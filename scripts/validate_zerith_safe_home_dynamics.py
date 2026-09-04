#!/usr/bin/env python3
"""Validate a Zerith home posture under gravity-compensated dynamics."""

import argparse
import json
import sys

from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_grasp_geometry import (
    PICK_RAIL_POSITION_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)
from src.zerith_online_env import LEFT_ARM_SERVO_CONFIGS, ZerithOnlineEnv
from src.zerith_pregrasp_collision import (
    apply_safety_clearance_filters,
    build_pregrasp_planning_model,
    configuration_clearance_metrics,
)

EVAL_PACKAGE_XML = (
    REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"
)
ZERITH_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Hold a saved home posture for five seconds with the online "
            "200 Hz gravity-compensated PD servo and report stability "
            "metrics."
        )
    )
    parser.add_argument("scene_dmd", type=Path)
    parser.add_argument("--scene-package-xml", type=Path, required=True)
    parser.add_argument(
        "--home-json",
        "--safe-home-json",
        dest="home_json",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--home-key",
        choices=("q_safe_home", "q_pick_home"),
        default="q_safe_home",
        help="Joint-vector key loaded from --home-json.",
    )
    parser.add_argument(
        "--eval-package-xml",
        type=Path,
        default=EVAL_PACKAGE_XML,
    )
    parser.add_argument(
        "--robot-model-dir",
        type=Path,
        default=ZERITH_MODEL_DIR,
    )
    parser.add_argument(
        "--rail-position",
        type=float,
        default=PICK_RAIL_POSITION_METERS,
    )
    parser.add_argument("--hold-duration", type=float, default=5.0)
    parser.add_argument("--physics-dt", type=float, default=0.001)
    parser.add_argument("--controller-dt", type=float, default=0.005)
    parser.add_argument("--policy-dt", type=float, default=0.1)
    parser.add_argument(
        "--maximum-final-joint-error",
        type=float,
        default=0.02,
    )
    parser.add_argument(
        "--maximum-final-joint-speed",
        type=float,
        default=0.05,
    )
    parser.add_argument(
        "--maximum-continuous-saturation-duration",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Defaults to safe_home_dynamics.json beside safe-home-json.",
    )
    return parser.parse_args()


def _load_home(path: Path, home_key: str) -> np.ndarray:
    """Load one selected seven-joint home posture from search output."""
    payload = json.loads(path.read_text(encoding="utf-8"))
    q_home = np.asarray(payload[home_key], dtype=float)
    if q_home.shape != (7,) or not np.all(np.isfinite(q_home)):
        raise ValueError(
            f"safe-home-json contains an invalid {home_key}"
        )
    return q_home


def _maximum_saturation_runs(samples, controller_dt: float) -> dict:
    """Return maximum continuous saturation duration for each arm joint."""
    maximum_runs = np.zeros(7, dtype=int)
    current_runs = np.zeros(7, dtype=int)
    for sample in samples:
        saturated = sample.saturated[:7]
        current_runs = np.where(saturated, current_runs + 1, 0)
        maximum_runs = np.maximum(maximum_runs, current_runs)
    return {
        config.name: float(run_count * controller_dt)
        for config, run_count in zip(
            LEFT_ARM_SERVO_CONFIGS,
            maximum_runs,
            strict=True,
        )
    }


def _torque_summary(samples) -> dict:
    """Summarize gravity, PD, raw, applied, and saturation signals."""
    quantities = (
        "gravity_torque",
        "pd_torque",
        "raw_torque",
        "applied_torque",
    )
    output = {}
    for joint_index, config in enumerate(LEFT_ARM_SERVO_CONFIGS):
        joint_output = {}
        for quantity in quantities:
            values = np.array(
                [getattr(sample, quantity)[joint_index] for sample in samples]
            )
            joint_output[f"maximum_absolute_{quantity}"] = float(
                np.max(np.abs(values))
            )
        joint_output["saturation_sample_count"] = int(
            sum(sample.saturated[joint_index] for sample in samples)
        )
        output[config.name] = joint_output
    return output


def main() -> None:
    """Run the selected home posture dynamics hold regression."""
    args = _parse_args()
    if args.hold_duration <= 0.0:
        raise ValueError("hold-duration must be positive")
    policy_steps = round(args.hold_duration / args.policy_dt)
    if not np.isclose(
        policy_steps * args.policy_dt,
        args.hold_duration,
        atol=1e-12,
    ):
        raise ValueError("hold-duration must be a multiple of policy-dt")

    scene_dmd = args.scene_dmd.resolve()
    scene_package_xml = args.scene_package_xml.resolve()
    eval_package_xml = args.eval_package_xml.resolve()
    robot_model_dir = args.robot_model_dir.resolve()
    home_json = args.home_json.resolve()
    q_home = _load_home(home_json, args.home_key)

    planning_model = build_pregrasp_planning_model(
        scene_dmd=scene_dmd,
        scene_package_xml=scene_package_xml,
        eval_package_xml=eval_package_xml,
        robot_model_dir=robot_model_dir,
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
        rail_position=args.rail_position,
    )
    apply_safety_clearance_filters(
        planning_model,
        influence_distance=0.017,
    )
    q_scene = planning_model.q_scene.copy()
    q_scene[planning_model.arm_indices] = q_home
    clearance_metrics = configuration_clearance_metrics(
        planning_model,
        q_scene,
        influence_distance=0.05,
    )

    env = ZerithOnlineEnv(
        scene_dmd=scene_dmd,
        scene_package_xml=scene_package_xml,
        additional_package_xmls=[eval_package_xml],
        robot_model_dir=robot_model_dir,
        target_model_name="living_room_box_0",
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
        rail_position=args.rail_position,
        q_home=q_home,
        physics_dt=args.physics_dt,
        controller_dt=args.controller_dt,
        policy_dt=args.policy_dt,
        episode_duration=args.hold_duration + args.policy_dt,
        max_joint_delta=0.1,
        realtime_rate=0.0,
    )
    initial_observation = env.reset()
    initial_penetrations = env.robot_penetrations()
    observations = [initial_observation]
    for _ in range(policy_steps):
        observation, _, _, _ = env.step(np.r_[np.zeros(7), 1.0])
        observations.append(observation)

    samples = env.control_log
    q_errors = np.vstack(
        [np.abs(observation["q_left"] - q_home) for observation in observations]
    )
    final_observation = observations[-1]
    final_joint_error = np.abs(final_observation["q_left"] - q_home)
    final_joint_speed = np.abs(final_observation["v_left"])
    saturation_runs = _maximum_saturation_runs(samples, args.controller_dt)
    maximum_saturation_duration = max(saturation_runs.values())
    final_penetrations = env.robot_penetrations()
    passed = bool(
        not initial_penetrations
        and not final_penetrations
        and np.max(final_joint_error) <= args.maximum_final_joint_error
        and np.max(final_joint_speed) <= args.maximum_final_joint_speed
        and maximum_saturation_duration
        <= args.maximum_continuous_saturation_duration
    )

    output = {
        "passed": passed,
        "home_key": args.home_key,
        args.home_key: q_home.tolist(),
        "distance_unit": "meters",
        f"{args.home_key}_clearance_metrics": clearance_metrics,
        "hold_duration_s": args.hold_duration,
        "physics_frequency_hz": 1.0 / args.physics_dt,
        "controller_frequency_hz": 1.0 / args.controller_dt,
        "policy_frequency_hz": 1.0 / args.policy_dt,
        "maximum_joint_error_during_hold_rad": float(np.max(q_errors)),
        "final_joint_error_rad": final_joint_error.tolist(),
        "maximum_final_joint_error_rad": float(np.max(final_joint_error)),
        "final_joint_speed_rad_per_s": final_joint_speed.tolist(),
        "maximum_final_joint_speed_rad_per_s": float(
            np.max(final_joint_speed)
        ),
        "maximum_continuous_saturation_duration_s": (
            maximum_saturation_duration
        ),
        "continuous_saturation_duration_by_joint_s": saturation_runs,
        "torque_summary": _torque_summary(samples),
        "initial_robot_penetrations": [
            vars(penetration) for penetration in initial_penetrations
        ],
        "final_robot_penetrations": [
            vars(penetration) for penetration in final_penetrations
        ],
        "thresholds": {
            "maximum_final_joint_error_rad": (
                args.maximum_final_joint_error
            ),
            "maximum_final_joint_speed_rad_per_s": (
                args.maximum_final_joint_speed
            ),
            "maximum_continuous_saturation_duration_s": (
                args.maximum_continuous_saturation_duration
            ),
        },
    }
    output_path = (
        args.output_json.resolve()
        if args.output_json is not None
        else home_json.parent / f"{args.home_key}_dynamics.json"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(
        f"{args.home_key} clearances: "
        f"nonpenetration={clearance_metrics['minimum_nonpenetration_distance']:.6f} m, "
        f"safety={clearance_metrics['minimum_safety_clearance']:.6f} m"
    )
    print(
        "Dynamics hold: "
        f"final_error={np.max(final_joint_error):.6f} rad, "
        f"final_speed={np.max(final_joint_speed):.6f} rad/s, "
        f"max_saturation_run={maximum_saturation_duration:.3f} s"
    )
    print(f"Diagnostics: {output_path}")
    if not passed:
        raise RuntimeError(
            f"{args.home_key} dynamics stability validation failed"
        )
    print(
        f"PASS: {args.home_key} remained dynamically stable for the hold "
        "period"
    )


if __name__ == "__main__":
    main()
