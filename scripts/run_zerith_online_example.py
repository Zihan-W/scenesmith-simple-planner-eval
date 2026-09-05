#!/usr/bin/env python3
"""Run external online policies against the calibrated Zerith pick scene."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.online_manipulation import (  # noqa: E402
    HoldPolicy,
    JointStepPolicy,
    JointStepPolicyConfig,
    NullTask,
    ObservedBodySpec,
    PlanarPoseRandomizationSpec,
    PickLiftPolicy,
    PickLiftPolicyConfig,
    PickLiftTask,
    PickLiftTaskConfig,
    Pose,
    ScenarioSpec,
    TimingConfig,
    VisualizationConfig,
    ZerithEnvironmentConfig,
    make_env,
    make_zerith_robot_spec,
    run_episodes,
)
from src.zerith_robot_config import (  # noqa: E402
    PICK_RAIL_POSITION_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)

EVAL_PACKAGE_XML = (
    REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"
)
ROBOT_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"
TARGET_MODEL_NAME = "living_room_box_0"
TARGET_BODY_NAME = "base_link"
TARGET_OBSERVATION_NAME = "pick_target"
SUPPORT_MODEL_NAME = "living_room_coffee_table_0"
SUPPORT_BODY_NAME = "base_link"
PICK_LIFT_CALIBRATION_JSON = (
    REPOSITORY_ROOT
    / "models"
    / "zerith_pick_eval"
    / "pick_lift_calibration.json"
)


def _parse_args() -> argparse.Namespace:
    """Parse the shared example command-line interface."""
    parser = argparse.ArgumentParser(
        description=(
            "Run HoldPolicy, JointStepPolicy, or physical PickLiftPolicy "
            "through the generic online environment and EpisodeRunner."
        )
    )
    parser.add_argument(
        "policy",
        choices=("hold", "joint-step", "pick-lift"),
    )
    parser.add_argument("scene_dmd", type=Path)
    parser.add_argument("--scene-package-xml", type=Path, required=True)
    parser.add_argument("--pick-home-json", type=Path, required=True)
    parser.add_argument("--pregrasp-json", type=Path)
    parser.add_argument(
        "--pick-lift-calibration-json",
        type=Path,
        default=PICK_LIFT_CALIBRATION_JSON,
    )
    parser.add_argument("--eval-package-xml", type=Path, default=EVAL_PACKAGE_XML)
    parser.add_argument("--robot-model-dir", type=Path, default=ROBOT_MODEL_DIR)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int)
    parser.add_argument("--joint-name", default="left_shoulder_pitch_joint")
    parser.add_argument("--joint-delta", type=float, default=0.03)
    parser.add_argument("--maximum-joint-step", type=float, default=0.01)
    parser.add_argument(
        "--maximum-cartesian-joint-step",
        type=float,
        default=0.02,
        help=(
            "Per-policy differential-IK joint bound; independent of the "
            "held-target slew bound in --maximum-joint-step."
        ),
    )
    parser.add_argument("--approach-distance", type=float)
    parser.add_argument("--closed-width", type=float, default=0.03)
    parser.add_argument("--lift-distance", type=float, default=0.1)
    parser.add_argument("--cartesian-step", type=float, default=0.003)
    parser.add_argument(
        "--target-xy-jitter-m",
        type=float,
        default=0.0,
        help="Symmetric reset-time target X/Y offset range.",
    )
    parser.add_argument(
        "--target-yaw-jitter-deg",
        type=float,
        default=0.0,
        help="Symmetric reset-time target yaw offset range in degrees.",
    )
    parser.add_argument(
        "--maximum-alignment-error",
        type=float,
        default=0.015,
    )
    parser.add_argument("--policy-dt", type=float, default=0.1)
    parser.add_argument("--controller-dt", type=float, default=0.005)
    parser.add_argument("--physics-dt", type=float, default=0.001)
    parser.add_argument("--meshcat", action="store_true")
    parser.add_argument("--meshcat-port", type=int)
    parser.add_argument("--record-html", action="store_true")
    parser.add_argument("--write-final-dmd", action="store_true")
    parser.add_argument("--realtime-rate", type=float, default=0.0)
    return parser.parse_args()


def _load_calibration(
    pick_home_json: Path,
    pregrasp_json: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Load PICK_HOME, PREGRASP, approach axis, and center distance."""
    home_payload = json.loads(pick_home_json.read_text(encoding="utf-8"))
    pregrasp_payload = json.loads(
        pregrasp_json.read_text(encoding="utf-8")
    )
    q_home = np.asarray(home_payload["q_pick_home"], dtype=float)
    q_pregrasp = np.asarray(home_payload["q_pregrasp"], dtype=float)
    solution = pregrasp_payload["solution"]
    axis = np.asarray(solution["grasp_axes_world"]["approach"], dtype=float)
    target = np.asarray(
        pregrasp_payload["target_collision_center_xyz_m"],
        dtype=float,
    )
    actual_pregrasp = np.asarray(
        solution["actual_pregrasp_pose"]["translation_xyz_m"],
        dtype=float,
    )
    if q_home.shape != (7,) or q_pregrasp.shape != (7,):
        raise ValueError("PICK_HOME and PREGRASP must each contain 7 joints")
    if axis.shape != (3,) or target.shape != (3,):
        raise ValueError("PREGRASP geometry has invalid dimensions")
    axis /= np.linalg.norm(axis)
    approach_distance = float(axis @ (target - actual_pregrasp))
    if approach_distance <= 0.0:
        raise ValueError("PREGRASP approach axis points away from the target")
    return q_home, q_pregrasp, axis, approach_distance


def _load_target_relative_pose(payload: dict, name: str) -> Pose:
    """Load one target-relative pose from the pick-lift calibration."""
    pose = payload[name]
    return Pose(
        tuple(pose["translation_m"]),
        tuple(pose["quaternion_wxyz"]),
    )


def main() -> None:
    """Construct the configured external policy, environment, and runner."""
    args = _parse_args()
    if args.episodes < 1:
        raise ValueError("episodes must be positive")
    if args.target_xy_jitter_m < 0.0:
        raise ValueError("target-xy-jitter-m must be nonnegative")
    if args.target_yaw_jitter_deg < 0.0:
        raise ValueError("target-yaw-jitter-deg must be nonnegative")
    pregrasp_json = (
        args.pregrasp_json.resolve()
        if args.pregrasp_json is not None
        else args.pick_home_json.resolve().parent / "pregrasp_ik.json"
    )
    q_home, q_pregrasp, approach_axis, calibrated_approach = (
        _load_calibration(args.pick_home_json.resolve(), pregrasp_json)
    )
    pick_lift_calibration = json.loads(
        args.pick_lift_calibration_json.resolve().read_text(encoding="utf-8")
    )
    staging_pose_in_target = _load_target_relative_pose(
        pick_lift_calibration,
        "staging_pose_in_target",
    )
    grasp_pose_in_target = _load_target_relative_pose(
        pick_lift_calibration,
        "grasp_pose_in_target",
    )
    timing = TimingConfig(
        physics_dt=args.physics_dt,
        controller_dt=args.controller_dt,
        policy_dt=args.policy_dt,
    )
    robot_spec = make_zerith_robot_spec(
        robot_model_dir=args.robot_model_dir.resolve(),
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
        rail_position=PICK_RAIL_POSITION_METERS,
        q_home_left=q_home,
    )
    target_qualified_name = f"{TARGET_MODEL_NAME}::{TARGET_BODY_NAME}"
    xy_jitter = args.target_xy_jitter_m
    yaw_jitter = np.deg2rad(args.target_yaw_jitter_deg)
    pose_randomizations = ()
    if xy_jitter > 0.0 or yaw_jitter > 0.0:
        pose_randomizations = (
            PlanarPoseRandomizationSpec(
                observation_name=TARGET_OBSERVATION_NAME,
                x_offset_range_m=(-xy_jitter, xy_jitter),
                y_offset_range_m=(-xy_jitter, xy_jitter),
                yaw_offset_range_rad=(-yaw_jitter, yaw_jitter),
            ),
        )
    scenario = ScenarioSpec(
        dmd_path=args.scene_dmd.resolve(),
        package_xmls=(
            args.scene_package_xml.resolve(),
            args.eval_package_xml.resolve(),
        ),
        observed_bodies=(
            ObservedBodySpec(
                observation_name=TARGET_OBSERVATION_NAME,
                model_instance_name=TARGET_MODEL_NAME,
                body_name=TARGET_BODY_NAME,
                write_back=True,
            ),
        ),
        pose_randomizations=pose_randomizations,
        visualization=VisualizationConfig(
            enabled=args.meshcat or args.record_html,
            port=args.meshcat_port,
            realtime_rate=args.realtime_rate,
        ),
        output_directory=args.output_root.resolve(),
    )

    task = NullTask()
    if args.policy == "hold":
        policy = HoldPolicy()
        default_max_steps = 20
    elif args.policy == "joint-step":
        policy = JointStepPolicy(
            JointStepPolicyConfig(args.joint_name, args.joint_delta)
        )
        default_max_steps = 20
    else:
        finger_bodies = tuple(
            f"{robot_spec.model_instance_name}::{name}"
            for name in (
                "left_jaw_left_finger_link",
                "left_jaw_right_finger_link",
            )
        )
        support_body = f"{SUPPORT_MODEL_NAME}::{SUPPORT_BODY_NAME}"
        task = PickLiftTask(
            PickLiftTaskConfig(
                target_observation_name=TARGET_OBSERVATION_NAME,
                gripper_contact_bodies=finger_bodies,
                target_contact_body=target_qualified_name,
                support_contact_bodies=(support_body,),
                required_lift_m=0.08,
                required_hold_s=3.0,
            )
        )
        policy = PickLiftPolicy(
            PickLiftPolicyConfig(
                target_observation_name=TARGET_OBSERVATION_NAME,
                arm_joint_names=robot_spec.controlled_joint_names[:7],
                pregrasp_joint_positions=tuple(q_pregrasp),
                end_effector_frame=robot_spec.end_effector_frame_name,
                approach_axis_world=tuple(approach_axis),
                finger_contact_bodies=finger_bodies,
                target_contact_body=target_qualified_name,
                support_contact_bodies=(support_body,),
                open_width_m=robot_spec.gripper.maximum_width_m,
                closed_width_m=args.closed_width,
                approach_distance_m=(
                    args.approach_distance
                    if args.approach_distance is not None
                    else calibrated_approach
                ),
                lift_distance_m=args.lift_distance,
                cartesian_step_m=args.cartesian_step,
                maximum_alignment_error_m=args.maximum_alignment_error,
                staging_pose_in_target=staging_pose_in_target,
                grasp_pose_in_target=grasp_pose_in_target,
            )
        )
        default_max_steps = 600

    max_steps = args.max_steps or default_max_steps
    env = make_env(
        ZerithEnvironmentConfig(
            scenario=scenario,
            robot_model_dir=args.robot_model_dir.resolve(),
            robot_xyz=ROBOT_BASE_XYZ_METERS,
            robot_yaw_deg=ROBOT_BASE_YAW_DEG,
            rail_position=PICK_RAIL_POSITION_METERS,
            q_home_left=tuple(q_home),
            timing=timing,
            target_model_name=TARGET_MODEL_NAME,
            target_body_name=TARGET_BODY_NAME,
            episode_duration=(max_steps + 1) * timing.policy_dt,
            max_joint_delta=args.maximum_joint_step,
            maximum_cartesian_joint_delta=(
                args.maximum_cartesian_joint_step
            ),
            task=task,
            enable_planning_query=args.policy == "pick-lift",
        ),
    )
    if env.backend.runtime.meshcat is not None:
        print(f"Meshcat URL: {env.backend.runtime.meshcat.web_url()}")
    results = run_episodes(
        env=env,
        policy=policy,
        seeds=tuple(args.seed + index for index in range(args.episodes)),
        max_steps=max_steps,
        output_root=args.output_root.resolve(),
        record_html=args.record_html,
        write_final_dmd=args.write_final_dmd,
    )
    for index, result in enumerate(results):
        print(
            f"Episode {index}: success={result.success}, "
            f"reason={result.summary['termination_reason']}, "
            f"steps={result.summary['policy_steps']}"
        )


if __name__ == "__main__":
    main()
