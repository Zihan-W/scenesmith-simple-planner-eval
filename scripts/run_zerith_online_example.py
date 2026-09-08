#!/usr/bin/env python3
"""Compatibility CLI forwarding to shared environment, policy and runner factories."""

import argparse
import dataclasses
import math
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.online_manipulation.recipes.pick_environment import make_config
from src.online_manipulation.recipes.pick_policy import build_policy
from src.online_manipulation.assembly import run
from src.online_manipulation import (
    HoldPolicy,
    JointStepPolicy,
    JointStepPolicyConfig,
    PlanarPoseRandomizationSpec,
    TimingConfig,
    VisualizationConfig,
)

EVAL_PACKAGE_XML = REPOSITORY_ROOT / "models/zerith_pick_eval/package.xml"
ROBOT_MODEL_DIR = REPOSITORY_ROOT / "models/zerith_drake"
PICK_LIFT_CALIBRATION_JSON = (
    REPOSITORY_ROOT / "experiments/inputs/pick_lift/pick_lift_calibration.json"
)


def _parse_args(argv=None) -> argparse.Namespace:
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
    parser.add_argument("--pick-home-json", type=Path)
    parser.add_argument("--environment-json", type=Path)
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
    return parser.parse_args(argv)


def build_run(args):
    """Resolve legacy flags through shared factories without reading unused experts.

    Legacy control defaults (.01 joint step, .03 closed width, duration from
    max_steps) stay explicit CLI overrides. Episode identity/initial state and
    Task criteria have one source: the environment recipe.
    """
    if args.episodes < 1:
        raise ValueError("episodes must be positive")
    if args.target_xy_jitter_m < 0 or args.target_yaw_jitter_deg < 0:
        raise ValueError("Target jitter must be nonnegative")
    pick = args.policy == "pick-lift"
    if pick and args.pick_home_json is None:
        raise ValueError("pick-lift requires --pick-home-json; Hold/JointStep do not")
    config = make_config(
        repository_root=REPOSITORY_ROOT,
        scene_root=args.scene_package_xml.resolve().parent,
        pick_artifact_root=args.scene_dmd.resolve().parent,
        scene_dmd=args.scene_dmd,
        scene_package_xml=args.scene_package_xml,
        eval_package_xml=args.eval_package_xml,
        robot_model_dir=args.robot_model_dir,
        settings_path=args.environment_json,
        task_enabled=pick,
    )
    max_steps = args.max_steps if args.max_steps is not None else (600 if pick else 20)
    if max_steps < 1:
        raise ValueError("max-steps must be positive")
    timing = TimingConfig(args.physics_dt, args.controller_dt, args.policy_dt)
    observed = config.scenario.observed_bodies[0]
    xy, yaw = args.target_xy_jitter_m, math.radians(args.target_yaw_jitter_deg)
    randomizations = (
        (
            PlanarPoseRandomizationSpec(
                observed.observation_name, (-xy, xy), (-xy, xy), (-yaw, yaw)
            ),
        )
        if xy or yaw
        else ()
    )
    config = dataclasses.replace(
        config,
        scenario=dataclasses.replace(
            config.scenario,
            pose_randomizations=randomizations,
            visualization=VisualizationConfig(
                enabled=args.meshcat or args.record_html,
                port=args.meshcat_port,
                realtime_rate=args.realtime_rate,
            ),
            output_directory=args.output_root.resolve(),
        ),
        timing=timing,
        episode_duration=(max_steps + 1) * timing.policy_dt,
        max_joint_delta=args.maximum_joint_step,
        maximum_cartesian_joint_delta=args.maximum_cartesian_joint_step,
    )
    if args.policy == "hold":
        policy = HoldPolicy()
    elif args.policy == "joint-step":
        policy = JointStepPolicy(
            JointStepPolicyConfig(args.joint_name, args.joint_delta)
        )
    else:
        overrides = dict(
            closed_width_m=args.closed_width,
            lift_distance_m=args.lift_distance,
            cartesian_step_m=args.cartesian_step,
            maximum_alignment_error_m=args.maximum_alignment_error,
        )
        if args.approach_distance is not None:
            overrides["approach_distance_m"] = args.approach_distance
        policy = build_policy(
            config,
            pick_artifact_root=args.pick_home_json.resolve().parent,
            pick_home_json=args.pick_home_json,
            pregrasp_json=args.pregrasp_json,
            calibration_json=args.pick_lift_calibration_json,
            policy_overrides=overrides,
        )
    return config, policy, max_steps


def main():
    """Forward parsed options to the same public example runner."""
    args = _parse_args()
    config, policy, max_steps = build_run(args)
    results = run(
        config,
        policy,
        seeds=tuple(args.seed + i for i in range(args.episodes)),
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
