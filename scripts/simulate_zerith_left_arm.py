#!/usr/bin/env python3
"""Run online Zerith hold-pose and per-joint step regression tests."""

import argparse
import sys

from pathlib import Path

import numpy as np

from pydrake.all import Meshcat

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_online_env import LEFT_ARM_SERVO_CONFIGS, ZerithOnlineEnv

ZERITH_MODEL_RELATIVE_PATH = Path("models/zerith_drake")


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Exercise Zerith through an online reset()/step(action) control "
            "environment without RRT, TOPPRA, or a trajectory file."
        )
    )
    parser.add_argument(
        "scene_dmd",
        type=Path,
        help="Path to house_furniture_welded.dmd.yaml.",
    )
    parser.add_argument(
        "--scene-package-xml",
        type=Path,
        help="Scene package.xml; inferred from scene_dmd when omitted.",
    )
    parser.add_argument(
        "--robot-model-dir",
        type=Path,
        default=REPOSITORY_ROOT / ZERITH_MODEL_RELATIVE_PATH,
        help="Generated Drake package containing Zerith's urdf/ and meshes/.",
    )
    parser.add_argument(
        "--target-model-name",
        default="living_room_box_0",
        help="Scene model instance exposed as red_box_pose in observations.",
    )
    parser.add_argument(
        "--robot-xyz",
        type=float,
        nargs=3,
        default=(3.05, 3.07, 0.1815),
        metavar=("X", "Y", "Z"),
        help=(
            "World position of the welded dipan_link in meters "
            "(default: east of the coffee table)."
        ),
    )
    parser.add_argument(
        "--robot-yaw-deg",
        type=float,
        default=180.0,
        help=(
            "World yaw of dipan_link in degrees "
            "(default: facing the coffee table)."
        ),
    )
    parser.add_argument(
        "--q-home",
        type=float,
        nargs=7,
        default=(0.0,) * 7,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        help="Seven-joint left-arm home posture in radians.",
    )
    parser.add_argument(
        "--time-step",
        type=float,
        default=0.001,
        help="Drake plant time step in seconds (default: 0.001, 1000 Hz).",
    )
    parser.add_argument(
        "--control-period",
        type=float,
        default=0.005,
        help="Gravity-compensated PD period (default: 0.005, 200 Hz).",
    )
    parser.add_argument(
        "--policy-period",
        type=float,
        default=0.1,
        help="Policy action period (default: 0.1, 10 Hz).",
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=5.0,
        help="Duration of the q_home hold regression in seconds.",
    )
    parser.add_argument(
        "--step-delta",
        type=float,
        default=0.1,
        help="Per-joint regression step size in radians.",
    )
    parser.add_argument(
        "--step-settle-duration",
        type=float,
        default=1.0,
        help="Hold time after each positive and negative joint step.",
    )
    parser.add_argument(
        "--realtime-rate",
        type=float,
        default=1.0,
        help="Target simulator realtime rate; zero runs as fast as possible.",
    )
    parser.add_argument(
        "--meshcat-port",
        type=int,
        help="Meshcat port; Drake selects an available port when omitted.",
    )
    parser.add_argument(
        "--initial-penetration-limit",
        type=float,
        help="Fail if an active initial penetration exceeds this depth.",
    )
    parser.add_argument(
        "--control-log",
        type=Path,
        default=Path("zerith_online_control.csv"),
        help="Controller-frequency CSV output path.",
    )
    parser.add_argument(
        "--record-html",
        type=Path,
        default=Path("zerith_online_control.html"),
        help="Static Meshcat recording output path.",
    )
    return parser.parse_args()


def _policy_steps(duration: float, policy_period: float) -> int:
    """Convert a duration to an exact positive number of policy steps."""
    if duration <= 0.0:
        raise ValueError("Test durations must be positive")
    steps = round(duration / policy_period)
    if steps < 1 or not np.isclose(
        steps * policy_period,
        duration,
        atol=1e-12,
    ):
        raise ValueError(
            f"Duration {duration} must be an integer multiple of "
            f"policy period {policy_period}"
        )
    return steps


def _run_zero_actions(env: ZerithOnlineEnv, count: int) -> dict:
    """Advance the environment with a held arm target and open gripper."""
    observation = None
    for _ in range(count):
        observation, _, done, _ = env.step(
            np.r_[np.zeros(7), 1.0]
        )
        if done:
            raise RuntimeError(
                "Episode ended before regression test completed"
            )
    assert observation is not None
    return observation


def _saturation_ratio(samples) -> float:
    """Return the fraction of arm actuator samples that were saturated."""
    if not samples:
        return 0.0
    saturated = np.vstack([sample.saturated[:7] for sample in samples])
    return float(np.mean(saturated))


def main() -> None:
    """Run online hold and step policies through the reset/step interface."""
    args = _parse_args()
    hold_steps = _policy_steps(args.hold_duration, args.policy_period)
    settle_steps = _policy_steps(
        args.step_settle_duration,
        args.policy_period,
    )
    episode_duration = (
        args.hold_duration
        + 2.0
        * len(LEFT_ARM_SERVO_CONFIGS)
        * (args.policy_period + args.step_settle_duration)
        + args.policy_period
    )

    meshcat = Meshcat(args.meshcat_port)
    env = ZerithOnlineEnv(
        scene_dmd=args.scene_dmd,
        scene_package_xml=args.scene_package_xml,
        robot_model_dir=args.robot_model_dir,
        target_model_name=args.target_model_name,
        robot_xyz=args.robot_xyz,
        robot_yaw_deg=args.robot_yaw_deg,
        q_home=args.q_home,
        physics_dt=args.time_step,
        controller_dt=args.control_period,
        policy_dt=args.policy_period,
        episode_duration=episode_duration,
        max_joint_delta=abs(args.step_delta),
        realtime_rate=args.realtime_rate,
        meshcat=meshcat,
    )

    meshcat.StartRecording()
    observation = env.reset()
    penetrations = env.robot_penetrations()
    print(f"Meshcat URL: {meshcat.web_url()}")
    print(
        "Frequencies: "
        f"physics={1.0 / env.physics_dt:.0f} Hz, "
        f"servo={1.0 / env.controller_dt:.0f} Hz, "
        f"policy={1.0 / env.policy_dt:.0f} Hz"
    )
    print(
        "Initial active robot penetration pairs: "
        f"{len(penetrations)}"
    )
    for penetration in penetrations:
        print(
            f"  {penetration.depth:.6f} m: "
            f"{penetration.frame_a} <-> {penetration.frame_b}"
        )
    if (
        args.initial_penetration_limit is not None
        and penetrations
        and penetrations[0].depth > args.initial_penetration_limit
    ):
        raise ValueError(
            f"Maximum initial penetration {penetrations[0].depth:.6f} m "
            f"exceeds limit {args.initial_penetration_limit:.6f} m"
        )

    hold_log_start = len(env.control_log)
    observation = _run_zero_actions(env, hold_steps)
    hold_samples = env.control_log[hold_log_start:]
    hold_error = np.abs(observation["q_left"] - env.q_home)
    print("q_home hold test completed")
    print(f"  max final joint error: {np.max(hold_error):.6f} rad")
    print(
        "  arm torque saturation ratio: "
        f"{_saturation_ratio(hold_samples):.6f}"
    )

    for joint_index, config in enumerate(LEFT_ARM_SERVO_CONFIGS):
        positive_action = np.r_[np.zeros(7), 1.0]
        positive_action[joint_index] = args.step_delta
        observation, _, _, positive_info = env.step(positive_action)
        observation = _run_zero_actions(env, settle_steps)
        positive_error = (
            positive_info["desired_q_left"][joint_index]
            - observation["q_left"][joint_index]
        )

        negative_action = np.r_[np.zeros(7), 1.0]
        negative_action[joint_index] = -args.step_delta
        observation, _, _, negative_info = env.step(negative_action)
        observation = _run_zero_actions(env, settle_steps)
        return_error = (
            negative_info["desired_q_left"][joint_index]
            - observation["q_left"][joint_index]
        )
        print(
            f"{config.name}: "
            f"step_error={positive_error:.6f} rad, "
            f"return_error={return_error:.6f} rad"
        )

    env.write_control_log(args.control_log)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    args.record_html.write_text(meshcat.StaticHtml())
    print(f"Control log: {args.control_log.resolve()}")
    print(f"Meshcat recording: {args.record_html.resolve()}")


if __name__ == "__main__":
    main()
