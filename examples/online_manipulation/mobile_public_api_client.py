"""Repository-external client for actual dual-arm / base / camera observations."""

import argparse
import dataclasses
import json
from pathlib import Path

import numpy as np
from PIL import Image

from src.online_manipulation import (
    BaseConfig,
    BaseVelocityAction,
    GripperAction,
    JointDeltaAction,
    RobotCommand,
    RuntimeConfig,
    ScenarioSpec,
    ZerithMobileRobotAdapter,
    make_env,
    make_zerith_camera_specs,
    make_zerith_dual_spec,
)


def main():
    """Construct without expert files and control every group in one step."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--mode", choices=("planar_kinematic", "wheel_dynamic"), required=True
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = args.repo_root.resolve()
    base = BaseConfig(
        args.mode, base_height_m=0.1816 if args.mode == "planar_kinematic" else 0.1808
    )
    spec = make_zerith_dual_spec(
        robot_model_dir=root / "models/zerith_drake",
        robot_xyz=(0, 0, base.base_height_m),
        robot_yaw_deg=0,
        rail_position=0.4,
        q_home_left=(0,) * 7,
    )
    spec = dataclasses.replace(
        spec,
        cameras=make_zerith_camera_specs(
            enabled_names=("head_camera", "left_wrist_camera", "right_wrist_camera"),
            width=160,
            height=120,
            update_period_s=0.1,
        ),
    )
    scene = root / "models/mobile_scene"
    env = make_env(
        RuntimeConfig(
            ScenarioSpec(
                scene / "obstacle.dmd.yaml",
                (scene / "package.xml",),
                ground_body_names=("ground::floor",),
            ),
            ZerithMobileRobotAdapter(spec, base),
        )
    )
    obs, reset_info = env.reset(seed=0)
    trace = [obs.as_dict()]
    for _ in range(5):
        action = RobotCommand(
            arms={
                side: JointDeltaAction((spec.arm_groups[side][0],), (-0.004,))
                for side in ("left", "right")
            },
            grippers={"left": GripperAction(0.06), "right": GripperAction(0.055)},
            base=BaseVelocityAction(0.06, 0.1),
        )
        obs, reward, terminated, truncated, info = env.step(action)
        if not info["action_decision"]["accepted"] or terminated or truncated:
            raise RuntimeError(info)
        trace.append(obs.as_dict())
        query = env.get_planning_query()
        if abs(query.state_time_s - obs.time_s) > 1e-9:
            raise AssertionError("Planning snapshot has a stale timestamp")
        for side, frame_name in spec.end_effector_frames.items():
            actual = obs.robot.end_effectors[side]
            planned = query.frame_pose_at(
                query.configuration(), spec.model_instance_name, frame_name
            )
            np.testing.assert_allclose(
                planned.translation_m, actual.translation_m, atol=1e-10
            )
    for name, sensor in obs.sensors.items():
        Image.fromarray(sensor.rgb).save(args.output / f"{name}_rgb.png")
        np.save(args.output / f"{name}_depth.npy", sensor.depth)
    result = {"reset": reset_info, "observations": trace, "last_step": info}
    (args.output / "client.json").write_text(json.dumps(result, indent=2))
    print(
        json.dumps(
            {
                "mode": args.mode,
                "time_s": obs.time_s,
                "grippers": dict(obs.robot.gripper_widths_m),
                "sensor_times": {
                    name: frame.timestamp_s for name, frame in obs.sensors.items()
                },
            }
        )
    )


if __name__ == "__main__":
    main()
