#!/usr/bin/env python3
"""Repository-external camera client using only the public project API."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

from src.online_manipulation import (
    HoldAction,
    NullTask,
    ScenarioSpec,
    ZerithEnvironmentConfig,
    make_env,
    make_zerith_camera_specs,
)


class ImageReadingPolicy:
    """Read an RGB observation and return a state-preserving action."""

    def __init__(self, camera_name: str):
        """Store the stable public camera name."""
        self.camera_name = camera_name
        self.last_rgb_mean = 0.0

    def reset(self, observation, info) -> None:
        """Reset policy telemetry without reaching into the environment."""
        del observation, info
        self.last_rgb_mean = 0.0

    def act(self, observation):
        """Read the latest held RGB frame and keep the robot still."""
        image = observation.sensors[self.camera_name].rgb
        self.last_rgb_mean = float(image.mean())
        return HoldAction()


def make_config(repository_root: Path, dmd_path: Path):
    """Build a head-camera environment from public configuration only."""
    scene_root = repository_root / "models" / "online_env_minimal_scene"
    return ZerithEnvironmentConfig(
        scenario=ScenarioSpec(
            dmd_path=dmd_path,
            package_xmls=(scene_root / "package.xml",),
        ),
        robot_model_dir=repository_root / "models" / "zerith_drake",
        robot_xyz=(0.0, 0.0, 0.2315),
        robot_yaw_deg=0.0,
        rail_position=0.4,
        q_home_left=(0.0,) * 7,
        episode_duration=1.0,
        task=NullTask(),
        cameras=make_zerith_camera_specs(
            enabled_names=("head_camera",),
            width=320,
            height=240,
            update_period_s=0.05,
        ),
    )


def main() -> None:
    """Save one RGB/depth sample and execute one image-reading policy step."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repository-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    parser.add_argument("--scene-dmd", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    repository_root = args.repository_root.resolve()
    scene_dmd = (
        args.scene_dmd.resolve()
        if args.scene_dmd is not None
        else repository_root
        / "models"
        / "online_env_minimal_scene"
        / "scene.dmd.yaml"
    )

    env = make_env(make_config(repository_root, scene_dmd))
    observation, info = env.reset(seed=args.seed)
    camera = observation.sensors["head_camera"]
    camera_metadata = camera.as_dict()
    if any(name in camera_metadata for name in ("rgb", "depth", "label")):
        raise RuntimeError("Default camera serialization unexpectedly includes images")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rgb_path = args.output_dir / "head_camera_rgb.png"
    depth_path = args.output_dir / "head_camera_depth_m.npy"
    Image.fromarray(camera.rgb).save(rgb_path)
    np.save(depth_path, camera.depth)

    valid_depth = np.isfinite(camera.depth) & (camera.depth > 0.0)
    if not np.any(valid_depth):
        raise RuntimeError("Head camera produced no valid metric depth pixels")
    policy = ImageReadingPolicy("head_camera")
    policy.reset(observation, info)
    observation, reward, terminated, truncated, step_info = env.step(
        policy.act(observation)
    )
    payload = {
        "camera": "head_camera",
        "frame": camera.frame,
        "timestamp_s": camera.timestamp_s,
        "intrinsics": camera.intrinsics.as_dict(),
        "rgb_shape": list(camera.rgb.shape),
        "rgb_dtype": str(camera.rgb.dtype),
        "depth_shape": list(camera.depth.shape),
        "depth_dtype": str(camera.depth.dtype),
        "depth_unit": "m",
        "default_serialization_contains_images": False,
        "image_metadata": camera_metadata["image_metadata"],
        "valid_depth_range_m": [
            float(camera.depth[valid_depth].min()),
            float(camera.depth[valid_depth].max()),
        ],
        "policy_rgb_mean": policy.last_rgb_mean,
        "simulation_time_s": observation.time_s,
        "reward": reward,
        "terminated": terminated,
        "truncated": truncated,
        "action_status": step_info["action_decision"]["status"],
        "rgb_path": str(rgb_path.resolve()),
        "depth_path": str(depth_path.resolve()),
    }
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
