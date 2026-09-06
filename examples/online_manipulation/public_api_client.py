#!/usr/bin/env python3
"""Repository-external-style client using only the public online API."""

import argparse
import json
from pathlib import Path

from src.online_manipulation import (
    HoldPolicy,
    NullTask,
    ObservedBodySpec,
    ScenarioSpec,
    ZerithEnvironmentConfig,
    make_env,
)


def make_config(repository_root: Path) -> ZerithEnvironmentConfig:
    """Build a portable config for the repository-owned minimal scene."""
    root = Path(repository_root).resolve()
    scene_root = root / "models" / "online_env_minimal_scene"
    return ZerithEnvironmentConfig(
        scenario=ScenarioSpec(
            dmd_path=scene_root / "scene.dmd.yaml",
            package_xmls=(scene_root / "package.xml",),
            observed_bodies=(
                ObservedBodySpec(
                    observation_name="portable_object",
                    model_instance_name="portable_test_box",
                    body_name="base_link",
                    write_back=True,
                ),
            ),
        ),
        robot_model_dir=root / "models" / "zerith_drake",
        # The floor top is z=0.0.  This height leaves the wheel collision
        # proxies 1.6 mm above it in the default configuration.
        robot_xyz=(0.0, 0.0, 0.2315),
        robot_yaw_deg=0.0,
        rail_position=0.4,
        q_home_left=(0.0,) * 7,
        episode_duration=1.0,
        task=NullTask(),
    )


def run_one_step(config: ZerithEnvironmentConfig, seed: int) -> dict:
    """Reset, ask an external policy for one action, and step once."""
    env = make_env(config)
    policy = HoldPolicy()
    observation, reset_info = env.reset(seed=seed)
    policy.reset(observation, reset_info)
    action = policy.act(observation)
    observation, reward, terminated, truncated, info = env.step(action)
    return {
        "seed": reset_info["seed"],
        "action_type": type(action).__name__,
        "simulation_time_s": observation.time_s,
        "joint_count": len(observation.robot.joint_names),
        "object_names": sorted(observation.objects),
        "reward": reward,
        "terminated": terminated,
        "truncated": truncated,
        "action_status": info["action_decision"]["status"],
    }


def main() -> None:
    """Run the public API smoke from any working directory."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repository-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    print(json.dumps(run_one_step(make_config(args.repository_root), args.seed)))


if __name__ == "__main__":
    main()
