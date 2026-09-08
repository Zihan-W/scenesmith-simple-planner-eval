#!/usr/bin/env python3
"""Repository-external-style client using only the public online API."""

import argparse
import json
from pathlib import Path

from src.online_manipulation import (
    make_minimal_config as make_config,
    HoldPolicy,
    ZerithEnvironmentConfig,
    make_env,
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
