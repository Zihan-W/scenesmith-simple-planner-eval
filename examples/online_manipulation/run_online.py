"""Run caller-supplied environment/policy pairs through the public API.

No robot, scene, task, action dimension, or policy implementation is selected
here. Separate trusted factories return an environment config and a policy.
"""

import argparse
import importlib
from collections.abc import Sequence
from pathlib import Path

from src.online_manipulation import (
    EnvironmentConfig,
    EpisodeResult,
    Policy,
    make_env,
    run_episodes,
)


def run(
    config: EnvironmentConfig,
    policy: Policy,
    *,
    output_root: Path,
    seeds: Sequence[int] = (0,),
    max_steps: int = 100,
    record_html: bool = False,
    write_final_dmd: bool = False,
) -> tuple[EpisodeResult, ...]:
    """Build an environment and run reset-isolated, recorded episodes.

    The config owns the robot, scene, task, timing, and visualization settings.
    The policy owns action generation. The public runner owns reset/step,
    termination, diagnostics, and artifact writing; do not duplicate that loop.
    """
    if max_steps <= 0 or not seeds:
        raise ValueError("max_steps must be positive and seeds must be nonempty")
    if not isinstance(policy, Policy):
        raise TypeError("policy must implement reset(observation, info) and act")
    env = make_env(config)
    return run_episodes(
        env=env,
        policy=policy,
        seeds=seeds,
        max_steps=max_steps,
        output_root=output_root,
        record_html=record_html,
        write_final_dmd=write_final_dmd,
    )


def main() -> None:
    """Load caller configuration and execute without preset task choices."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--env-factory", required=True,
        help="module:function called with no arguments to return EnvironmentConfig",
    )
    parser.add_argument(
        "--policy-factory", required=True,
        help="module:function called with the environment config to return Policy",
    )
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--seeds", nargs="+", type=int, default=[0])
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--record-html", action="store_true")
    parser.add_argument("--write-final-dmd", action="store_true")
    args = parser.parse_args()
    config = load_factory(args.env_factory)()
    policy = load_factory(args.policy_factory)(config)
    results = run(
        config, policy,
        output_root=args.output_root,
        seeds=args.seeds,
        max_steps=args.max_steps,
        record_html=args.record_html,
        write_final_dmd=args.write_final_dmd,
    )
    for result in results:
        summary = result.summary
        print(
            f"seed={summary['seed']} success={result.success} "
            f"reason={summary['termination_reason']} "
            f"steps={summary['policy_steps']}"
        )
    print(f"Artifacts: {args.output_root.resolve()}")


def load_factory(reference: str):
    """Import a trusted caller-owned factory without choosing its implementation."""
    module_name, separator, function_name = reference.partition(":")
    if not separator or not module_name or not function_name:
        raise ValueError("Factory must have the form module:function")
    return getattr(importlib.import_module(module_name), function_name)


if __name__ == "__main__":
    main()
