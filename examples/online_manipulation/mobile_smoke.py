"""Record actual base/dual-arm state using only the public environment API."""

import argparse
import csv
import dataclasses
import json
from pathlib import Path

from src.online_manipulation import (
    BaseConfig,
    BaseVelocityAction,
    HoldAction,
    RuntimeConfig,
    ScenarioSpec,
    VisualizationConfig,
    ZerithMobileRobotAdapter,
    make_env,
    make_zerith_dual_spec,
)


from src.online_manipulation.recipes.mobile import make_config

def main():
    """Run settle, forward, reverse, spin, curve and stop; report, never fake PASS."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", required=True, choices=("planar_kinematic", "wheel_dynamic")
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--meshcat", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = make_config(args.mode, meshcat=args.meshcat)
    env = make_env(config)
    obs, info = env.reset(0)
    if args.meshcat:
        env.start_recording()
    rows = []
    phases = [
        ("settle", 20, HoldAction()),
        ("forward", 30, BaseVelocityAction(0.1, 0)),
        ("stop_forward", 20, HoldAction()),
        ("reverse", 30, BaseVelocityAction(-0.1, 0)),
        ("stop_reverse", 20, HoldAction()),
        ("spin", 30, BaseVelocityAction(0, 0.3)),
        ("curve", 30, BaseVelocityAction(0.08, 0.2)),
        ("stop", 30, HoldAction()),
    ]
    for phase, count, action in phases:
        for _ in range(count):
            obs, _, terminated, truncated, info = env.step(action)
            rows.append(
                {
                    "phase": phase,
                    "time_s": obs.time_s,
                    "base": obs.base,
                    "q": obs.robot.q,
                    "saturated": obs.robot.torque_saturated,
                    "contacts": [c.as_dict() for c in obs.contacts],
                }
            )
            if terminated or truncated:
                raise RuntimeError("Unexpected episode termination")
        print(phase, json.dumps(obs.base), flush=True)
    (args.output / "states.json").write_text(json.dumps(rows, indent=2))
    with (args.output / "states.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=("phase", "time_s", "base"))
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    k: json.dumps(row[k]) if k == "base" else row[k]
                    for k in writer.fieldnames
                }
            )
    if args.meshcat:
        env.save_recording(args.output / "simulation.html")


if __name__ == "__main__":
    main()
