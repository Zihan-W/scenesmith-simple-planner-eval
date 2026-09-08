"""Public API static navigation example; runner alone advances the simulator."""

import argparse
import dataclasses
import json
from pathlib import Path

from src.online_manipulation import (
    HoldAction,
    NavigationGoal,
    Navigator,
    Pose,
    TimingConfig,
    build_navigation_map,
    build_planning_query,
    make_env,
)
from examples.online_manipulation.mobile_smoke import make_config


def main():
    """Plan around an actual static obstacle and evaluate measured parking."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", required=True, choices=("planar_kinematic", "wheel_dynamic")
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--meshcat", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = dataclasses.replace(
        make_config(args.mode, meshcat=args.meshcat, obstacle=True),
        episode_duration=180,
    )
    # A compact elbow-flexed navigation pose, checked before starting motion.
    spec = config.robot_adapter.spec
    home = list(spec.home_positions)
    for side in ("left", "right"):
        home[spec.controlled_joint_names.index(f"{side}_elbow_joint")] = 1.4
    adapter = type(config.robot_adapter)(
        dataclasses.replace(spec, home_positions=tuple(home)),
        config.robot_adapter.base_config,
    )
    config = dataclasses.replace(config, robot_adapter=adapter)
    query = build_planning_query(
        scenario=config.scenario,
        robot_adapter=config.robot_adapter,
        timing=config.timing,
    )
    posture_check = query.check_configuration(query.configuration())
    (args.output / "navigation_posture.json").write_text(
        json.dumps(dataclasses.asdict(posture_check), indent=2)
    )
    if not posture_check.valid:
        raise RuntimeError("Navigation posture did not pass collision validation")
    navigation_map = build_navigation_map(
        query,
        navigation_frame=config.robot_adapter.navigation_frame_name,
        ground_body_names=config.scenario.ground_body_names,
        ground_geometries=config.scenario.ground_geometries,
    )
    (args.output / "map.json").write_text(
        json.dumps(dataclasses.asdict(navigation_map), indent=2)
    )
    navigator = Navigator(navigation_map)
    env = make_env(config)
    obs, _ = env.reset(0)
    if args.meshcat:
        env.start_recording()
    for _ in range(20):
        obs, *_ = env.step(HoldAction())
    navigator.set_goal(NavigationGoal(Pose((2.8, 0, 0), (1, 0, 0, 0))), obs)
    (args.output / "path.json").write_text(json.dumps(navigator.path))
    print("radius", navigation_map.robot_radius_m, "path", navigator.path, flush=True)
    rows = []
    for step in range(1750):
        action = navigator.act(obs)
        obs, _, terminated, truncated, info = env.step(action)
        rows.append(
            {
                "time_s": obs.time_s,
                "base": obs.base,
                "status": navigator.status,
                "errors": getattr(navigator, "errors", {}),
                "action": dataclasses.asdict(action),
            }
        )
        if step % 50 == 0:
            print(
                step,
                navigator.status,
                obs.base["pose"],
                getattr(navigator, "errors", {}),
                flush=True,
            )
        if (
            navigator.status in ("arrived", "no_path", "blocked", "timeout")
            or terminated
            or truncated
        ):
            break
    (args.output / "states.json").write_text(json.dumps(rows, indent=2))
    (args.output / "result.json").write_text(
        json.dumps(
            {
                "status": navigator.status,
                "errors": getattr(navigator, "errors", {}),
                "time_s": obs.time_s,
                "stable_since": navigator.stable_since,
                "base": obs.base,
            },
            indent=2,
        )
    )
    if args.meshcat:
        env.save_recording(args.output / "simulation.html")
    print("navigation result:", navigator.status, flush=True)
    if navigator.status != "arrived":
        raise RuntimeError(f"Navigation did not pass: {navigator.status}")


if __name__ == "__main__":
    main()
