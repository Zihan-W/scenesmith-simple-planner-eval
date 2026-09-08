"""Repeatable actual-state local-goal arrival and kinematic blocked-motion audit."""

import argparse
import dataclasses
import json
import math
from pathlib import Path

import numpy as np
from pydrake.all import Quaternion, RigidTransform, RollPitchYaw

from src.online_manipulation import (
    BaseVelocityAction,
    HoldAction,
    NavigationGoal,
    Navigator,
    Pose,
    StaticNavigationMap,
    make_env,
)
from src.online_manipulation.recipes.mobile import make_config


def local_arrival(mode, output):
    """Use a rotated, tilted wrist frame to express a planar navigation goal."""
    config = make_config(mode)
    adapter = config.robot_adapter
    home = list(adapter.spec.home_positions)
    for side in ("left", "right"):
        home[
            adapter.spec.controlled_joint_names.index(adapter.spec.arm_groups[side][3])
        ] = 1.4
    spec = dataclasses.replace(
        adapter.spec,
        home_positions=tuple(home),
        base_pose=Pose(
            adapter.spec.base_pose.translation_m, (math.sqrt(0.5), 0, 0, math.sqrt(0.5))
        ),
    )
    adapter = type(adapter)(spec, adapter.base_config)
    env = make_env(dataclasses.replace(config, robot_adapter=adapter))
    obs, _ = env.reset(0)
    for _ in range(20):
        obs, *_ = env.step(HoldAction())
    reference_name = "right_wrist_pitch_link"
    reference = obs.robot.frame_poses_world[reference_name]
    X_WR = RigidTransform(
        Quaternion(reference.quaternion_wxyz), reference.translation_m
    )
    X_WG = RigidTransform(RollPitchYaw(0, 0, -math.pi + 0.1), [-0.25, 0.1, 0])
    X_RG = X_WR.inverse() @ X_WG
    goal = NavigationGoal(
        Pose(tuple(X_RG.translation()), tuple(X_RG.rotation().ToQuaternion().wxyz())),
        reference_name,
    )
    navigator = Navigator(StaticNavigationMap((), 0.42))
    navigator.set_goal(goal, obs)
    frozen = navigator.world_goal
    rows = []
    for _ in range(500):
        action = navigator.act(obs)
        obs, _, terminated, truncated, info = env.step(action)
        rows.append(
            {
                "time_s": obs.time_s,
                "base": obs.base,
                "status": navigator.status,
                "errors": navigator.errors,
                "action": dataclasses.asdict(action),
                "decision": info["action_decision"],
            }
        )
        if navigator.world_goal != frozen:
            raise AssertionError("Local goal changed while the reference moved")
        if (
            navigator.status in ("arrived", "blocked", "timeout")
            or terminated
            or truncated
        ):
            break
    result = {
        "mode": mode,
        "status": navigator.status,
        "local_goal": dataclasses.asdict(goal),
        "resolved_world_goal": frozen.as_dict(),
        "accepted_time_s": navigator.accepted_time_s,
        "stable_since": navigator.stable_since,
        "final": rows[-1],
    }
    (output / f"{mode}_local_trace.json").write_text(json.dumps(rows, indent=2))
    (output / f"{mode}_local_result.json").write_text(json.dumps(result, indent=2))
    print(mode, navigator.status, navigator.errors, flush=True)
    if navigator.status != "arrived":
        raise AssertionError(result)


def blocked_motion(output):
    """Drive into a real obstacle; the prescribed backend must stop at its edge."""
    env = make_env(make_config("planar_kinematic", obstacle=True))
    obs, _ = env.reset(0)
    rows = []
    for _ in range(140):
        obs, *_ = env.step(BaseVelocityAction(0.15, 0))
        rows.append(
            {
                "time_s": obs.time_s,
                "base": obs.base,
                "contacts": [c.as_dict() for c in obs.contacts],
            }
        )
        if obs.base["blocked"]:
            break
    (output / "kinematic_blocked_trace.json").write_text(json.dumps(rows, indent=2))
    if not obs.base["blocked"]:
        raise AssertionError("Expected real swept-geometry blockage")
    if any("obstacle" in c.body_a + c.body_b for c in obs.contacts):
        raise AssertionError("Robot penetrated obstacle before reporting blocked")
    before = np.array(obs.base["pose"]["translation_m"])
    for _ in range(20):
        obs, *_ = env.step(BaseVelocityAction(-0.1, 0))
    if obs.base["blocked"] or obs.base["pose"]["translation_m"][0] > before[0] - 0.05:
        raise AssertionError(
            "Safe reverse motion from blocked position did not release"
        )
    print("kinematic swept blockage and reverse recovery passed", flush=True)


def main():
    """Run both actual mobile systems and save all failures before raising."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    for mode in ("wheel_dynamic", "planar_kinematic"):
        local_arrival(mode, args.output)
    blocked_motion(args.output)


if __name__ == "__main__":
    main()
