"""Run delta/absolute Cartesian controls through the public API only."""

import argparse
import dataclasses
import json
import math

import numpy as np

from examples.online_manipulation.cartesian_policy import CartesianGoalPolicy
from examples.online_manipulation.minimal_setup import make_env_config
from src.online_manipulation import (
    CartesianDeltaAction,
    HoldAction,
    Pose,
    make_env,
    make_zerith_robot_spec,
)


def main() -> None:
    """Send five 2 mm deltas, then track an absolute position/orientation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-json", required=True)
    args = parser.parse_args()
    config = dataclasses.replace(
        make_env_config(), enable_planning_query=True,
        episode_duration=20., maximum_cartesian_joint_delta=0.02)
    robot = make_zerith_robot_spec(
        robot_model_dir=config.robot_model_dir, robot_xyz=config.robot_xyz,
        robot_yaw_deg=config.robot_yaw_deg, rail_position=config.rail_position,
        q_home_left=config.q_home_left)
    env = make_env(config)
    obs, info = env.reset(seed=0)
    for _ in range(10):
        obs, _, _, _, info = env.step(HoldAction())
    initial = obs.robot.end_effector_pose
    rows = []
    for _ in range(5):
        obs, _, _, _, info = env.step(CartesianDeltaAction(
            robot.end_effector_frame_name, "world", (0., 0., 0.002), (0., 0., 0.)))
        rows.append({"phase": "delta", "time_s": obs.time_s,
                     "pose": obs.robot.end_effector_pose.as_dict(),
                     "decision": info["action_decision"]})
    delta_height = obs.robot.end_effector_pose.translation_m[2] - initial.translation_m[2]
    # A nonzero orientation target exercises six-dimensional pose control.
    # World Z rotation of 0.01 rad, left multiplied into the initial quaternion.
    w, x, y, z = initial.quaternion_wxyz
    c, s = math.cos(0.005), math.sin(0.005)
    goal = Pose(initial.translation_m, (c*w-s*z, c*x-s*y, c*y+s*x, c*z+s*w))
    policy = CartesianGoalPolicy(robot.end_effector_frame_name, goal)
    policy.reset(obs, info)
    for _ in range(100):
        obs, _, terminated, truncated, info = env.step(policy.act(obs))
        rows.append({"phase": "absolute", "time_s": obs.time_s,
                     "pose": obs.robot.end_effector_pose.as_dict(),
                     "decision": info["action_decision"]})
        if terminated or truncated:
            break
    position_error = float(np.linalg.norm(
        np.subtract(obs.robot.end_effector_pose.translation_m, goal.translation_m)))
    quaternion = np.asarray(obs.robot.end_effector_pose.quaternion_wxyz)
    goal_q = np.asarray(goal.quaternion_wxyz)
    dot = abs(float(quaternion @ goal_q / np.linalg.norm(quaternion) / np.linalg.norm(goal_q)))
    orientation_error = 2 * math.acos(min(1., dot))
    result = {"delta_height_m": delta_height, "position_error_m": position_error,
              "orientation_error_rad": orientation_error,
              "rejected_steps": sum(row["decision"]["status"] == "rejected" for row in rows),
              "trace": rows}
    with open(args.output_json, "w", encoding="utf-8") as output:
        json.dump(result, output, indent=2)
    print(json.dumps({key: value for key, value in result.items() if key != "trace"}))
    if not (delta_height > 0.005 and position_error < 0.002 and orientation_error < 0.01
            and result["rejected_steps"] == 0):
        raise RuntimeError("Cartesian smoke failed; inspect the saved trace")


if __name__ == "__main__":
    main()
