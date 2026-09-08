"""Local dual-arm Cartesian precision and commanded-frame semantics checks."""

import dataclasses
import json
import unittest
from unittest import mock

import numpy as np
from pydrake.all import Quaternion, RotationMatrix

from src.online_manipulation import (
    CartesianDeltaAction,
    CartesianPoseAction,
    HoldAction,
    JointDeltaAction,
    RobotCommand,
    make_env,
)
from test_dual_runtime import dual_config


def rotation(pose):
    """Return the orientation used only for independent error measurement."""
    return RotationMatrix(Quaternion(pose.quaternion_wxyz))


class CartesianHandoffTest(unittest.TestCase):
    """Validate a small reachable neighborhood, not the whole workspace."""

    def test_repeated_abs_and_delta_world_commanded_baseline(self):
        config = dual_config()
        adapter = config.robot_adapter
        home = list(adapter.spec.home_positions)
        for names in adapter.spec.arm_groups.values():
            home[adapter.spec.controlled_joint_names.index(names[3])] = 1.4
        adapter = type(adapter)(
            dataclasses.replace(adapter.spec, home_positions=tuple(home))
        )
        env = make_env(dataclasses.replace(config, robot_adapter=adapter))
        obs, _ = env.reset(0)
        for _ in range(5):
            obs, *_ = env.step(HoldAction())
        spec = adapter.spec
        query = env.get_planning_query()
        reachable = np.array(home)
        for names in spec.arm_groups.values():
            reachable[spec.controlled_joint_names.index(names[0])] -= 0.025
            reachable[spec.controlled_joint_names.index(names[-1])] += 0.015
        self.assertTrue(query.check_edge(home, reachable).valid)
        goals = {
            side: query.frame_pose_at(reachable, spec.model_instance_name, frame)
            for side, frame in spec.end_effector_frames.items()
        }
        command = RobotCommand(
            arms={
                side: CartesianPoseAction(spec.end_effector_frames[side], "world", goal)
                for side, goal in goals.items()
            }
        )
        rows = []
        for _ in range(60):
            obs, _, _, _, info = env.step(command)
            self.assertTrue(
                info["action_decision"]["accepted"], info["action_decision"]
            )
            row = {"time_s": obs.time_s}
            for side, goal in goals.items():
                actual = obs.robot.end_effectors[side]
                row[side] = {
                    "position_error_m": float(
                        np.linalg.norm(
                            np.array(actual.translation_m) - goal.translation_m
                        )
                    ),
                    "orientation_error_rad": float(
                        (rotation(goal) @ rotation(actual).inverse())
                        .ToAngleAxis()
                        .angle()
                    ),
                }
            rows.append(row)
        # One millimeter is below the 5 mm planning margin; one degree is a
        # local alignment test, not a guarantee of arbitrary grasp accuracy.
        for side in goals:
            for row in rows[-10:]:
                self.assertLess(row[side]["position_error_m"], 0.001)
                self.assertLess(row[side]["orientation_error_rad"], np.deg2rad(1))
        print(
            "CARTESIAN_ABS_METRICS="
            + json.dumps(
                {"goals": {s: p.as_dict() for s, p in goals.items()}, "trace": rows}
            )
        )

        # Produce real servo lag. Delta FK must use held command, not measured q.
        obs, *_ = env.step(
            RobotCommand(
                arms={
                    side: JointDeltaAction((names[0],), (-0.015,))
                    for side, names in spec.arm_groups.items()
                }
            )
        )
        commanded = np.array(obs.robot.q_commanded)
        self.assertGreater(np.linalg.norm(commanded - obs.robot.q), 1e-5)
        query = env.get_planning_query()
        poses = {
            s: query.frame_pose_at(commanded, spec.model_instance_name, f)
            for s, f in spec.end_effector_frames.items()
        }
        delta = RobotCommand(
            arms={
                s: CartesianDeltaAction(f, "world", (0, 0.0005, 0), (0.003, 0, 0))
                for s, f in spec.end_effector_frames.items()
            }
        )
        with mock.patch.object(
            query, "differential_ik_step", wraps=query.differential_ik_step
        ) as solve:
            obs, _, _, _, info = env.step(delta)
        self.assertTrue(info["action_decision"]["accepted"], info)
        directions = {}
        for call in solve.call_args_list:
            indices = [
                spec.controlled_joint_names.index(n)
                for n in call.kwargs["active_joint_names"]
            ]
            np.testing.assert_allclose(
                np.array(call.kwargs["seed"])[indices], commanded[indices], atol=1e-12
            )
            np.testing.assert_allclose(call.kwargs["translation_m"], [0, 0.0005, 0])
            np.testing.assert_allclose(
                call.kwargs["rotation_vector_rad"], [0.003, 0, 0]
            )
        for side, frame in spec.end_effector_frames.items():
            after = query.frame_pose_at(
                obs.robot.q_commanded, spec.model_instance_name, frame
            )
            d = np.array(after.translation_m) - poses[side].translation_m
            aa = (rotation(after) @ rotation(poses[side]).inverse()).ToAngleAxis()
            rv = aa.axis() * aa.angle()
            self.assertGreater(d[1], 0)
            self.assertGreater(rv[0], 0)
            self.assertLess(np.linalg.norm(d - [0, 0.0005, 0]), 0.00015)
            self.assertLess(np.linalg.norm(rv - [0.003, 0, 0]), 0.001)
            directions[side] = {
                "translation_world_m": d.tolist(),
                "left_rotation_vector_rad": rv.tolist(),
            }
        held = obs.robot.q_commanded
        obs, *_ = env.step(HoldAction())
        self.assertEqual(held, obs.robot.q_commanded)
        print("CARTESIAN_DELTA_METRICS=" + json.dumps(directions))


if __name__ == "__main__":
    unittest.main()
