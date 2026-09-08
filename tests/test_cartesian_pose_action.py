"""Absolute Cartesian control contracts and world-frame error regression."""

import dataclasses
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
from pydrake.all import RollPitchYaw

from src.online_manipulation import (
    CartesianPoseAction,
    CompositeAction,
    DifferentialIkResult,
    EdgeCheck,
    GripperAction,
    PairContactPolicy,
    PickLiftTask,
    PickLiftTaskConfig,
    PlanningQuery,
    Pose,
)
from src.online_manipulation.adapters.zerith import ZerithLegacyActionTranslator
from test_zerith_robot_adapter import _adapter
from test_online_environment import _carrier_query


class CartesianPoseActionTest(unittest.TestCase):
    """Keep absolute goals nonaccumulating and collision checked."""

    def test_action_requires_pose_and_frames(self):
        with self.assertRaises(ValueError):
            CartesianPoseAction("", "world", Pose((0, 0, 0), (1, 0, 0, 0)))
        with self.assertRaises(TypeError):
            CartesianPoseAction("tool", "world", (0, 0, 0))

    def test_absolute_error_uses_world_rotation_and_forwards_safety(self):
        current_rotation = RollPitchYaw(0.3, -0.4, 0.7).ToRotationMatrix()
        world_increment = RollPitchYaw(0, 0, 0.05).ToRotationMatrix()
        current = Pose((1, 2, 3), tuple(current_rotation.ToQuaternion().wxyz()))
        goal = Pose((1.01, 2.02, 2.97), tuple(
            (world_increment @ current_rotation).ToQuaternion().wxyz()
        ))
        query = SimpleNamespace(
            robot_adapter=SimpleNamespace(spec=SimpleNamespace(model_instance_name="robot")),
            frame_pose_at=Mock(return_value=current),
            differential_ik_step=Mock(return_value="result"),
        )
        contact = PairContactPolicy.from_pairs("test", [("a", "b")])
        kwargs = dict(target_pose=goal, frame_name="tool", seed=(0.1, 0.2),
                      validation_start=(0, 0), maximum_joint_delta=0.02,
                      contact_policy=contact)
        self.assertEqual(PlanningQuery.differential_ik_to_pose(query, **kwargs), "result")
        sent = query.differential_ik_step.call_args.kwargs
        np.testing.assert_allclose(sent["translation_m"], [0.01, 0.02, -0.03])
        np.testing.assert_allclose(sent["rotation_vector_rad"], [0, 0, 0.05], atol=1e-12)
        self.assertIs(sent["contact_policy"], contact)
        self.assertEqual(sent["validation_start"], (0, 0))
        # The same absolute goal becomes a zero command at arrival, including
        # equivalent quaternion signs and non-unit quaternion input.
        query.frame_pose_at.return_value = dataclasses.replace(
            goal, quaternion_wxyz=tuple(-2 * v for v in goal.quaternion_wxyz))
        PlanningQuery.differential_ik_to_pose(query, **kwargs)
        sent = query.differential_ik_step.call_args.kwargs
        np.testing.assert_allclose(sent["translation_m"], 0, atol=1e-12)
        np.testing.assert_allclose(sent["rotation_vector_rad"], 0, atol=1e-12)

    def test_shortest_rotation_crosses_pi_without_full_turn(self):
        query = SimpleNamespace(
            robot_adapter=SimpleNamespace(spec=SimpleNamespace(model_instance_name="robot")),
            frame_pose_at=Mock(return_value=Pose((0, 0, 0), tuple(
                RollPitchYaw(0, 0, math.radians(179)).ToQuaternion().wxyz()))),
            differential_ik_step=Mock(),
        )
        PlanningQuery.differential_ik_to_pose(
            query, target_pose=Pose((0, 0, 0), tuple(
                RollPitchYaw(0, 0, math.radians(-179)).ToQuaternion().wxyz())),
            frame_name="tool", seed=(0,), maximum_joint_delta=0.02)
        np.testing.assert_allclose(
            query.differential_ik_step.call_args.kwargs["rotation_vector_rad"],
            [0, 0, math.radians(2)], atol=1e-12)

    def test_adapter_dispatch_and_atomic_collision_rejection(self):
        spec = _adapter().spec
        result = DifferentialIkResult(
            success=True, reason="success", configuration=(0.01,) + (0.,) * 8,
            requested_twist=(0.,) * 6, achieved_twist=(0.,) * 6,
            joint_delta_scaled=False,
            edge=EdgeCheck(True, 2, 0.01, 0.01, 0., 0.))
        query = SimpleNamespace(differential_ik_to_pose=Mock(return_value=result))
        translator = ZerithLegacyActionTranslator(
            spec, planning_query=query, maximum_joint_delta=0.1,
            maximum_cartesian_joint_delta=0.02)
        action = CartesianPoseAction(spec.end_effector_frame_name, "world",
                                     Pose((1, 2, 3), (1, 0, 0, 0)))
        np.testing.assert_allclose(translator.translate(action)[:7], [0.01] + [0.] * 6)
        self.assertEqual(
            query.differential_ik_to_pose.call_args.kwargs["target_pose"],
            action.pose,
        )
        query.differential_ik_to_pose.return_value = dataclasses.replace(
            result, success=False, reason="edge_collision_or_clearance",
            edge=dataclasses.replace(result.edge, valid=False))
        rejected = translator.translate(CompositeAction(arm=action, gripper=GripperAction(0)))
        np.testing.assert_allclose(rejected[:7], 0)
        self.assertAlmostEqual(rejected[7], 1.)
        self.assertEqual(translator.last_decision["status"], "rejected")
        with self.assertRaisesRegex(NotImplementedError, "world"):
            translator.translate(dataclasses.replace(action, reference_frame="tool"))
        with self.assertRaisesRegex(ValueError, "RobotSpec"):
            translator.translate(dataclasses.replace(action, end_effector_frame="wrong"))

    def test_absolute_and_composite_keep_carried_body_checks(self):
        task = PickLiftTask(PickLiftTaskConfig(
            target_observation_name="box", target_contact_body="object::body",
            gripper_contact_bodies=("robot::left", "robot::right")))
        pose = Pose((0, 0, 1), (1, 0, 0, 0))
        task._contact_state = Mock(return_value={"bilateral_gripper_contact": True})
        task._target = Mock(return_value=SimpleNamespace(pose=pose))
        env = SimpleNamespace(observation=SimpleNamespace(
            robot=SimpleNamespace(end_effector_pose=pose)))
        env.get_planning_query = lambda: _carrier_query(pose)
        action = CartesianPoseAction("tool", "world", pose)
        for command in (action, CompositeAction(arm=action, gripper=GripperAction(0))):
            contact = task.allowed_contacts(env, command)
            self.assertEqual(contact.carried_bodies[0].body_name, "object::body")
            self.assertEqual(contact.carried_bodies[0].carrier_frame_name, "tool")


if __name__ == "__main__":
    unittest.main()
