"""Tests for replaceable example policies outside the environment."""

import dataclasses
import unittest

from src.online_manipulation import (
    CartesianDeltaAction,
    CompositeAction,
    ContactObservation,
    GripperAction,
    HoldAction,
    HoldPolicy,
    JointDeltaAction,
    JointPositionAction,
    JointStepPolicy,
    JointStepPolicyConfig,
    ObjectObservation,
    Observation,
    PickLiftPolicy,
    PickLiftPolicyConfig,
    Pose,
    RobotObservation,
    SpatialVelocity,
)


def _observation(
    *,
    q=(0.0, 0.0),
    v=(0.0, 0.0),
    q_commanded=None,
    ee_x=0.0,
    ee_y=0.0,
    ee_z=0.5,
    ee_quaternion=(1.0, 0.0, 0.0, 0.0),
    target_x=0.1,
    target_y=0.0,
    target_z=0.5,
    target_quaternion=(1.0, 0.0, 0.0, 0.0),
    contacts=(),
    gripper_width=0.08,
) -> Observation:
    """Build one small policy observation."""
    if q_commanded is None:
        q_commanded = q
    pose = Pose((ee_x, ee_y, ee_z), ee_quaternion)
    twist = SpatialVelocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    target_pose = Pose(
        (target_x, target_y, target_z),
        target_quaternion,
    )
    return Observation(
        time_s=0.0,
        robot=RobotObservation(
            joint_names=("joint_a", "joint_b"),
            q=q,
            v=v,
            q_commanded=q_commanded,
            torque_commanded=(0.0, 0.0),
            torque_applied=(0.0, 0.0),
            torque_saturated=(False, False),
            end_effector_pose=pose,
            end_effector_twist=twist,
            gripper_width_m=gripper_width,
        ),
        objects={"target": ObjectObservation(target_pose, twist)},
        contacts=contacts,
        task={},
    )


def _pick_policy() -> PickLiftPolicy:
    """Return a short-threshold staged policy for unit testing."""
    return PickLiftPolicy(
        PickLiftPolicyConfig(
            target_observation_name="target",
            arm_joint_names=("joint_a", "joint_b"),
            pregrasp_joint_positions=(0.1, 0.2),
            end_effector_frame="tool",
            approach_axis_world=(1.0, 0.0, 0.0),
            finger_contact_bodies=("robot::left", "robot::right"),
            target_contact_body="scene::target",
            open_width_m=0.08,
            closed_width_m=0.03,
            approach_distance_m=0.1,
            lift_distance_m=0.05,
            cartesian_step_m=0.01,
            stable_pregrasp_steps=2,
            stable_contact_steps=2,
            stable_verify_steps=2,
        )
    )


def _target_relative_pick_policy() -> PickLiftPolicy:
    """Return a short target-relative ALIGN/APPROACH policy."""
    return PickLiftPolicy(
        dataclasses.replace(
            _pick_policy().config,
            staging_pose_in_target=Pose(
                (-0.1, 0.0, 0.0),
                (1.0, 0.0, 0.0, 0.0),
            ),
            grasp_pose_in_target=Pose(
                (-0.02, 0.0, 0.0),
                (1.0, 0.0, 0.0, 0.0),
            ),
            stable_target_pose_steps=2,
        )
    )


class OnlinePoliciesTest(unittest.TestCase):
    """Validate external policy reset and observation-driven transitions."""

    def test_hold_and_joint_step_have_no_environment_dependency(self):
        observation = _observation()
        hold = HoldPolicy()
        hold.reset(observation, {})
        self.assertIsInstance(hold.act(observation), HoldAction)

        step = JointStepPolicy(JointStepPolicyConfig("joint_a", 0.02))
        step.reset(observation, {})
        first = step.act(observation)
        self.assertIsInstance(first, JointDeltaAction)
        self.assertEqual(first.joint_names, ("joint_a",))
        self.assertIsInstance(step.act(observation), HoldAction)

    def test_pick_lift_transitions_only_from_observed_state(self):
        policy = _pick_policy()
        initial = _observation()
        policy.reset(initial, {})
        command = policy.act(initial)
        self.assertIsInstance(command, CompositeAction)

        at_pregrasp = _observation(q=(0.1, 0.2))
        policy.act(at_pregrasp)
        transition = policy.act(at_pregrasp)
        self.assertIsInstance(transition, HoldAction)
        self.assertEqual(policy.stage, "approach")

        approach = policy.act(at_pregrasp)
        self.assertIsInstance(approach, CompositeAction)
        self.assertIsInstance(approach.arm, CartesianDeltaAction)
        close = policy.act(_observation(q=(0.1, 0.2), ee_x=0.098))
        self.assertIsInstance(close, GripperAction)
        self.assertEqual(policy.stage, "close")

        contacts = (
            ContactObservation("robot::left", "scene::target", 0.001),
            ContactObservation("robot::right", "scene::target", 0.001),
        )
        policy.act(_observation(q=(0.1, 0.2), ee_x=0.098, contacts=contacts))
        transition = policy.act(
            _observation(q=(0.1, 0.2), ee_x=0.098, contacts=contacts)
        )
        self.assertIsInstance(transition, CompositeAction)
        self.assertIsInstance(transition.arm, JointPositionAction)
        self.assertEqual(policy.stage, "verify")

        policy.act(
            _observation(
                q=(0.1, 0.2),
                ee_x=0.098,
                target_z=0.51,
                contacts=contacts,
            )
        )
        transition = policy.act(
            _observation(
                q=(0.1, 0.2),
                ee_x=0.098,
                target_z=0.51,
                contacts=contacts,
            )
        )
        self.assertIsInstance(transition, HoldAction)
        self.assertEqual(policy.stage, "lift")

        lift = policy.act(_observation(q=(0.1, 0.2), ee_x=0.098))
        self.assertIsInstance(lift, CompositeAction)
        self.assertIsInstance(lift.arm, CartesianDeltaAction)
        self.assertEqual(lift.gripper, GripperAction(0.03))
        hold = policy.act(
            _observation(
                q=(0.1, 0.2),
                ee_x=0.098,
                target_z=0.55,
            )
        )
        self.assertIsInstance(hold, HoldAction)
        self.assertEqual(policy.stage, "hold")

    def test_cartesian_step_waits_for_servo_target_to_settle(self):
        policy = _pick_policy()
        at_pregrasp = _observation(q=(0.1, 0.2))
        policy.reset(at_pregrasp, {})
        policy.act(at_pregrasp)
        policy.act(at_pregrasp)

        waiting = _observation(
            q=(0.1, 0.2),
            q_commanded=(0.2, 0.2),
        )
        self.assertIsInstance(policy.act(waiting), HoldAction)
        self.assertEqual(policy.stage, "approach")

    def test_approach_rejects_lateral_misalignment_without_closing(self):
        policy = _pick_policy()
        at_pregrasp = _observation(q=(0.1, 0.2))
        policy.reset(at_pregrasp, {})
        policy.act(at_pregrasp)
        policy.act(at_pregrasp)

        misaligned = _observation(
            q=(0.1, 0.2),
            ee_x=0.098,
            ee_y=0.02,
        )
        self.assertIsInstance(policy.act(misaligned), HoldAction)
        self.assertEqual(policy.stage, "failed")

    def test_approach_uses_only_the_calibrated_axis(self):
        policy = _pick_policy()
        at_pregrasp = _observation(q=(0.1, 0.2))
        policy.reset(at_pregrasp, {})
        policy.act(at_pregrasp)
        policy.act(at_pregrasp)

        action = policy.act(at_pregrasp)
        self.assertIsInstance(action, CompositeAction)
        self.assertEqual(action.arm.translation_m, (0.01, 0.0, 0.0))

    def test_approach_corrects_observed_lateral_drift(self):
        policy = _pick_policy()
        at_pregrasp = _observation(q=(0.1, 0.2))
        policy.reset(at_pregrasp, {})
        policy.act(at_pregrasp)
        policy.act(at_pregrasp)

        action = policy.act(
            _observation(q=(0.1, 0.2), ee_y=0.005)
        )
        self.assertIsInstance(action, CompositeAction)
        self.assertLess(action.arm.translation_m[1], 0.0)
        self.assertAlmostEqual(
            sum(value * value for value in action.arm.translation_m) ** 0.5,
            0.01,
        )

    def test_target_relative_alignment_and_approach_use_live_pose(self):
        policy = _target_relative_pick_policy()
        at_pregrasp = _observation(q=(0.1, 0.2))
        policy.reset(at_pregrasp, {})
        policy.act(at_pregrasp)
        policy.act(at_pregrasp)
        self.assertEqual(policy.stage, "align")

        policy.act(at_pregrasp)
        transition = policy.act(at_pregrasp)
        self.assertIsInstance(transition, HoldAction)
        self.assertEqual(policy.stage, "approach")

        approach = policy.act(at_pregrasp)
        self.assertIsInstance(approach, CompositeAction)
        self.assertEqual(approach.arm.translation_m, (0.01, 0.0, 0.0))
        at_grasp = _observation(q=(0.1, 0.2), ee_x=0.08)
        policy.act(at_grasp)
        close = policy.act(at_grasp)
        self.assertIsInstance(close, GripperAction)
        self.assertEqual(policy.stage, "close")

    def test_target_relative_motion_rejects_target_drift(self):
        policy = _target_relative_pick_policy()
        at_pregrasp = _observation(q=(0.1, 0.2))
        policy.reset(at_pregrasp, {})
        policy.act(at_pregrasp)
        policy.act(at_pregrasp)

        action = policy.act(
            _observation(q=(0.1, 0.2), target_x=0.12)
        )
        self.assertIsInstance(action, HoldAction)
        self.assertEqual(policy.stage, "failed")
        self.assertEqual(
            policy.diagnostics()["failure_reason"],
            "target_moved_before_grasp",
        )

    def test_verify_requires_target_to_leave_support(self):
        policy = PickLiftPolicy(
            dataclasses.replace(
                _pick_policy().config,
                support_contact_bodies=("scene::support",),
            )
        )
        bilateral = (
            ContactObservation("robot::left", "scene::target", 0.001),
            ContactObservation("robot::right", "scene::target", 0.001),
        )
        supported = bilateral + (
            ContactObservation("scene::support", "scene::target", 0.001),
        )
        at_pregrasp = _observation(q=(0.1, 0.2))
        policy.reset(at_pregrasp, {})
        policy.act(at_pregrasp)
        policy.act(at_pregrasp)
        policy.act(_observation(q=(0.1, 0.2), ee_x=0.098))
        policy.act(
            _observation(q=(0.1, 0.2), ee_x=0.098, contacts=bilateral)
        )
        policy.act(
            _observation(q=(0.1, 0.2), ee_x=0.098, contacts=bilateral)
        )
        self.assertEqual(policy.stage, "verify")

        action = policy.act(
            _observation(
                q=(0.1, 0.2),
                ee_x=0.098,
                target_z=0.51,
                contacts=supported,
                gripper_width=0.04,
            )
        )
        self.assertIsInstance(action, CompositeAction)
        self.assertIsInstance(action.arm, CartesianDeltaAction)
        self.assertEqual(policy.stage, "verify")


if __name__ == "__main__":
    unittest.main()
