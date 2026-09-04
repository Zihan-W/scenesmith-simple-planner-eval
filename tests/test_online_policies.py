"""Tests for replaceable example policies outside the environment."""

import unittest

from src.online_manipulation import (
    CartesianDeltaAction,
    CompositeAction,
    ContactObservation,
    GripperAction,
    HoldAction,
    HoldPolicy,
    JointDeltaAction,
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
    ee_x=0.0,
    target_z=0.5,
    contacts=(),
) -> Observation:
    """Build one small policy observation."""
    pose = Pose((ee_x, 0.0, 0.5), (1.0, 0.0, 0.0, 0.0))
    twist = SpatialVelocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    target_pose = Pose((0.1, 0.0, target_z), (1.0, 0.0, 0.0, 0.0))
    return Observation(
        time_s=0.0,
        robot=RobotObservation(
            joint_names=("joint_a", "joint_b"),
            q=q,
            v=v,
            q_commanded=q,
            torque_commanded=(0.0, 0.0),
            torque_applied=(0.0, 0.0),
            torque_saturated=(False, False),
            end_effector_pose=pose,
            end_effector_twist=twist,
            gripper_width_m=0.08,
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
            approach_distance_m=0.02,
            lift_distance_m=0.05,
            cartesian_step_m=0.01,
            stable_pregrasp_steps=2,
            stable_contact_steps=2,
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
        close = policy.act(_observation(q=(0.1, 0.2), ee_x=0.02))
        self.assertIsInstance(close, GripperAction)
        self.assertEqual(policy.stage, "close")

        contacts = (
            ContactObservation("robot::left", "scene::target", 0.001),
            ContactObservation("robot::right", "scene::target", 0.001),
        )
        policy.act(_observation(q=(0.1, 0.2), ee_x=0.02, contacts=contacts))
        transition = policy.act(
            _observation(q=(0.1, 0.2), ee_x=0.02, contacts=contacts)
        )
        self.assertIsInstance(transition, HoldAction)
        self.assertEqual(policy.stage, "lift")

        lift = policy.act(_observation(q=(0.1, 0.2), ee_x=0.02))
        self.assertIsInstance(lift, CompositeAction)
        self.assertIsInstance(lift.arm, CartesianDeltaAction)
        self.assertEqual(lift.gripper, GripperAction(0.03))
        hold = policy.act(
            _observation(
                q=(0.1, 0.2),
                ee_x=0.02,
                target_z=0.55,
            )
        )
        self.assertIsInstance(hold, HoldAction)
        self.assertEqual(policy.stage, "hold")


if __name__ == "__main__":
    unittest.main()
