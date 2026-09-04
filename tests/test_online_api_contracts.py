"""Drake-independent contract tests for the public online API."""

import unittest
from pathlib import Path

import numpy as np

from src.online_manipulation import (
    PUBLIC_API_VERSION,
    CompositeAction,
    ContactObservation,
    GripperAction,
    GripperSpec,
    HoldAction,
    JointDeltaAction,
    JointSpec,
    ObjectObservation,
    Observation,
    Pose,
    RobotAdapter,
    RobotObservation,
    RobotSpec,
    SpatialVelocity,
    Task,
    TaskEvaluation,
    TimingConfig,
)


def _pose() -> Pose:
    """Return an identity test pose."""
    return Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))


def _twist() -> SpatialVelocity:
    """Return a zero test spatial velocity."""
    return SpatialVelocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def _robot_spec() -> RobotSpec:
    """Return a two-joint mock robot description."""
    joints = (
        JointSpec("joint_a", "revolute", -1.0, 1.0, 2.0, 3.0, 10.0, 2.0),
        JointSpec("joint_b", "revolute", -2.0, 2.0, 2.0, 3.0, 10.0, 2.0),
    )
    return RobotSpec(
        name="mock",
        model_instance_name="mock_robot",
        package_name="mock_package",
        model_path=Path("models/mock/model.urdf"),
        base_link_name="base",
        base_pose=_pose(),
        controlled_joints=joints,
        locked_joint_positions={"rail": 0.4},
        end_effector_frame_name="tool",
        home_positions=(0.0, 0.0),
        gripper=GripperSpec(("finger_left", "finger_right"), 0.0, 0.08),
    )


class _FakeAdapter:
    """Minimal structural RobotAdapter implementation for contract tests."""

    def __init__(self) -> None:
        self._spec = _robot_spec()

    @property
    def spec(self) -> RobotSpec:
        return self._spec

    def add_model(self, parser):
        return "model_instance"

    def configure_model(self, plant, model_instance) -> None:
        return None

    def initialize_state(self, plant, plant_context, model_instance) -> None:
        return None

    def make_robot_observation(
        self,
        plant,
        plant_context,
        model_instance,
        controller_state,
    ) -> RobotObservation:
        return _robot_observation()

    def gripper_position_targets(self, width_m: float):
        return {"finger_left": -0.5 * width_m, "finger_right": 0.5 * width_m}


class _AllowNone:
    """Contact policy that rejects every pair."""

    @property
    def name(self) -> str:
        return "allow_none"

    def permits(self, body_a: str, body_b: str) -> bool:
        return False


class _FakeTask:
    """Minimal structural Task implementation for contract tests."""

    def reset(self, env, rng):
        return {"seed_sample": int(rng.integers(1000))}

    def observe(self, env):
        return {"phase": "idle"}

    def evaluate(self, env):
        return TaskEvaluation()

    def allowed_contacts(self, env, action):
        return _AllowNone()

    def finalize(self, env):
        return {"complete": True}


def _robot_observation() -> RobotObservation:
    """Return consistent mock robot telemetry."""
    return RobotObservation(
        joint_names=("joint_a", "joint_b"),
        q=(0.0, 0.0),
        v=(0.0, 0.0),
        q_commanded=(0.0, 0.0),
        torque_commanded=(0.0, 0.0),
        torque_applied=(0.0, 0.0),
        torque_saturated=(False, False),
        end_effector_pose=_pose(),
        end_effector_twist=_twist(),
        gripper_width_m=0.08,
    )


class PublicContractTest(unittest.TestCase):
    """Validate stable public action, observation, and protocol behavior."""

    def test_public_api_version_is_explicit(self) -> None:
        self.assertEqual(PUBLIC_API_VERSION, "0.1")

    def test_default_timing_has_expected_integer_schedule(self) -> None:
        timing = TimingConfig()
        self.assertEqual(timing.physics_steps_per_controller, 5)
        self.assertEqual(timing.controller_steps_per_policy, 20)

    def test_nonintegral_schedule_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "controller_dt"):
            TimingConfig(physics_dt=0.003, controller_dt=0.005)
        with self.assertRaisesRegex(ValueError, "policy_dt"):
            TimingConfig(controller_dt=0.006, policy_dt=0.1)

    def test_typed_actions_are_immutable_and_unit_explicit(self) -> None:
        arm = JointDeltaAction(["joint_a", "joint_b"], [0.1, -0.1])
        gripper = GripperAction(width_m=0.04)
        action = CompositeAction(arm=arm, gripper=gripper)
        self.assertEqual(action.arm.deltas, (0.1, -0.1))
        self.assertEqual(action.gripper.width_m, 0.04)
        self.assertIsInstance(HoldAction(), HoldAction)

    def test_action_name_value_mismatch_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "equal length"):
            JointDeltaAction(("joint_a",), (0.1, 0.2))

    def test_robot_spec_exposes_adapter_order(self) -> None:
        spec = _robot_spec()
        self.assertEqual(
            spec.controlled_joint_names,
            ("joint_a", "joint_b"),
        )
        self.assertEqual(spec.locked_joint_positions["rail"], 0.4)

    def test_adapter_and_task_are_runtime_checkable(self) -> None:
        self.assertIsInstance(_FakeAdapter(), RobotAdapter)
        self.assertIsInstance(_FakeTask(), Task)

    def test_observation_uses_generic_object_mapping(self) -> None:
        observation = Observation(
            time_s=0.1,
            robot=_robot_observation(),
            objects={"arbitrary_object": ObjectObservation(_pose(), _twist())},
            contacts=(
                ContactObservation("robot::tool", "scene::box", 0.0),
            ),
            task={"phase": "idle"},
        )
        payload = observation.as_dict()
        self.assertEqual(
            set(payload),
            {"time", "robot", "objects", "contacts", "task"},
        )
        self.assertIn("arbitrary_object", payload["objects"])
        self.assertNotIn("red_box_pose", payload)


if __name__ == "__main__":
    unittest.main()
