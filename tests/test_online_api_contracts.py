"""Drake-independent contract tests for the public online API."""

import unittest
from pathlib import Path

import numpy as np

from src.online_manipulation import (
    PUBLIC_API_VERSION,
    CameraSpec,
    CompositeAction,
    ContactObservation,
    EnvironmentConfig,
    GripperAction,
    GripperSpec,
    HoldAction,
    JointDeltaAction,
    JointSpec,
    ObjectObservation,
    ObservedBodySpec,
    Observation,
    OnlineManipulationEnv,
    OnlineEnvironment,
    PlanarPoseRandomizationSpec,
    Pose,
    RobotAdapter,
    RobotObservation,
    RobotSpec,
    ScenarioSpec,
    SpatialVelocity,
    Task,
    TaskEvaluation,
    TimingConfig,
    make_env,
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


class _ThreeJointMockAdapter:
    """RobotAdapter proving the contract has no seven-axis assumption."""

    def __init__(self, cameras: tuple[CameraSpec, ...] = ()) -> None:
        joints = tuple(
            JointSpec(
                f"axis_{index}",
                "revolute",
                -1.0,
                1.0,
                2.0,
                3.0,
                10.0,
                2.0,
            )
            for index in range(3)
        )
        self._spec = RobotSpec(
            name="three_axis_mock",
            model_instance_name="three_axis_model",
            package_name="three_axis_package",
            model_path=Path("models/mock/three_axis.urdf"),
            base_link_name="root",
            base_pose=_pose(),
            controlled_joints=joints,
            locked_joint_positions={},
            end_effector_frame_name="tip",
            home_positions=(0.0, 0.0, 0.0),
            gripper=None,
            cameras=cameras,
        )

    @property
    def spec(self) -> RobotSpec:
        return self._spec

    def add_model(self, parser):
        del parser
        return "three_axis_model"

    def configure_model(self, plant, model_instance) -> None:
        del plant, model_instance

    def initialize_state(self, plant, plant_context, model_instance) -> None:
        del plant, plant_context, model_instance

    def make_robot_observation(
        self,
        plant,
        plant_context,
        model_instance,
        controller_state,
    ) -> RobotObservation:
        del plant, plant_context, model_instance, controller_state
        pose = _pose()
        twist = _twist()
        return RobotObservation(
            joint_names=self.spec.controlled_joint_names,
            q=(0.0, 0.0, 0.0),
            v=(0.0, 0.0, 0.0),
            q_commanded=(0.0, 0.0, 0.0),
            torque_commanded=(0.0, 0.0, 0.0),
            torque_applied=(0.0, 0.0, 0.0),
            torque_saturated=(False, False, False),
            end_effector_pose=pose,
            end_effector_twist=twist,
        )

    def gripper_position_targets(self, width_m: float):
        del width_m
        return {}


class _MockAdapterRuntimeBackend:
    """Drive the generic environment from an arbitrary mock adapter spec."""

    def __init__(self, adapter: _ThreeJointMockAdapter) -> None:
        """Initialize state using only the adapter's public RobotSpec."""
        self.adapter = adapter
        self._q = np.asarray(adapter.spec.home_positions, dtype=float)

    def reset(self, rng: np.random.Generator):
        """Restore the adapter-defined home position."""
        del rng
        self._q = np.asarray(self.adapter.spec.home_positions, dtype=float)
        return self._observation(), {"backend": "mock_adapter"}

    def step(self, action, contact_policy):
        """Apply named joint deltas without assuming an action dimension."""
        del contact_policy
        if isinstance(action, JointDeltaAction):
            name_to_index = {
                name: index
                for index, name in enumerate(
                    self.adapter.spec.controlled_joint_names
                )
            }
            for name, delta in zip(
                action.joint_names,
                action.deltas,
                strict=True,
            ):
                self._q[name_to_index[name]] += delta
        elif not isinstance(action, HoldAction):
            raise TypeError(f"Unsupported mock action: {type(action)}")
        return self._observation(time_s=0.1), False, {
            "action_decision": {"status": "accepted"},
        }

    def write_updated_scenario(self, output_path):
        """Expose the complete backend contract without scene state."""
        del output_path
        return ()

    def start_recording(self) -> None:
        """Provide the optional recording hook."""

    def save_recording(self, output_path) -> None:
        """Provide the optional recording hook."""
        del output_path

    def _observation(self, time_s: float = 0.0) -> Observation:
        """Return telemetry sized and named from the mock adapter."""
        values = tuple(float(value) for value in self._q)
        zeros = (0.0,) * len(values)
        return Observation(
            time_s,
            RobotObservation(
                joint_names=self.adapter.spec.controlled_joint_names,
                q=values,
                v=zeros,
                q_commanded=values,
                torque_commanded=zeros,
                torque_applied=zeros,
                torque_saturated=(False,) * len(values),
                end_effector_pose=_pose(),
                end_effector_twist=_twist(),
                gripper_width_m=None,
            ),
            {},
            (),
            {},
        )


class _PublicEnvironment:
    """Small environment returned by the public composition contract."""

    def reset(self, seed=None):
        return Observation(0.0, _robot_observation(), {}, (), {}), {
            "seed": seed,
        }

    def step(self, action):
        del action
        return (
            Observation(0.1, _robot_observation(), {}, (), {}),
            0.0,
            False,
            False,
            {},
        )

    def write_updated_scenario(self, output_path):
        del output_path
        return ()

    def finalize_episode(self):
        return {"success": False}

    def start_recording(self):
        return None

    def save_recording(self, output_path):
        del output_path


class _PublicEnvironmentConfig:
    """Minimal EnvironmentConfig implementation for factory testing."""

    def build_environment(self):
        return _PublicEnvironment()


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
        self.assertEqual(PUBLIC_API_VERSION, "0.2.dev1")

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

    def test_mock_adapter_has_arbitrary_dofs_and_no_gripper(self) -> None:
        adapter = _ThreeJointMockAdapter()
        self.assertIsInstance(adapter, RobotAdapter)
        self.assertEqual(adapter.spec.controlled_joint_names, (
            "axis_0",
            "axis_1",
            "axis_2",
        ))
        self.assertIsNone(adapter.spec.gripper)
        self.assertEqual(
            len(adapter.make_robot_observation(None, None, None, {}).q),
            3,
        )

    def test_mock_adapter_can_declare_zero_or_multiple_cameras(self) -> None:
        self.assertEqual(_ThreeJointMockAdapter().spec.cameras, ())
        cameras = (
            CameraSpec(
                "camera_a",
                "frame_a",
                Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
                Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
            ),
            CameraSpec(
                "camera_b",
                "frame_b",
                Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
                Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
            ),
        )
        self.assertEqual(_ThreeJointMockAdapter(cameras).spec.cameras, cameras)

    def test_mock_adapter_drives_environment_with_named_partial_action(self):
        adapter = _ThreeJointMockAdapter()
        env = OnlineManipulationEnv(_MockAdapterRuntimeBackend(adapter))
        observation, info = env.reset(seed=9)

        self.assertEqual(info["seed"], 9)
        self.assertEqual(
            observation.robot.joint_names,
            ("axis_0", "axis_1", "axis_2"),
        )
        self.assertIsNone(observation.robot.gripper_width_m)

        observation, _, _, _, step_info = env.step(
            JointDeltaAction(("axis_0", "axis_2"), (0.25, -0.5))
        )
        self.assertEqual(observation.robot.q, (0.25, 0.0, -0.5))
        self.assertEqual(
            step_info["action_decision"]["status"],
            "accepted",
        )

    def test_make_env_accepts_only_the_public_config_contract(self) -> None:
        config = _PublicEnvironmentConfig()
        self.assertIsInstance(config, EnvironmentConfig)
        env = make_env(config)
        self.assertIsInstance(env, OnlineEnvironment)
        observation, info = env.reset(seed=0)
        action = HoldAction()
        observation, reward, terminated, truncated, _ = env.step(action)
        self.assertEqual(info["seed"], 0)
        self.assertAlmostEqual(observation.time_s, 0.1)
        self.assertEqual(reward, 0.0)
        self.assertFalse(terminated)
        self.assertFalse(truncated)

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
            {"time", "robot", "objects", "contacts", "task", "sensors"},
        )
        self.assertIn("arbitrary_object", payload["objects"])
        self.assertNotIn("red_box_pose", payload)
        self.assertEqual(payload["sensors"], {})

    def test_initial_pose_requires_a_declared_observed_body(self) -> None:
        with self.assertRaisesRegex(ValueError, "observed bodies"):
            ScenarioSpec(
                dmd_path=Path("scene.dmd.yaml"),
                initial_object_poses={"missing": _pose()},
            )
        scenario = ScenarioSpec(
            dmd_path=Path("scene.dmd.yaml"),
            initial_object_poses={"object": _pose()},
            observed_bodies=(
                ObservedBodySpec("object", "model", "body"),
            ),
        )
        self.assertEqual(scenario.initial_object_poses["object"], _pose())

    def test_pose_randomization_requires_a_declared_observed_body(self):
        randomization = PlanarPoseRandomizationSpec(
            observation_name="object",
            x_offset_range_m=(-0.01, 0.01),
        )
        with self.assertRaisesRegex(ValueError, "observed bodies"):
            ScenarioSpec(
                dmd_path=Path("scene.dmd.yaml"),
                pose_randomizations=(randomization,),
            )
        scenario = ScenarioSpec(
            dmd_path=Path("scene.dmd.yaml"),
            observed_bodies=(ObservedBodySpec("object", "model", "body"),),
            pose_randomizations=(randomization,),
        )
        self.assertEqual(scenario.pose_randomizations, (randomization,))


if __name__ == "__main__":
    unittest.main()
