"""Contract tests for the generic OnlineManipulationEnv facade."""

import unittest
from types import SimpleNamespace
from pathlib import Path

from src.online_manipulation import (
    CartesianDeltaAction,
    CompositeAction,
    ContactObservation,
    HoldAction,
    ObjectObservation,
    Observation,
    OnlineEnvironment,
    OnlineManipulationEnv,
    Pose,
    RobotObservation,
    SpatialVelocity,
    PickLiftTask,
    PickLiftTaskConfig,
)


def _observation(
    time_s: float,
    *,
    target_height_m: float | None = None,
    contacts=(),
) -> Observation:
    """Return a minimal normalized observation at one simulation time."""
    pose = Pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    twist = SpatialVelocity((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    robot = RobotObservation(
        joint_names=("joint",),
        q=(0.0,),
        v=(0.0,),
        q_commanded=(0.0,),
        torque_commanded=(0.0,),
        torque_applied=(0.0,),
        torque_saturated=(False,),
        end_effector_pose=pose,
        end_effector_twist=twist,
    )
    objects = {}
    if target_height_m is not None:
        objects["target"] = ObjectObservation(
            Pose(
                (0.0, 0.0, target_height_m),
                (1.0, 0.0, 0.0, 0.0),
            ),
            twist,
        )
    return Observation(time_s, robot, objects, tuple(contacts), {})


def _carrier_query(pose):
    """Minimal planning contract for task-only tests, not a physical simulator."""
    return SimpleNamespace(
        robot_adapter=SimpleNamespace(spec=SimpleNamespace(
            model_instance_name="robot", grippers={},
            gripper=SimpleNamespace(contact_body_names=("left", "right"), joint_names=()),
            arm_groups={}, end_effector_frames={}, controlled_joint_names=("joint",),
            end_effector_frame_name="tool")),
        frame_pose=lambda *args: pose)


class _FakeRuntimeBackend:
    """Deterministic backend used to isolate facade behavior."""

    def __init__(self) -> None:
        self.time_s = 0.0
        self.actions = []
        self.reset_sample = None

    def reset(self, rng):
        self.reset_sample = int(rng.integers(1_000_000))
        self.time_s = 0.0
        self.actions.clear()
        return _observation(self.time_s), {"backend_reset": True}

    def step(self, action, contact_policy):
        del contact_policy
        self.actions.append(action)
        self.time_s += 0.1
        return _observation(self.time_s), self.time_s >= 0.2, {
            "backend_steps": len(self.actions)
        }

    def write_updated_scenario(self, output_path):
        self.output_path = Path(output_path)
        return ("target",)

    def get_planning_query(self):
        return _carrier_query(_observation(self.time_s).robot.end_effector_pose)


class OnlineManipulationEnvTest(unittest.TestCase):
    """Validate public reset, step, and time-limit semantics."""

    def test_runtime_protocol_and_reset_seed(self) -> None:
        env = OnlineManipulationEnv(_FakeRuntimeBackend())
        self.assertIsInstance(env, OnlineEnvironment)
        observation, info = env.reset(seed=17)
        self.assertEqual(observation.time_s, 0.0)
        self.assertEqual(info["backend_reset"], True)
        self.assertEqual(info["seed"], 17)
        self.assertEqual(info["task_reset"], {"task_name": "null"})
        self.assertEqual(observation.task["task_name"], "null")

    def test_reset_seed_controls_backend_randomization(self) -> None:
        backend = _FakeRuntimeBackend()
        env = OnlineManipulationEnv(backend)
        env.reset(seed=29)
        first_sample = backend.reset_sample
        env.step(HoldAction())
        env.reset(seed=29)
        self.assertEqual(backend.reset_sample, first_sample)
        env.reset(seed=30)
        self.assertNotEqual(backend.reset_sample, first_sample)

    def test_step_forwards_typed_action_and_reports_truncation(self) -> None:
        backend = _FakeRuntimeBackend()
        env = OnlineManipulationEnv(backend)
        env.reset()
        action = HoldAction()
        observation, reward, terminated, truncated, info = env.step(action)
        self.assertIs(backend.actions[0], action)
        self.assertEqual(observation.time_s, 0.1)
        self.assertEqual(reward, 0.0)
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertEqual(info["backend_steps"], 1)
        self.assertEqual(info["contact_policy"], "free_motion")

        _, _, terminated, truncated, _ = env.step(HoldAction())
        self.assertFalse(terminated)
        self.assertTrue(truncated)

    def test_write_back_delegates_to_runtime_backend(self) -> None:
        backend = _FakeRuntimeBackend()
        env = OnlineManipulationEnv(backend)
        updated = env.write_updated_scenario("updated.dmd.yaml")
        self.assertEqual(updated, ("target",))
        self.assertEqual(backend.output_path, Path("updated.dmd.yaml"))

    def test_pick_lift_task_terminates_after_required_stable_time(self) -> None:
        class LiftBackend(_FakeRuntimeBackend):
            """Expose a target lifted after the first policy step."""

            def reset(self, rng):
                del rng
                self.time_s = 0.0
                self.actions.clear()
                return _observation(0.0, target_height_m=0.5), {}

            def step(self, action, contact_policy):
                del contact_policy
                self.actions.append(action)
                self.time_s += 0.1
                contacts = (
                    ContactObservation(
                        "robot::left", "scene::target", 0.001
                    ),
                    ContactObservation(
                        "robot::right", "scene::target", 0.001
                    ),
                )
                return (
                    _observation(
                        self.time_s,
                        target_height_m=0.6,
                        contacts=contacts,
                    ),
                    False,
                    {},
                )

        task = PickLiftTask(
            PickLiftTaskConfig(
                target_observation_name="target",
                gripper_contact_bodies=("robot::left", "robot::right"),
                target_contact_body="scene::target",
                support_contact_bodies=("scene::support",),
                required_lift_m=0.08,
                required_hold_s=0.2,
            )
        )
        env = OnlineManipulationEnv(LiftBackend(), task=task)
        observation, _ = env.reset(seed=3)
        self.assertEqual(observation.task["lift_m"], 0.0)

        observation, reward, terminated, _, info = env.step(HoldAction())
        self.assertFalse(terminated)
        self.assertEqual(reward, 0.0)
        self.assertAlmostEqual(observation.task["lift_m"], 0.1)
        self.assertEqual(info["contact_policy"], "pick_lift_target_contact")

        observation, reward, terminated, _, info = env.step(HoldAction())
        self.assertTrue(terminated)
        self.assertEqual(reward, 1.0)
        self.assertTrue(observation.task["success"])
        self.assertEqual(info["task"]["reason"], "lift_held")

    def test_pick_lift_task_rejects_support_only_height(self) -> None:
        class SupportedLiftBackend(_FakeRuntimeBackend):
            """Expose a lifted target that remains supported by the table."""

            def reset(self, rng):
                del rng
                self.time_s = 0.0
                self.actions.clear()
                return _observation(0.0, target_height_m=0.5), {}

            def step(self, action, contact_policy):
                del contact_policy
                self.actions.append(action)
                self.time_s += 0.1
                contacts = (
                    ContactObservation(
                        "robot::left", "scene::target", 0.001
                    ),
                    ContactObservation(
                        "robot::right", "scene::target", 0.001
                    ),
                    ContactObservation(
                        "scene::support", "scene::target", 0.001
                    ),
                )
                return (
                    _observation(
                        self.time_s,
                        target_height_m=0.6,
                        contacts=contacts,
                    ),
                    False,
                    {},
                )

        task = PickLiftTask(
            PickLiftTaskConfig(
                target_observation_name="target",
                gripper_contact_bodies=("robot::left", "robot::right"),
                target_contact_body="scene::target",
                support_contact_bodies=("scene::support",),
                required_lift_m=0.08,
                required_hold_s=0.2,
            )
        )
        env = OnlineManipulationEnv(SupportedLiftBackend(), task=task)
        env.reset(seed=4)
        observation, _, terminated, _, info = env.step(HoldAction())
        observation, _, terminated, _, info = env.step(HoldAction())
        self.assertFalse(terminated)
        self.assertFalse(observation.task["success"])
        self.assertTrue(observation.task["support_contact"])
        self.assertFalse(info["task"]["metrics"]["stable_lift"])

    def test_pick_lift_declares_carried_body_after_bilateral_contact(self):
        contacts = (
            ContactObservation("robot::left", "scene::target", 1e-6),
            ContactObservation("robot::right", "scene::target", 1e-6),
            ContactObservation("scene::support", "scene::target", 1e-6),
        )
        observation = _observation(
            0.0,
            target_height_m=0.5,
            contacts=contacts,
        )
        task = PickLiftTask(
            PickLiftTaskConfig(
                target_observation_name="target",
                gripper_contact_bodies=("robot::left", "robot::right"),
                target_contact_body="scene::target",
                support_contact_bodies=("scene::support",),
            )
        )

        class _Env:
            pass

        env = _Env()
        env.observation = observation
        env.get_planning_query = lambda: _carrier_query(observation.robot.end_effector_pose)
        action = CompositeAction(
            arm=CartesianDeltaAction(
                end_effector_frame="tool",
                reference_frame="world",
                translation_m=(0.0, 0.0, 0.001),
                rotation_vector_rad=(0.0, 0.0, 0.0),
            )
        )
        contact_policy = task.allowed_contacts(env, action)
        self.assertEqual(len(contact_policy.carried_bodies), 1)
        self.assertEqual(
            contact_policy.carried_bodies[0].body_name,
            "scene::target",
        )
        self.assertTrue(
            contact_policy.permits("scene::target", "scene::support")
        )


if __name__ == "__main__":
    unittest.main()
