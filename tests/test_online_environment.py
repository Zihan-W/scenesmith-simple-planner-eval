"""Contract tests for the generic OnlineManipulationEnv facade."""

import unittest

from src.online_manipulation import (
    HoldAction,
    Observation,
    OnlineEnvironment,
    OnlineManipulationEnv,
    Pose,
    RobotObservation,
    SpatialVelocity,
)


def _observation(time_s: float) -> Observation:
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
    return Observation(time_s, robot, {}, (), {})


class _FakeRuntimeBackend:
    """Deterministic backend used to isolate facade behavior."""

    def __init__(self) -> None:
        self.time_s = 0.0
        self.actions = []

    def reset(self):
        self.time_s = 0.0
        self.actions.clear()
        return _observation(self.time_s), {"backend_reset": True}

    def step(self, action):
        self.actions.append(action)
        self.time_s += 0.1
        return _observation(self.time_s), self.time_s >= 0.2, {
            "backend_steps": len(self.actions)
        }


class OnlineManipulationEnvTest(unittest.TestCase):
    """Validate public reset, step, and time-limit semantics."""

    def test_runtime_protocol_and_reset_seed(self) -> None:
        env = OnlineManipulationEnv(_FakeRuntimeBackend())
        self.assertIsInstance(env, OnlineEnvironment)
        observation, info = env.reset(seed=17)
        self.assertEqual(observation.time_s, 0.0)
        self.assertEqual(info, {"backend_reset": True, "seed": 17})

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

        _, _, terminated, truncated, _ = env.step(HoldAction())
        self.assertFalse(terminated)
        self.assertTrue(truncated)

    def test_write_back_is_explicitly_unavailable_before_phase_five(self) -> None:
        env = OnlineManipulationEnv(_FakeRuntimeBackend())
        with self.assertRaisesRegex(NotImplementedError, "Phase 5"):
            env.write_updated_scenario("unused.dmd.yaml")


if __name__ == "__main__":
    unittest.main()
