"""A02: actual non-seven-axis mechanism through the shared Drake runtime."""

import dataclasses
from pathlib import Path
import unittest

from src.online_manipulation import (
    HoldAction,
    JointDeltaAction,
    JointSpec,
    Pose,
    RobotSpec,
    ScenarioSpec,
    make_env,
)
from src.online_manipulation.adapters.description import DescriptionRobotAdapter
from src.online_manipulation.runtime import RuntimeConfig


def fixture_config():
    """Return a self-contained two-axis, no-gripper real physical mechanism."""
    root = Path(__file__).resolve().parents[1]
    robot = RobotSpec(
        name="fixture",
        model_instance_name="two_joint_fixture",
        package_name="runtime_fixture",
        model_path=root / "models/runtime_fixture/urdf/two_joint.urdf",
        base_link_name="pedestal",
        base_pose=Pose((0, 0, 0.5), (1, 0, 0, 0)),
        controlled_joints=tuple(
            JointSpec(name, "revolute", -1.5, 1.5, 2, 10, 100, 20)
            for name in ("hinge_a", "hinge_b")
        ),
        home_positions=(0, 0),
        locked_joint_positions={},
        end_effector_frame_name="tip",
    )
    scene = root / "models/online_env_minimal_scene"
    return RuntimeConfig(
        ScenarioSpec(scene / "scene.dmd.yaml", (scene / "package.xml",)),
        DescriptionRobotAdapter(robot),
    )


class DescriptionRuntimeTest(unittest.TestCase):
    """No Mock backend, no gripper and no array-size assumptions."""

    def test_two_joint_step_reset_and_model_agreement(self):
        env = make_env(fixture_config())
        initial, _ = env.reset(seed=0)
        self.assertEqual(initial.robot.joint_names, ("hinge_a", "hinge_b"))
        self.assertIsNone(initial.robot.gripper_width_m)
        obs, _, _, _, info = env.step(
            JointDeltaAction(("hinge_a", "hinge_b"), (0.02, -0.03))
        )
        self.assertTrue(info["action_decision"]["accepted"], info)
        self.assertAlmostEqual(obs.time_s, 0.1)
        self.assertGreater(obs.robot.q[0], 0.001)
        self.assertLess(obs.robot.q[1], -0.001)
        self.assertEqual(info["control_updates"], 20)
        self.assertEqual(info["physics_steps_per_control"], 5)
        obs, *_ = env.step(HoldAction())
        self.assertEqual(obs.robot.q_commanded, (0.02, -0.03))
        again, _ = env.reset(seed=0)
        self.assertEqual(initial.robot.q, again.robot.q)
        self.assertIs(env.backend.planning.plant, env.backend.plant)
        self.assertIsNot(env.backend.planning.context, env.backend.plant_context)


if __name__ == "__main__":
    unittest.main()
