"""Regression tests for policy-independent environment composition."""

import ast
import dataclasses
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from examples.online_manipulation import minimal_setup as minimal
from examples.online_manipulation.example_policies import (
    make_hold_policy, make_joint_step_policy,
)
from examples.online_manipulation.pick_lift_demo import minimal_setup
from examples.online_manipulation.pick_lift_demo import policy as expert
from src.online_manipulation import HoldPolicy, JointStepPolicy, PickLiftPolicy


class PickLiftDemoExampleTest(unittest.TestCase):
    """Keep expert files and construction out of the environment factory."""

    def setUp(self):
        """Build environment config with an empty expert-artifact directory."""
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.artifacts = Path(temporary.name)
        self.root = Path(__file__).resolve().parents[1]
        self.robot = SimpleNamespace(
            model_instance_name="fixture_robot",
            controlled_joint_names=tuple(f"joint_{i}" for i in range(7)) + (
                "finger_left", "finger_right"
            ),
            gripper=SimpleNamespace(
                joint_names=("finger_left", "finger_right"),
                maximum_width_m=0.07628,
                contact_body_names=("finger_left_link", "finger_right_link"),
            ),
            end_effector_frame_name="fixture_grasp_frame",
        )
        with mock.patch.object(
            minimal_setup, "make_zerith_robot_spec", return_value=self.robot
        ):
            self.config = minimal_setup.make_config(
                repository_root=self.root,
                scene_root=self.artifacts / "scene",
                pick_artifact_root=self.artifacts,
            )

    def _write_expert_fixture(self):
        """Write files that only the expert policy requires."""
        home = {
            "daogui_joint_position_m": self.config.rail_position,
            "q_pick_home": self.config.q_home_left,
            "q_pregrasp": [0.1] * 7,
        }
        pregrasp = {
            "daogui_joint_position_m": self.config.rail_position,
            "robot_base_xyz_m": self.config.robot_xyz,
            "robot_base_yaw_deg": self.config.robot_yaw_deg,
            "target_collision_center_xyz_m": [0.1, 0.0, 0.5],
            "solution": {
                "grasp_axes_world": {"approach": [1.0, 0.0, 0.0]},
                "actual_pregrasp_pose": {"translation_xyz_m": [0.0, 0.0, 0.5]},
            },
        }
        (self.artifacts / "pick_home.json").write_text(json.dumps(home))
        (self.artifacts / "pregrasp_ik.json").write_text(json.dumps(pregrasp))

    def _expert(self, config):
        """Create the expert without building a simulator."""
        with mock.patch.object(
            expert, "make_zerith_robot_spec", return_value=self.robot
        ):
            return expert.build_policy(
                config, pick_artifact_root=self.artifacts,
                calibration_json=(
                    self.root / "experiments/inputs/pick_lift/pick_lift_calibration.json"
                ),
            )

    def test_environment_and_other_policies_need_no_expert_files(self):
        """Removing expert calibration must not prevent env/Hold/step creation."""
        self.assertEqual(list(self.artifacts.iterdir()), [])
        self.assertEqual(self.config.rail_position, 0.4)
        self.assertIsInstance(make_hold_policy(self.config), HoldPolicy)
        self.assertIsInstance(make_joint_step_policy(self.config), JointStepPolicy)
        self.assertEqual(self.config.task.config.required_lift_m, 0.08)
        self.assertEqual(self.config.task.config.required_hold_s, 3.0)
        with self.assertRaises(FileNotFoundError):
            self._expert(self.config)

    def test_three_policies_leave_same_environment_configuration_unchanged(self):
        """The expert consumes bindings but never modifies environment settings."""
        self._write_expert_fixture()
        before = dataclasses.asdict(self.config)
        before.pop("task")
        task_before = self.config.task.config
        policies = (
            make_hold_policy(self.config), make_joint_step_policy(self.config),
            self._expert(self.config),
        )
        self.assertIsInstance(policies[2], PickLiftPolicy)
        after = dataclasses.asdict(self.config)
        after.pop("task")
        self.assertEqual(before, after)
        self.assertEqual(task_before, self.config.task.config)
        self.assertEqual(
            policies[2].config.finger_contact_bodies,
            self.config.task.config.gripper_contact_bodies,
        )

    def test_expert_rejects_mismatched_environment(self):
        """The policy must not silently overwrite the chosen initial state."""
        self._write_expert_fixture()
        with self.assertRaisesRegex(ValueError, "does not match"):
            self._expert(dataclasses.replace(self.config, rail_position=0.3))

    def test_environment_modules_do_not_import_or_create_policies(self):
        """Guard the dependency boundary in both example environments."""
        for module in (minimal, minimal_setup):
            source = Path(module.__file__).read_text()
            tree = ast.parse(source)
            for node in ast.walk(tree):
                if isinstance(node, ast.ImportFrom):
                    self.assertFalse((node.module or "").endswith("policy"))
                    for alias in node.names:
                        self.assertNotIn("Policy", alias.name)
                if isinstance(node, ast.ClassDef):
                    self.assertNotIn("Policy", node.name)
            for filename in (
                "pregrasp_ik.json", "pick_home.json", "pick_lift_calibration.json"
            ):
                self.assertNotIn(filename, source)


if __name__ == "__main__":
    unittest.main()
