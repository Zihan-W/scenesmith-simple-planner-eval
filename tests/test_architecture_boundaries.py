"""Regression tests for generic online-environment module boundaries."""

import ast
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
GENERIC_CORE_FILES = (
    "actions.py",
    "contact.py",
    "controller.py",
    "dmd_finalizer.py",
    "drake_utils.py",
    "environment.py",
    "factory.py",
    "observations.py",
    "planning.py",
    "protocols.py",
    "runner.py",
    "specs.py",
    "runtime.py",
    "model.py",
    "sensors.py",
    "base.py",
    "navigation.py",
    "navigation_geometry.py",
)
FORBIDDEN_MARKERS = (
    "zerith",
    "living_room",
    "red_box",
    "coffee_table",
)
ZERITH_IMPLEMENTATION_MARKERS = (
    "left_shoulder_pitch_joint",
    "left_jaw_left_finger_joint",
    "dipan_link",
    "daogui_joint",
    "gripper_max_opening",
)
PICK_LIFT_OWNED_MARKERS = (
    "pickliftpolicy",
    "picklifttask",
    "_pregrasp",
    "_align",
    "_approach",
    "_close",
    "_verify",
    "_lift",
)


class ArchitectureBoundaryTest(unittest.TestCase):
    """Keep robot, task, and current-scene details out of generic core."""

    def test_generic_core_has_no_current_robot_or_scene_markers(self) -> None:
        core = REPOSITORY_ROOT / "src" / "online_manipulation"
        for filename in GENERIC_CORE_FILES:
            source = (core / filename).read_text(encoding="utf-8").lower()
            for marker in FORBIDDEN_MARKERS:
                self.assertNotIn(marker, source, f"{marker} in {filename}")

    def test_generic_core_does_not_import_robot_adapter(self) -> None:
        core = REPOSITORY_ROOT / "src" / "online_manipulation"
        for filename in GENERIC_CORE_FILES:
            source = (core / filename).read_text(encoding="utf-8")
            self.assertNotIn("adapters.zerith", source, filename)
            self.assertNotIn("src.zerith_", source, filename)

    def test_zerith_joint_link_and_gripper_details_are_adapter_owned(self):
        core = REPOSITORY_ROOT / "src" / "online_manipulation"
        for filename in GENERIC_CORE_FILES:
            source = (core / filename).read_text(encoding="utf-8").lower()
            for marker in ZERITH_IMPLEMENTATION_MARKERS:
                self.assertNotIn(marker, source, f"{marker} in {filename}")

    def test_pick_lift_state_and_contacts_do_not_leak_into_core(self) -> None:
        core = REPOSITORY_ROOT / "src" / "online_manipulation"
        for filename in GENERIC_CORE_FILES:
            source = (core / filename).read_text(encoding="utf-8").lower()
            for marker in PICK_LIFT_OWNED_MARKERS:
                self.assertNotIn(marker, source, f"{marker} in {filename}")

        policy_source = (core / "policies.py").read_text(encoding="utf-8")
        task_source = (core / "tasks.py").read_text(encoding="utf-8")
        self.assertIn("class PickLiftPolicy", policy_source)
        self.assertIn("class PickLiftTask", task_source)
        for source, filename in (
            (policy_source, "policies.py"),
            (task_source, "tasks.py"),
        ):
            self.assertNotIn("adapters.zerith", source, filename)
            self.assertNotIn("zerith_online_env", source, filename)

    def test_external_client_imports_only_public_online_api(self) -> None:
        examples = REPOSITORY_ROOT / "examples" / "online_manipulation"
        for filename in (
            "public_api_client.py",
            "camera_public_api_client.py",
            "mobile_public_api_client.py",
        ):
            source = (examples / filename).read_text(encoding="utf-8")
            tree = ast.parse(source)
            project_imports = [
                node.module
                for node in ast.walk(tree)
                if isinstance(node, ast.ImportFrom)
                and node.module is not None
                and node.module.startswith("src.")
            ]
            self.assertEqual(
                project_imports,
                ["src.online_manipulation"],
                filename,
            )
            for marker in (
                "pydrake",
                ".backend",
                ".runtime",
                "plant_context",
                "position_start",
            ):
                self.assertNotIn(marker, source, filename)


if __name__ == "__main__":
    unittest.main()
