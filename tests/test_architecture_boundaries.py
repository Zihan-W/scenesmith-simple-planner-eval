"""Regression tests for generic online-environment module boundaries."""

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
    "observations.py",
    "planning.py",
    "protocols.py",
    "runner.py",
    "specs.py",
)
FORBIDDEN_MARKERS = (
    "zerith",
    "living_room",
    "red_box",
    "coffee_table",
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


if __name__ == "__main__":
    unittest.main()
