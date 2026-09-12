"""Tests for synchronized BT state in saved Meshcat recordings."""

import json
import tempfile
import unittest
from pathlib import Path

from src.online_manipulation.recording_overlay import (
    MARKER,
    build_behavior_tree_timeline,
    inject_behavior_tree_overlay,
    policy_behavior_tree_timeline,
)


TREE = {
    "kind": "root", "name": "", "args": [], "children": [{
        "kind": "sequence", "name": "", "args": [], "children": [
            {"kind": "action", "name": "Wait", "args": ["1.0"], "children": []},
            {"kind": "action", "name": "Execute", "args": [], "children": []},
        ],
    }],
}


def row(time_s, path, status="RUNNING", stage="align"):
    diagnostics = {"controller": "behavior_tree", "active_path": path,
                   "tree_status": status, "reason": "", "expert": {"stage": stage}}
    return {"simulation_time_s": time_s, "task_reason": "running",
            "policy_diagnostics_json": json.dumps(diagnostics)}


class Policy:
    def behavior_tree_visualization(self):
        return {"title": "Test tree", "tree": TREE}


class RecordingOverlayTest(unittest.TestCase):
    def test_builds_paths_and_compacts_unchanged_samples(self):
        payload = build_behavior_tree_timeline(
            definition={"title": "Test tree", "tree": TREE},
            trace=[row(10.1, "0.0.0"), row(10.2, "0.0.0"),
                   row(10.3, "0.0.1", "SUCCESS", "hold")],
            recording_start_time_s=10.0,
        )
        self.assertEqual([node["path"] for node in payload["nodes"]],
                         ["0", "0.0", "0.0.0", "0.0.1"])
        self.assertEqual(len(payload["samples"]), 2)
        self.assertAlmostEqual(payload["samples"][0]["time_s"], 0.1)
        self.assertEqual(payload["samples"][-1]["active_label"], "Execute")

    def test_reads_optional_policy_definition(self):
        payload = policy_behavior_tree_timeline(
            policy=Policy(), trace=[row(0.1, "0.0.0")], recording_start_time_s=0.0)
        self.assertEqual(payload["title"], "Test tree")

    def test_policy_without_definition_is_ignored(self):
        self.assertIsNone(policy_behavior_tree_timeline(
            policy=object(), trace=[row(0.1, "0.0.0")], recording_start_time_s=0.0))

    def test_injects_once_before_body(self):
        payload = build_behavior_tree_timeline(
            definition={"title": "Test tree", "tree": TREE},
            trace=[row(0.1, "0.0.0")], recording_start_time_s=0.0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "simulation.html"
            path.write_text("<!doctype html><body><canvas></canvas></body></html>")
            self.assertTrue(inject_behavior_tree_overlay(path, payload))
            text = path.read_text()
            self.assertIn(MARKER, text)
            self.assertIn("__SCENESMITH_BT_TIMELINE__", text)
            self.assertLess(text.index(MARKER), text.index("</body>"))
            self.assertFalse(inject_behavior_tree_overlay(path, payload))
            updated = dict(payload, title="Updated tree")
            self.assertTrue(inject_behavior_tree_overlay(path, updated))
            text = path.read_text()
            self.assertEqual(text.count(f"<!-- {MARKER} -->"), 1)
            self.assertIn("Updated tree", text)


if __name__ == "__main__":
    unittest.main()
