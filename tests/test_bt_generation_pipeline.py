"""The shared generation boundary accepts both current task families."""

import hashlib
import json
import shutil
import tempfile
import unittest
from pathlib import Path

from planner.src.bt.generation import (
    ChatCompletion, REQUEST_SCHEMA, RESULT_SCHEMA, generate, load_request, write_result,
)
from planner.src.bt.navigation import load_generated_plan


class RecordedClient:
    def __init__(self, response):
        self.response = response

    def complete(self, *, model, messages):
        assert messages and messages[-1]["role"] == "user"
        return ChatCompletion(self.response, model, "recorded", "stop")


class SharedPipelineTest(unittest.TestCase):
    def test_picklift_uses_same_generic_request_and_output(self):
        source = Path(__file__).resolve().parents[1] / "experiments/inputs/bt_picklift_first_person"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "inputs"
            shutil.copytree(source, root)
            request = json.loads((root / "generation_request.json").read_text())
            request["schema"] = REQUEST_SCHEMA
            (root / "generation_request.json").write_text(json.dumps(request))
            raw = json.loads((root / "generated_plan.json").read_text())["raw_response"]
            result = generate(load_request(root / "generation_request.json"), RecordedClient(raw))
            self.assertEqual(result["schema"], RESULT_SCHEMA)
            self.assertEqual(result["generation"]["profile"], "picklift")
            self.assertTrue(result["generation"]["multimodal"])
            self.assertEqual(len(result["generation"]["views"]), 2)
            self.assertTrue(write_result(result, root / "output").is_file())

    def test_navigation_replays_through_common_pipeline(self):
        source = Path(__file__).resolve().parents[1] / "experiments/inputs/bt_navigation"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            references = {}
            for name in ("environment.json", "task_plan.json"):
                data = (source / name).read_bytes()
                (root / name).write_bytes(data)
                references[name] = hashlib.sha256(data).hexdigest()
            request = {
                "schema": REQUEST_SCHEMA,
                "request_id": "navigation-replay",
                "environment": {"path": "environment.json", "sha256": references["environment.json"]},
                "task": {
                    "description": "Navigate around the obstacle and park both arms.",
                    "plan_path": "task_plan.json",
                    "plan_sha256": references["task_plan.json"],
                },
                "observations": [],
                "model": {"id": "recorded", "max_attempts": 1},
            }
            (root / "request.json").write_text(json.dumps(request))
            raw = json.loads((source / "generated_plan.json").read_text())["raw_response"]
            result = generate(load_request(root / "request.json"), RecordedClient(raw))
            self.assertEqual(result["schema"], RESULT_SCHEMA)
            self.assertEqual(result["generation"]["profile"], "navigation")
            self.assertFalse(result["generation"]["multimodal"])
            self.assertEqual(result["tree"]["children"][0]["children"][1]["children"][1]["name"], "NavigateTo")
            output = write_result(result, root / "output")
            self.assertTrue((output.parent / "generated_bt.html").is_file())
            self.assertIn("NavigateTo", (output.parent / "generated_bt.mmd").read_text())
            reloaded, _ = load_generated_plan(
                output,
                json.loads((root / "environment.json").read_text()),
                json.loads((root / "task_plan.json").read_text()),
            )
            self.assertEqual(reloaded["mdsl_sha256"], result["mdsl_sha256"])

    def test_rejects_changed_bound_environment(self):
        source = Path(__file__).resolve().parents[1] / "experiments/inputs/bt_navigation"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "environment.json").write_bytes((source / "environment.json").read_bytes())
            (root / "task_plan.json").write_bytes((source / "task_plan.json").read_bytes())
            request = {
                "schema": REQUEST_SCHEMA, "request_id": "tamper",
                "environment": {"path": "environment.json", "sha256": "0" * 64},
                "task": {"description": "Navigate", "plan_path": "task_plan.json",
                         "plan_sha256": hashlib.sha256((root / "task_plan.json").read_bytes()).hexdigest()},
                "observations": [], "model": {"id": "recorded", "max_attempts": 1},
            }
            (root / "request.json").write_text(json.dumps(request))
            with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                load_request(root / "request.json")


if __name__ == "__main__":
    unittest.main()
