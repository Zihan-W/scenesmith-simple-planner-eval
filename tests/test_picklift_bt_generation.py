"""Contract tests for the scene-and-task-to-BT module."""

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from examples.online_manipulation.picklift_bt_generation import (
    CAMERA_ORDER,
    ChatCompletion,
    GenerationError,
    REQUEST_SCHEMA,
    RESULT_SCHEMA,
    build_messages,
    generate,
    load_request,
    write_result,
)


PNG = b"\x89PNG\r\n\x1a\ncontract-test"


def digest(data):
    return hashlib.sha256(data).hexdigest()


class FakeClient:
    def __init__(self, responses):
        self.responses = iter(responses)
        self.messages = []

    def complete(self, *, model, messages):
        self.messages.append(json.loads(json.dumps(messages, default=str)))
        content = next(self.responses)
        return ChatCompletion(
            content=content,
            model=model,
            response_id=f"response-{len(self.messages)}",
            finish_reason="stop",
            usage={"total_tokens": 10},
        )


class PickLiftBtGenerationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        camera_data = {}
        visibility = {}
        observations = []
        for camera in CAMERA_ORDER:
            path = self.root / f"{camera}.png"
            data = PNG + camera.encode()
            path.write_bytes(data)
            sha = digest(data)
            camera_data[camera] = {
                "frame": camera,
                "timestamp_s": 0.0,
                "pose": {"translation_m": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]},
                "intrinsics": {"width": 1, "height": 1},
                "files": {"rgb": {"path": path.name, "sha256": sha}},
            }
            visibility[camera] = {"visible": True, "pixel_count": 1}
            observations.append({
                "camera": camera,
                "path": path.name,
                "sha256": sha,
                "media_type": "image/png",
            })
        environment = {
            "schema": "scenesmith.verigraph_pick.environment.v1",
            "runtime_snapshot": {
                "robot": {"name": "test_robot"},
                "task": {"required_lift_m": 0.08, "required_hold_s": 3.0},
                "robot_camera_observations": camera_data,
                "target_camera_visibility": visibility,
                "available_bt_skills": ["Wait", "ExecutePickLift", "PickLiftSucceeded"],
            },
            "verigraph": {
                "parser": "verigraph.core.parse.parse_llm_response_to_graph",
                "metadata": {"environment": {
                    "semantic_map_locations": {"station": "test"},
                    "objects_metadata": {"pick_target": {"kind": "object"}},
                    "assets_metadata": {"table": {"kind": "asset"}},
                    "asset_object_relations": {"table": ["(on_something)pick_target"]},
                    "location_asset_relations": {"station": ["table"]},
                    "scene_graph": {
                        "nodes": ["table", "pick_target"],
                        "relations": ["pick_target, on, table"],
                    },
                }},
            },
            "available_skills": ["Wait", "ExecutePickLift", "PickLiftSucceeded"],
        }
        task_plan = {
            "schema": "scenesmith.bt.picklift_task_plan.v1",
            "task": "pick_target",
            "success_condition": "PickLiftSucceeded",
            "steps": [
                {"skill": "Wait", "duration_s": 1.0},
                {"skill": "ExecutePickLift"},
            ],
        }
        environment_bytes = (json.dumps(environment, indent=2) + "\n").encode()
        task_bytes = (json.dumps(task_plan, indent=2) + "\n").encode()
        (self.root / "environment.json").write_bytes(environment_bytes)
        (self.root / "task_plan.json").write_bytes(task_bytes)
        request = {
            "schema": REQUEST_SCHEMA,
            "request_id": "contract-test",
            "environment": {
                "path": "environment.json", "sha256": digest(environment_bytes),
            },
            "task": {
                "description": (
                    "Wait 1.0 seconds, pick up pick_target, lift it 0.08 m, and hold 3.0 seconds."
                ),
                "plan_path": "task_plan.json",
                "plan_sha256": digest(task_bytes),
            },
            "observations": observations,
            "model": {"id": "gpt-4.1-mini-2025-04-14", "max_attempts": 3},
        }
        self.request_path = self.root / "request.json"
        self.request_path.write_text(json.dumps(request, indent=2) + "\n")

    def tearDown(self):
        self.temporary.cleanup()

    def test_fixed_request_builds_first_person_messages(self):
        loaded = load_request(self.request_path)
        messages = build_messages(loaded)
        content = messages[-1]["content"]
        self.assertEqual([item["type"] for item in content], ["text", "image_url", "image_url"])
        self.assertIn("no external or third-person image", content[0]["text"])
        self.assertEqual(
            [Path(item["image_url"]["path"]).stem for item in content[1:]],
            list(CAMERA_ORDER),
        )

    def test_invalid_response_is_repaired_then_compiled(self):
        invalid = json.dumps({
            "MAIN_SEQUENCE": ["Wait", "ExecutePickLift"],
            "ULTIMATE_GOAL": "lift",
            "WHERE_TO_CHECK_GOAL": "station",
        })
        valid = json.dumps({
            "MAIN_SEQUENCE": (
                'sequence { action [Wait, "1.0"] action [ExecutePickLift] }'
            ),
            "ULTIMATE_GOAL": "pick_target is lifted 0.08 m and held for 3.0 seconds",
        })
        client = FakeClient([invalid, valid])
        result = generate(load_request(self.request_path), client)
        self.assertEqual(result["schema"], RESULT_SCHEMA)
        self.assertEqual(result["generation"]["api_calls_completed"], 2)
        self.assertEqual(result["generation"]["accepted_attempt"], 2)
        self.assertIsNotNone(result["generation"]["attempts"][0]["validation_error"])
        self.assertIsNone(result["generation"]["attempts"][1]["validation_error"])
        self.assertEqual(result["generation"]["reference_answer_used"], False)
        self.assertEqual(result["generation"]["robot_first_person_only"], True)
        self.assertEqual(
            set(result),
            {
                "schema", "request_id", "request_sha256", "raw_response",
                "parsed_response", "generation", "mdsl", "mdsl_sha256", "tree",
            },
        )
        output = write_result(result, self.root / "output")
        self.assertTrue(output.is_file())
        self.assertTrue((output.parent / "generated_bt.mmd").is_file())

    def test_rejects_image_not_bound_to_environment(self):
        request = json.loads(self.request_path.read_text())
        altered = PNG + b"altered"
        (self.root / "head_camera.png").write_bytes(altered)
        request["observations"][0]["sha256"] = digest(altered)
        self.request_path.write_text(json.dumps(request))
        with self.assertRaisesRegex(ValueError, "not bound"):
            load_request(self.request_path)

    def test_rejects_non_robot_camera_order(self):
        request = json.loads(self.request_path.read_text())
        request["observations"].reverse()
        self.request_path.write_text(json.dumps(request))
        with self.assertRaisesRegex(ValueError, "must be head_camera"):
            load_request(self.request_path)

    def test_exhausted_compiler_attempts_raise_without_output(self):
        request = json.loads(self.request_path.read_text())
        request["model"]["max_attempts"] = 1
        self.request_path.write_text(json.dumps(request))
        client = FakeClient(["not json"])
        with self.assertRaisesRegex(GenerationError, "No response passed"):
            generate(load_request(self.request_path), client)


if __name__ == "__main__":
    unittest.main()
