"""Tests for declarative SceneSmith BT generation."""

import json
import unittest
from pathlib import Path

from planner.src.bt.navigation import (
    compile_model_response,
    generate_tree,
    load_generation_inputs,
    planning_prompt,
    to_mdsl,
)


class GeneratedBtNavigationTest(unittest.TestCase):
    def setUp(self):
        root = Path(__file__).parents[1]
        inputs = root / "experiments/inputs/bt_navigation"
        self.environment, self.plan = load_generation_inputs(
            inputs / "environment.json", inputs / "task_plan.json"
        )

    def test_generates_canonical_bt_components(self):
        tree = generate_tree(self.environment, self.plan)
        self.assertEqual(tree.kind, "root")
        self.assertEqual(tree.children[0].kind, "selector")
        self.assertEqual(tree.children[0].children[0].kind, "condition")
        self.assertEqual(tree.children[0].children[1].kind, "sequence")
        mdsl = to_mdsl(tree)
        self.assertIn("condition [ParkedDualTargetsReached]", mdsl)
        self.assertIn("action [NavigateTo", mdsl)
        self.assertNotIn("TAMP", mdsl)

    def test_rejects_unavailable_plan_skill(self):
        plan = json.loads(json.dumps(self.plan))
        plan["steps"][0]["skill"] = "PlanTaskAndMotion"
        with self.assertRaisesRegex(ValueError, "Unsupported skill"):
            generate_tree(self.environment, plan)

    def test_accepts_runtime_extracted_environment(self):
        environment = json.loads(json.dumps(self.environment))
        environment["schema"] = "scenesmith.bt.environment.v2"
        environment["extraction"] = {
            "method": "scenesmith_runtime_reset_and_drake_proximity_geometry"
        }
        environment["navigation_geometry"] = {
            "obstacle_aabbs_xy_m": [[1.0, -0.3, 2.0, 0.3]]
        }
        path = self._testMethodName + ".json"
        from tempfile import TemporaryDirectory
        with TemporaryDirectory() as directory:
            environment_path = Path(directory) / path
            environment_path.write_text(json.dumps(environment))
            loaded, _ = load_generation_inputs(
                environment_path,
                Path(__file__).parents[1] / "experiments/inputs/bt_navigation/task_plan.json",
            )
        self.assertEqual(loaded["schema"], "scenesmith.bt.environment.v2")

    def test_v2_prompt_uses_extracted_planning_view(self):
        environment = json.loads(json.dumps(self.environment))
        environment.update({
            "schema": "scenesmith.bt.environment.v2",
            "extraction": {
                "method": "scenesmith_runtime_reset_and_drake_proximity_geometry",
                "reset_seed": 0,
                "reset_time_s": 0.0,
                "experiment_sha256": "abc",
            },
            "scene": {"dmd_path": "scene.dmd.yaml"},
            "robot": {
                "name": "dual",
                "arm_groups": {"left": [], "right": []},
                "grippers": {
                    "left": {"minimum_width_m": 0, "maximum_width_m": 0.07},
                    "right": {"minimum_width_m": 0, "maximum_width_m": 0.07},
                },
                "controlled_joints": [{"name": "large_unused_field"}],
            },
            "base": {"mode": "wheel_dynamic", "frame": "nav", "pose": {}, "blocked": False},
            "objects": {},
            "navigation_geometry": {"obstacle_aabbs_xy_m": [[1, -0.3, 2, 0.3]]},
        })
        prompt = planning_prompt(environment, self.plan)
        self.assertIn("obstacle_aabbs_xy_m", prompt)
        self.assertNotIn("large_unused_field", prompt)

    def test_compiles_strict_vlm_response(self):
        raw = json.dumps({
            "MAIN_SEQUENCE": """sequence {
action [Wait, \"2.0\"]
action [NavigateTo, \"2.8\", \"0.0\", \"0.0\", \"world\"]
action [SetDualJointTargets, \"-0.08\", \"0.06\", \"0.055\", \"0.4\"]
}""",
            "ULTIMATE_GOAL": "Park at the goal and hold both arm targets.",
        })
        response, tree = compile_model_response(self.environment, self.plan, raw)
        self.assertEqual(tree.kind, "root")
        self.assertEqual(response["ULTIMATE_GOAL"], "Park at the goal and hold both arm targets.")

    def test_rejects_vlm_plan_value_change(self):
        raw = json.dumps({
            "MAIN_SEQUENCE": """sequence {
action [Wait, \"2.0\"]
action [NavigateTo, \"2.7\", \"0.0\", \"0.0\", \"world\"]
action [SetDualJointTargets, \"-0.08\", \"0.06\", \"0.055\", \"0.4\"]
}""",
            "ULTIMATE_GOAL": "Use a changed goal.",
        })
        with self.assertRaisesRegex(ValueError, "preserve the validated task plan"):
            compile_model_response(self.environment, self.plan, raw)


if __name__ == "__main__":
    unittest.main()
