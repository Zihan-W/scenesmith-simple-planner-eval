"""Tests for strict generated PickLift Behavior Trees."""

import json
import unittest

from planner.src.bt.picklift import (
    Parser, canonical_tree, compile_response, to_mdsl)


class PickLiftBtTest(unittest.TestCase):
    def setUp(self):
        self.environment = {
            "schema": "scenesmith.verigraph_pick.environment.v1",
            "verigraph": {"parser": "verigraph.core.parse.parse_llm_response_to_graph"},
            "available_skills": ["Wait", "ExecutePickLift", "PickLiftSucceeded"],
        }
        self.plan = {"success_condition": "PickLiftSucceeded", "steps": [
            {"skill": "Wait", "duration_s": 1.0}, {"skill": "ExecutePickLift"}]}

    def test_canonical_components(self):
        mdsl = to_mdsl(canonical_tree(self.plan))
        self.assertIn("root {", mdsl)
        self.assertIn("selector {", mdsl)
        self.assertIn("condition [PickLiftSucceeded]", mdsl)
        self.assertIn("sequence {", mdsl)
        self.assertIn("action [ExecutePickLift]", mdsl)

    def test_strict_model_response(self):
        raw = json.dumps({"MAIN_SEQUENCE": "sequence {\n"
                          "action [Wait, \"1.0\"]\n"
                          "action [ExecutePickLift]\n}",
                          "ULTIMATE_GOAL": "The red target is lifted and held."})
        response, root = compile_response(self.environment, self.plan, raw)
        self.assertEqual(response["ULTIMATE_GOAL"], "The red target is lifted and held.")
        self.assertEqual(root.kind, "root")

    def test_rejects_extra_action(self):
        raw = json.dumps({"MAIN_SEQUENCE": "sequence {\n"
                          "action [Wait, \"1.0\"]\n"
                          "action [ExecutePickLift]\n"
                          "action [ExecutePickLift]\n}", "ULTIMATE_GOAL": "lift"})
        with self.assertRaisesRegex(ValueError, "changed"):
            compile_response(self.environment, self.plan, raw)

    def test_parser_rejects_unknown_skill(self):
        with self.assertRaisesRegex(ValueError, "Unknown"):
            Parser("action [Teleport]").parse()


if __name__ == "__main__":
    unittest.main()
