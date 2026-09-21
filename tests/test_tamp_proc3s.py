"""Deterministic contract tests; not substitutes for live model acceptance."""

import copy
import json
import unittest

from examples.online_manipulation.tamp_hierarchy import PredicateGoal, WorldState, picklift_registry
from examples.online_manipulation.tamp_proc3s import parse_proc3s_program
from examples.online_manipulation.tamp_skill_planning import StripsProgramGenerator


class PRoC3SProgramTest(unittest.TestCase):
    def setUp(self):
        self.registry = picklift_registry()
        self.world = WorldState({"red_cube": {}}, frozenset({
            PredicateGoal("observed", ("red_cube",)), PredicateGoal("gripper_empty", ())}))
        self.goals = (PredicateGoal("holding", ("red_cube",)),)
        self.document = {
            "schema": "scenesmith.proc3s.program.v1",
            "steps": [
                {"skill": "NavigateToPick", "arguments": {"object": "red_cube"},
                 "continuous_variables": {"base_pose": "$b0"}},
                {"skill": "PickLift", "arguments": {"object": "red_cube"},
                 "continuous_variables": {"grasp_pose": "$g0", "approach_pose": "$a0"}},
            ],
            "domains": [
                {"variable": "b0", "sampler": "scene_base_pose"},
                {"variable": "g0", "sampler": "calibrated_grasp_pose"},
                {"variable": "a0", "sampler": "calibrated_approach_pose"},
            ],
        }

    def parse(self, document=None):
        return parse_proc3s_program(json.dumps(document or self.document), registry=self.registry,
                                   world=self.world, goals=self.goals)

    def test_open_program_and_domains_are_preserved(self):
        program = self.parse()
        self.assertEqual(program.steps[1].continuous_variables["grasp_pose"], "g0")
        self.assertEqual(program.parameter_domains["g0"], "calibrated_grasp_pose")

    def test_numeric_solution_rejected(self):
        self.document["steps"][0]["continuous_variables"]["base_pose"] = [0.4, 0.2, 0.0]
        with self.assertRaisesRegex(ValueError, "open"):
            self.parse()

    def test_domain_binding_rejected(self):
        for edit in ("missing", "numeric", "wrong_type", "duplicate"):
            with self.subTest(edit=edit):
                doc = copy.deepcopy(self.document)
                if edit == "missing":
                    doc["domains"].pop()
                elif edit == "numeric":
                    doc["domains"][0]["value"] = 0.4
                elif edit == "wrong_type":
                    doc["domains"][0]["sampler"] = "calibrated_grasp_pose"
                else:
                    doc["domains"].append(doc["domains"][0])
                with self.assertRaises(ValueError):
                    self.parse(doc)

    def test_missing_precondition_rejected(self):
        self.document["steps"].pop(0)
        self.document["domains"].pop(0)
        with self.assertRaisesRegex(ValueError, "precondition"):
            self.parse()

    def test_strips_is_explicit_non_model_baseline(self):
        planner = StripsProgramGenerator(self.registry)
        program = planner.generate(self.world, self.goals)
        self.assertEqual(planner.name, "strips")
        self.assertEqual(planner.calls, 0)
        self.assertEqual([step.skill for step in program.steps], ["NavigateToPick", "PickLift"])


if __name__ == "__main__":
    unittest.main()
