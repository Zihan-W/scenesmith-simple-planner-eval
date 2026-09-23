import json
import unittest
from types import SimpleNamespace

from planner.src.tamp.program import (
    ProgramGrounder, SketchGrounder, parse_program,
)
from planner.src.tamp.planner import (
    Subgoal, SymbolicOperator, SymbolicVlmProposer, plan_with_reprompting,
)


class Domain:
    skill_names = frozenset({"navigate", "pick"})
    program_schema = {"navigate": {"required": ["x"]}}
    sampler_hints = {"base_candidates": [1, 2]}

    def check(self, skill, parameters, state):
        if skill.skill == "pick" and state["base_x"] != 2:
            return False, "ik_unreachable", {}
        return True, "valid", {"checked": True}

    def predict(self, skill, parameters, state):
        if skill.skill == "navigate":
            state["base_x"] = parameters["x"]
        return state


class ProgramTest(unittest.TestCase):
    def test_live_two_request_contract_keeps_images_and_symbolic_order(self):
        program = {
            "schema": "scenesmith.tamp.program.v1",
            "variables": {"x": {"type": "discrete", "values": [1, 2]}},
            "steps": [
                {"skill": "navigate", "arguments": {"target": "red"},
                 "parameters": {"x": {"var": "x"}}},
                {"skill": "pick", "arguments": {"target": "red"},
                 "parameters": {"grasp": {"const": "side"}}},
            ],
        }

        class Client:
            def __init__(self):
                self.calls = []

            def complete(self, *, model, messages):
                self.calls.append((model, messages))
                content = ({"schema": "scenesmith.tamp.goals.v2",
                            "subgoals": [{"predicate": "holding", "target": "red"}]}
                           if len(self.calls) == 1 else program)
                return SimpleNamespace(content=json.dumps(content))

        client = Client()
        images = ({"path": "/tmp/head.png", "sha256": "a" * 64},)
        proposer = SymbolicVlmProposer(
            client=client, model="test-model", images=images,
            skill_descriptions={"navigate": "move", "pick": "grasp"},
            predicates={"holding": "stable lift"},
            operators=(
                SymbolicOperator("navigate", frozenset({"observed"}),
                                 frozenset({"at_pick_pose"})),
                SymbolicOperator("pick", frozenset({"at_pick_pose"}),
                                 frozenset({"holding"})),
            ),
        )
        grounder = SketchGrounder(Domain(), world={"objects": {"red": {}}},
                                  client=client, model="test-model", images=images)
        result = plan_with_reprompting(
            proposer=proposer, grounder=grounder, goal="lift red",
            world={"objects": {"red": {}}}, initial_state={"base_x": 0},
            max_proposals=1,
        )
        self.assertTrue(result.success)
        self.assertEqual([step.skill for step in result.steps], ["navigate", "pick"])
        self.assertEqual(len(client.calls), 2)
        self.assertTrue(all(call[1][1]["content"][1]["type"] == "image_url"
                            for call in client.calls))
        sketch_prompt = json.loads(client.calls[1][1][1]["content"][0]["text"])
        self.assertEqual(sketch_prompt["skill_parameters"], Domain.program_schema)
        self.assertEqual(sketch_prompt["sampler_hints"], Domain.sampler_hints)

    def test_joint_parameter_sampling_and_rollout_gate(self):
        sketch = parse_program(json.dumps({
            "schema": "scenesmith.tamp.program.v1",
            "variables": {"x": {"type": "discrete", "values": [1, 2]}},
            "steps": [
                {"skill": "navigate", "arguments": {},
                 "parameters": {"x": {"var": "x"}}},
                {"skill": "pick", "arguments": {},
                 "parameters": {"grasp": {"const": "side"}}},
            ],
        }), Domain.skill_names)
        rolled = []

        def rollout(steps):
            rolled.append(steps[0].parameters["x"])
            return True, "success", {"physical": True}

        result = ProgramGrounder(Domain(), rollout=rollout).solve(
            sketch, {"base_x": 0})
        self.assertTrue(result.success)
        self.assertEqual(result.candidates_checked, 2)
        self.assertEqual(result.failures[0]["reason"], "ik_unreachable")
        self.assertEqual(rolled, [2])

    def test_rejects_executable_model_content(self):
        with self.assertRaisesRegex(ValueError, "schema, variables and steps only"):
            parse_program(json.dumps({"schema": "scenesmith.tamp.program.v1",
                                      "variables": {}, "steps": [], "python": "exec(1)"}),
                          Domain.skill_names)

    def test_sketch_must_match_symbolic_refinement(self):
        program = json.dumps({
            "schema": "scenesmith.tamp.program.v1",
            "variables": {},
            "steps": [{"skill": "pick", "arguments": {},
                       "parameters": {"grasp": {"const": "side"}}}],
        })
        result = SketchGrounder(Domain(), world={}, recorded_program=program).solve(
            (Subgoal("navigate", {}),), {"base_x": 0})
        self.assertFalse(result.success)
        self.assertEqual(result.failures[0]["reason"], "invalid_program")


if __name__ == "__main__":
    unittest.main()
