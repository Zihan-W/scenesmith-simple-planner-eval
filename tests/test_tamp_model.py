"""Legacy live-model failures remain bounded and never bypass validation."""

import json
import unittest
from types import SimpleNamespace

from planner.src.bt.generation import GenerationError
from planner.src.tamp.model import PlannerModelError, validated_completion
from planner.src.tamp.planner import (
    JsonVlmProposer, Subgoal, plan_with_reprompting,
)
from planner.src.tamp.program import SketchGrounder


class SequenceClient:
    def __init__(self, values):
        self.values = iter(values)
        self.calls = []

    def complete(self, **kwargs):
        self.calls.append(kwargs)
        value = next(self.values)
        if isinstance(value, Exception):
            raise value
        return SimpleNamespace(content=value)


class LegacyModelTest(unittest.TestCase):
    def test_transport_and_json_errors_retry_with_configured_temperature(self):
        client = SequenceClient([GenerationError("provider invalid JSON"), "{", '{"ok":true}'])
        attempts = []
        value, _ = validated_completion(
            client=client, model="configured", messages=[], validate=json.loads,
            max_attempts=3, temperature=0.4, attempts=attempts)
        self.assertEqual(value, {"ok": True})
        self.assertEqual([item["valid"] for item in attempts], [False, False, True])
        self.assertTrue(all(call["temperature"] == 0.4 for call in client.calls))
        self.assertIn("Invalid JSON/schema", client.calls[-1]["messages"][-1]["content"])

    def test_exhaustion_is_a_typed_failure(self):
        client = SequenceClient(["bad", "bad"])
        with self.assertRaises(PlannerModelError):
            validated_completion(client=client, model="test", messages=[],
                                 validate=json.loads, max_attempts=2)
        self.assertEqual(len(client.calls), 2)

    def test_legacy_proposer_records_terminal_failure_without_grounding(self):
        client = SequenceClient(["bad", "bad"])
        proposer = JsonVlmProposer(client=client, model="test", max_attempts=2,
                                   skill_descriptions={"pick": "Pick the object"})
        result = plan_with_reprompting(proposer=proposer, grounder=None,
                                      goal="pick red", world={}, initial_state={})
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "planner_model_error")
        self.assertEqual(len(proposer.model_attempts), 2)

    def test_program_retries_changed_skeleton_before_any_geometry(self):
        class Domain:
            skill_names = frozenset({"pick"})

            def check(self, subgoal, parameters, state):
                self.checked = subgoal.arguments["target"]
                return True, "valid", {}

            def predict(self, subgoal, parameters, state):
                return state

        def program(target):
            return json.dumps({"schema": "scenesmith.tamp.program.v1",
                               "variables": {"x": {"type": "discrete", "values": [1]}},
                               "steps": [{"skill": "pick", "arguments": {"target": target},
                                          "parameters": {"x": {"var": "x"}}}]})

        client = SequenceClient([program("wrong"), program("red")])
        domain = Domain()
        grounder = SketchGrounder(domain, world={}, client=client, model="test",
                                  max_attempts=2, temperature=0.1)
        result = grounder.solve((Subgoal("pick", {"target": "red"}),), {})
        self.assertTrue(result.success)
        self.assertEqual(domain.checked, "red")
        self.assertEqual(len(client.calls), 2)
        self.assertEqual(len(grounder.programs[0]["model_attempts"]), 2)

    def test_program_provider_exhaustion_returns_failure_evidence(self):
        domain = SimpleNamespace(skill_names=frozenset({"pick"}))
        grounder = SketchGrounder(domain, world={}, model="test", max_attempts=2,
                                  client=SequenceClient([GenerationError("bad"), "invalid"]))
        result = grounder.solve((Subgoal("pick", {"target": "red"}),), {})
        self.assertFalse(result.success)
        self.assertEqual(result.failures[0]["reason"], "planner_model_error")
        self.assertEqual(result.candidates_checked, 0)


if __name__ == "__main__":
    unittest.main()
