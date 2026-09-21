"""Provider response-format requests supplement, never replace, local validation."""

import io
import json
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from examples.online_manipulation.bt_generation import OpenAICompatibleChatClient
from examples.online_manipulation.tamp_model import goal_response_format, json_response_format
from examples.online_manipulation.tamp_semantic import ModelSettings, SemanticSubgoalPlanner


class StructuredOutputTest(unittest.TestCase):
    def test_schema_carries_exact_symbolic_objects_predicates_and_arities(self):
        response_format = goal_response_format(
            "json_schema", schema_name="scenesmith.tamp.goals.v3",
            predicate_arity={"inside": 2, "empty": 0}, known_objects={"cube", "bin"})
        self.assertTrue(response_format["json_schema"]["strict"])
        schema = response_format["json_schema"]["schema"]
        self.assertFalse(schema["additionalProperties"])
        variants = schema["properties"]["subgoals"]["items"]["anyOf"]
        by_predicate = {item["properties"]["predicate"]["enum"][0]: item for item in variants}
        for name, count in (("inside", 2), ("empty", 0)):
            item = by_predicate[name]
            self.assertFalse(item["additionalProperties"])
            arguments = item["properties"]["arguments"]
            self.assertEqual((arguments["minItems"], arguments["maxItems"]), (count, count))
            self.assertEqual(arguments["items"]["enum"], ["bin", "cube"])

    def test_local_validation_retries_even_if_provider_ignores_schema(self):
        calls = []

        class Client:
            def complete(self, **kwargs):
                calls.append(kwargs)
                return SimpleNamespace(content=json.dumps({
                    "schema": "scenesmith.tamp.goals.v3", "subgoals": [
                        {"predicate": "holding", "arguments": ["unknown" if len(calls) == 1 else "cube"]}]}))

        planner = SemanticSubgoalPlanner(
            Client(), ModelSettings(model="configured", response_format="json_schema"))
        goals = planner.propose(task="pick cube", world={"objects": {"cube": {}}},
                                predicate_arity={"holding": 1})
        self.assertEqual(goals[0].arguments, ("cube",))
        self.assertEqual(planner.calls, 2)
        self.assertTrue(all(call["response_format"]["type"] == "json_schema" for call in calls))

    def test_transport_preserves_format_and_default_bt_request_is_unchanged(self):
        client = OpenAICompatibleChatClient(base_url="https://example.invalid/v1", api_key="unused")
        payload = json.dumps({"choices": [{"message": {"content": "{}"}, "finish_reason": "stop"}]})
        for response_format in (None, {"type": "json_object"}):
            with patch("examples.online_manipulation.bt_generation.urlopen",
                       return_value=io.StringIO(payload)) as request:
                client.complete(model="test", messages=[], response_format=response_format)
            body = json.loads(request.call_args.args[0].data)
            if response_format is None:
                self.assertNotIn("response_format", body)
                self.assertNotIn("temperature", body)
            else:
                self.assertEqual(body["response_format"], response_format)

    def test_free_key_legacy_program_does_not_silently_downgrade_strict_schema(self):
        with self.assertRaises(ValueError):
            json_response_format("json_schema")
        self.assertEqual(json_response_format("json_object"), {"type": "json_object"})


if __name__ == "__main__":
    unittest.main()
