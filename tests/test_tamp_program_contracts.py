"""Open-variable skeleton parsing and replaceable solver semantic invariants."""

import dataclasses
import json
import unittest

from planner.src.tamp.geometry import ExternalGeometrySolverAdapter, SamplingSolver
from planner.src.tamp.hierarchy import (
    SkillRegistry, SkillSpec, WorldState, parse_skill_program,
)


class SkillProgramContractTest(unittest.TestCase):
    def setUp(self):
        self.registry = SkillRegistry((SkillSpec(
            "Transfer", ("object", "destination"), ("offset",), (), ()),))
        self.world = WorldState({"cube": {}, "bin": {}}, frozenset())
        self.document = {"schema": "scenesmith.tamp.skill_program.v2", "steps": [
            {"skill": "Transfer", "arguments": {"object": "cube", "destination": "bin"},
             "continuous_variables": {"offset": "shared_offset"}},
            {"skill": "Transfer", "arguments": {"object": "cube", "destination": "bin"},
             "continuous_variables": {"offset": "shared_offset"}},
        ]}

    def parse(self):
        return parse_skill_program(json.dumps(self.document), registry=self.registry,
                                   known_objects=frozenset(self.world.objects))

    def test_rejects_numeric_domains_and_unknown_nonprimary_objects(self):
        self.document["steps"][0]["continuous_variables"]["offset"] = {"low": 0, "high": 1}
        with self.assertRaisesRegex(ValueError, "symbolic identifiers"):
            self.parse()
        self.document["steps"][0]["continuous_variables"]["offset"] = "shared_offset"
        self.document["steps"][0]["arguments"]["destination"] = "unknown_bin"
        with self.assertRaisesRegex(ValueError, "symbolic skill arguments"):
            self.parse()

    def solve(self):
        class Domain:
            def samples(self, skill, state):
                for offset in ((2, 1) if state.get("first_done") else (1,)):
                    yield {"offset": offset}

            def check(self, skill, candidate, state):
                # The n-ary destination must survive the SceneSmith object->target binding.
                if skill.arguments != {"target": "cube", "destination": "bin"}:
                    raise AssertionError("Solver dropped a symbolic argument")
                return True, "valid", {}

            def predict(self, skill, candidate, state):
                return {**state, "first_done": True}

        program = self.parse()
        result = SamplingSolver(self.registry, Domain()).solve(self.world, program, {})
        return program, result

    def test_shared_variable_is_not_overwritten_by_later_skill(self):
        _, result = self.solve()
        self.assertEqual(result.assignments, {"shared_offset": 1})
        self.assertEqual([action.geometric_parameters["offset"] for action in result.actions], [1, 1])
        self.assertEqual(result.constraints[0].constraint, "shared_variable_conflict")

    def test_external_adapter_rejects_changed_symbolic_task(self):
        program, result = self.solve()

        class Backend:
            def solve(self, *args, **kwargs):
                return result

        adapter = ExternalGeometrySolverAdapter(Backend())
        self.assertEqual(adapter.solve(self.world, program, {}), result)
        result = dataclasses.replace(result, actions=(dataclasses.replace(
            result.actions[0], symbolic_args={"object": "bin", "destination": "cube"}),
            result.actions[1]))
        with self.assertRaisesRegex(ValueError, "changed the symbolic"):
            adapter.solve(self.world, program, {})

    def test_external_backend_cannot_mutate_the_input_task(self):
        program, result = self.solve()

        class MutatingBackend:
            def solve(self, world, program, state, **kwargs):
                program.steps[0].arguments["object"] = "bin"
                world.objects.clear()
                state["changed"] = True
                return dataclasses.replace(result, actions=(dataclasses.replace(
                    result.actions[0], symbolic_args=program.steps[0].arguments),
                    result.actions[1]))

        state = {}
        with self.assertRaisesRegex(ValueError, "changed the symbolic"):
            ExternalGeometrySolverAdapter(MutatingBackend()).solve(self.world, program, state)
        self.assertEqual(program.steps[0].arguments["object"], "cube")
        self.assertEqual(set(self.world.objects), {"cube", "bin"})
        self.assertEqual(state, {})


if __name__ == "__main__":
    unittest.main()
