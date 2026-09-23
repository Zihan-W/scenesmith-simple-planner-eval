"""Hierarchical semantic and skill-skeleton contracts."""

import json
import unittest

from planner.src.tamp.hierarchy import (
    PredicateGoal, SkillRegistry, SkillSpec, WorldState, parse_semantic_goals,
    picklift_registry, refine_goals,
)
from planner.src.tamp.semantic import (
    ModelSettings, SemanticModelError, SemanticSubgoalPlanner,
)
from planner.src.tamp.geometry import (
    GeometricUnsat, SamplingSolver, abstract_constraint_feedback,
)


class HierarchicalContractsTest(unittest.TestCase):
    def test_transport_malformed_response_retries_without_joint_state_in_prompt(self):
        from planner.src.bt.generation import GenerationError

        class Client:
            calls = 0

            def complete(self, *, model, messages, temperature):
                self.calls += 1
                payload = json.loads(messages[1]["content"][0]["text"])
                if "robot" in payload["world"] or "translation_m" in payload["world"]["objects"]["cube"]:
                    raise AssertionError("Numeric robot/scene geometry leaked into semantic input")
                if payload["world"]["objects"]["cube"]["category"] != "manipuland":
                    raise AssertionError("Object metadata was lost")
                if self.calls == 1:
                    raise GenerationError("Provider returned an invalid Chat Completions response")
                return type("Response", (), {"content": json.dumps({
                    "schema": "scenesmith.tamp.goals.v3",
                    "subgoals": [{"predicate": "holding", "arguments": ["cube"]}],
                })})()

        events = []
        planner = SemanticSubgoalPlanner(Client(), ModelSettings(), trace=events.append)
        result = planner.propose(
            task="pick cube", predicate_arity={"holding": 1},
            world={"objects": {"cube": {"category": "manipuland", "translation_m": [1, 2, 3]}},
                   "robot": {"q": [0.1, 0.2]}},
        )
        self.assertEqual(result, (PredicateGoal("holding", ("cube",)),))
        self.assertEqual(planner.calls, 2)
        self.assertEqual([e["event"] for e in events], [
            "semantic_model_request", "semantic_model_error",
            "semantic_model_request", "semantic_model_response"])

    def test_batch_ranking_and_trace_include_every_evaluated_candidate(self):
        obj = ("$object",)
        registry = SkillRegistry((SkillSpec(
            "Reach", ("object",), ("distance",), (),
            (PredicateGoal("reachable", obj),)),))
        world = WorldState({"cube": {}}, frozenset())
        program = refine_goals(world, (PredicateGoal("reachable", ("cube",)),), registry)

        class Domain:
            def samples(self, skill, state):
                yield {"distance": 3.0}
                yield {"distance": 1.0}
                yield {"distance": 2.0}

            def check(self, skill, candidate, state):
                valid = candidate["distance"] != 2.0
                return valid, "valid" if valid else "collision", {}

            def rank_candidate(self, skill, candidate, checks):
                return (candidate["distance"],)

            def predict(self, skill, candidate, state):
                return state

        events = []
        result = SamplingSolver(registry, Domain(), trace=events.append).solve(world, program, {})
        self.assertEqual(result.assignments["distance_0"], 1.0)
        self.assertEqual(len(events), 3)
        self.assertEqual([e["feasible"] for e in events], [True, True, False])
        self.assertEqual(result.constraints[0].constraint, "collision")

    def test_basic_pick_inserts_prerequisite_and_keeps_geometry_open(self):
        world = WorldState(
            {"red_cube": {}},
            frozenset({PredicateGoal("observed", ("red_cube",)),
                       PredicateGoal("gripper_empty", ())}),
        )
        program = refine_goals(
            world, (PredicateGoal("holding", ("red_cube",)),), picklift_registry()
        )
        self.assertEqual([step.skill for step in program.steps],
                         ["NavigateToPick", "PickLift"])
        self.assertEqual(program.steps[0].continuous_variables,
                         {"base_pose": "base_pose_0"})
        self.assertEqual(program.steps[1].continuous_variables,
                         {"grasp_pose": "grasp_pose_1",
                          "approach_pose": "approach_pose_1"})

    def test_nary_semantic_goal_rejects_numeric_geometry(self):
        content = json.dumps({"schema": "scenesmith.tamp.goals.v3", "subgoals": [
            {"predicate": "inside", "arguments": ["red_cube", "target_bin"]}]})
        self.assertEqual(parse_semantic_goals(
            content, predicate_arity={"inside": 2},
            known_objects=frozenset({"red_cube", "target_bin"})
        ), (PredicateGoal("inside", ("red_cube", "target_bin")),))
        numeric = json.dumps({"schema": "scenesmith.tamp.goals.v3", "subgoals": [
            {"predicate": "inside", "arguments": ["red_cube", "0.22"]}]})
        with self.assertRaises(ValueError):
            parse_semantic_goals(numeric, predicate_arity={"inside": 2},
                                 known_objects=frozenset({"red_cube", "target_bin"}))

    def test_generic_registry_can_insert_new_prerequisite(self):
        object_arg = ("$object",)
        registry = SkillRegistry((
            SkillSpec("Open", ("object",), (),
                      (PredicateGoal("observed", object_arg),),
                      (PredicateGoal("open", object_arg),)),
            SkillSpec("Take", ("object",), ("grasp_pose",),
                      (PredicateGoal("open", object_arg),),
                      (PredicateGoal("holding", object_arg),)),
        ))
        world = WorldState({"drawer": {}},
                           frozenset({PredicateGoal("observed", ("drawer",))}))
        program = refine_goals(world, (PredicateGoal("holding", ("drawer",)),), registry)
        self.assertEqual([step.skill for step in program.steps], ["Open", "Take"])

    def test_semantic_model_retries_bad_json_and_never_accepts_numeric_pose(self):
        class Client:
            def __init__(self):
                self.responses = [
                    '{"schema":"scenesmith.tamp.goals.v3","subgoals":[{"predicate":"holding","arguments":["2.77"]}]}',
                    '{"schema":"scenesmith.tamp.goals.v3","subgoals":[{"predicate":"holding","arguments":["red_cube"]}]}',
                ]

            def complete(self, *, model, messages, temperature):
                self_model = model
                self_temperature = temperature
                self.assertions = (self_model, self_temperature, len(messages))
                return type("Response", (), {"content": self.responses.pop(0)})()

        client = Client()
        planner = SemanticSubgoalPlanner(client, ModelSettings(max_attempts=2))
        goals = planner.propose(
            task="pick red_cube", world={"objects": {"red_cube": {}}},
            predicate_arity={"holding": 1},
        )
        self.assertEqual(goals, (PredicateGoal("holding", ("red_cube",)),))
        self.assertEqual(planner.calls, 2)
        self.assertEqual(client.assertions, ("gpt-4.1-mini", 0.2, 4))

        class InvalidClient:
            def complete(self, *, model, messages, temperature):
                return type("Response", (), {"content": "not JSON"})()

        with self.assertRaises(SemanticModelError):
            SemanticSubgoalPlanner(InvalidClient(), ModelSettings(max_attempts=2)).propose(
                task="pick red_cube", world={"objects": {"red_cube": {}}},
                predicate_arity={"holding": 1},
            )

    def test_geometry_solver_tries_next_candidate_without_changing_skill(self):
        class Domain:
            def __init__(self):
                self.checked = []

            def samples(self, skill, state):
                del state
                if skill.skill == "NavigateToPick":
                    yield {"base_x_m": 1.0, "base_y_m": 0.0, "base_yaw_rad": 0.0}
                    yield {"base_x_m": 2.0, "base_y_m": 0.0, "base_yaw_rad": 0.0}
                else:
                    yield {"target": "red_cube", "arm": "left",
                           "grasp_lateral_offset_m": 0.0}

            def check(self, skill, candidate, state):
                self.checked.append((skill.skill, dict(candidate)))
                if skill.skill == "NavigateToPick" and candidate["base_x_m"] == 1.0:
                    return False, "corridor", {"message": "blocked"}
                return True, "valid", {"ik": {}}

            def predict(self, skill, candidate, state):
                if skill.skill == "NavigateToPick":
                    state["base_pose"] = candidate["base_x_m"]
                return state

        world = WorldState(
            {"red_cube": {}},
            frozenset({PredicateGoal("observed", ("red_cube",)),
                       PredicateGoal("gripper_empty", ())}),
        )
        program = refine_goals(
            world, (PredicateGoal("holding", ("red_cube",)),), picklift_registry()
        )
        domain = Domain()
        solver = SamplingSolver(picklift_registry(), domain, batch_size=2)
        result = solver.solve(world, program, {"base_height_m": 0.18})
        self.assertEqual([action.skill_name for action in result.actions],
                         ["NavigateToPick", "PickLift"])
        self.assertEqual(result.assignments["base_pose_0"], (2.0, 0.0, 0.0))
        self.assertEqual(result.constraints[0].constraint, "corridor")
        self.assertEqual(result.actions[-1].expected_effects,
                         (PredicateGoal("holding", ("red_cube",)),))

        class InfeasibleDomain(Domain):
            def check(self, skill, candidate, state):
                return False, "ik", {"message": "unreachable"}

        with self.assertRaises(GeometricUnsat) as failure:
            SamplingSolver(picklift_registry(), InfeasibleDomain()).solve(
                world, program, {"base_height_m": 0.18})
        self.assertTrue(failure.exception.constraints)
        abstract = abstract_constraint_feedback(
            failure.exception.constraints, skill="NavigateToPick", target="red_cube")
        self.assertEqual(abstract["type"], "GEOMETRIC_INFEASIBLE")
        self.assertEqual(abstract["reason"], "ik")
        self.assertEqual(abstract["involved_objects"], ["red_cube"])
        self.assertNotIn("joint_positions", abstract)


if __name__ == "__main__":
    unittest.main()
