"""Constraint search must gate every proposed skill before execution."""

import json
import unittest

from planner.src.tamp.planner import (
    ConstraintGrounder, GoalPredicate, Subgoal, SymbolicOperator,
    parse_goal_proposal, parse_proposal, plan_with_reprompting, refine_goals,
)


class TwoStepDomain:
    skill_names = frozenset({"navigate", "pick"})

    def samples(self, subgoal, state):
        if subgoal.skill == "navigate":
            yield {"x": 1}
            yield {"x": 2}
        else:
            yield {"grasp": "side"}

    def check(self, subgoal, parameters, state):
        if subgoal.skill == "pick" and state["base_x"] != 2:
            return False, "ik_unreachable", {"base_x": state["base_x"]}
        return True, "valid", {"collision_checked": True}

    def predict(self, subgoal, parameters, state):
        if subgoal.skill == "navigate":
            state["base_x"] = parameters["x"]
        return state


class ReplanningProposer:
    def __init__(self):
        self.feedback = []

    def propose(self, *, goal, world, feedback):
        self.feedback.append(feedback)
        if not feedback:
            return (Subgoal("pick", {"object": "red"}),)
        return (Subgoal("navigate", {}), Subgoal("pick", {"object": "red"}))


class TampPlannerTest(unittest.TestCase):
    def test_grounder_backtracks_when_pick_is_unreachable(self):
        steps = (Subgoal("navigate", {}), Subgoal("pick", {"object": "red"}))
        result = ConstraintGrounder(TwoStepDomain()).solve(steps, {"base_x": 0})
        self.assertTrue(result.success)
        self.assertEqual(result.steps[0].parameters, {"x": 2})
        self.assertEqual(result.candidates_checked, 4)
        self.assertEqual(result.failures[0]["reason"], "ik_unreachable")

    def test_failed_grounding_is_fed_back_before_replanning(self):
        proposer = ReplanningProposer()
        result = plan_with_reprompting(
            proposer=proposer, grounder=ConstraintGrounder(TwoStepDomain()),
            goal="lift red object", world={}, initial_state={"base_x": 0},
        )
        self.assertTrue(result.success)
        self.assertEqual(result.proposals[0]["failure_counts"], {"ik_unreachable": 1})
        self.assertEqual(proposer.feedback[1][0]["failure_counts"]["ik_unreachable"], 1)

    def test_physics_failure_backtracks_to_next_candidate(self):
        calls = []

        def rollout(steps):
            x = steps[0].parameters["x"]
            calls.append(x)
            return (x == 2, "unstable_grasp", {"base_x": x})

        result = ConstraintGrounder(TwoStepDomain(), rollout=rollout).solve(
            (Subgoal("navigate", {}),), {"base_x": 0})
        self.assertTrue(result.success)
        self.assertEqual(calls, [1, 2])
        self.assertEqual(result.failures[0]["reason"], "unstable_grasp")

    def test_model_cannot_inject_unknown_skill_or_code(self):
        allowed = frozenset({"navigate", "pick"})
        with self.assertRaisesRegex(ValueError, "unavailable"):
            parse_proposal(json.dumps({"schema": "scenesmith.tamp.subgoals.v1",
                                       "subgoals": [{"skill": "exec_python", "arguments": {}}]}),
                           allowed)
        with self.assertRaisesRegex(ValueError, "schema and subgoals only"):
            parse_proposal(json.dumps({"schema": "scenesmith.tamp.subgoals.v1",
                                       "subgoals": [], "code": "print(1)"}), allowed)

    def test_symbolic_goal_refines_through_prerequisite(self):
        goals = parse_goal_proposal(json.dumps({
            "schema": "scenesmith.tamp.goals.v2",
            "subgoals": [{"predicate": "holding", "target": "red"}],
        }), frozenset({"holding"}))
        operators = (
            SymbolicOperator("NavigateToPick", frozenset({"observed"}),
                             frozenset({"at_pick_pose"})),
            SymbolicOperator("PickLift", frozenset({"at_pick_pose"}),
                             frozenset({"holding"})),
        )
        steps = refine_goals(goals, operators,
                             frozenset({GoalPredicate("observed", "red")}))
        self.assertEqual([step.skill for step in steps],
                         ["NavigateToPick", "PickLift"])


if __name__ == "__main__":
    unittest.main()
