"""Regression tests for the three-level failure information boundary."""

import json
import unittest

from examples.online_manipulation.tamp_failures import (
    LowLevelFailure, ProgramFailure, SemanticFailure,
)
from examples.online_manipulation.tamp_hierarchy import ConstraintResult, PredicateGoal


class FailureBoundaryTest(unittest.TestCase):
    def test_numeric_diagnostics_remain_only_in_low_level_failure(self):
        low = LowLevelFailure("PickLift", "pick_ik_grasp", ("cube", "drawer"),
                              {"q": [123.456], "particle_cost": 98.765})
        program = low.abstract()
        semantic = SemanticFailure(PredicateGoal("holding", ("cube",)), (program,))
        self.assertEqual(low.details["q"], [123.456])
        self.assertEqual(program.failed_constraints, ("ik",))
        self.assertNotIn("123.456", json.dumps(program.as_feedback()))
        self.assertNotIn("98.765", json.dumps(semantic.as_feedback()))
        self.assertNotIn("PickLift", json.dumps(semantic.as_feedback()))
        self.assertEqual(semantic.as_feedback()["blocking_failures"][0]["involved_objects"],
                         ["cube", "drawer"])

    def test_ik_endpoint_collision_is_not_reported_as_unreachable(self):
        for label in ("staging_pose_in_target", "grasp_pose_in_target", "lift_pose_world", "lift_waypoint"):
            key = "lift_waypoints_world" if label == "lift_waypoint" else label
            details = {key: {"reason": "endpoint_collision_or_clearance", "distance": -0.002}}
            low = LowLevelFailure("PickLift", "pick_ik_" + label, ("cube",), details)
            self.assertEqual(low.abstract().failed_constraints, ("collision",))
            constraint = ConstraintResult(False, low.reason, "g0", low.reason, ("cube",), details)
            feedback = ProgramFailure.from_constraints((constraint,), skill="PickLift").as_feedback()
            self.assertEqual(feedback["failed_constraints"], ["collision"])
            self.assertNotIn("-0.002", json.dumps(feedback))
        solver_failed = LowLevelFailure("PickLift", "pick_ik_staging_pose_in_target", (),
                                       {"staging_pose_in_target": {"reason": "solver_failed"}})
        self.assertEqual(solver_failed.abstract().failed_constraints, ("ik",))
        nav = LowLevelFailure("NavigateToPick", "navigation_pick_infeasible", (), {
            "pick_failures": [{"reason": low.reason, "details": details}]})
        self.assertEqual(nav.abstract().failed_constraints, ("collision",))

    def test_arbitrary_constraint_text_cannot_be_a_prompt_category(self):
        feedback = ProgramFailure.from_feedback({
            "skill": "PickLift", "failed_constraints": ["IK failed at q=123.456", "pick_ik_grasp"],
            "involved_objects": ["cube"], "details": {"cost": 98.765},
        }).as_feedback()
        self.assertEqual(feedback["failed_constraints"], ["other", "ik"])
        self.assertNotIn("123.456", json.dumps(feedback))
        self.assertNotIn("details", feedback)

    def test_budget_failure_keeps_search_exhaustion_explicit(self):
        constraint = ConstraintResult(False, "pick_joint_edge_approach", "g0", "q=123.456",
                                      ("cube",), {"distance": -0.01})
        failure = ProgramFailure.from_constraints((constraint,), skill="PickLift")
        self.assertTrue(failure.program_unsat)
        self.assertTrue(failure.search_budget_exhausted)
        self.assertEqual(failure.failed_constraints, ("collision",))


if __name__ == "__main__":
    unittest.main()
