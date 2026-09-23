"""Regression tests for the three-level failure information boundary."""

import json
import unittest

from planner.src.tamp.failures import (
    LowLevelFailure, ProgramFailure, SemanticFailure,
)
from planner.src.tamp.hierarchy import ConstraintResult, PredicateGoal


class FailureBoundaryTest(unittest.TestCase):
    def test_runtime_joint_limit_rejection_preserves_actual_cause(self):
        edge = {"joint_limits_valid": False, "nonpenetration_valid": True,
                "safety_clearance_valid": True, "joint_configuration": [123.456]}
        for details in ({"edge": edge}, {"last_action_rejection": {"edge": edge}}):
            failure = LowLevelFailure("PickLift", "skill_runtime_failed:joint_edge_rejected",
                                      ("cube",), details).abstract()
            self.assertEqual(failure.failed_constraints, ("joint_limits",))
            self.assertNotIn("123.456", json.dumps(failure.as_feedback()))
        failure = LowLevelFailure("PickLift", "joint_edge_rejected", (), {
            "edge": {"joint_limits_valid": True, "nonpenetration_valid": False}})
        self.assertEqual(failure.abstract().failed_constraints, ("collision",))

    def test_planning_edge_joint_limits_are_not_reported_as_collision(self):
        for label in ("staging_pose_in_target", "grasp_pose_in_target", "lift_waypoint"):
            key = "lift_waypoints_world" if label == "lift_waypoint" else label
            details = {key: {"joint_edge": {"joint_limits_valid": False}}}
            low = LowLevelFailure("PickLift", "pick_joint_edge_" + label, (), details)
            self.assertEqual(low.abstract().failed_constraints, ("joint_limits",))

    def test_physical_skill_failures_use_known_program_categories(self):
        categories = {"planned_hold_timeout": "verification",
                      "planned_lost_contact": "grasp_validity",
                      "planned_bilateral_contact_timeout": "grasp_validity",
                      "planned_staging_timeout": "execution",
                      "planned_grasp_timeout": "execution",
                      "planned_lift_waypoint_timeout": "execution"}
        for reason, category in categories.items():
            for prefix in ("", "skill_runtime_failed:"):
                low = LowLevelFailure("PickLift", prefix + reason)
                self.assertEqual(low.abstract().failed_constraints, (category,))

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
        failure = ProgramFailure.from_constraints((constraint,), skill="PickLift",
            search_budget_exhausted=True, failure_source="sampling_budget_exhausted")
        self.assertFalse(failure.program_unsat)
        self.assertTrue(failure.search_budget_exhausted)
        self.assertEqual(failure.failed_constraints, ("collision",))


if __name__ == "__main__":
    unittest.main()
