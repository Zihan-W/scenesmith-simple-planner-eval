"""Full assignment CCSP contracts using a small analytic skill domain."""

import random
import unittest

from planner.src.tamp.ccsp import PRoC3SProgramUnsat, Proc3sCCSPSolver
from planner.src.tamp.hierarchy import PredicateGoal, WorldState, picklift_registry
from planner.src.tamp.skill_planning import StripsProgramGenerator


class AnalyticDomain:
    """A grasp succeeds iff navigation reached its sampled reachable base."""

    def __init__(self, force_unsat=False):
        self.sampled = 0
        self.checked = 0
        self.force_unsat = force_unsat

    def sample_candidate(self, skill, state, rng):
        self.sampled += 1
        if skill.skill == "NavigateToPick":
            return {"base_x_m": 1.0, "base_y_m": 0.0, "base_yaw_rad": 0.0}
        return {"grasp_lateral_offset_m": rng.uniform(-0.02, 0.02),
                "approach_height_offset_m": 0.0, "approach_segments": 8}

    def parameter_control_keys(self, parameter):
        return {"base_pose": ("base_x_m", "base_y_m", "base_yaw_rad"),
                "grasp_pose": ("grasp_lateral_offset_m",),
                "approach_pose": ("approach_height_offset_m", "approach_segments")}[parameter]

    def predict(self, skill, candidate, state):
        if skill.skill == "NavigateToPick":
            state["base_x"] = candidate["base_x_m"]
        return state

    def check(self, skill, candidate, state):
        self.checked += 1
        if self.sampled % 2:
            raise AssertionError("Checked before sampling the whole program")
        if skill.skill == "NavigateToPick":
            if state.get("base_x") != 0.0:
                raise AssertionError("Validation was not reset to initial state")
            return True, "valid", {}
        feasible = state.get("base_x") == 1.0 and not self.force_unsat
        return feasible, "valid" if feasible else "ik", {
            "ik": {"grasp_pose_in_target": {"x": candidate["grasp_lateral_offset_m"]},
                   "staging_pose_in_target": {"z": 0.1}}}


class CCSPTest(unittest.TestCase):
    def setUp(self):
        self.registry = picklift_registry()
        self.world = WorldState({"cube": {}}, frozenset({
            PredicateGoal("observed", ("cube",)), PredicateGoal("gripper_empty", ())}))
        self.program = StripsProgramGenerator(self.registry).generate(
            self.world, (PredicateGoal("holding", ("cube",)),))

    def test_full_assignment_precedes_checks_and_initial_state_is_preserved(self):
        domain = AnalyticDomain()
        state = {"base_x": 0.0}
        result = Proc3sCCSPSolver(self.registry, domain).solve(self.world, self.program, state)
        self.assertEqual(len(result.actions), 2)
        self.assertEqual(domain.sampled, 2)
        self.assertEqual(domain.checked, 2)
        self.assertEqual(state, {"base_x": 0.0})
        self.assertEqual(len(result.assignments), 3)

    def test_unsat_names_failed_skill_and_does_not_leak_numeric_details(self):
        solver = Proc3sCCSPSolver(self.registry, AnalyticDomain(True), max_samples=3)
        with self.assertRaises(PRoC3SProgramUnsat) as caught:
            solver.solve(self.world, self.program, {"base_x": 0.0})
        error = caught.exception
        self.assertEqual(error.samples, 3)
        self.assertEqual(error.program_feedback["skill"], "PickLift")
        self.assertEqual(error.program_feedback["failed_constraints"], ["ik"])
        self.assertTrue(error.program_feedback["search_budget_exhausted"])
        self.assertNotIn("details", error.program_feedback)
        self.assertEqual(len(error.constraints), 3)

    def test_resolving_keeps_a_reproducible_advancing_random_stream(self):
        def solve_twice():
            rng = random.Random(500)
            return [Proc3sCCSPSolver(self.registry, AnalyticDomain(), rng=rng).solve(
                self.world, self.program, {"base_x": 0.0}).assignments
                    for _ in range(2)]

        first, second = solve_twice()
        self.assertNotEqual(first, second)
        self.assertEqual([first, second], solve_twice())


if __name__ == "__main__":
    unittest.main()
