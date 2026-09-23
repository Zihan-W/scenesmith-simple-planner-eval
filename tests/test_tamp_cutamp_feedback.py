"""Distinguish optimizer evidence, finite budgets, and exact step failures."""
import unittest
from types import SimpleNamespace

from planner.src.tamp.cutamp_feedback import optimizer_failure_feedback, exact_failure_feedback
from planner.src.tamp.failures import ProgramFailure
from planner.src.tamp.hierarchy import ConstraintResult, SkillProgram, SkillStep


class CuTAMPFailureSemanticsTests(unittest.TestCase):
    def test_approximate_failure_has_no_invented_first_step_or_unsat_proof(self):
        candidates = [{'constraint_diagnostics': {
            'Collision': {'robot_to_world': {'satisfied': False}},
            'KinematicConstraint': {'pos_err': {'satisfied': False}},
            'Motion': {'joint_limit': {'satisfied': True}}}}]
        feedback = optimizer_failure_feedback(candidates,
            {'timed_out': False, 'num_opt_steps': 7}, max_steps=20, target='target')
        self.assertEqual(feedback['skill'], '')
        self.assertIsNone(feedback['program_step'])
        self.assertEqual(feedback['attribution_scope'], 'global')
        self.assertEqual(feedback['failure_source'], 'approximate_tolerance_unmet')
        self.assertEqual(set(feedback['failed_constraints']), {'ik', 'collision'})
        self.assertFalse(feedback['program_unsat'])
        self.assertFalse(feedback['search_budget_exhausted'])
        self.assertEqual(feedback['budget_scope'], 'none')
        self.assertEqual(ProgramFailure.from_feedback(feedback).as_feedback(), feedback)

    def test_iteration_budget_is_observed_not_assumed(self):
        feedback = optimizer_failure_feedback([], {'timed_out': False, 'num_opt_steps': 20},
                                               max_steps=20, target='target')
        self.assertTrue(feedback['search_budget_exhausted'])
        self.assertEqual(feedback['budget_scope'], 'optimizer_steps')
        self.assertFalse(feedback['program_unsat'])

    def test_exact_step_attribution_requires_consistent_indices(self):
        program = SkillProgram((SkillStep('NavigateToPick', {'object':'target'}, {'base_pose':'b'}),
                                SkillStep('PickLift', {'object':'target'}, {'grasp_pose':'g','approach_pose':'a'})))
        failure = ConstraintResult(False, 'pick_ik_grasp', 'g', 'rejected', ('target',), {'program_step':1})
        kwargs = dict(checks=[{}], candidate_count=5, max_postchecks=1, target='target')
        feedback = exact_failure_feedback(program, [failure], **kwargs)
        self.assertEqual((feedback['skill'], feedback['program_step']), ('PickLift',1))
        self.assertEqual(feedback['attribution_scope'], 'step')
        self.assertEqual(feedback['failure_source'], 'exact_postcheck_rejected')
        self.assertTrue(feedback['search_budget_exhausted'])
        self.assertEqual(feedback['budget_scope'], 'exact_postchecks')
        self.assertFalse(feedback['program_unsat'])
        # A quality-window stop can coincide with the count limit. Budget
        # feedback must use independent count evidence, not stop precedence.
        feedback = exact_failure_feedback(
            program, [failure], **kwargs, stop_reason='quality_window_complete',
            count_exhausted=True)
        self.assertTrue(feedback['search_budget_exhausted'])
        self.assertEqual(feedback['budget_scope'], 'exact_postchecks')
        feedback = exact_failure_feedback(
            program, [failure], checks=[{}], candidate_count=5,
            max_postchecks=2, target='target', stop_reason='postcheck_count')
        self.assertFalse(feedback['search_budget_exhausted'])
        self.assertEqual(feedback['budget_scope'], 'none')
        global_failure = ConstraintResult(False,'collision','','rejected',('target',),{})
        feedback = exact_failure_feedback(program, [failure,global_failure], **kwargs)
        self.assertEqual(feedback['skill'],'')
        self.assertEqual(feedback['attribution_scope'],'global')
        self.assertIsNone(feedback['program_step'])

    def test_legacy_geometric_tag_does_not_imply_unsatisfiable(self):
        feedback = ProgramFailure.from_feedback({'type':'GEOMETRIC_INFEASIBLE','skill':'PickLift'})
        self.assertFalse(feedback.program_unsat)


if __name__ == '__main__':
    unittest.main()
