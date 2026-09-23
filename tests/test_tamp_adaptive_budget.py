"""Admission, selection and evidence contracts for the optional budget policy."""
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from planner.src.tamp.postcheck_budget import AdaptivePostcheckBudget
from planner.src.tamp.cutamp import postcheck_cutamp_candidates
from planner.src.tamp.hierarchy import SkillProgram, SkillStep, picklift_registry

SETTINGS = {'execution_reserve_s': 20., 'first_pass_remaining_s': 10., 'native_call_guard_s': 2.}


class AdaptiveBudgetTests(unittest.TestCase):
    def test_transition_uses_tighter_deadline_and_never_switches_back(self):
        budget = AdaptivePostcheckBudget(SETTINGS, 100., 70.)
        self.assertIsNone(budget.before_candidate(50., 0, False))
        self.assertEqual(budget.mode, 'quality_window')
        self.assertIsNone(budget.before_candidate(58., 1, False))
        self.assertEqual(budget.mode, 'first_pass')
        self.assertEqual(budget.before_candidate(59., 2, True), 'adaptive_first_pass_found')
        self.assertEqual(budget.transition['after_checks'], 1)
        budget.after_candidate(73.)
        self.assertEqual(budget.evidence(73.)['soft_deadline_overrun_s'], 3.)

    def test_reserve_and_guard_stop_admission_without_feasible_candidate(self):
        budget = AdaptivePostcheckBudget(SETTINGS, 100., 200.)
        self.assertEqual(budget.deadline, 80.)
        self.assertEqual(budget.before_candidate(78., 3, False), 'adaptive_native_guard')
        self.assertFalse(budget.transition['had_feasible_candidate'])

    def test_explicit_finite_contract_required(self):
        for config in ({}, {**SETTINGS, 'extra': 1}, {**SETTINGS, 'native_call_guard_s': True},
                       {**SETTINGS, 'execution_reserve_s': float('nan')}):
            with self.assertRaises(ValueError): AdaptivePostcheckBudget.validate(config)
        with self.assertRaises(ValueError): AdaptivePostcheckBudget(SETTINGS, None, 100.)

    def test_execution_path_stops_after_first_complete_pass(self):
        class Domain:
            config = SimpleNamespace(robot_adapter=SimpleNamespace(spec=SimpleNamespace(arm_groups={'left':['arm']})))
            def check(self, skill, params, state):
                q = params['grasp_arm_joint_positions']
                return q[0] > 0, 'valid' if q[0] > 0 else 'collision', {'ik':{'grasp_pose_in_target':{'arm_joint_positions':q}}}
            def rank_candidate(self, *args): return (0,)
            def predict(self, *args): return {}
        world = SimpleNamespace(objects={'target':{}}, robot={'base_link_pose':{
            'translation_m':[0,0,.2], 'quaternion_wxyz':[1,0,0,0]}})
        program = SkillProgram((SkillStep('PickLift', {'object':'target'}, {'grasp_pose':'g','approach_pose':'a'}),))
        candidates = [{'particle_index':i, 'optimizer_feasible':True,'hard_constraint_cost':i,
            'base_world_pose':[0,0,.2,1,0,0,0], 'arm_joint_names':['arm'],
            'arm_joint_positions':[float(i)], 'grasp_lateral_offset_m':0.} for i in range(3)]
        with tempfile.TemporaryDirectory() as folder:
            plan, checks, failures = postcheck_cutamp_candidates(picklift_registry(),Domain(),world,program,
                candidates,{},folder,postcheck_order='cost',adaptive_budget=SETTINGS,
                run_deadline_monotonic_s=time.perf_counter()+25.)
            evidence=json.loads((Path(folder)/'candidate_selection.json').read_text())
        self.assertEqual(len(checks),2)
        self.assertTrue(checks[-1]['completed'])
        self.assertEqual(evidence['selected_particle'],1)
        self.assertEqual(evidence['stop_reason'],'adaptive_first_pass_found')
        self.assertEqual(plan.actions[0].geometric_parameters['grasp_arm_joint_positions'],[1.])
        self.assertEqual(len(failures),1)

    def test_guard_without_candidates_has_budget_feedback(self):
        from planner.src.tamp.cutamp_feedback import postcheck_budget_feedback
        for reason in ('postcheck_wall_time', 'adaptive_native_guard'):
            feedback = postcheck_budget_feedback(reason, 'target')
            self.assertTrue(feedback['search_budget_exhausted'])
            self.assertFalse(feedback['program_unsat'])
            self.assertEqual(feedback['budget_scope'], 'exact_postcheck_wall_time')
        self.assertIsNone(postcheck_budget_feedback('adaptive_first_pass_found', 'target'))
