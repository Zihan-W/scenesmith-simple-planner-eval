"""Local budgets must preserve safety, evidence and global recovery time."""
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from planner.src.tamp.cutamp import CuTAMPSettings, CuTAMPSolver, postcheck_cutamp_candidates
from planner.src.tamp.geometry import GeometricUnsat, GeometryDeadlineExceeded
from planner.src.tamp.hierarchy import SkillProgram, SkillStep, picklift_registry
from planner.src.tamp.model_budget import model_stage_budget
from planner.src.tamp.scenesmith import SceneSmithPickDomain
from planner.src.tamp.semantic import ModelSettings
from simulation.src.geometry.planning import _canonical_ik_seed


class BudgetTests(unittest.TestCase):
    def test_ik_roundoff_canonicalization_never_clamps_real_violations(self):
        specs = [SimpleNamespace(position_lower=-0.03814, position_upper=0.)] * 3
        measured = np.array([1.0873995615933327e-11, 2e-8, -0.02])
        original = measured.copy()
        solver_input = _canonical_ik_seed(measured, specs)
        np.testing.assert_array_equal(measured, original)
        np.testing.assert_array_equal(solver_input, [0., 2e-8, -0.02])

    def test_model_stage_limits_and_restores_shared_client(self):
        class Client:
            timeout_s = 180.
            deadline_monotonic_s = time.perf_counter() + 300
            def set_deadline(self, value):
                self.deadline_monotonic_s = value
        client = Client(); parent = client.deadline_monotonic_s
        with self.assertRaisesRegex(ValueError, 'fixture'):
            with model_stage_budget(client, ModelSettings()) as deadline:
                self.assertLessEqual(deadline-time.perf_counter(), 90.)
                self.assertEqual(client.timeout_s, 30.)
                self.assertEqual(client.deadline_monotonic_s, deadline)
                raise ValueError('fixture')
        self.assertEqual(client.deadline_monotonic_s, parent)
        self.assertEqual(client.timeout_s, 180.)

    def test_nested_check_deadline_is_not_geometry_rejection(self):
        domain = object.__new__(SceneSmithPickDomain)
        domain.set_deadline(time.perf_counter() - 1)
        with self.assertRaises(GeometryDeadlineExceeded):
            domain.check(SimpleNamespace(arguments={'target':'target'}), {}, {})

    def test_local_solve_timeout_is_recoverable_but_global_timeout_is_not(self):
        with tempfile.TemporaryDirectory() as tmp:
            settings = CuTAMPSettings(**{key: tmp for key in (
                'gpu_python', 'cutamp_root', 'kinematics_template', 'robot_template',
                'multipliers', 'tolerances', 'grasp_calibration')})
            solver = CuTAMPSolver(picklift_registry(), SimpleNamespace(target_name='target'),
                settings=settings, resolved_config={}, repository_root=tmp, output_root=Path(tmp)/'solve')
            with patch.object(solver, '_solve', side_effect=GeometryDeadlineExceeded):
                solver.set_deadline(time.perf_counter()+100)
                with self.assertRaises(GeometricUnsat) as caught:
                    solver.solve(None, None, {})
                self.assertEqual(caught.exception.program_feedback['budget_scope'], 'geometry_wall_time')
                self.assertFalse(caught.exception.program_feedback['program_unsat'])
                solver.set_deadline(time.perf_counter()-1)
                with self.assertRaises(GeometryDeadlineExceeded):
                    solver.solve(None, None, {})

    def test_postcheck_timeout_retains_prior_full_certificate(self):
        class Domain:
            config = SimpleNamespace(robot_adapter=SimpleNamespace(
                spec=SimpleNamespace(arm_groups={'left':['arm']})))
            def check(self, skill, params, state):
                if params['grasp_arm_joint_positions'][0] > 1:
                    raise GeometryDeadlineExceeded()
                return True, 'valid', {'ik':{'grasp_pose_in_target':{'arm_joint_positions':[1.]}}}
            def rank_candidate(self,*args): return (0, -0.1, 0)
            def predict(self,*args): return {}
        world=SimpleNamespace(objects={'target':{}}, robot={'base_link_pose':{
            'translation_m':[0,0,.2], 'quaternion_wxyz':[1,0,0,0]}})
        program=SkillProgram((SkillStep('PickLift',{'object':'target'},
                                       {'grasp_pose':'g','approach_pose':'a'}),))
        cs=[{'particle_index':i, 'optimizer_feasible':True, 'hard_constraint_cost':i,
             'base_world_pose':[0,0,.2,1,0,0,0], 'arm_joint_names':['arm'],
             'arm_joint_positions':[float(i+1)], 'grasp_lateral_offset_m':0.} for i in range(3)]
        with tempfile.TemporaryDirectory() as tmp:
            plan,checks,failures=postcheck_cutamp_candidates(picklift_registry(),Domain(),world,program,cs,{},tmp)
            stop=json.loads((Path(tmp)/'candidate_selection.json').read_text())
        self.assertIsNotNone(plan)
        self.assertEqual(len(checks),2)
        self.assertFalse(checks[1]['completed'])
        self.assertFalse(checks[1]['approximate_exact_disagreement'])
        self.assertEqual(failures,())
        self.assertEqual(stop['stop_reason'],'postcheck_wall_time')
        self.assertEqual(stop['candidates_completed'],1)

    def test_outside_domain_rejected_before_expensive_check(self):
        class Domain:
            candidates=((-1,-1,0),(-1,1,0),(1,-1,0),(1,1,0))
            config=SimpleNamespace(robot_adapter=SimpleNamespace(spec=SimpleNamespace(arm_groups={'left':['arm']})))
            def check(self,*args): raise AssertionError('must not spend IK on outside-domain candidate')
        world=SimpleNamespace(objects={'target':{}},robot={'base_link_pose':{
            'translation_m':[0,0,.2],'quaternion_wxyz':[1,0,0,0]}})
        program=SkillProgram((SkillStep('NavigateToPick',{'object':'target'},{'base_pose':'b'}),))
        cs=[{'particle_index':0,'optimizer_feasible':False,'hard_constraint_cost':1.,
             'base_world_pose':[2,0,.2,1,0,0,0], 'arm_joint_names':['arm'],
             'arm_joint_positions':[0.], 'grasp_lateral_offset_m':0.}]
        with tempfile.TemporaryDirectory() as tmp:
            plan,checks,failures=postcheck_cutamp_candidates(picklift_registry(),Domain(),world,program,cs,{},tmp)
        self.assertIsNone(plan)
        self.assertEqual(failures[0].constraint,'reachability')
        self.assertEqual(checks[0]['details']['check_stage'],'base_domain_halfspaces')
