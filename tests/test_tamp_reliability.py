"""Recovery, constrained search and external deadline contracts, not physics proof."""
import dataclasses
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError

import numpy as np
from planner.src.bt.generation import OpenAICompatibleChatClient, ProviderError
from planner.src.tamp.cutamp import order_postcheck_candidates, postcheck_cutamp_candidates
from planner.src.tamp.geometry import SamplingSolver
from planner.src.tamp.hierarchy import PredicateGoal, SkillProgram, SkillStep, picklift_registry
from planner.src.tamp.online import IncrementalTampRunner, RecoveryLimits
from planner.src.tamp.provenance import record_validation_commit
from planner.src.tamp.semantic import SemanticModelError, SemanticSubgoalPlanner, ModelSettings
from planner.src.tamp.subdomains import (validate_subdomain, validate_program_subdomains,
    restrict_base_candidates, candidate_in_subdomain, restrict_sample, grasp_range)
from planner.src.tamp.supervision import supervise_simulation


class ReliabilityTests(unittest.TestCase):
    def test_provider_status_classification(self):
        client = OpenAICompatibleChatClient(base_url='https://example.invalid/v1', api_key='private')
        for code, retry in ((401,False),(403,False),(429,True),(503,True)):
            with self.subTest(code=code), patch('planner.src.bt.generation.urlopen',
                    side_effect=HTTPError('secret-url',code,'private-body',{},None)):
                with self.assertRaises(ProviderError) as caught:
                    client.complete(model='model',messages=[{'role':'user','content':'x'}])
                self.assertEqual(caught.exception.retryable,retry)
                self.assertNotIn('private',str(caught.exception))
        with patch('planner.src.bt.generation.urlopen', side_effect=URLError(TimeoutError())):
            with self.assertRaises(ProviderError) as caught:
                client.complete(model='model',messages=[{'role':'user','content':'x'}])
            self.assertEqual(caught.exception.kind,'timeout')

    def test_stage_recovery_is_separate_and_bounded(self):
        from test_tamp_online import Domain, Executor, Observer
        holding=PredicateGoal('holding',('red_cube',))
        initial=(PredicateGoal('observed',('red_cube',)),PredicateGoal('gripper_empty',()))
        for failures, retryable, expected in ((2,True,'task_goals_satisfied'),(3,True,'provider_retry_exhausted'),(1,False,'provider_configuration_error')):
            class Semantic:
                calls=0
                def propose(self, **kwargs):
                    self.calls+=1
                    if self.calls<=failures:
                        raise SemanticModelError('fixture',provider_failure={'kind':'timeout','retryable':retryable})
                    return (holding,)
            events=[]; semantic=Semantic(); executor=Executor(initial)
            runner=IncrementalTampRunner(semantic=semantic,registry=picklift_registry(),
                solver_factory=lambda world:SamplingSolver(picklift_registry(),Domain()),
                executor=executor,observer=Observer(('red_cube',)),trace=events.append,
                limits=RecoveryLimits(provider_retry_delay_s=0))
            result=runner.run(task='pick',task_goals=(holding,),initial_observation=initial,
                initial_geometry_state={},predicate_arity={'holding':1})
            if failures==2:
                self.assertTrue(result.success)
                self.assertEqual(result.metrics['num_skill_replans'],0)
                self.assertEqual(result.metrics['num_geometry_retries'],0)
            else:
                self.assertEqual(result.reason,expected)
                self.assertEqual(executor.calls,[])
            self.assertEqual(result.metrics['provider_stage_retries'],min(failures,2) if retryable else 0)

    def test_satisfied_semantic_goals_are_removed_and_raw_retained(self):
        response=SimpleNamespace(content=json.dumps({'schema':'scenesmith.tamp.goals.v3','subgoals':[
            {'predicate':'gripper_empty','arguments':[]}, {'predicate':'holding','arguments':['red_cube']}]}))
        client=SimpleNamespace(complete=lambda **kwargs:response)
        events=[]
        planner=SemanticSubgoalPlanner(client,ModelSettings(max_attempts=1),trace=events.append)
        goals=planner.propose(task='pick',world={'objects':{'red_cube':{}},'facts':[{'predicate':'gripper_empty','arguments':[]}]},predicate_arity={'holding':1,'gripper_empty':0})
        self.assertEqual(goals,(PredicateGoal('holding',('red_cube',)),))
        self.assertTrue(any(e['event']=='semantic_goals_filtered' for e in events))

    def test_quantiles_visit_each_cost_band_before_second_rank(self):
        candidates=[{'particle_index':i,'hard_constraint_cost':i,'grasp_lateral_offset_m':0.,'arm_joint_positions':[0.]} for i in range(64)]
        ordered=order_postcheck_candidates(candidates,'cost_quantiles')
        self.assertEqual([c['particle_index'] for c in ordered[:8]],[0,16,32,48,1,17,33,49])
        self.assertEqual(len({c['particle_index'] for c in ordered}),64)

    def test_shared_witness_runs_full_pick_once_and_never_accepts_bad_pick(self):
        class Domain:
            candidates=((-1,-1,0),(-1,1,0),(1,-1,0),(1,1,0))
            config=SimpleNamespace(robot_adapter=SimpleNamespace(spec=SimpleNamespace(arm_groups={'left':['arm']})))
            def __init__(self, valid): self.valid=valid; self.calls=[]
            def check_navigation_corridor(self,*args):
                self.calls.append('corridor'); return True,'valid',{'navigation_goal':[0,0,0,'world'],'base_xyz_yaw':[0,0,.2,0]}
            def check(self,skill,candidate,state):
                self.calls.append(skill.skill)
                return self.valid,'valid' if self.valid else 'pick_ik_grasp',{'ik':{'grasp_pose_in_target':{'arm_joint_positions':[.1]}}}
            def predict(self,skill,candidate,state): return dict(state)
            def rank_candidate(self,*args): return (0.,)
        world=SimpleNamespace(objects={'target':{}},robot={'base_link_pose':{'translation_m':[0,0,.2],'quaternion_wxyz':[1,0,0,0]}})
        program=SkillProgram((SkillStep('NavigateToPick',{'object':'target'},{'base_pose':'b'}),SkillStep('PickLift',{'object':'target'},{'grasp_pose':'g','approach_pose':'a'})))
        candidate={'particle_index':0,'optimizer_feasible':True,'hard_constraint_cost':0.,'base_world_pose':[0,0,.2,1,0,0,0],'arm_joint_names':['arm'],'arm_joint_positions':[.1],'grasp_lateral_offset_m':0.}
        for valid in (True,False):
            domain=Domain(valid)
            with tempfile.TemporaryDirectory() as out:
                plan, checks, failures=postcheck_cutamp_candidates(picklift_registry(),domain,world,program,[candidate],{},out)
            self.assertEqual(domain.calls,['corridor','PickLift'])
            self.assertEqual(plan is not None,valid)
            if valid:
                self.assertEqual(plan.actions[0].geometric_parameters['checks']['navigation_witness_source'],'same_particle_complete_pick_check')
            else:
                self.assertEqual(failures[0].details['program_step'],1)

    def test_subdomain_stays_in_rotated_hull_and_rejects_expansion(self):
        from scipy.spatial import ConvexHull
        from test_tamp_proc3s import PRoC3SProgramTest
        fixture=PRoC3SProgramTest();fixture.setUp()
        fixture.document['domains'][0]['subdomain']={'forward':[.2,.8],'lateral':[.1,.7]}
        fixture.document['domains'][1]['subdomain']={'lateral':[.25,.75]}
        program=fixture.parse();validate_program_subdomains(program,picklift_registry())
        yaw=.7; rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
        xy=np.array([[0,0],[2,0],[0,2]])@rotation.T
        candidates=tuple((*p,yaw) for p in xy)
        clipped=restrict_base_candidates(candidates,program.parameter_subdomains['b0'])
        half=ConvexHull(xy).equations
        self.assertLessEqual(float(np.max(np.asarray(clipped)[:,:2]@half[:,:2].T+half[:,2])),1e-14)
        domain=SimpleNamespace(candidates=candidates,program_schema={'PickLift':{'grasp_lateral_offset_m_range':[-.02,.02]}})
        self.assertTrue(np.allclose(grasp_range(domain,program.parameter_subdomains['g0']),[-.01,.01]))
        step=program.steps[1]
        inside=restrict_sample(domain,program,step,{'grasp_lateral_offset_m':.02})
        self.assertTrue(candidate_in_subdomain(domain,program,step,inside))
        self.assertFalse(candidate_in_subdomain(domain,program,step,{'grasp_lateral_offset_m':.02}))
        for invalid in ({'lateral':[-.1,1]},{'height':[0,1]},{'lateral':[True,1]},{'lateral':[.5,.5]}):
            with self.assertRaises(ValueError):validate_subdomain('calibrated_grasp_pose',invalid)
        with self.assertRaises(ValueError):
            validate_program_subdomains(dataclasses.replace(program,parameter_subdomains={'g0':{'lateral':[-1,1]}}),picklift_registry())

    def test_external_deadline_terminates_worker_and_writes_result(self):
        with tempfile.TemporaryDirectory() as root:
            out=Path(root)/'run'
            started=time.perf_counter()
            code=supervise_simulation([sys.executable,'-c','import time; time.sleep(30)'],output=out,started_at=started,max_wall_time_s=.15)
            result=json.loads((out/'result.json').read_text())
            self.assertEqual(code,2)
            self.assertEqual(result['reason'],'wall_time_budget_exhausted')
            self.assertLess(time.perf_counter()-started,3)

    def test_snapshot_commit_preserves_dirty_index_head_and_ignored_tests(self):
        with tempfile.TemporaryDirectory() as root:
            repo=Path(root);(repo/'tests').mkdir();(repo/'tests/test_x.py').write_text('x=1\n')
            (repo/'experiments/cutamp').mkdir(parents=True)
            (repo/'experiments/cutamp/README.md').write_text('worker evidence\n')
            (repo/'experiments/cutamp/upstream-local.patch').write_text('upstream delta\n')
            (repo/'.gitignore').write_text('tests/\n')
            def git(*args):return subprocess.check_output(['git','-C',root,*args])
            git('init','-q');git('config','user.name','Validation Test');git('config','user.email','test@example.invalid')
            git('add','.gitignore');git('commit','-qm','base')
            (repo/'README.md').write_text('staged\n');git('add','README.md');(repo/'README.md').write_text('unstaged\n')
            head=git('rev-parse','HEAD');index=(repo/'.git/index').read_bytes()
            evidence=record_validation_commit(repo,repo/'evidence',ref='refs/validation/test')
            self.assertEqual(git('rev-parse','HEAD'),head);self.assertEqual((repo/'.git/index').read_bytes(),index)
            self.assertEqual(git('show',evidence['validation_commit']+':README.md'),b'unstaged\n')
            self.assertEqual(git('show',evidence['validation_commit']+':tests/test_x.py'),b'x=1\n')
            self.assertEqual(git('show',evidence['validation_commit']+':experiments/cutamp/README.md'),
                             b'worker evidence\n')
            self.assertEqual(git('show',evidence['validation_commit']+':experiments/cutamp/upstream-local.patch'),
                             b'upstream delta\n')

    def test_unsupported_backend_does_not_ignore_subdomain(self):
        from test_tamp_online import Domain
        from test_tamp_proc3s import PRoC3SProgramTest
        from planner.src.tamp.geometry import GeometricUnsat
        fixture=PRoC3SProgramTest();fixture.setUp()
        fixture.document['domains'][1]['subdomain']={'lateral':[.3,.7]}
        program=fixture.parse()
        with self.assertRaises(GeometricUnsat) as caught:
            SamplingSolver(fixture.registry,Domain()).solve(fixture.world,program,{})
        self.assertFalse(caught.exception.retryable_search)
        self.assertEqual(caught.exception.constraints[0].reason,'unsupported_subdomain')

    def test_declared_grasp_subdomain_reaches_ccsp(self):
        from test_tamp_proc3s import PRoC3SProgramTest
        from planner.src.tamp.ccsp import Proc3sCCSPSolver
        fixture=PRoC3SProgramTest();fixture.setUp()
        fixture.document['domains'][1]['subdomain']={'lateral':[.3,.7]}
        program=fixture.parse();seen=[]
        class Domain:
            program_schema={'PickLift':{'grasp_lateral_offset_m_range':[-.02,.02]}}
            def sample_candidate(self,skill,state,rng):
                return {'base_x_m':0.,'base_y_m':0.,'base_yaw_rad':0.} if skill.skill=='NavigateToPick' else {'grasp_lateral_offset_m':.02,'approach_height_offset_m':0.}
            def parameter_control_keys(self,role):
                return {'base_pose':['base_x_m','base_y_m','base_yaw_rad'],'grasp_pose':['grasp_lateral_offset_m'],'approach_pose':['approach_height_offset_m']}[role]
            def predict(self,skill,candidate,state):return dict(state)
            def check(self,skill,candidate,state):
                seen.append(candidate);return True,'valid',{}
        Proc3sCCSPSolver(fixture.registry,Domain(),max_samples=1).solve(fixture.world,program,{})
        self.assertAlmostEqual(seen[-1]['grasp_lateral_offset_m'],.008)

    def test_provider_retry_pool_is_shared_between_semantic_and_program(self):
        from test_tamp_online import Domain, Executor, Observer
        from planner.src.tamp.proc3s import PRoC3SGenerationFailure
        from planner.src.tamp.skill_planning import StripsProgramGenerator
        holding=PredicateGoal('holding',('red_cube',));initial=(PredicateGoal('observed',('red_cube',)),PredicateGoal('gripper_empty',()))
        class Semantic:
            calls=0
            def propose(self,**kwargs):
                self.calls+=1
                if self.calls==1:raise SemanticModelError('fixture',provider_failure={'kind':'connection','retryable':True})
                return (holding,)
        class Generator(StripsProgramGenerator):
            calls=0
            def generate(self,*args,**kwargs):
                self.calls+=1
                raise PRoC3SGenerationFailure('fixture',provider_failure={'kind':'timeout','retryable':True})
        executor=Executor(initial);generator=Generator(picklift_registry())
        runner=IncrementalTampRunner(semantic=Semantic(),registry=picklift_registry(),solver_factory=lambda w:SamplingSolver(picklift_registry(),Domain()),executor=executor,observer=Observer(('red_cube',)),trace=lambda e:None,limits=RecoveryLimits(provider_retry_delay_s=0),program_generator=generator)
        result=runner.run(task='pick',task_goals=(holding,),initial_observation=initial,initial_geometry_state={},predicate_arity={'holding':1})
        self.assertEqual(result.reason,'provider_retry_exhausted');self.assertEqual(result.metrics['provider_stage_retries'],2)
        self.assertEqual(generator.calls,2);self.assertEqual(executor.calls,[])
