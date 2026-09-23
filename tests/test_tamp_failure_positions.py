"""Checked-sample locations reach program repair without region-UNSAT claims."""
import dataclasses
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from planner.src.bt.generation import ChatCompletion
from planner.src.tamp.cutamp import postcheck_cutamp_candidates
from planner.src.tamp.cutamp_feedback import exact_failure_feedback
from planner.src.tamp.failure_positions import normalized_failure_positions, validate_positions
from planner.src.tamp.failures import ProgramFailure
from planner.src.tamp.hierarchy import PredicateGoal, SkillProgram, SkillStep, WorldState, picklift_registry
from planner.src.tamp.proc3s import PRoC3SProgramGenerator
from planner.src.tamp.scenesmith import SceneSmithPickDomain
from planner.src.tamp.semantic import ModelSettings
from planner.src.tamp.snapshot import world_token


class PositionFeedbackTests(unittest.TestCase):
    def test_actual_postcheck_failure_reaches_model_repair(self):
        class Domain:
            config=SimpleNamespace(robot_adapter=SimpleNamespace(spec=SimpleNamespace(arm_groups={'left':['arm']})))
            program_schema={'PickLift': {'grasp_lateral_offset_m_range':[-.02,.02]}}
            failure_positions=SceneSmithPickDomain.failure_positions
            def check(self,*args): return False,'pick_joint_edge_lift_waypoint',{}
        actual_world=WorldState({'target':{'translation_m':[0,0,0], 'quaternion_wxyz':[1,0,0,0]}},
            frozenset({PredicateGoal('observed',('target',)), PredicateGoal('at_pick_pose',('target',)),
                       PredicateGoal('gripper_empty',())}), observation_id='0.000000',
            robot={'base_link_pose':{'translation_m':[0,0,.2], 'quaternion_wxyz':[1,0,0,0]},
                   'joint_names':['arm'], 'q':[.1]})
        Domain.snapshot=SimpleNamespace(token=world_token(actual_world))
        registry=picklift_registry()
        program=SkillProgram((SkillStep('PickLift',{'object':'target'},{'grasp_pose':'g','approach_pose':'a'}),))
        world=SimpleNamespace(objects={'target':{}},robot={'base_link_pose':{
            'translation_m':[0,0,.2],'quaternion_wxyz':[1,0,0,0]}})
        candidates=[dict(particle_index=i,optimizer_feasible=True,hard_constraint_cost=i,
            base_world_pose=[0,0,.2,1,0,0,0],arm_joint_names=['arm'],arm_joint_positions=[.1],
            grasp_lateral_offset_m=offset) for i,offset in enumerate((-.01,.01))]
        with tempfile.TemporaryDirectory() as folder:
            plan,checks,failures=postcheck_cutamp_candidates(registry,Domain(),world,program,candidates,{},folder)
        self.assertIsNone(plan)
        feedback=exact_failure_feedback(program,failures,checks=checks,candidate_count=2,max_postchecks=8,target='target')
        self.assertFalse(feedback['program_unsat'])
        self.assertEqual([p['position']['lateral'] for p in feedback['sampled_failure_positions']],[.25,.75])
        document={'schema':'scenesmith.proc3s.program.v1','steps':[dict(skill='PickLift',arguments={'object':'target'},
            continuous_variables={'grasp_pose':'$g','approach_pose':'$a'})],
            'domains':[dict(variable='g',sampler='calibrated_grasp_pose'),dict(variable='a',sampler='calibrated_approach_pose')]}
        requests=[]
        class Client:
            def complete(self,**kwargs):
                requests.append(json.loads(kwargs['messages'][1]['content']))
                return ChatCompletion(json.dumps(document),'fixture','fixture')
        generator=PRoC3SProgramGenerator(Client(),ModelSettings(),registry)
        generator.last_program=document
        generator.generate(actual_world,(PredicateGoal('holding',('target',)),),feedback=(feedback,))
        received=requests[0]['constraint_feedback'][0]
        self.assertEqual(received['sampled_failure_positions'],feedback['sampled_failure_positions'])
        self.assertIn('not_full_assignments_or_region_unsat',received['sampled_failure_scope'])
        self.assertNotIn('sampled_failure_positions',ProgramFailure.from_feedback(feedback).abstract())
        # Real current-station fallback strips navigation before geometry. Its
        # checked step 0 must be attributed to model-program step 1 on repair.
        document['steps'].insert(0, dict(skill='NavigateToPick',arguments={'object':'target'},
                                       continuous_variables={'base_pose':'$b'}))
        document['domains'].append(dict(variable='b',sampler='scene_base_pose'))
        generator.last_program=document
        generator.generate(actual_world,(PredicateGoal('holding',('target',)),),feedback=(feedback,))
        remapped=requests[-1]['constraint_feedback'][0]['sampled_failure_positions']
        self.assertTrue(all(point['program_step']==1 for point in remapped))
        self.assertEqual([point['position'] for point in remapped],
                         [point['position'] for point in feedback['sampled_failure_positions']])
        changed_world=dataclasses.replace(actual_world, observation_id='1.000000')
        generator.generate(changed_world,(PredicateGoal('holding',('target',)),),feedback=(feedback,))
        self.assertNotIn('sampled_failure_positions', requests[-1]['constraint_feedback'][0])



    def test_normalization_uses_full_envelope_and_does_not_clamp_outside(self):
        domain=SimpleNamespace(candidates=((0.,0.,0.),(2.,0.,0.),(0.,4.,0.),(2.,4.,0.)),
                               snapshot=SimpleNamespace(token='a'*64))
        step=SkillStep('NavigateToPick',{'object':'target'},{'base_pose':'base'})
        point=normalized_failure_positions(domain,step,0,{'base_x_m':1.,'base_y_m':1.},'collision')
        self.assertEqual(point[0]['position'],{'forward':.5,'lateral':.25})
        self.assertEqual(normalized_failure_positions(domain,step,0,{'base_x_m':3.,'base_y_m':1.},'collision'),())
        self.assertEqual(normalized_failure_positions(domain,step,0,{},'excluded_assignment'),())
        for bad in ({**point[0],'position':{'forward':float('nan'),'lateral':.2}},
                    {**point[0],'position':{'forward':.2,'lateral':True}},
                    {**point[0],'arbitrary_text':'injected'},
                    {**point[0],'failed_constraint':'untrusted'}):
            with self.assertRaises(ValueError): validate_positions([bad])
