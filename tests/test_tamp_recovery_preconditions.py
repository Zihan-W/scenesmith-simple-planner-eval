"""A contact failure must not imply an empty gripper or authorize navigation."""
import unittest
from types import SimpleNamespace
from planner.src.tamp.scenesmith_online import failed_pick_recovery_status, SceneSmithWorldObserver
from planner.src.tamp.hierarchy import PredicateGoal, picklift_registry
from planner.src.tamp.geometry import SamplingSolver
from planner.src.tamp.online import IncrementalTampRunner, SkillExecution
from test_tamp_online import Semantic, Domain, Observer, Executor

class RecoveryPreconditionsTest(unittest.TestCase):
    def task(self,**changes):
        return SimpleNamespace(time_s=1.,task={'support_contact':True,'finger_contacts':[False,False],'bilateral_gripper_contact':False,'unexpected_target_contacts':[],**changes})

    def test_single_finger_contact_blocks_recovery(self):
        status=failed_pick_recovery_status(self.task(finger_contacts=[False,True]),True,'valid')
        self.assertFalse(status['allowed']);self.assertIn('target_finger_contact',status['blockers'])

    def test_no_contact_does_not_override_invalid_navigation_start(self):
        status=failed_pick_recovery_status(self.task(),False,'corridor alpha=0.00')
        self.assertFalse(status['allowed']);self.assertEqual(status['navigation_check_reason'],'corridor alpha=0.00')

    def test_unsupported_target_blocks_recovery(self):
        self.assertFalse(failed_pick_recovery_status(self.task(support_contact=False),True,'valid')['allowed'])

    def test_supported_clear_state_retains_existing_recovery(self):
        self.assertTrue(failed_pick_recovery_status(self.task(),True,'valid')['allowed'])

    def test_observer_does_not_label_unilateral_contact_empty(self):
        observer=SceneSmithWorldObserver.__new__(SceneSmithWorldObserver)
        observer.executor=SimpleNamespace(last_navigation_goal=None);observer.object_metadata={};observer.target_name='red_cube'
        observation=self.task(finger_contacts=[False,True])
        observation.objects={};observation.base={'pose':{},'frame':'nav','base_link_pose':{},'base_link_frame':'base'}
        observation.robot=SimpleNamespace(joint_names=(),q=(),gripper_widths_m={})
        self.assertNotIn(PredicateGoal('gripper_empty',()),observer.observe(observation).facts)

    def test_runner_replans_without_unsafe_motion(self):
        seen=PredicateGoal('observed',('red_cube',));empty=PredicateGoal('gripper_empty',());holding=PredicateGoal('holding',('red_cube',))
        class ContactFailureExecutor(Executor):
            def execute(self,action):
                if action.skill_name=='PickLift':
                    self.calls.append(action.skill_name)
                    return SkillExecution(frozenset(self.facts),False,'planned_bilateral_contact_timeout',failure_details={'recovery_preconditions':{'allowed':False,'blockers':['target_finger_contact']}})
                return super().execute(action)
        executor=ContactFailureExecutor((seen,empty));events=[];registry=picklift_registry()
        runner=IncrementalTampRunner(semantic=Semantic(((holding,), (holding,), (holding,))),registry=registry,solver_factory=lambda world:SamplingSolver(registry,Domain(),batch_size=2),executor=executor,observer=Observer(('red_cube',)),trace=events.append)
        result=runner.run(task='pick red_cube',task_goals=(holding,),initial_observation=frozenset((seen,empty)),initial_geometry_state={'base_height_m':.18},predicate_arity={'holding':1,'observed':1,'gripper_empty':0})
        self.assertEqual(result.reason,'recovery_preconditions_failed');self.assertFalse(result.success)
        self.assertEqual(executor.calls,['NavigateToPick','PickLift'])
        self.assertTrue(any(e['event']=='recovery_blocked' for e in events))
        self.assertGreaterEqual(result.metrics['num_skill_replans'], 1)

if __name__=='__main__':unittest.main()
