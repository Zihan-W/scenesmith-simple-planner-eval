"""Bounded measured-height corrections and safe-stop controller contracts."""
from types import SimpleNamespace
import unittest
from planner.src.skills.picklift import JointWaypointPickLiftSkill, validate_compensation

BOUNDS = dict(max_attempts=3, max_step_m=.01, max_total_m=.03, height_margin_m=.006)


class CompensationTests(unittest.TestCase):
    def skill(self, planner):
        config=SimpleNamespace(contact_cartesian_tracking_tolerance=.05,
            cartesian_velocity_tolerance=.1, stable_contact_steps=2, closed_width_m=0.,
            maximum_lost_contact_steps=3)
        plan=dict(arm_joint_names=list('abcdefg'),staging_joint_positions=[0.]*7,
                  grasp_joint_positions=[0.]*7,lift_waypoints=[[0.]*7])
        skill=JointWaypointPickLiftSkill(config,plan,compensation=BOUNDS,compensation_planner=planner)
        skill.stage='hold'
        observation=SimpleNamespace(robot=SimpleNamespace(joint_names=list('abcdefg'),q=[0.]*7,v=[0.]*7),
            task=dict(lift_m=.076,required_lift_m=.08,bilateral_gripper_contact=True))
        return skill,observation

    def test_only_settled_bilateral_underheight_requests_certified_correction(self):
        calls=[]
        def planner(observation,distance):
            calls.append(distance)
            return [[.02]*7], {'reason':'certified'}
        skill,observation=self.skill(planner)
        for _ in range(3): skill.act(observation)
        self.assertEqual(calls,[])
        skill.act(observation)
        self.assertEqual(calls,[.01])
        self.assertEqual(skill.stage,'verify')
        self.assertEqual(skill.compensation_attempts,1)
        observation.task['bilateral_gripper_contact']=False
        skill.act(observation)
        self.assertEqual(skill.stop_reason,'planned_compensation_contact_lost')

    def test_rejected_plan_exhausted_budget_and_missing_contact_stop(self):
        for mode in ('rejected','exhausted','contact','missing'):
            calls=[]
            def planner(*args):
                calls.append(args)
                return None,{'reason':'compensation_edge_rejected'}
            skill,observation=self.skill(planner)
            if mode=='exhausted': skill.compensation_attempts=3
            if mode=='contact': observation.task['bilateral_gripper_contact']=False
            if mode=='missing': observation.task.pop('lift_m')
            for _ in range(4): skill.act(observation)
            self.assertEqual(skill.stage,'failed')
            self.assertEqual(len(calls),int(mode=='rejected'))

    def test_actual_height_above_threshold_does_not_request_more_motion(self):
        skill,observation=self.skill(lambda *args: self.fail('Must hold above threshold'))
        observation.task['lift_m']=.081
        for _ in range(10): skill.act(observation)
        self.assertEqual(skill.stage,'hold')
        self.assertEqual(skill.compensation_attempts,0)

    def test_explicit_bounds_are_enforced(self):
        for values in ({}, {**BOUNDS,'max_attempts':4}, {**BOUNDS,'max_total_m':.1},
                       {**BOUNDS,'height_margin_m':float('nan')}):
            with self.assertRaises(ValueError): validate_compensation(values)

    def test_moving_arm_does_not_trigger_another_correction(self):
        calls=[]
        def planner(*args):
            calls.append(args)
            return [[.02]*7], {'reason':'certified'}
        skill,observation=self.skill(planner)
        for _ in range(4):skill.act(observation)
        self.assertEqual(len(calls),1)
        observation.robot.v=[.2]*7
        for _ in range(20):skill.act(observation)
        self.assertEqual(skill.stage,'verify')
        self.assertEqual(len(calls),1)
