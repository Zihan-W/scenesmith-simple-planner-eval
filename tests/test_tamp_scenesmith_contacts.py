"""The geometric grasp check must permit only task-authorized contact pairs."""

import unittest

from planner.src.tamp.scenesmith import SceneSmithPickDomain
from simulation.src.geometry.contact import permits_contact, penetration_limit
from simulation.src.tasks.tasks import PickLiftTaskConfig
from simulation.src.geometry.planning import PlanningQuery


class PlannedGraspContactTest(unittest.TestCase):
    def test_fingers_and_support_have_independent_bounds(self):
        domain = object.__new__(SceneSmithPickDomain)
        domain.task_config = PickLiftTaskConfig(
            target_observation_name="pick_target",
            gripper_contact_bodies=("robot::left_finger", "robot::right_finger"),
            target_contact_body="box::base_link",
            support_contact_bodies=("table::base_link",),
            maximum_allowed_contact_penetration_m=0.0005,
            maximum_allowed_support_penetration_m=0.0035,
        )
        policy = domain._grasp_contact_policy()
        self.assertTrue(permits_contact(policy, "robot::left_finger", "box::base_link"))
        self.assertTrue(permits_contact(policy, "table::base_link", "box::base_link"))
        self.assertFalse(permits_contact(policy, "robot::elbow", "box::base_link"))
        self.assertEqual(penetration_limit(policy, "robot::left_finger", "box::base_link"),
                         0.0005)
        self.assertEqual(penetration_limit(policy, "table::base_link", "box::base_link"),
                         0.0035)

    def test_query_adds_wheel_support_to_existing_task_support(self):
        domain = object.__new__(SceneSmithPickDomain)
        domain.task_config = PickLiftTaskConfig(
            target_observation_name="pick_target",
            gripper_contact_bodies=("robot::left_finger", "robot::right_finger"),
            target_contact_body="box::base_link",
            support_contact_bodies=("table::base_link",),
            maximum_allowed_contact_penetration_m=0.0005,
            maximum_allowed_support_penetration_m=0.0035,
        )
        query = object.__new__(PlanningQuery)
        query.support_limits_m = {}
        wheel = ("robot::wheel", "tire")
        floor = ("room::body", "floor_collision")
        query.support_geometry_limits_m = {tuple(sorted((wheel, floor))): 0.002}
        policy = query._contact_policy(domain._grasp_contact_policy())
        self.assertTrue(permits_contact(policy, wheel[0], floor[0], wheel[1], floor[1]))
        self.assertEqual(penetration_limit(
            policy, wheel[0], floor[0], wheel[1], floor[1]), 0.002)
        self.assertFalse(permits_contact(
            policy, wheel[0], floor[0], wheel[1], "wall_collision"))
        self.assertEqual(penetration_limit(
            policy, "robot::left_finger", "box::base_link"), 0.0005)
        self.assertEqual(penetration_limit(
            policy, "table::base_link", "box::base_link"), 0.0035)
        self.assertIs(query._contact_policy(policy), policy)


if __name__ == "__main__":
    unittest.main()
