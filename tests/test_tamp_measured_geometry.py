"""Actual planning queries start from measured joints and a snapshot-scoped cache."""

import random
import unittest
from types import SimpleNamespace

import numpy as np

from planner.src.tamp.snapshot import PlanningSnapshot
from planner.src.tamp.scenesmith import SceneSmithPickDomain
from planner.src.tamp.planner import Subgoal
from simulation.src import BaseVelocityAction, JointDeltaAction, Pose, RobotCommand, make_env
from simulation.src.robots.adapters.description import drake_pose
from simulation.src.recipes.mobile import make_config


class MeasuredGeometryTest(unittest.TestCase):
    def test_lift_reserves_ik_error_and_reuses_shared_skill_height(self):
        domain = object.__new__(SceneSmithPickDomain)
        domain.task_config = SimpleNamespace(required_lift_m=0.08)
        domain.nominal_lift_distance_m = 0.1
        self.assertEqual(domain._lift_distance(), 0.1)
        domain.task_config.required_lift_m = 0.12
        self.assertAlmostEqual(domain._lift_distance(), 0.126)
        self.assertGreater(domain._lift_distance() - domain.lift_ik_tolerance_m,
                           domain.task_config.required_lift_m)

    def test_approach_heights_are_solver_owned_and_scaled_to_gripper(self):
        domain = object.__new__(SceneSmithPickDomain)
        domain.target_name = "red"
        domain.config = SimpleNamespace(robot_adapter=SimpleNamespace(spec=SimpleNamespace(
            grippers={"left": SimpleNamespace(maximum_width_m=0.08)})))
        samples = list(domain.samples(Subgoal("PickLift", {"target": "red"}), {}))
        self.assertEqual(len(samples), 36)
        self.assertEqual({sample["approach_height_offset_m"] for sample in samples},
                         {0.0, 0.04, 0.08})
        self.assertTrue(all(sample["target"] == "red" for sample in samples))
        self.assertEqual({sample["approach_segments"] for sample in samples}, {1, 8})

    def test_ccsp_staging_height_varies_within_existing_domain(self):
        domain = object.__new__(SceneSmithPickDomain)
        domain.target_name = "red"
        domain.config = SimpleNamespace(robot_adapter=SimpleNamespace(spec=SimpleNamespace(
            grippers={"left": SimpleNamespace(maximum_width_m=0.08)})))
        rng = random.Random(501)
        samples = [domain.sample_candidate(Subgoal("PickLift", {"target": "red"}), {}, rng)
                   for _ in range(16)]
        heights = [sample["approach_height_offset_m"] for sample in samples]
        self.assertEqual(len(set(heights)), 16)
        self.assertTrue(all(0 <= height <= 0.08 for height in heights))

    def test_query_uses_measured_joints_and_independent_candidates(self):
        config = make_config("wheel_dynamic")
        env = make_env(config)
        observation, _ = env.reset(4)
        name = config.robot_adapter.spec.arm_groups["left"][0]
        for _ in range(5):
            observation, *_ = env.step(RobotCommand(
                arms={"left": JointDeltaAction((name,), (-0.01,))},
                base=BaseVelocityAction(0.08, 0.0)))
        measured = dict(zip(observation.robot.joint_names, observation.robot.q, strict=True))
        self.assertLess(measured[name], -0.005)
        domain = object.__new__(SceneSmithPickDomain)
        domain.config = config
        domain.snapshot = PlanningSnapshot(observation, env.get_planning_query())
        domain._navigation_query_cache = None
        pose = observation.base["base_link_pose"]
        base = Pose(tuple(pose["translation_m"]), tuple(pose["quaternion_wxyz"]))
        query, adapter = domain._candidate_query(base)
        np.testing.assert_allclose(query.configuration(), [
            measured[joint] for joint in adapter.spec.controlled_joint_names])
        self.assertIsNot(domain._candidate_query(base)[0], query)
        moved = Pose((base.translation_m[0] + 0.1, *base.translation_m[1:]),
                     base.quaternion_wxyz)
        moved_query, _ = domain._candidate_query(moved)
        self.assertIsNot(moved_query, query)
        np.testing.assert_allclose(moved_query.configuration(), query.configuration())
        nav_query, _ = domain._navigation_goal(moved)
        np.testing.assert_allclose(nav_query.configuration(), query.configuration())
        actual_base = nav_query.frame_pose(adapter.spec.model_instance_name,
                                          adapter.spec.base_link_name)
        np.testing.assert_allclose(actual_base.translation_m, base.translation_m,
                                   atol=1e-10)
        body = nav_query.plant.GetBodyByName(adapter.spec.base_link_name,
                                            nav_query.robot_model_instance)
        nav_query.plant.SetFreeBodyPose(nav_query.context, body, drake_pose(moved))
        restored_query, _ = domain._navigation_goal(moved)
        self.assertIsNot(restored_query, nav_query)
        restored_base = restored_query.frame_pose(adapter.spec.model_instance_name,
                                             adapter.spec.base_link_name)
        np.testing.assert_allclose(restored_base.translation_m, base.translation_m, atol=1e-10)
        next_query, _ = domain._navigation_goal(base, start_pose=moved)
        predicted_start = next_query.frame_pose(adapter.spec.model_instance_name,
                                                adapter.spec.base_link_name)
        np.testing.assert_allclose(predicted_start.translation_m, moved.translation_m, atol=1e-10)
        finger = adapter.spec.grippers["left"].joint_names[0]
        lower = query.joint_limits()[finger][0]
        measured[finger] = lower - 1e-11
        query.set_observed_joint_positions(measured)
        index = adapter.spec.controlled_joint_names.index(finger)
        self.assertEqual(query.configuration()[index], lower - 1e-11)
        self.assertTrue(query.check_configuration(query.configuration()).within_joint_limits)
        measured[finger] = lower - 1e-4
        query.set_observed_joint_positions(measured)
        self.assertFalse(query.check_configuration(query.configuration()).within_joint_limits)


if __name__ == "__main__":
    unittest.main()
