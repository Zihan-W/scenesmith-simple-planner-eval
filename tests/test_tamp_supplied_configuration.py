"""The shared domain must validate, never replace, solver-supplied grasp joints."""

from types import SimpleNamespace
import unittest

from examples.online_manipulation.tamp_scenesmith import SceneSmithPickDomain
from src.online_manipulation.observations import Pose
from src.online_manipulation.planning import ClearanceMetrics, ConfigurationCheck
from src.online_manipulation.tasks import PickLiftTaskConfig


class SuppliedConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.domain = object.__new__(SceneSmithPickDomain)
        self.domain.task_config = PickLiftTaskConfig(
            target_observation_name="target", target_contact_body="box::body",
            gripper_contact_bodies=("robot::finger_a", "robot::finger_b"))
        self.spec = SimpleNamespace(
            arm_groups={"left": ("j0", "j1")}, controlled_joint_names=("j0", "finger", "j1"),
            model_instance_name="robot", end_effector_frames={"left": "tcp"})
        self.desired = Pose((0, 0, 0), (1, 0, 0, 0))
        self.actual = self.desired
        self.checked = None
        self.collision_valid = True
        self.query = SimpleNamespace(
            robot_adapter=SimpleNamespace(spec=self.spec),
            check_configuration=self.check_configuration,
            frame_pose=lambda *args: self.actual,
            solve_ik=lambda *args, **kwargs: self.fail("Supplied grasp must not be re-solved"))

    def check_configuration(self, q, **kwargs):
        self.checked = tuple(q)
        clearance = ClearanceMetrics(0.01, 0.01, None, None, 0.05)
        return ConfigurationCheck(self.collision_valid, tuple(q), clearance)

    def test_keeps_supplied_arm_and_seeded_non_arm_values_exactly(self):
        result = self.domain._check_supplied_grasp_configuration(
            self.query, self.desired, [0, 0.012, 0], [0.31, -0.27])
        self.assertTrue(result.success)
        self.assertEqual(result.configuration, (0.31, 0.012, -0.27))
        self.assertEqual(self.checked, result.configuration)
        self.assertEqual(result.solver_result, "external_configuration_checked_without_ik")

    def test_matches_existing_axiswise_ik_position_box(self):
        self.actual = Pose((0.0009, 0.0009, 0.0009), (1, 0, 0, 0))
        result = self.domain._check_supplied_grasp_configuration(
            self.query, self.desired, [0, 0, 0], [0, 0])
        self.assertTrue(result.success)
        self.assertGreater(result.position_error_m, 0.001)
        self.actual = Pose((0.0011, 0, 0), (1, 0, 0, 0))
        result = self.domain._check_supplied_grasp_configuration(
            self.query, self.desired, [0, 0, 0], [0, 0])
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "supplied_grasp_pose_error")

    def test_collision_is_not_overridden_by_matching_pose(self):
        self.collision_valid = False
        result = self.domain._check_supplied_grasp_configuration(
            self.query, self.desired, [0, 0, 0], [0, 0])
        self.assertFalse(result.success)

    def test_bad_dimensions_and_nonfinite_values_rejected(self):
        for values in ([0], [0, float("nan")], [float("inf"), 0]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.domain._check_supplied_grasp_configuration(
                    self.query, self.desired, [0, 0, 0], values)


if __name__ == "__main__":
    unittest.main()
