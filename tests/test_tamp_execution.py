"""A grounded joint program must keep its stage and contact gates."""

import unittest
from types import SimpleNamespace

from examples.online_manipulation.tamp_execution import JointWaypointPickLiftSkill


NAMES = tuple(f"joint_{index}" for index in range(7))


def observation(q, *, bilateral=False):
    return SimpleNamespace(
        robot=SimpleNamespace(joint_names=NAMES, q=tuple(q), q_commanded=tuple(q),
                              v=(0.0,) * 7),
        task={"bilateral_gripper_contact": bilateral},
    )


class PlannedPickLiftTest(unittest.TestCase):
    def setUp(self):
        config = SimpleNamespace(open_width_m=0.076,
                                 closed_width_m=0.0,
                                 stable_contact_steps=2,
                                 maximum_close_steps=10,
                                 maximum_lost_contact_steps=3,
                                 contact_cartesian_tracking_tolerance=0.04,
                                 cartesian_velocity_tolerance=0.2)
        plan = {"arm_joint_names": NAMES,
                "staging_joint_positions": [0.1] * 7,
                "grasp_joint_positions": [0.2] * 7,
                "lift_waypoints": [[0.25] * 7, [0.3] * 7]}
        self.config, self.plan = config, plan
        self.skill = JointWaypointPickLiftSkill(config, plan)

    def test_consumes_every_certified_approach_waypoint_before_closing(self):
        plan = {**self.plan, "approach_waypoints": [[0.15] * 7, [0.2] * 7]}
        skill = JointWaypointPickLiftSkill(self.config, plan)
        for _ in range(4):
            skill.act(observation([0.1] * 7))
        self.assertEqual(skill.stage, "align")
        for _ in range(4):
            command = skill.act(observation([0.15] * 7))
        self.assertEqual(skill.approach_index, 1)
        self.assertEqual(skill.stage, "align")
        self.assertEqual(command.arms["left"].positions, tuple([0.2] * 7))
        for _ in range(4):
            skill.act(observation([0.2] * 7))
        self.assertEqual(skill.stage, "close")

    def test_rejects_approach_path_ending_at_a_different_grasp(self):
        with self.assertRaisesRegex(ValueError, "terminate at the planned grasp"):
            JointWaypointPickLiftSkill(self.config, {
                **self.plan, "approach_waypoints": [[0.15] * 7]})

    def test_runtime_limits_preserve_joint_edge_direction_instead_of_axis_clipping(self):
        target = [0.12, -0.28, 0.15, 0.01, 0.0, 0.0, 0.0]
        skill = JointWaypointPickLiftSkill(
            self.config, {**self.plan, "staging_joint_positions": target},
            joint_step_limits={name: 0.1 for name in NAMES})
        actual = observation([0.0] * 7)
        first = skill.act(actual).arms["left"].positions
        for value, goal in zip(first, target, strict=True):
            self.assertAlmostEqual(value, goal * (0.1 / 0.28))
            self.assertLessEqual(abs(value), 0.1)
        # Runtime clips relative to the previous COMMAND, not measured lag.
        actual.robot.q_commanded = first
        second = skill.act(actual).arms["left"].positions
        for value, goal in zip(second, target, strict=True):
            self.assertAlmostEqual(value, goal * (0.2 / 0.28))

    def test_tracks_grasp_contact_and_lift_waypoints(self):
        for _ in range(4):
            self.skill.act(observation([0.1] * 7))
        self.assertEqual(self.skill.stage, "align")
        for _ in range(4):
            self.skill.act(observation([0.2] * 7))
        self.assertEqual(self.skill.stage, "close")
        for _ in range(2):
            self.skill.act(observation([0.2] * 7, bilateral=True))
        self.assertEqual(self.skill.stage, "verify")
        for _ in range(4):
            self.skill.act(observation([0.25] * 7, bilateral=True))
        self.assertEqual(self.skill.waypoint_index, 1)
        for _ in range(4):
            self.skill.act(observation([0.3] * 7, bilateral=True))
        self.assertEqual(self.skill.stage, "hold")
        self.assertIsNone(self.skill.stop_reason)

    def test_lost_contact_stops_lift(self):
        self.skill.stage = "verify"
        for _ in range(4):
            self.skill.act(observation([0.2] * 7, bilateral=False))
        self.assertEqual(self.skill.stop_reason, "planned_lost_contact")

    def test_loaded_waypoint_uses_shared_contact_tolerance_and_velocity_gate(self):
        self.skill.stage = "verify"
        loaded = observation([0.22] * 7, bilateral=True)
        loaded.robot.v = (0.3,) * 7
        for _ in range(4):
            self.skill.act(loaded)
        self.assertEqual(self.skill.waypoint_index, 0)
        loaded.robot.v = (0.0,) * 7
        for _ in range(4):
            self.skill.act(loaded)
        self.assertEqual(self.skill.waypoint_index, 1)

    def test_loaded_error_outside_shared_tolerance_does_not_advance(self):
        self.skill.stage = "verify"
        for _ in range(4):
            self.skill.act(observation([0.20] * 7, bilateral=True))
        self.assertEqual(self.skill.waypoint_index, 0)


if __name__ == "__main__":
    unittest.main()
