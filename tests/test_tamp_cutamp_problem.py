"""Reject unsupported skeletons and stale observations before any model export."""

import unittest
from types import SimpleNamespace

from examples.online_manipulation.tamp_cutamp_problem import ContinuousProblemBuilder
from examples.online_manipulation.tamp_hierarchy import SkillProgram, SkillStep, WorldState


class ContinuousProblemBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.builder = object.__new__(ContinuousProblemBuilder)
        self.pose = {"translation_m": [1., 2., 0.2], "quaternion_wxyz": [1., 0., 0., 0.]}
        self.observation = SimpleNamespace(time_s=8.8, base={"base_link_pose": self.pose},
                                           robot=SimpleNamespace(joint_names=["arm"], q=[0.]), objects={})
        self.builder.domain = SimpleNamespace(target_name="target", observation=self.observation)
        self.program = SkillProgram((SkillStep("PickLift", {"object": "target"},
                                              {"grasp_pose": "g0", "approach_pose": "a0"}),))
        self.robot = {"base_link_pose": self.pose, "joint_names": ["arm"], "q": [0.]}

    def test_navigation_only_is_not_silently_given_a_pick(self):
        program = SkillProgram((SkillStep("NavigateToPick", {"object": "target"},
                                          {"base_pose": "b0"}),))
        world = WorldState({"target": {}}, frozenset(), "8.800000", self.robot)
        with self.assertRaisesRegex(ValueError, "Unsupported fixed"):
            self.builder.build(program, world, "unused_must_not_be_created")

    def test_rejects_stale_timestamp(self):
        world = WorldState({"target": {}}, frozenset(), "0.000000", self.robot)
        with self.assertRaisesRegex(ValueError, "timestamps"):
            self.builder.build(self.program, world, "unused_must_not_be_created")

    def test_rejects_conflicting_joint_measurement(self):
        world = WorldState({"target": {}}, frozenset(), "8.800000", {**self.robot, "q": [0.1]})
        with self.assertRaisesRegex(ValueError, "joint states"):
            self.builder.build(self.program, world, "unused_must_not_be_created")

    def test_rejects_conflicting_base_measurement(self):
        pose = {**self.pose, "translation_m": [1.01, 2., 0.2]}
        world = WorldState({"target": {}}, frozenset(), "8.800000", {**self.robot, "base_link_pose": pose})
        with self.assertRaisesRegex(ValueError, "base poses"):
            self.builder.build(self.program, world, "unused_must_not_be_created")


if __name__ == "__main__":
    unittest.main()
