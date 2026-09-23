"""Reject unsupported skeletons and stale observations before any model export."""

import unittest
from types import SimpleNamespace

from planner.src.tamp.snapshot import PlanningSnapshot
from simulation.src import Pose
from planner.src.tamp.cutamp_problem import ContinuousProblemBuilder
from planner.src.tamp.hierarchy import SkillProgram, SkillStep, WorldState


class ContinuousProblemBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.builder = object.__new__(ContinuousProblemBuilder)
        self.pose = {"translation_m": [1., 2., 0.2], "quaternion_wxyz": [1., 0., 0., 0.]}
        self.observation = SimpleNamespace(time_s=8.8, base={"base_link_pose": self.pose},
                                           robot=SimpleNamespace(joint_names=["arm"], q=[0.]),
                                           objects={"target": SimpleNamespace(pose=Pose((1,2,0.2),(1,0,0,0)))})
        self.builder.domain = SimpleNamespace(target_name="target", observation=self.observation,
            snapshot=PlanningSnapshot(self.observation))
        self.program = SkillProgram((SkillStep("PickLift", {"object": "target"},
                                              {"grasp_pose": "g0", "approach_pose": "a0"}),))
        self.robot = {"base_link_pose": self.pose, "joint_names": ["arm"], "q": [0.]}

    def test_repeated_navigation_is_not_silently_rewritten(self):
        program = SkillProgram((SkillStep("NavigateToPick", {"object": "target"},
                                          {"base_pose": "b0"}),
                                SkillStep("NavigateToPick", {"object": "target"},
                                          {"base_pose": "b1"})))
        world = WorldState({"target": self.pose}, frozenset(), "8.800000", self.robot)
        with self.assertRaisesRegex(ValueError, "Unsupported cuTAMP"):
            self.builder.build(program, world, "unused_must_not_be_created")

    def test_rejects_stale_timestamp(self):
        world = WorldState({"target": self.pose}, frozenset(), "0.000000", self.robot)
        with self.assertRaisesRegex(ValueError, "snapshot token"):
            self.builder.build(self.program, world, "unused_must_not_be_created")

    def test_rejects_conflicting_joint_measurement(self):
        world = WorldState({"target": self.pose}, frozenset(), "8.800000", {**self.robot, "q": [0.1]})
        with self.assertRaisesRegex(ValueError, "snapshot token"):
            self.builder.build(self.program, world, "unused_must_not_be_created")

    def test_rejects_conflicting_base_measurement(self):
        pose = {**self.pose, "translation_m": [1.01, 2., 0.2]}
        world = WorldState({"target": self.pose}, frozenset(), "8.800000", {**self.robot, "base_link_pose": pose})
        with self.assertRaisesRegex(ValueError, "snapshot token"):
            self.builder.build(self.program, world, "unused_must_not_be_created")


if __name__ == "__main__":
    unittest.main()
