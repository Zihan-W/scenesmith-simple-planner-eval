"""cuTAMP preserves the program, GPU assignments, exclusions and parking domain."""

import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from planner.src.tamp.cutamp import CuTAMPSettings, postcheck_cutamp_candidates
from planner.src.tamp.cutamp_base_domain import base_domain_halfspaces
from planner.src.tamp.geometry import assignment_identity
from planner.src.tamp.hierarchy import SkillProgram, SkillStep, picklift_registry


class CuTAMPIntegrationTests(unittest.TestCase):
    def test_quality_window_preserves_rank_selection_and_names_early_stop(self):
        class Domain:
            config = SimpleNamespace(robot_adapter=SimpleNamespace(
                spec=SimpleNamespace(arm_groups={"left": ["arm"]})))

            def check(self, skill, parameters, state):
                del skill, state
                return True, "valid", {"ik": {"grasp_pose_in_target": {
                    "arm_joint_positions": parameters["grasp_arm_joint_positions"]}}}

            def predict(self, skill, parameters, state):
                del skill, parameters
                return dict(state)

            def rank_candidate(self, skill, parameters, checks):
                del skill, checks
                return (-parameters["grasp_lateral_offset_m"],)

        world = SimpleNamespace(objects={"target": {}}, robot={"base_link_pose": {
            "translation_m": [0, 0, 0.2], "quaternion_wxyz": [1, 0, 0, 0]}})
        program = SkillProgram((SkillStep("PickLift", {"object": "target"},
                                         {"grasp_pose": "g0", "approach_pose": "a0"}),))
        particles = [{"particle_index": index, "optimizer_feasible": True,
                      "hard_constraint_cost": float(index),
                      "base_world_pose": [0, 0, 0.2, 1, 0, 0, 0],
                      "arm_joint_names": ["arm"], "arm_joint_positions": [0.1 + index],
                      "grasp_lateral_offset_m": index * 0.001} for index in range(3)]
        for window, expected_particle, expected_reason, expected_checks in (
                (None, 2, "all_candidates_checked", 3),
                (0, 0, "first_pass_found", 1),
                (1, 1, "quality_window_complete", 2)):
            with self.subTest(window=window), tempfile.TemporaryDirectory() as folder:
                plan, checks, failures = postcheck_cutamp_candidates(
                    picklift_registry(), Domain(), world, program, particles, {}, folder,
                    postcheck_order="cost", quality_window_after_first_pass=window)
                selection = json.loads((Path(folder) / "candidate_selection.json").read_text())
                self.assertFalse(failures)
                self.assertEqual(len(checks), expected_checks)
                self.assertEqual(selection["stop_reason"], expected_reason)
                self.assertEqual(selection["first_pass_particle"], 0)
                self.assertEqual(selection["selected_particle"], expected_particle)
                self.assertFalse(selection["count_exhausted"])
                self.assertEqual(plan.actions[0].geometric_parameters[
                    "grasp_arm_joint_positions"], [0.1 + expected_particle])

        with tempfile.TemporaryDirectory() as folder:
            postcheck_cutamp_candidates(
                picklift_registry(), Domain(), world, program, particles, {}, folder,
                max_postchecks=2, postcheck_order="cost",
                quality_window_after_first_pass=1)
            selection = json.loads((Path(folder) / "candidate_selection.json").read_text())
            self.assertEqual(selection["stop_reason"], "quality_window_complete")
            self.assertTrue(selection["count_exhausted"])

    def test_rotated_domain_rejects_aabb_corner(self):
        yaw = 0.37
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                             [math.sin(yaw), math.cos(yaw)]])
        xy = np.array([[-1, -2], [-1, 2], [1, -2], [1, 2]]) @ rotation.T
        anchors = np.column_stack((xy, np.full(4, yaw)))
        halfspaces = base_domain_halfspaces(anchors, yaw)
        self.assertTrue(np.all(halfspaces[:, 2] < 0))
        corner = xy.max(axis=0)
        self.assertGreater(np.max(halfspaces[:, :2] @ corner + halfspaces[:, 2]), 0.1)

    def test_navigation_witness_does_not_insert_executable_pick(self):
        class Domain:
            candidates = ((-1, -1, 0), (-1, 1, 0), (1, -1, 0), (1, 1, 0))
            config = SimpleNamespace(robot_adapter=SimpleNamespace(
                spec=SimpleNamespace(arm_groups={"left": ["arm"]})))

            def check(self, skill, parameters, state):
                if skill.skill == "NavigateToPick":
                    return True, "fixture", {"navigation_goal": [0, 0, 0],
                                             "base_xyz_yaw": [0, 0, 0],
                                             "pick_witness": {"parameters": {}, "checks": {}}}
                raise AssertionError("Navigation must not bind an undeclared GPU grasp configuration")

            def predict(self, *args):
                return {}

            def rank_candidate(self, *args):
                return (0,)

        world = SimpleNamespace(objects={"target": {}}, robot={"base_link_pose": {
            "translation_m": [0, 0, 0.2], "quaternion_wxyz": [1, 0, 0, 0]}})
        program = SkillProgram((SkillStep("NavigateToPick", {"object": "target"},
                                         {"base_pose": "b0"}),))
        particles = [{"particle_index": 0, "optimizer_feasible": False, "hard_constraint_cost": 0.01,
                      "base_world_pose": [0, 0, 0.2, 1, 0, 0, 0],
                      "arm_joint_names": ["arm"], "arm_joint_positions": [0.4],
                      "grasp_lateral_offset_m": 0.0}]
        with tempfile.TemporaryDirectory() as folder:
            plan, checks, failures = postcheck_cutamp_candidates(
                picklift_registry(), Domain(), world, program, particles, {}, folder)
            self.assertEqual([a.skill_name for a in plan.actions], ["NavigateToPick"])
            details = checks[0]["steps"][0]["details"]
            self.assertFalse(details["gpu_grasp_configuration_is_execution_binding"])
            self.assertNotIn("grasp_arm_joint_positions", plan.actions[0].geometric_parameters)
            self.assertTrue(checks[0]["approximate_exact_disagreement"])
            self.assertFalse(checks[0]["optimizer_feasible"])
            excluded = frozenset({assignment_identity(plan.assignments)})
            rejected, _, errors = postcheck_cutamp_candidates(
                picklift_registry(), Domain(), world, program, particles, {}, folder,
                excluded_assignments=excluded)
            self.assertIsNone(rejected)
            self.assertEqual(errors[-1].constraint, "excluded_assignment")

    def test_settings_preserve_virtualenv_python_symlink(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            executable = root / "python-real"
            executable.touch()
            (root / "python").symlink_to(executable)
            names = ("cutamp_root", "kinematics_template", "robot_template", "multipliers",
                     "tolerances", "grasp_calibration")
            for name in names:
                (root / name).touch()
            settings = {name: name for name in names}
            settings["gpu_python"] = "python"
            path = root / "settings.json"
            path.write_text(json.dumps(settings))
            loaded = CuTAMPSettings.from_file(path)
            self.assertEqual(loaded.gpu_python, str(root / "python"))


if __name__ == "__main__":
    unittest.main()
