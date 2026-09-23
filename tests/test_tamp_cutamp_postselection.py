"""Numeric postselection contract only; no physical feasibility is simulated."""

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from scripts.cutamp_observed_resolve import postcheck_candidates
from planner.src.tamp.scenesmith import SceneSmithPickDomain


class PostselectionContractTests(unittest.TestCase):
    def test_bounded_batch_uses_existing_gap_score_without_replacing_q(self):
        class Domain:
            target_name = "target"
            config = SimpleNamespace(robot_adapter=SimpleNamespace(
                base_config=SimpleNamespace(base_height_m=0.2),
                spec=SimpleNamespace(arm_groups={"left": ["arm"]})))
            rank_candidate = SceneSmithPickDomain.rank_candidate

            def predict(self, subgoal, assignment, state):
                return dict(state)

            def check(self, subgoal, assignment, state):
                q = assignment["grasp_arm_joint_positions"]
                return True, "fixture_only", {"ik": {
                    "grasp_pose_in_target": {"arm_joint_positions": list(q)},
                    "grasp_contacts": {"gap_imbalance_m": 0.01 - assignment["grasp_lateral_offset_m"]},
                    "lift_waypoints_world": {"waypoints": [{"min_joint_margin_rad": 0.1}]}}}

        world = SimpleNamespace(objects={"target": {}}, robot={"base_link_pose": {
            "translation_m": [0., 0., 0.2], "quaternion_wxyz": [1., 0., 0., 0.]}})
        candidates = [{"particle_index": index, "optimizer_feasible": True, "hard_constraint_cost": index,
                       "base_world_pose": [0., 0., 0.2, 1., 0., 0., 0.],
                       "arm_joint_names": ["arm"], "arm_joint_positions": [index + 0.1],
                       "grasp_lateral_offset_m": index * 0.001} for index in range(3)]
        original = json.dumps(candidates)
        with tempfile.TemporaryDirectory() as directory:
            assignment, checks = postcheck_candidates(Domain(), world, candidates, directory, max_postchecks=2)
            selection = json.loads((Path(directory) / "candidate_selection.json").read_text())
        self.assertEqual(len(checks), 2)
        self.assertEqual(selection["selected_particle"], 1)
        self.assertEqual(assignment["grasp_arm_joint_positions"], [1.1])
        self.assertEqual(json.dumps(candidates), original)


if __name__ == "__main__":
    unittest.main()
