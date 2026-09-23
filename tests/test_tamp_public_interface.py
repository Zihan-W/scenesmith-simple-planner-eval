"""Contracts for TAMP standalone execution and independent live planning snapshots."""

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

import test_picklift_close_target as close_contract
from planner.src.skills.runtime import SingleSkillPolicy, SkillInvocation
from planner.src.skills.navigation import certified_local_map
from planner.src.bt.core import Node
from planner.src.bt.runtime import _certified_local_map
from planner.src.tamp.snapshot import PlanningSnapshot
from planner.src.tamp.scenesmith import SceneSmithPickDomain
from simulation.src import BaseVelocityAction, Pose, make_env
from simulation.src.recipes.mobile import make_config


class StandaloneCloseContractTest(close_contract.PickLiftCloseTargetStateTest):
    """Run the established close/feedback contract against the independent adapter."""

    def setUp(self):
        super().setUp()
        self.policy = SingleSkillPolicy(
            SkillInvocation("ExecutePickLift"), self.policy.task, self.expert,
            None, {"mode": "tamp"}, 0.1)
        self.policy.reset(self.initial, {})

    def test_completion_requires_observed_success(self):
        self.expert.stage = "hold"
        self.expert.act = lambda observation: close_contract.RobotCommand()
        observed = close_contract._observation()
        observed.task.update(lift_m=0.09, held_above_threshold_s=2.999999999,
                             bilateral_gripper_contact=True, support_contact=False)
        self.policy.act(observed)
        self.assertEqual(self.policy.diagnostics()["skill_status"], "RUNNING")
        observed.task["success"] = True
        self.policy.act(observed)
        self.assertEqual(self.policy.diagnostics()["skill_status"], "SUCCESS")


class PublicPlanningSnapshotTest(unittest.TestCase):
    def test_full_state_fork_and_candidate_queries_do_not_mutate_live_state(self):
        config = make_config("wheel_dynamic")
        env = make_env(config)
        env.reset(4)
        observation, *_ = env.step(BaseVelocityAction(0.08, 0.0))
        original = env.get_planning_query()
        snapshot = original.fork()
        q = original.plant.GetPositions(original.context).copy()
        v = original.plant.GetVelocities(original.context).copy()
        np.testing.assert_array_equal(snapshot.plant.GetPositions(snapshot.context), q)
        np.testing.assert_array_equal(snapshot.plant.GetVelocities(snapshot.context), v)
        self.assertEqual(snapshot.state_time_s, observation.time_s)
        self.assertIsNot(snapshot.context, original.context)
        pose = observation.base["base_link_pose"]
        base = Pose(tuple(pose["translation_m"]), tuple(pose["quaternion_wxyz"]))
        moved = Pose((base.translation_m[0] + 0.1, *base.translation_m[1:]),
                     base.quaternion_wxyz)
        domain = object.__new__(SceneSmithPickDomain)
        domain.config = config
        domain.snapshot = PlanningSnapshot(observation, snapshot)
        domain._navigation_query_cache = None
        candidate, adapter = domain._candidate_query(moved)
        np.testing.assert_allclose(candidate.frame_pose(adapter.spec.model_instance_name,
            adapter.spec.base_link_name).translation_m, moved.translation_m, atol=1e-10)
        np.testing.assert_array_equal(original.plant.GetPositions(original.context), q)
        np.testing.assert_array_equal(snapshot.plant.GetPositions(snapshot.context), q)
        # Check all scene positions, including non-observed objects and locked joints.
        candidate.set_robot_base_pose(base)
        np.testing.assert_allclose(candidate.plant.GetPositions(candidate.context), q, atol=1e-10)
        np.testing.assert_array_equal(candidate.plant.GetVelocities(candidate.context), v)
        # Geometry mutations of an older query cannot leak into a fresh env snapshot.
        original.set_robot_base_pose(moved)
        refreshed = env.get_planning_query()
        np.testing.assert_array_equal(refreshed.plant.GetPositions(refreshed.context), q)
        env.step(BaseVelocityAction(0.08, 0.0))
        self.assertGreater(env.get_planning_query().state_time_s, snapshot.state_time_s)
        np.testing.assert_array_equal(snapshot.plant.GetPositions(snapshot.context), q)

    def test_navigation_preserves_baseline_result_and_caller_snapshot(self):
        config = make_config("wheel_dynamic")
        env = make_env(config)
        observation, _ = env.reset(4)
        query = env.get_planning_query()
        positions = query.plant.GetPositions(query.context).copy()
        x, y = observation.base["pose"]["translation_m"][:2]
        for dx, yaw in ((0.0, 0.0), (0.5, 1.57)):
            args = (str(x + dx), str(y), str(yaw), "world")
            def outcome(check, snapshot, invocation):
                try:
                    return (True, check(snapshot, config.robot_adapter, config.scenario, invocation))
                except ValueError as error:
                    return (False, str(error))
            baseline = outcome(_certified_local_map, query.fork(), Node("action", "NavigateTo", args))
            standalone = outcome(certified_local_map, query, SkillInvocation("NavigateTo", args))
            self.assertEqual(baseline[0], standalone[0])
            if not baseline[0]:
                self.assertEqual(baseline[1], standalone[1])
            np.testing.assert_array_equal(query.plant.GetPositions(query.context), positions)

    def test_stale_live_query_is_rejected_before_loading_artifacts(self):
        with self.assertRaisesRegex(ValueError, "same timestamp"):
            SceneSmithPickDomain(environment_config=None,
                observation=SimpleNamespace(time_s=2.0),
                planning_query=SimpleNamespace(state_time_s=1.0),
                calibration_path=Path("unused"), pick_home_path=Path("unused"))

    def test_runtime_and_geometry_have_no_bt_or_backend_dependency(self):
        repo = Path(__file__).resolve().parents[1]
        files = ("planner/src/tamp/scenesmith.py", "planner/src/tamp/scenesmith_online.py",
                 "planner/src/skills/runtime.py", "planner/src/skills/navigation.py")
        for relative in files:
            with self.subTest(file=relative):
                tree = ast.parse((repo / relative).read_text())
                for node in ast.walk(tree):
                    if isinstance(node, ast.ImportFrom):
                        self.assertFalse((node.module or "").startswith("planner.src.bt"))
                    if isinstance(node, ast.Attribute):
                        self.assertNotIn(node.attr, ("backend", "plant_context"))


if __name__ == "__main__":
    unittest.main()
