"""A TAMP-selected grasp must reach the standalone skill runtime unchanged."""

import dataclasses
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from planner.src.tamp.hierarchy import (
    ParameterizedSkillAction, SkillRegistry, picklift_registry,
)
from planner.src.skills.runtime import SkillInvocation
from planner.src.tamp.scenesmith_online import (
    SceneSmithSkillExecutor, parameter_consumption,
)


class BindingTest(unittest.TestCase):
    def test_pick_parameters_are_consumed_by_shared_runtime(self):
        self._check_pick_binding("PickLift", picklift_registry())

    def test_expiry_before_first_tick_returns_result_without_motion(self):
        self._check_pick_binding("PickLift", picklift_registry(), expired=True)

    def test_registered_skill_name_dispatches_without_executor_name_branch(self):
        registry = SkillRegistry((dataclasses.replace(
            picklift_registry()["PickLift"], name="ConfiguredPick"),))
        self._check_pick_binding("ConfiguredPick", registry)

    def _check_pick_binding(self, name, registry, *, expired=False):
        positions = [0.1] * 7
        checks = {"ik": {
            "staging_pose_in_target": {"arm_joint_positions": positions},
            "grasp_pose_in_target": {"arm_joint_positions": positions},
            "approach_waypoints_world": [{"joint_positions": [0.05] * 7},
                                          {"joint_positions": positions}],
            "lift_waypoints_world": {"success": True,
                                     "arm_joint_names": [f"joint_{i}" for i in range(7)],
                                     "waypoints": [{"joint_positions": positions}]},
        }}
        action = ParameterizedSkillAction(
            name, {"object": "red_cube"},
            {"grasp_lateral_offset_m": -0.00293, "checks": checks,
             "g0": {"lateral_offset_m": -0.00293, "ik": checks["ik"]["grasp_pose_in_target"]},
             "a0": checks["ik"]["staging_pose_in_target"]}, (), True,
            {"grasp_pose": "g0", "approach_pose": "a0"},
        )
        observation = SimpleNamespace(
            time_s=0.1, task={"success": True}, base={}, objects={},
            robot=SimpleNamespace(as_dict=lambda: {}),
        )
        env = SimpleNamespace(step=lambda command: (
            observation, 0.0, True, False, {"action_decision": {"accepted": True}}))
        config = SimpleNamespace(
            robot_adapter=SimpleNamespace(spec=SimpleNamespace(model_instance_name="robot")),
            scenario=SimpleNamespace(observed_bodies=[]))
        experiment = SimpleNamespace(
            environment_config=config,
            resolved_config={"user_config": {"policy_options": {}}},
        )
        captured = []
        action_results = []

        class Policy:
            stop_reason = None

            def reset(self, observation, info):
                pass

            def act(self, observation):
                return object()

            def record_action_result(self, observation, info):
                action_results.append((observation.time_s, dict(info["action_decision"])))

            def diagnostics(self):
                return {"skill_status": "SUCCESS"}

        def factory(context, invocation, *, navigation_query=None):
            self.assertEqual(invocation.name, "ExecutePickLift")
            self.assertIsNone(navigation_query)
            captured.append(context.options)
            return Policy()

        with tempfile.TemporaryDirectory() as directory:
            with patch("planner.src.tamp.scenesmith_online.make_skill_policy",
                       side_effect=factory):
                executor = SceneSmithSkillExecutor(
                    env=env, experiment=experiment,
                    repository_root=Path(directory), output_root=Path(directory),
                    observation=observation, reset_info={},
                    registry=registry,
                )
                if expired:
                    executor.set_deadline(0.0)
                    env.step = lambda command: self.fail("No command after deadline")
                result = executor.execute(action)
            if expired:
                self.assertFalse(result.success)
                self.assertEqual(result.reason, "wall_time_budget_exhausted")
                self.assertEqual(action_results, [])
                self.assertTrue((Path(directory) / "skills/skill_001/result.json").exists())
                return
            self.assertTrue(result.success)
            self.assertEqual(action_results, [(0.1, {"accepted": True})])
            self.assertTrue(result.geometry_guarantee)
            self.assertEqual(result.geometric_conditioning, {"grasp_pose": True, "approach_pose": True})
            self.assertEqual(captured[0]["expert_grasp_lateral_offset_m"], -0.00293)
            self.assertEqual(captured[0]["tamp_joint_skill_plan"]["lift_waypoints"],
                             [positions])
            self.assertEqual(captured[0]["tamp_joint_skill_plan"]["approach_waypoints"],
                             [[0.05] * 7, positions])
            invocation = json.loads((Path(directory) / "skills/skill_001/skill.json").read_text())
            self.assertEqual(invocation, {"name": "ExecutePickLift", "args": []})
            self.assertNotIn("bt_json_input", captured[0])
            self.assertFalse(list(Path(directory).rglob("skill_bt.json")))

    def test_missing_or_mismatched_assignment_cannot_claim_conditioning(self):
        spec = picklift_registry()["NavigateToPick"]
        goal = ["1", "2", "0", "world"]
        action = ParameterizedSkillAction("NavigateToPick", {"object": "cube"},
            {"b0": (1.0, 2.0, 0.0), "checks": {
                "base_xyz_yaw": [1.0, 2.0, 0.0], "navigation_goal": goal}}, (), True,
            {"base_pose": "b0"})
        leaf = SkillInvocation("NavigateTo", tuple(goal))
        self.assertEqual(parameter_consumption(action, spec, leaf, {}), {"base_pose": True})
        unbound = dataclasses.replace(action, parameter_bindings={})
        self.assertEqual(parameter_consumption(unbound, spec, leaf, {}), {"base_pose": False})
        mismatched = dataclasses.replace(action, geometric_parameters={
            **action.geometric_parameters, "b0": (9.0, 2.0, 0.0)})
        self.assertEqual(parameter_consumption(mismatched, spec, leaf, {}), {"base_pose": False})


if __name__ == "__main__":
    unittest.main()
