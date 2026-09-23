"""TAMP completion must be observed, never predicted from the next clock tick."""

import unittest
from types import SimpleNamespace

from planner.src.bt.core import Node, Status
from planner.src.bt.runtime import JsonBtPolicy
from planner.src.tamp.scenesmith_online import observed_pick_success
from simulation.src import RobotCommand


class ObservedCompletionTest(unittest.TestCase):
    def _pick_outcome(self, mode, *, succeeded=False):
        task = SimpleNamespace(config=SimpleNamespace(
            required_lift_m=0.08, required_hold_s=3.0,
            maximum_target_translational_speed_m_s=0.01,
            maximum_target_rotational_speed_rad_s=0.3))
        expert = SimpleNamespace(stage="hold", stop_reason=None,
                                 act=lambda observation: RobotCommand())
        policy = JsonBtPolicy(
            Node("root"), task, expert, None, {"generation": {"mode": mode}}, 0.1)
        observation = SimpleNamespace(task={
            "lift_m": 0.0802, "held_above_threshold_s": 2.999999999999403,
            "bilateral_gripper_contact": True, "support_contact": False,
            "unexpected_target_contacts": [],
            "target_translational_speed_m_s": 0.002,
            "target_rotational_speed_rad_s": 0.2, "success": succeeded})
        return policy._picklift(Node("action", name="ExecutePickLift"), "0", observation)

    def test_tamp_waits_for_actual_task_completion(self):
        self.assertEqual(self._pick_outcome("tamp").status, Status.RUNNING)
        self.assertEqual(self._pick_outcome("tamp", succeeded=True).status, Status.SUCCESS)

    def test_ordinary_bt_keeps_original_completion_behavior(self):
        self.assertEqual(self._pick_outcome("reviewed_design").status, Status.SUCCESS)

    def test_terminal_task_success_matches_exact_pick_action(self):
        action = SimpleNamespace(skill_name="PickLift", symbolic_args={"object": "target"})
        self.assertTrue(observed_pick_success(action, {"success": True, "target": "target"}))
        self.assertFalse(observed_pick_success(action, {"success": False, "target": "target"}))
        self.assertFalse(observed_pick_success(action, {"success": True, "target": "other"}))
        self.assertFalse(observed_pick_success(
            SimpleNamespace(skill_name="PickLift", symbolic_args={}), {"success": True}))

    def test_global_task_success_does_not_certify_navigation(self):
        action = SimpleNamespace(skill_name="NavigateToPick", symbolic_args={"object": "target"})
        self.assertFalse(observed_pick_success(action, {"success": True, "target": "target"}))


if __name__ == "__main__":
    unittest.main()
