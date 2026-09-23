import unittest
from types import SimpleNamespace

from planner.src.bt.core import Node
from planner.src.bt.runtime import JsonBtPolicy
from simulation.src import GripperAction, RobotCommand


class _CloseExpert:
    def __init__(self):
        self.config = SimpleNamespace(
            closed_width_m=0.0,
            maximum_close_steps=100,
        )
        self.stage = "close"
        self.stop_reason = None
        self.calls = 0

    def reset(self, observation, info):
        del observation, info
        self.stage = "close"
        self.stop_reason = None
        self.calls = 0

    def act(self, observation):
        del observation
        self.calls += 1
        return GripperAction(0.0)

    def diagnostics(self):
        return {"stage": self.stage, "close_steps": self.calls}


def _observation(width=0.07628, *, time_s=1.0, contacts=(False, False), bilateral=False):
    return SimpleNamespace(
        time_s=time_s,
        robot=SimpleNamespace(
            gripper_widths_m={"left": width},
            gripper_width_m=width,
        ),
        task={
            "success": False,
            "finger_contacts": tuple(contacts),
            "bilateral_gripper_contact": bilateral,
            "lift_m": 0.0,
            "support_contact": True,
            "unexpected_target_contacts": (),
            "target_translational_speed_m_s": 0.0,
            "target_rotational_speed_rad_s": 0.0,
            "held_above_threshold_s": 0.0,
        },
    )


def _decision(accepted, reason="accepted", **extra):
    return {"action_decision": {"accepted": accepted, "reason": reason, **extra}}


def _target(action):
    if not isinstance(action, RobotCommand):
        raise AssertionError(f"expected RobotCommand, got {type(action).__name__}")
    return action.grippers["left"].width_m


class PickLiftCloseTargetStateTest(unittest.TestCase):
    def setUp(self):
        root = Node("root", children=(Node("action", "ExecutePickLift"),))
        self.expert = _CloseExpert()
        task = SimpleNamespace(config=SimpleNamespace(
            required_lift_m=0.08,
            required_hold_s=3.0,
            maximum_target_translational_speed_m_s=0.03,
            maximum_target_rotational_speed_rad_s=0.3,
            maximum_allowed_contact_penetration_m=0.0001,
        ))
        metadata = {
            "mdsl_sha256": "test",
            "generation": {"mode": "reviewed_design"},
        }
        self.policy = JsonBtPolicy(root, task, self.expert, None, metadata, 0.1)
        self.initial = _observation()
        self.policy.reset(self.initial, {})

    def _closure(self):
        return self.policy.diagnostics()["closure"]

    def test_close_republishes_zero_when_measured_width_stalls(self):
        first = self.policy.act(self.initial)
        self.assertAlmostEqual(_target(first), 0.0)
        self.policy.record_action_result(self.initial, _decision(True))

        second = self.policy.act(_observation(width=0.07628, time_s=1.1))
        self.assertAlmostEqual(_target(second), 0.0)
        self.assertAlmostEqual(self._closure()["previous_accepted_target_m"], 0.0)
        self.assertEqual(self._closure()["target_mode"], "direct_expert_target")

    def test_rejected_zero_target_is_rechecked_without_backoff(self):
        first = self.policy.act(self.initial)
        self.policy.record_action_result(
            self.initial,
            _decision(False, "joint_edge_rejected", minimum_distance_m=-0.0002),
        )
        second = self.policy.act(_observation(time_s=1.1))

        self.assertAlmostEqual(_target(first), 0.0)
        self.assertAlmostEqual(_target(second), 0.0)
        closure = self._closure()
        self.assertAlmostEqual(closure["last_accepted_close_target_m"], 0.07628)
        self.assertEqual(closure["rejection_count"], 1)
        self.assertEqual(closure["last_rejection"]["reason"], "joint_edge_rejected")

    def test_precheck_or_request_does_not_count_as_final_acceptance(self):
        requested = self.policy.act(self.initial)
        self.assertAlmostEqual(_target(requested), 0.0)
        self.assertAlmostEqual(self._closure()["last_accepted_close_target_m"], 0.07628)

        self.policy.record_action_result(
            self.initial,
            _decision(False, "joint_edge_rejected", precheck_accepted=True),
        )
        self.assertAlmostEqual(self._closure()["last_accepted_close_target_m"], 0.07628)

    def test_feedback_uses_new_measured_state(self):
        self.policy.act(self.initial)
        after = _observation(width=0.0751, time_s=1.1)
        self.policy.record_action_result(after, _decision(True))
        self.assertAlmostEqual(self._closure()["measured_width_m"], 0.0751)
        self.assertTrue(self._closure()["actual_progress"])

    def test_actual_progress_prevents_transient_rejection_from_blocking(self):
        self.policy.act(self.initial)
        moving = _observation(width=0.0750, time_s=1.1)
        self.policy.record_action_result(moving, _decision(False, "joint_edge_rejected"))
        self.assertIsNone(self.policy.stop_reason)
        self.assertEqual(self._closure()["no_safe_or_actual_progress_ticks"], 0)

    def test_rejections_continue_until_expert_close_budget(self):
        for index in range(20):
            action = self.policy.act(_observation(time_s=1.0 + index * 0.1))
            self.assertAlmostEqual(_target(action), 0.0)
            self.policy.record_action_result(
                _observation(time_s=1.1 + index * 0.1),
                _decision(False, "joint_edge_rejected", rejected_edge=index),
            )

        self.assertIsNone(self.policy.stop_reason)
        closure = self._closure()
        self.assertEqual(closure["candidate_close_target_m"], 0.0)
        self.assertEqual(closure["rejection_count"], 20)
        self.assertEqual(closure["last_rejection"]["reason"], "joint_edge_rejected")
        self.assertIn("remaining_close_budget", closure)

    def test_duplicate_tick_does_not_advance_or_call_expert_again(self):
        first = self.policy.act(self.initial)
        duplicate = self.policy.act(self.initial)
        self.assertAlmostEqual(_target(first), _target(duplicate))
        self.assertEqual(self.expert.calls, 1)

    def test_reset_and_new_close_stage_do_not_leak_target(self):
        first = self.policy.act(self.initial)
        self.policy.record_action_result(self.initial, _decision(True))
        self.assertAlmostEqual(_target(first), 0.0)

        reset_observation = _observation(width=0.06, time_s=0.0)
        self.policy.reset(reset_observation, {})
        after_reset = self.policy.act(reset_observation)
        self.assertAlmostEqual(_target(after_reset), 0.0)

    def test_cancel_discards_pending_target_and_reset_starts_fresh(self):
        self.policy.act(self.initial)
        self.policy.cancel()
        self.assertIsNone(self.policy.diagnostics()["closure"])
        self.assertEqual(self.policy.stop_reason, "cancelled")

        fresh = _observation(width=0.05, time_s=0.0)
        self.policy.reset(fresh, {})
        self.assertAlmostEqual(_target(self.policy.act(fresh)), 0.0)

    def test_leaving_close_clears_pending_closure_and_does_not_fake_contact(self):
        self.policy.act(self.initial)
        self.policy.record_action_result(self.initial, _decision(True))
        self.expert.stage = "verify"
        verify = self.policy.act(_observation(bilateral=True, contacts=(True, True), time_s=1.1))
        self.assertAlmostEqual(_target(verify), 0.0)
        self.assertIsNone(self.policy.diagnostics()["closure"])
        self.assertFalse(self.initial.task["bilateral_gripper_contact"])

    def test_rejected_unsafe_targets_are_never_recorded_as_accepted(self):
        candidates = []
        for index in range(4):
            action = self.policy.act(_observation(time_s=1.0 + index * 0.1))
            candidates.append(_target(action))
            self.policy.record_action_result(
                _observation(time_s=1.1 + index * 0.1),
                _decision(False, "joint_edge_rejected"),
            )
        self.assertEqual(candidates, [0.0] * 4)
        self.assertAlmostEqual(self._closure()["last_accepted_close_target_m"], 0.07628)


if __name__ == "__main__":
    unittest.main()
