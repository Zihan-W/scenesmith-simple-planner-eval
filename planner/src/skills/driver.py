"""Shared navigation and PickLift command/acceptance state machines.

Owning policies supply state storage and their explicit completion policy.
This module contains no behavior-tree interpreter.
"""
import dataclasses
import enum
import math
from collections.abc import Mapping
from simulation.src import (BaseVelocityAction, CompositeAction, GripperAction,
    HoldAction, NavigationGoal, Pose, RobotCommand)

_CLOSE_PROGRESS_M = 0.00001

class Status(enum.Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"

@dataclasses.dataclass(frozen=True)
class SkillOutcome:
    status: Status
    action: object | None = None
    path: str = ""
    reason: str = ""

class SkillDriver:
    """Shared skill transitions; subclasses choose observed/predicted completion."""
    def _pick_complete(self, observation):
        return bool(observation.task.get("success"))

    def _active_closure(self):
        for state in self._leaf_state.values():
            if "closure" in state:
                return state["closure"]
        return None


    def _clear_closure(self, path, *, archive=False):
        state = self._leaf_state.get(path)
        closure = state.pop("closure", None) if state is not None else None
        if archive and closure is not None:
            self._closure_archive = {**closure, "active": False}
        self._pending_close = None


    @staticmethod
    def _observed_gripper_width(observation):
        width = observation.robot.gripper_widths_m.get(
            "left", observation.robot.gripper_width_m)
        if width is None:
            raise ValueError("PickLift requires an observed left gripper width")
        return float(width)


    def _remaining_close_budget(self):
        diagnostics = self.expert.diagnostics()
        used = diagnostics.get("close_steps")
        if used is None and diagnostics.get("stage") == "close":
            used = diagnostics.get("stage_ticks")
        used = 0 if used is None else int(used)
        return max(0, int(self.expert.config.maximum_close_steps) - used)


    def record_action_result(self, observation, info):
        """Commit a close target only after Runtime reports final acceptance."""
        pending = self._pending_close
        if pending is None:
            return
        decision = info.get("action_decision") if isinstance(info, Mapping) else None
        if not isinstance(decision, Mapping) or not isinstance(decision.get("accepted"), bool):
            raise RuntimeError(
                "PickLift close command requires a final boolean action_decision.accepted")
        path = pending["path"]
        leaf = self._leaf_state.get(path)
        if leaf is None or "closure" not in leaf:
            self._pending_close = None
            return
        state = leaf["closure"]
        candidate = pending["candidate_target_m"]
        previous = pending["previous_accepted_target_m"]
        measured = self._observed_gripper_width(observation)
        contacts = tuple(bool(value) for value in observation.task.get("finger_contacts", ()))
        bilateral = bool(observation.task.get("bilateral_gripper_contact", False))
        actual_progress = pending["measured_width_m"] - measured >= _CLOSE_PROGRESS_M
        contact_progress = (
            bilateral and not pending["bilateral_contact"]
            or any(current and not old for current, old in zip(
                contacts, pending["finger_contacts"], strict=False))
        )
        accepted = decision["accepted"]
        target_progress = accepted and previous - candidate >= _CLOSE_PROGRESS_M

        state.update({
            "measured_width_m": measured,
            "previous_accepted_target_m": previous,
            "candidate_close_target_m": candidate,
            "candidate_close_step_m": pending["candidate_step_m"],
            "accepted": accepted,
            "target_progress": target_progress,
            "actual_progress": actual_progress,
            "contact_progress": contact_progress,
            "finger_contacts": contacts,
            "bilateral_gripper_contact": bilateral,
            "last_action_decision": dict(decision),
        })
        if accepted:
            state["last_accepted_close_target_m"] = candidate
        else:
            state["rejection_count"] += 1
            state["last_rejection"] = dict(decision)

        if target_progress or actual_progress or contact_progress:
            state["no_safe_or_actual_progress_ticks"] = 0
        else:
            state["no_safe_or_actual_progress_ticks"] += 1
        self._pending_close = None


    def _navigate(self, node, path, observation):
        if self.navigator is None:
            return SkillOutcome(Status.FAILURE, path=path, reason="navigator_unavailable")
        state = self._leaf_state.setdefault(path, {})
        if not state.get("started"):
            x, y, yaw = map(float, node.args[:3])
            quaternion = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
            self.navigator.set_goal(
                NavigationGoal(Pose((x, y, 0), quaternion), node.args[3]), observation)
            self._navigation_goal = (x, y, yaw)
            state["started"] = True
        action = self.navigator.act(observation)
        if self.navigator.status in ("blocked", "no_path", "timeout"):
            return SkillOutcome(Status.FAILURE, action, path,
                               f"navigation_{self.navigator.status}")
        if self.navigator.status == "arrived":
            owner = self.navigator.config.control_owner
            self.navigator.release()
            return SkillOutcome(Status.SUCCESS, BaseVelocityAction(
                0, 0, control_owner=owner, release_control=True), path)
        return SkillOutcome(Status.RUNNING, action, path)


    def _picklift(self, node, path, observation):
        if self.expert is None:
            return SkillOutcome(Status.FAILURE, path=path, reason="picklift_unavailable")
        if self.navigator and (self.navigator.status != "arrived" or self.navigator.owner):
            return SkillOutcome(Status.FAILURE, path=path, reason="base_not_released")
        if self._pending_close is not None:
            if self._pending_close["path"] != path:
                raise RuntimeError("Unresolved PickLift close command belongs to another leaf")
            return SkillOutcome(Status.RUNNING, self._pending_close["action"], path)

        action = self.expert.act(observation)
        # The calibrated expert uses the fixed-arm CompositeAction contract.
        # The mobile Runtime accepts the same public components as named groups.
        if isinstance(action, CompositeAction):
            action = RobotCommand(
                arms={"left": action.arm} if action.arm is not None else {},
                grippers={"left": action.gripper} if action.gripper is not None else {},
            )
        elif isinstance(action, GripperAction):
            action = RobotCommand(grippers={"left": action})
        elif not isinstance(action, (HoldAction, RobotCommand)):
            action = RobotCommand(arms={"left": action})
        # Match the direct PickLift policy: keep publishing its final close
        # target instead of replacing it with measured-width increments.
        # Runtime still validates the command from the current measured state,
        # and final acceptance is recorded separately from the request.
        if isinstance(action, RobotCommand) and "left" in action.grippers:
            gripper = action.grippers["left"]
            closing = (self.expert.stage == "close"
                       and gripper.width_m <= self.expert.config.closed_width_m + 1e-9)
            if closing:
                width = self._observed_gripper_width(observation)
                contacts = tuple(observation.task.get("finger_contacts", ()))
                bilateral = bool(observation.task.get("bilateral_gripper_contact", False))
                leaf = self._leaf_state.setdefault(path, {})
                close_state = leaf.get("closure")
                if close_state is None:
                    close_state = {
                        "active": True,
                        "target_mode": "direct_expert_target",
                        "initialized_from_measured_width_m": width,
                        "last_accepted_close_target_m": width,
                        "previous_accepted_target_m": width,
                        "candidate_close_target_m": None,
                        "candidate_close_step_m": None,
                        "measured_width_m": width,
                        "accepted": None,
                        "target_progress": False,
                        "actual_progress": False,
                        "contact_progress": False,
                        "finger_contacts": tuple(bool(value) for value in contacts),
                        "bilateral_gripper_contact": False,
                        "rejection_count": 0,
                        "no_safe_or_actual_progress_ticks": 0,
                        "close_command_count": 0,
                        "remaining_close_budget": self._remaining_close_budget(),
                        "last_rejection": None,
                        "last_action_decision": None,
                    }
                    leaf["closure"] = close_state
                    self._closure_archive = None
                previous_target = close_state["last_accepted_close_target_m"]
                close_target = gripper.width_m
                candidate_step = max(0.0, previous_target - close_target)
                close_state.update({
                    "measured_width_m": width,
                    "previous_accepted_target_m": previous_target,
                    "candidate_close_target_m": close_target,
                    "candidate_close_step_m": candidate_step,
                    "finger_contacts": tuple(bool(value) for value in contacts),
                    "bilateral_gripper_contact": bilateral,
                    "accepted": None,
                })
                close_state["close_command_count"] += 1
                close_state["remaining_close_budget"] = self._remaining_close_budget()
                action = RobotCommand(
                    arms=action.arms,
                    grippers={**action.grippers, "left": GripperAction(close_target)},
                    base=action.base,
                )
                self._pending_close = {
                    "path": path,
                    "action": action,
                    "previous_accepted_target_m": previous_target,
                    "candidate_target_m": close_target,
                    "candidate_step_m": candidate_step,
                    "measured_width_m": width,
                    "finger_contacts": tuple(bool(value) for value in contacts),
                    "bilateral_contact": bilateral,
                }
            else:
                self._clear_closure(path)
        if self.expert.stop_reason:
            self._clear_closure(path, archive=True)
            return SkillOutcome(Status.FAILURE, action, path, self.expert.stop_reason)
        return SkillOutcome(Status.SUCCESS if self._pick_complete(observation)
                            else Status.RUNNING, action, path)
