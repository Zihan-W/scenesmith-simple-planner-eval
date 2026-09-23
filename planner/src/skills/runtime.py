"""Execute one grounded skill without a behavior tree or tree interpreter."""

from __future__ import annotations

import dataclasses
import enum
import json
import math
from collections.abc import Mapping

from planner.src.skills.driver import SkillDriver, SkillOutcome, Status
from planner.src.skills.navigation import certified_local_map
from planner.src.skills.picklift import JointWaypointPickLiftSkill
from simulation.src import (
    BaseVelocityAction, CompositeAction, GripperAction, HoldAction,
    NavigationConfig, NavigationGoal, Navigator, Pose, RobotCommand,
)
from simulation.src.robots.adapters.zerith import ZerithEnvironmentConfig
from simulation.src.recipes.pick_policy import build_policy


_CLOSE_PROGRESS_M = 0.00001


@dataclasses.dataclass(frozen=True)
class SkillInvocation:
    """Runtime skill name and ordered string arguments; contains no control flow."""

    name: str
    args: tuple[str, ...] = ()

    def __post_init__(self):
        signatures = {"NavigateTo": 4, "ExecutePickLift": 0}
        if self.name not in signatures or len(self.args) != signatures[self.name]:
            raise ValueError(f"Invalid skill invocation: {self.name}")
        if any(not isinstance(value, str) for value in self.args):
            raise ValueError("Skill arguments must be strings")


class SingleSkillPolicy(SkillDriver):
    """Adapt the existing controllers to public environment actions and feedback.

    Retains the established close-command acceptance tracking. Completion of a
    pick requires observed task success, never prediction of the next time step.
    """

    required_capabilities = {"arms": ("left",), "grippers": ("left",)}

    def __init__(self, invocation, task, expert, navigator, metadata, policy_dt):
        self.invocation = invocation
        self.task, self.expert, self.navigator = task, expert, navigator
        self.config, self.policy_dt = metadata, policy_dt
        self._leaf_state = {}
        self._status = Status.RUNNING
        self._path = self._reason = ""
        self._ticks = 0
        self._navigation_goal = None
        self._pending_close = None
        self._closure_archive = None
        self.actions = {"NavigateTo": self._navigate, "ExecutePickLift": self._picklift}

    def act(self, observation):
        self._ticks += 1
        outcome = self.actions[self.invocation.name](self.invocation, "skill", observation)
        self._status, self._path, self._reason = outcome.status, outcome.path, outcome.reason
        return outcome.action if outcome.action is not None else HoldAction()

    def reset(self, observation, info):
        if self.expert:
            self.expert.reset(observation, info)
        if self.navigator:
            self.navigator.reset()
        self._leaf_state.clear()
        self._status = Status.RUNNING
        self._path = self._reason = ""
        self._ticks = 0
        self._navigation_goal = None
        self._pending_close = None
        self._closure_archive = None

    def cancel(self):
        """Discard command-tracking state when an owning executor cancels."""
        self._pending_close = None
        self._closure_archive = None
        for state in self._leaf_state.values():
            state.pop("closure", None)
        self._status = Status.FAILURE
        self._reason = "cancelled"

    @property
    def stop_reason(self):
        return self._reason if self._status is Status.FAILURE else None

    def diagnostics(self):
        closure = self._active_closure()
        if closure is not None:
            closure = dict(closure)
        elif self._closure_archive is not None:
            closure = dict(self._closure_archive)
        return {"controller": "single_skill", "skill_status": self._status.value,
                "active_path": self._path, "reason": self._reason, "tick_count": self._ticks,
                "navigation_status": self.navigator.status if self.navigator else None,
                "expert": self.expert.diagnostics() if self.expert else None,
                "closure": closure,
                "skill": self.invocation.name, "generation": dict(self.config)}









def make_skill_policy(context, invocation, *, navigation_query=None):
    """Bind one certified TAMP skill to existing navigation/pick controllers."""
    options, repo = context.options, context.repository_root
    config = context.environment_config
    task = config.task_factory()
    task = getattr(task, "task", task)
    navigator = None
    if invocation.name == "NavigateTo":
        query = navigation_query
        if query is None:
            raise ValueError("NavigateTo requires a synchronized public planning query")
        navigation_map = certified_local_map(
            query, config.robot_adapter, config.scenario, invocation)
        position_tolerance = float(options.get(
            "navigation_position_tolerance_m", 0.03))
        if not 0.002 <= position_tolerance <= 0.03:
            raise ValueError("Navigation position tolerance must be 2-30 mm")
        navigator = Navigator(navigation_map, NavigationConfig(
            position_tolerance_m=position_tolerance))
    expert = None
    if invocation.name == "ExecutePickLift":
        if options.get("tamp_joint_skill_plan") is None:
            raise ValueError("ExecutePickLift requires a grounded joint skill plan")
        spec = context.robot_spec
        calibration_path = repo / options["expert_calibration"]
        calibration = json.loads(calibration_path.read_text())
        lateral_offset = float(options.get("expert_grasp_lateral_offset_m", 0.0))
        if not math.isfinite(lateral_offset) or abs(lateral_offset) > 0.02:
            raise ValueError("Expert grasp lateral offset must be within 20 mm")
        relative = calibration["grasp_pose_in_target"]
        relative_translation = tuple(relative["translation_m"])
        grasp_pose = Pose((relative_translation[0],
                           relative_translation[1] + lateral_offset,
                           relative_translation[2]),
                          tuple(relative["quaternion_wxyz"]))
        calibrated = ZerithEnvironmentConfig(
            scenario=config.scenario, robot_model_dir=spec.model_path.parent.parent,
            robot_xyz=tuple(options["calibrated_base_xyz_m"]),
            robot_yaw_deg=float(options["calibrated_base_yaw_deg"]),
            rail_position=float(calibration["rail_position_m"]),
            q_home_left=tuple(spec.home_positions[:7]), task=task,
            cameras=spec.cameras)
        policy_overrides = dict(options.get("expert_policy_overrides") or {})
        policy_overrides["grasp_pose_in_target"] = grasp_pose
        expert = build_policy(
            calibrated, pick_artifact_root=repo / options["expert_inputs"],
            calibration_json=calibration_path,
            policy_overrides=policy_overrides,
        )
        joint_limits = {joint.name: min(
            config.maximum_joint_delta, joint.velocity_limit * config.timing.policy_dt)
            for joint in spec.controlled_joints if joint.name in spec.arm_groups["left"]}
        expert = JointWaypointPickLiftSkill(
            expert.config, options["tamp_joint_skill_plan"],
            joint_step_limits=joint_limits)
    metadata = {"mode": "tamp", **options.get("tamp_generation", {})}
    return SingleSkillPolicy(invocation, task, expert, navigator, metadata,
                             config.timing.policy_dt)
