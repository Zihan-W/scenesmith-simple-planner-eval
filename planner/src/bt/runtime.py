"""Execute validated BT JSON by dispatching leaves to existing robot interfaces."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from pathlib import Path

import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw

from planner.src.bt.core import Node, SKILLS, Status, TickOutcome, tick_tree, to_dict, to_mdsl
from planner.src.skills.driver import SkillDriver
from planner.src.skills.navigation import certified_local_map
from planner.src.skills.picklift import JointWaypointPickLiftSkill
from simulation.src import (
    BaseVelocityAction, CompositeAction, GripperAction, HoldAction,
    NavigationConfig, NavigationGoal, Navigator, Pose, RobotCommand,
    StaticNavigationMap, build_planning_query,
)
from simulation.src.robots.adapters.zerith import ZerithEnvironmentConfig
from simulation.src.recipes.pick_policy import build_policy
from simulation.src.geometry.scene_geometry import resolve_ground_geometries


# A condition is a reusable interface to the navigator, not a new tree runtime.
_EXTRA_CONDITIONS = {"BaseParkedAtPickPose": ()}


_CLOSE_PROGRESS_M = 0.00001


def load_tree(path: Path) -> Node:
    """Reject malformed trees and skills without a registered runtime handler."""
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    count = 0

    def parse(value, depth=0):
        nonlocal count
        count += 1
        if depth > 16 or count > 64:
            raise ValueError("BT exceeds depth/node limit")
        if not isinstance(value, dict) or set(value) != {"kind", "name", "args", "children"}:
            raise ValueError("BT node must contain kind, name, args, children")
        kind, name, args, children = (value[key] for key in
                                      ("kind", "name", "args", "children"))
        if kind not in ("root", "selector", "sequence", "action", "condition"):
            raise ValueError(f"Unsupported BT node kind: {kind}")
        if not isinstance(name, str) or not isinstance(args, list) or not isinstance(children, list):
            raise ValueError("Invalid BT node fields")
        if kind in ("action", "condition"):
            spec = SKILLS.get(name)
            signature = _EXTRA_CONDITIONS.get(name)
            if spec is not None:
                signature = spec["arguments"]
            if signature is None or (spec and spec["class"] != kind):
                raise ValueError(f"Unregistered {kind} skill: {name}")
            if len(args) != len(signature) or children or any(not isinstance(a, str) for a in args):
                raise ValueError(f"Invalid {name} arguments or children")
        elif name or args or not children:
            raise ValueError("Composite BT node must have children and no name/args")
        parsed = tuple(parse(child, depth + 1) for child in children)
        if kind == "root" and (depth != 0 or len(parsed) != 1):
            raise ValueError("BT root must contain one child")
        return Node(kind, name, tuple(args), parsed)

    root = parse(document)
    if root.kind != "root":
        raise ValueError("BT must start at root")
    return root


def _leaves(node):
    if node.kind in ("action", "condition"):
        yield node
    for child in node.children:
        yield from _leaves(child)


_certified_local_map = certified_local_map


class JsonBtPolicy(SkillDriver):
    """One BT interpreter; each skill is bound once to a public robot API."""

    required_capabilities = {"arms": ("left",), "grippers": ("left",)}

    def __init__(self, root, task, expert, navigator, metadata, policy_dt):
        self.root, self.task, self.expert, self.navigator = root, task, expert, navigator
        self.config, self.policy_dt = metadata, policy_dt
        self._sequence_indexes = {}
        self._leaf_state = {}
        self._status = Status.RUNNING
        self._path = self._reason = ""
        self._ticks = 0
        self._navigation_goal = None
        self._pending_close = None
        self._closure_archive = None
        self.actions = {"Wait": self._wait, "NavigateTo": self._navigate,
                        "ExecutePickLift": self._picklift}
        self.conditions = {"PickLiftSucceeded": self._task_succeeded,
                           "BaseParkedAtPickPose": self._base_parked}
        missing = {node.name for node in _leaves(root) if
                   node.name not in (self.actions if node.kind == "action" else self.conditions)}
        if missing:
            raise ValueError(f"No robot interface registered for skills: {sorted(missing)}")

    def reset(self, observation, info):
        if self.expert:
            self.expert.reset(observation, info)
        if self.navigator:
            self.navigator.reset()
        self._sequence_indexes.clear()
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

    def act(self, observation):
        self._ticks += 1
        outcome = tick_tree(self.root, "0", observation, self._sequence_indexes,
                            self._condition, self._action)
        self._status, self._path, self._reason = outcome.status, outcome.path, outcome.reason
        return outcome.action if outcome.action is not None else HoldAction()

    def diagnostics(self):
        closure = self._active_closure()
        if closure is not None:
            closure = dict(closure)
        elif self._closure_archive is not None:
            closure = dict(self._closure_archive)
        return {"controller": "json_behavior_tree", "tree_status": self._status.value,
                "active_path": self._path, "reason": self._reason, "tick_count": self._ticks,
                "navigation_status": self.navigator.status if self.navigator else None,
                "expert": self.expert.diagnostics() if self.expert else None,
                "closure": closure,
                "mdsl_sha256": self.config["mdsl_sha256"],
                "tamp_used": self.config["generation"]["mode"] == "tamp"}






    def behavior_tree_visualization(self):
        return {"title": "JSON Behavior Tree", "tree": to_dict(self.root)}

    def _condition(self, node, path, observation):
        return self.conditions[node.name](node, path, observation)

    def _action(self, node, path, observation):
        return self.actions[node.name](node, path, observation)

    def _task_succeeded(self, node, path, observation):
        return TickOutcome(Status.SUCCESS if observation.task.get("success") else Status.FAILURE,
                           path=path)

    def _base_parked(self, node, path, observation):
        if not self.navigator or not self._navigation_goal:
            return TickOutcome(Status.FAILURE, path=path, reason="no_navigation_goal")
        x, y, yaw = self._navigation_goal
        pose = observation.base["pose"]
        px, py = pose["translation_m"][:2]
        w, qx, qy, qz = pose["quaternion_wxyz"]
        actual_yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        parked = (self.navigator.status == "arrived" and not self.navigator.owner
                  and math.hypot(px - x, py - y) <= 0.03
                  and abs(math.atan2(math.sin(actual_yaw - yaw), math.cos(actual_yaw - yaw))) <= math.radians(3)
                  and np.linalg.norm(observation.base["linear_velocity_world_m_s"][:2]) <= 0.01
                  and abs(observation.base["yaw_rate_rad_s"]) <= 0.02)
        return TickOutcome(Status.SUCCESS if parked else Status.FAILURE, path=path,
                           reason="" if parked else "base_not_parked")

    def _wait(self, node, path, observation):
        seconds = float(node.args[0])
        if not math.isfinite(seconds) or seconds < 0:
            return TickOutcome(Status.FAILURE, path=path, reason="invalid_wait_duration")
        state = self._leaf_state.setdefault(path, {})
        state.setdefault("started_s", observation.time_s)
        running = observation.time_s - state["started_s"] < seconds - 1e-9
        return TickOutcome(Status.RUNNING if running else Status.SUCCESS,
                           HoldAction() if running else None, path)



    def _navigate(self, node, path, observation):
        outcome = super()._navigate(node, path, observation)
        return TickOutcome(Status(outcome.status.value), outcome.action,
                           outcome.path, outcome.reason)

    def _picklift(self, node, path, observation):
        outcome = super()._picklift(node, path, observation)
        return TickOutcome(Status(outcome.status.value), outcome.action,
                           outcome.path, outcome.reason)

    def _pick_complete(self, observation):
        if observation.task.get("success"):
            return True
        if self.config["generation"]["mode"] == "tamp":
            return False
        state, config = observation.task, self.task.config
        stable = (state.get("lift_m", 0.0) >= config.required_lift_m
                  and state.get("bilateral_gripper_contact", False)
                  and not state.get("support_contact", True)
                  and not state.get("unexpected_target_contacts", ())
                  and state.get("target_translational_speed_m_s", math.inf)
                      <= config.maximum_target_translational_speed_m_s
                  and state.get("target_rotational_speed_rad_s", math.inf)
                      <= config.maximum_target_rotational_speed_rad_s)
        return (self.expert.stage == "hold" and stable
                and state.get("held_above_threshold_s", 0.0) + self.policy_dt
                    >= config.required_hold_s - 1e-9)


def make_policy(context, *, navigation_query=None):
    """Build one runtime from the BT JSON and the skills it actually contains."""
    options = context.options
    repo = context.repository_root
    tree_path = repo / options["bt_json_input"]
    root = load_tree(tree_path)
    leaves = list(_leaves(root))
    names = {node.name for node in leaves}
    config = context.environment_config
    task = config.task_factory()
    task = getattr(task, "task", task)
    navigator = None
    if "NavigateTo" in names:
        goals = [node for node in leaves if node.name == "NavigateTo"]
        if len(goals) != 1:
            raise ValueError("Local corridor currently accepts one NavigateTo goal")
        query = navigation_query
        if query is None:
            query = build_planning_query(
                scenario=config.scenario, robot_adapter=config.robot_adapter,
                timing=config.timing)
        navigation_map = _certified_local_map(
            query, config.robot_adapter, config.scenario, goals[0])
        position_tolerance = float(options.get(
            "navigation_position_tolerance_m", 0.03))
        if not 0.002 <= position_tolerance <= 0.03:
            raise ValueError("Navigation position tolerance must be 2-30 mm")
        navigator = Navigator(navigation_map, NavigationConfig(
            position_tolerance_m=position_tolerance))
    expert = None
    if "ExecutePickLift" in names:
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
        if options.get("tamp_joint_skill_plan") is not None:
            joint_limits = {joint.name: min(
                config.maximum_joint_delta, joint.velocity_limit * config.timing.policy_dt)
                for joint in spec.controlled_joints if joint.name in spec.arm_groups["left"]}
            expert = JointWaypointPickLiftSkill(
                expert.config, options["tamp_joint_skill_plan"],
                joint_step_limits=joint_limits)
    mdsl = to_mdsl(root)
    generation = options.get("tamp_generation") or {
        "mode": "reviewed_design", "model_called": False}
    if generation.get("mode") not in ("tamp", "reviewed_design"):
        raise ValueError("Unsupported BT generation provenance")
    metadata = {"kind": "json_behavior_tree", "bt_json_sha256": hashlib.sha256(
        tree_path.read_bytes()).hexdigest(), "mdsl_sha256": hashlib.sha256(
        mdsl.encode()).hexdigest(), "mdsl": mdsl, "tree": to_dict(root),
        "generation": generation}
    return JsonBtPolicy(root, task, expert, navigator, metadata, config.timing.policy_dt)
