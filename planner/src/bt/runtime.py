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


def _certified_local_map(query, adapter, scenario, navigate: Node):
    """Check translation and the differential-drive turning envelope.

    Sample all headings between the initial, forward/reverse travel, and final
    headings along the corridor. This conservatively covers the controller's
    blended turning; it remains a geometric precheck, not a dynamics proof.
    """
    plant, context = query.plant, query.context
    instance = query.robot_model_instance
    body = plant.GetBodyByName(adapter.spec.base_link_name, instance)
    base_frame = plant.GetFrameByName(adapter.spec.base_link_name, instance)
    navigation_frame = plant.GetFrameByName(adapter.navigation_frame_name, instance)
    X_BN = base_frame.CalcPoseInWorld(context).inverse() @ navigation_frame.CalcPoseInWorld(context)
    x, y, yaw = map(float, navigate.args[:3])
    if navigate.args[3] != "world" or not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError("NavigateTo must have a finite world-frame goal")
    X_WB_start = base_frame.CalcPoseInWorld(context)
    X_WN_start = navigation_frame.CalcPoseInWorld(context)
    start_xy = X_WN_start.translation()[:2]
    start_yaw = X_WN_start.rotation().ToRollPitchYaw().yaw_angle()
    delta = np.array([x, y]) - start_xy
    wrap = lambda angle: math.atan2(math.sin(angle), math.cos(angle))
    heading = math.atan2(delta[1], delta[0]) if np.linalg.norm(delta) > 1e-9 else start_yaw
    if abs(wrap(heading - start_yaw)) > math.pi / 2:
        heading += math.pi  # Match Navigator's reverse-drive choice.
    travel_yaw = start_yaw + wrap(heading - start_yaw)
    end_yaw = travel_yaw + wrap(yaw - travel_yaw)
    low, high = min(start_yaw, travel_yaw, end_yaw), max(start_yaw, travel_yaw, end_yaw)
    headings = np.linspace(low, high, max(2, math.ceil((high - low) / math.radians(2)) + 1))
    count = max(31, math.ceil(np.linalg.norm(delta) / 0.01) + 1)
    poses = [(0.0, start_yaw, X_WB_start)]
    for alpha in np.linspace(0, 1, count):
        xy = start_xy + alpha * delta
        for sample_yaw in headings:
            X_WN = RigidTransform(RollPitchYaw(0, 0, sample_yaw), [*xy, 0])
            poses.append((alpha, sample_yaw, X_WN @ X_BN.inverse()))
    floor_ids = resolve_ground_geometries(
        plant, plant.get_geometry_query_input_port().Eval(context).inspector(),
        scenario.ground_body_names, scenario.ground_geometries,
    )
    for alpha, sample_yaw, pose in poses:
        plant.SetFreeBodyPose(context, body, pose)
        geometry_query = plant.get_geometry_query_input_port().Eval(context)
        inspector = geometry_query.inspector()
        for pair in geometry_query.ComputeSignedDistancePairwiseClosestPoints(0.01):
            if pair.id_A in floor_ids or pair.id_B in floor_ids:
                continue
            a = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_A))
            b = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_B))
            a_robot, b_robot = a.model_instance() == instance, b.model_instance() == instance
            if not (a_robot or b_robot):
                continue
            if a_robot and b_robot and {a.name(), b.name()} == {"neck_yaw_link", "neck_camera_link"}:
                continue  # fixed, measured CAD overlap; present before motion
            required = 0.005 if a_robot != b_robot else 0.0
            if pair.distance < required:
                raise ValueError(
                    f"Navigation corridor collision at alpha={alpha:.2f}, yaw={sample_yaw:.4f}: "
                    f"{a.name()} / {b.name()} = {pair.distance:.4f} m")
    bounds = (-4.5, -4.5, 4.5, 4.5)
    return StaticNavigationMap(obstacles=(), robot_radius_m=0.0, bounds=bounds,
                               resolution_m=0.05)


class JsonBtPolicy:
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
        if self.navigator is None:
            return TickOutcome(Status.FAILURE, path=path, reason="navigator_unavailable")
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
            return TickOutcome(Status.FAILURE, action, path,
                               f"navigation_{self.navigator.status}")
        if self.navigator.status == "arrived":
            owner = self.navigator.config.control_owner
            self.navigator.release()
            return TickOutcome(Status.SUCCESS, BaseVelocityAction(
                0, 0, control_owner=owner, release_control=True), path)
        return TickOutcome(Status.RUNNING, action, path)

    def _picklift(self, node, path, observation):
        if self.expert is None:
            return TickOutcome(Status.FAILURE, path=path, reason="picklift_unavailable")
        if self.navigator and (self.navigator.status != "arrived" or self.navigator.owner):
            return TickOutcome(Status.FAILURE, path=path, reason="base_not_released")
        if self._pending_close is not None:
            if self._pending_close["path"] != path:
                raise RuntimeError("Unresolved PickLift close command belongs to another leaf")
            return TickOutcome(Status.RUNNING, self._pending_close["action"], path)

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
            return TickOutcome(Status.FAILURE, action, path, self.expert.stop_reason)
        state = observation.task
        task_config = self.task.config
        stable = (state.get("lift_m", 0.0) >= task_config.required_lift_m
                  and state.get("bilateral_gripper_contact", False)
                  and not state.get("support_contact", True)
                  and not state.get("unexpected_target_contacts", ())
                  and state.get("target_translational_speed_m_s", math.inf)
                      <= task_config.maximum_target_translational_speed_m_s
                  and state.get("target_rotational_speed_rad_s", math.inf)
                      <= task_config.maximum_target_rotational_speed_rad_s)
        completes_next = (self.expert.stage == "hold" and stable
                          and state.get("held_above_threshold_s", 0.0) + self.policy_dt
                              >= task_config.required_hold_s - 1e-9)
        if self.config["generation"]["mode"] == "tamp":
            # Online verification consumes the post-step task observation. A
            # predicted completion can still be short of the task timer (or
            # lose stability on the next tick), causing an unnecessary regrasp.
            # Keep ordinary BT baseline completion semantics unchanged.
            completes_next = False
        return TickOutcome(Status.SUCCESS if state.get("success") or completes_next
                           else Status.RUNNING, action, path)


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
