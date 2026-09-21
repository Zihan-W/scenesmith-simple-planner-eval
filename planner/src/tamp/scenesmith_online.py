"""SceneSmith bindings for the shared BT skill runtime and online verifier."""

from __future__ import annotations

import dataclasses
import hashlib
import json
import math
import time
import warnings
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from planner.src.bt.core import Node, SKILLS, to_dict
from planner.src.bt.runtime import make_policy, _certified_local_map
from planner.src.tamp.hierarchy import PredicateGoal, WorldState, picklift_registry
from planner.src.tamp.online import JsonlTrace, SkillExecution
from simulation.src import Pose
from simulation.src.io.experiment import FactoryContext


def _joint_pick_plan(checks):
    ik = checks.get("ik", {})
    waypoints = ik.get("lift_waypoints_world", {})
    if not waypoints.get("success"):
        raise ValueError("Grounded PickLift has no certified lift waypoints")
    plan = {
        "arm_joint_names": waypoints["arm_joint_names"],
        "staging_joint_positions": ik["staging_pose_in_target"]["arm_joint_positions"],
        "grasp_joint_positions": ik["grasp_pose_in_target"]["arm_joint_positions"],
        "lift_waypoints": [point["joint_positions"] for point in waypoints["waypoints"]],
    }
    if "approach_waypoints_world" in ik:
        plan["approach_waypoints"] = [
            point["joint_positions"] for point in ik["approach_waypoints_world"]]
    return plan


def observed_pick_success(action, task):
    """Match an already-observed task success to this exact PickLift action."""
    target = action.symbolic_args.get("object")
    return (action.skill_name == "PickLift"
            and isinstance(target, str) and bool(target)
            and task.get("success") is True
            and task.get("target") == target)


def parameter_consumption(action, spec, leaf, options):
    """Audit assignments against values actually passed to the existing runtime.

    Unknown/missing bindings are unverified, not implicitly supported. A true
    entry means conditioning is enforced, not guaranteed physical task success.
    """
    geometry = action.geometric_parameters
    bindings = action.parameter_bindings
    consumed = {name: False for name in set(spec.geometric_parameters) | set(bindings)}
    if not action.supports_geometric_conditioning:
        return consumed
    checks = geometry.get("checks", {})
    if spec.runtime_action == "NavigateTo":
        variable = bindings.get("base_pose")
        selected = geometry.get(variable) if variable else None
        checked = checks.get("base_xyz_yaw")
        consumed["base_pose"] = (
            selected is not None and checked is not None
            and tuple(selected) == tuple(checked)
            and tuple(leaf.args) == tuple(checks.get("navigation_goal", ()))
        )
    elif spec.runtime_action == "ExecutePickLift":
        ik = checks.get("ik", {})
        plan = options.get("tamp_joint_skill_plan", {})
        variable = bindings.get("grasp_pose")
        selected = geometry.get(variable) if variable else None
        consumed["grasp_pose"] = (
            isinstance(selected, dict)
            and selected.get("ik") is not None
            and selected["ik"].get("arm_joint_positions") is not None
            and selected.get("ik") == ik.get("grasp_pose_in_target")
            and selected.get("lateral_offset_m") == options.get("expert_grasp_lateral_offset_m")
            and plan.get("grasp_joint_positions") == selected["ik"].get("arm_joint_positions")
        )
        variable = bindings.get("approach_pose")
        selected = geometry.get(variable) if variable else None
        consumed["approach_pose"] = (
            isinstance(selected, dict) and selected == ik.get("staging_pose_in_target")
            and selected.get("arm_joint_positions") is not None
            and plan.get("staging_joint_positions") == selected.get("arm_joint_positions")
        )
    return consumed


def failed_pick_recovery_status(observation, navigation_valid, navigation_reason):
    """Require observed support, no target contact and the existing nav precheck.

    No verified local contact-release skill exists. Do not synthesize empty or
    navigation-ready facts when this gate fails.
    """
    task = observation.task
    fingers = tuple(task.get("finger_contacts", ()))
    blockers = []
    if not task.get("support_contact", False):
        blockers.append("target_not_observed_supported")
    if len(fingers) != 2:
        blockers.append("finger_contact_state_unavailable")
    if any(fingers) or task.get("bilateral_gripper_contact", False):
        blockers.append("target_finger_contact")
    if task.get("unexpected_target_contacts", ()):
        blockers.append("unexpected_target_contact")
    if not navigation_valid:
        blockers.append("navigation_precheck_failed")
    return {"allowed": not blockers, "blockers": blockers,
            "observation_time_s": observation.time_s,
            "support_contact": task.get("support_contact"),
            "finger_contacts": list(fingers),
            "navigation_check_reason": navigation_reason,
            "navigation_check_scope": "existing zero-travel corridor check from measured state",
            "verified_contact_release_skill_available": False}


class SceneSmithSkillExecutor:
    """Run one grounded action with the same JsonBtPolicy leaves as BT baseline."""

    def __init__(self, *, env, experiment, repository_root: Path,
                 output_root: Path, observation, reset_info,
                 max_skill_steps: int = 600,
                 model_called: bool = False, registry=None):
        if type(max_skill_steps) is not int or max_skill_steps < 1:
            raise ValueError("max_skill_steps must be a positive integer")
        self.env, self.experiment = env, experiment
        self.repo, self.output = Path(repository_root), Path(output_root)
        self.observation, self.last_info = observation, reset_info
        self.max_skill_steps = max_skill_steps
        self.model_called = model_called
        self.registry = picklift_registry() if registry is None else registry
        self.bindings = {"NavigateTo": self._navigate_binding,
                         "ExecutePickLift": self._pick_binding}
        self.skill_count = 0
        self.last_navigation_goal = None
        self.step_trace = JsonlTrace(self.output / "skill_steps.jsonl")

    def execute(self, action) -> SkillExecution:
        spec = self.registry[action.skill_name]
        if set(action.symbolic_args) != set(spec.symbolic_parameters):
            raise ValueError("Skill arguments differ from the registered contract")
        if action.supports_geometric_conditioning != spec.supports_geometric_conditioning:
            raise ValueError("Skill geometric-conditioning claim differs from its registry")
        if spec.runtime_action not in self.bindings or spec.runtime_action not in SKILLS:
            raise ValueError(f"No registered runtime binding for {action.skill_name}")
        self.skill_count += 1
        destination = self.output / "skills" / f"skill_{self.skill_count:03d}"
        destination.mkdir(parents=True)
        options = dict(self.experiment.resolved_config["user_config"]["policy_options"])
        leaf = self.bindings[spec.runtime_action](action, options)
        conditioning = parameter_consumption(action, spec, leaf, options)
        geometry_guarantee = bool(conditioning) and all(conditioning.values())
        if not geometry_guarantee:
            warnings.warn(
                f"{action.skill_name}: geometry_guarantee=false; unverified parameters: "
                f"{sorted(name for name, used in conditioning.items() if not used)}",
                RuntimeWarning, stacklevel=2,
            )
        (destination / "parameter_consumption.json").write_text(json.dumps({
            "supports_geometric_conditioning": conditioning,
            "geometry_guarantee": geometry_guarantee,
            "parameter_bindings": dict(action.parameter_bindings),
            "scope": "runtime_parameter_consumption_not_physical_success",
        }, indent=2) + "\n")
        tree = Node("root", children=(Node("sequence", children=(leaf,)),))
        bt_path = destination / "skill_bt.json"
        bt_path.write_text(json.dumps(to_dict(tree), indent=2) + "\n")
        options["bt_json_input"] = str(bt_path)
        options["tamp_generation"] = {
            "mode": "tamp", "model_called": self.model_called,
            "candidate": str(destination),
        }
        context = FactoryContext(
            self.experiment.environment_config,
            self.experiment.environment_config.robot_adapter.spec,
            options, self.repo,
        )
        if spec.runtime_action == "NavigateTo":
            # Corridor validation must start at the measured post-skill scene,
            # not the experiment's reset pose. The query owns a separate context.
            backend = self.env.backend
            query = backend.planning
            query.synchronize_state(backend.plant_context)
            try:
                policy = make_policy(context, navigation_query=query)
            finally:
                query.synchronize_state(backend.plant_context)
        else:
            policy = make_policy(context)
        policy.reset(self.observation, self.last_info)
        start = time.perf_counter()
        reason = "skill_step_limit"
        success = False
        episode_finished = False
        retryable = True
        last_rejection = {}
        for index in range(self.max_skill_steps):
            command = policy.act(self.observation)
            self.observation, _, terminated, truncated, self.last_info = self.env.step(command)
            action_result = getattr(policy, "record_action_result", None)
            if action_result is not None:
                if not callable(action_result):
                    raise TypeError("policy.record_action_result must be callable")
                action_result(self.observation, self.last_info)
            episode_finished = terminated or truncated
            diagnostics = policy.diagnostics()
            decision = self.last_info.get("action_decision", {})
            if not decision.get("accepted", True):
                last_rejection = dict(decision)
            self.step_trace({
                "skill_index": self.skill_count, "step": index,
                "skill": action.skill_name, "time_s": self.observation.time_s,
                "runtime_action": spec.runtime_action,
                "policy": diagnostics,
                "action_decision": self.last_info.get("action_decision"),
                "task": dict(self.observation.task),
                "robot": self.observation.robot.as_dict(),
                "objects": {name: item.pose.as_dict()
                            for name, item in self.observation.objects.items()},
                "base": dict(self.observation.base),
            })
            if episode_finished and observed_pick_success(action, self.observation.task):
                # The environment may finish on this step before the BT sees
                # the new observation on its next tick. Never step a finished
                # episode or predict completion from elapsed hold time.
                success, reason = True, "observed_task_success"
                break
            if diagnostics["tree_status"] == "SUCCESS":
                success, reason = True, "skill_runtime_success"
                break
            stage = (diagnostics.get("expert") or {}).get("stage")
            if (decision.get("reason") == "joint_edge_rejected"
                    and stage in ("pregrasp", "align", "verify")):
                # A rejected arm path needs fresh geometry. Closing-only
                # rejection remains under the existing adaptive gripper skill.
                reason, retryable = "skill_runtime_failed:joint_edge_rejected", False
                break
            if policy.stop_reason:
                reason = f"skill_runtime_failed:{policy.stop_reason}"
                retryable = not policy.stop_reason.startswith("planned_")
                break
            if terminated or truncated:
                reason = "episode_finished"
                break
        failure_details = {} if success else {
            "last_action_rejection": last_rejection,
            "runtime_diagnostics": diagnostics,
        }
        if not success and action.skill_name == "PickLift":
            query = self.env.backend.planning
            query.synchronize_state(self.env.backend.plant_context)
            pose = self.observation.base["pose"]
            w, x, y, z = pose["quaternion_wxyz"]
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            goal = Node("action", "NavigateTo", tuple(str(v) for v in
                        (*pose["translation_m"][:2], yaw)) + ("world",))
            valid, nav_reason = True, "valid"
            try:
                _certified_local_map(query, self.experiment.environment_config.robot_adapter,
                                     self.experiment.environment_config.scenario, goal)
            except ValueError as error:
                valid, nav_reason = False, str(error)
            finally:
                query.synchronize_state(self.env.backend.plant_context)
            failure_details["recovery_preconditions"] = failed_pick_recovery_status(
                self.observation, valid, nav_reason)
            if not failure_details["recovery_preconditions"]["allowed"]:
                retryable = False
        involved_objects = set(action.symbolic_args.values())
        aliases = {item.model_instance_name: item.observation_name
                   for item in self.experiment.environment_config.scenario.observed_bodies}
        robot_name = self.experiment.environment_config.robot_adapter.spec.model_instance_name
        pair = (last_rejection.get("edge") or {}).get("limiting_nonpenetration_pair") or {}
        for key in ("body_a", "body_b"):
            model = pair.get(key, "").split("::", 1)[0]
            if model and model != robot_name:
                involved_objects.add(aliases.get(model, model))
        result = SkillExecution(
            self.observation, success, reason, time.perf_counter() - start,
            episode_finished, retryable, failure_details, tuple(sorted(involved_objects)),
            conditioning, geometry_guarantee)
        (destination / "result.json").write_text(json.dumps({
            "success": success, "reason": reason,
            "duration_s": result.duration_s,
            "episode_finished": episode_finished,
            "retryable": retryable,
            "failure_details": failure_details,
            "involved_objects": sorted(involved_objects),
            "geometric_conditioning": conditioning,
            "geometry_guarantee": geometry_guarantee,
            "task": dict(self.observation.task),
        }, indent=2, default=str) + "\n")
        return result

    def _navigate_binding(self, action, options):
        del options
        goal = action.geometric_parameters["checks"]["navigation_goal"]
        self.last_navigation_goal = tuple(float(value) for value in goal[:3])
        return Node("action", "NavigateTo", tuple(goal))

    def _pick_binding(self, action, options):
        geometry = action.geometric_parameters
        options["expert_grasp_lateral_offset_m"] = geometry.get("grasp_lateral_offset_m", 0.0)
        options["tamp_joint_skill_plan"] = _joint_pick_plan(geometry["checks"])
        return Node("action", "ExecutePickLift")


class SceneSmithWorldObserver:
    """Convert each measured observation into symbolic facts and fresh RGBs."""

    def __init__(self, executor: SceneSmithSkillExecutor,
                 target_name: str, output_root: Path, *, scene_metadata: Path | None = None):
        self.executor = executor
        self.target_name = target_name
        self.output = Path(output_root)
        self.capture_count = 0
        task = executor.experiment.environment_config.task_factory()
        self.target_body = getattr(task, "task", task).config.target_contact_body
        supports = getattr(task, "task", task).config.support_contact_bodies
        records = (json.loads(scene_metadata.read_text())["objects"]
                   if scene_metadata is not None else ())
        by_model = {item["model_instance"]: item for item in records}
        self.object_metadata = {}
        self.object_bodies = {}
        for spec in executor.experiment.environment_config.scenario.observed_bodies:
            body = f"{spec.model_instance_name}::{spec.body_name}"
            self.object_bodies[spec.observation_name] = body
            metadata = by_model.get(spec.model_instance_name)
            self.object_metadata[spec.observation_name] = {
                "category": metadata.get("semantic_category") if metadata else None,
                "movable": not (metadata["static_in_sdf"] or metadata["fixed_by_directive"])
                if metadata else None,
                "articulated": any(joint.get("type") != "fixed" for joint in metadata["joints"])
                if metadata else None,
                "surface": True if body in supports else None,
            }

    def observe(self, observation):
        objects = {
            name: {**dataclasses.asdict(item.pose), **self.object_metadata.get(name, {})}
            for name, item in observation.objects.items()
        }
        facts = {PredicateGoal("observed", (name,)) for name in objects}
        if (not observation.task.get("bilateral_gripper_contact", False)
                and not any(observation.task.get("finger_contacts", ()))):
            facts.add(PredicateGoal("gripper_empty", ()))
        if observation.task.get("success", False):
            facts.add(PredicateGoal("holding", (self.target_name,)))
        if self.executor.last_navigation_goal is not None:
            goal_x, goal_y, goal_yaw = self.executor.last_navigation_goal
            pose = observation.base["pose"]
            x, y = pose["translation_m"][:2]
            w, qx, qy, qz = pose["quaternion_wxyz"]
            yaw = math.atan2(2 * (w * qz + qx * qy),
                             1 - 2 * (qy * qy + qz * qz))
            if (math.hypot(x - goal_x, y - goal_y) <= 0.03
                    and abs(math.atan2(math.sin(yaw - goal_yaw),
                                       math.cos(yaw - goal_yaw))) <= math.radians(3)
                    and np.linalg.norm(observation.base["linear_velocity_world_m_s"][:2]) <= 0.01
                    and abs(observation.base["yaw_rate_rad_s"]) <= 0.02
                    and observation.base["control_owner"] is None
                    and self.executor.last_info.get("action_decision", {}).get("accepted", True)):
                facts.add(PredicateGoal("at_pick_pose", (self.target_name,)))
        return WorldState(objects, frozenset(facts),
                          observation_id=f"{observation.time_s:.6f}",
                          robot={"base_pose": dict(observation.base["pose"]),
                                 "base_pose_frame": observation.base["frame"],
                                 "base_link_pose": dict(observation.base["base_link_pose"]),
                                 "base_link_frame": observation.base["base_link_frame"],
                                 "joint_names": observation.robot.joint_names,
                                 "q": observation.robot.q,
                                 "gripper_widths_m": dict(
                                     observation.robot.gripper_widths_m)})

    def geometry_state(self, observation, previous):
        state = dict(previous)
        base = observation.base["base_link_pose"]
        state["base_pose"] = Pose(tuple(base["translation_m"]),
                                  tuple(base["quaternion_wxyz"]))
        return state

    def capture_images(self, observation):
        cameras = self.executor.env.backend.capture_cameras()
        self.capture_count += 1
        folder = self.output / "observations" / f"observation_{self.capture_count:03d}"
        folder.mkdir(parents=True)
        images = []
        for name in ("head_camera", "left_wrist_camera"):
            camera = cameras.get(name)
            if camera is None or camera.rgb is None:
                continue
            if not math.isclose(camera.timestamp_s, observation.time_s, abs_tol=1e-9,
                                rel_tol=0.0):
                raise ValueError("Camera capture and world observation are not synchronized")
            path = folder / f"{name}_rgb.png"
            Image.fromarray(camera.rgb).save(path)
            image_path = path
            annotations = []
            if camera.label is not None:
                annotated = Image.fromarray(camera.rgb)
                draw = ImageDraw.Draw(annotated)
                for object_name, object_body in self.object_bodies.items():
                    if object_name not in observation.objects:
                        continue
                    model_name = object_body.split("::", 1)[0]
                    labels = [number for number, body in camera.label_names.items()
                              if body.split("::", 1)[0] == model_name]
                    if not labels:
                        continue
                    mask = np.isin(camera.label, labels)
                    ys, xs = np.where(mask)
                    if len(xs):
                        bounds = [int(xs.min()), int(ys.min()),
                                  int(xs.max()), int(ys.max())]
                        line_width = max(1, min(3, min(bounds[2] - bounds[0] + 1,
                                                       bounds[3] - bounds[1] + 1) // 8))
                        draw.rectangle(bounds, outline=(255, 220, 0), width=line_width)
                        draw.text((bounds[0], max(0, bounds[1] - 14)),
                                  object_name, fill=(255, 220, 0))
                        annotations.append({"object": object_name,
                                            "body": object_body,
                                            **self.object_metadata[object_name],
                                            "bbox_xyxy": bounds,
                                            "source": "simulator_label_mask"})
                if annotations:
                    image_path = folder / f"{name}_annotated.png"
                    annotated.save(image_path)
            (folder / f"{name}_annotations.json").write_text(
                json.dumps(annotations, indent=2) + "\n")
            images.append({"camera": name, "path": str(image_path),
                           "sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
                           "timestamp_s": camera.timestamp_s,
                           "observation_time_s": observation.time_s,
                           "annotations": annotations})
        (folder / "manifest.json").write_text(json.dumps(images, indent=2) + "\n")
        return tuple(images)
