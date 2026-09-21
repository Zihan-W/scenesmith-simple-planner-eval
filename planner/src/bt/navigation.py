"""Generate and execute a small SceneSmith behavior tree from JSON inputs.

The tree owns task sequencing.  Leaves call the repository's public online
robot actions and Navigator; no TAMP query or task-level planner is used.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from pathlib import Path

import numpy as np

from planner.src.bt.core import (
    MdslParser as BaseMdslParser, Node, Status, TickOutcome,
    condition_names, skill_registry, skill_signatures, tick_tree, to_dict, to_mdsl, to_mermaid,
)
from simulation.src import (
    BaseVelocityAction,
    GripperAction,
    HoldAction,
    JointPositionAction,
    NavigationGoal,
    Navigator,
    Pose,
    RobotCommand,
    build_navigation_map,
    build_planning_query,
)


_SIGNATURES = skill_signatures("navigation")

class MdslParser(BaseMdslParser):
    """Navigation skill validation using the common MDSL parser."""

    def __init__(self, text):
        super().__init__(text, signatures=_SIGNATURES,
                         conditions=condition_names("navigation"), max_depth=16)


def planning_prompt(environment: Mapping, task_plan: Mapping) -> str:
    """Build the VLM prompt from the SceneSmith environment and task plan."""
    planning_environment = environment
    if environment.get("schema") == "scenesmith.bt.environment.v2":
        # Retain the complete extracted file as provenance, but send only fields
        # that can affect this BT. This avoids spending model context on every
        # joint gain and camera calibration scalar.
        planning_environment = {
            "schema": environment["schema"],
            "extraction": {
                key: environment["extraction"].get(key)
                for key in ("method", "reset_seed", "reset_time_s", "experiment_sha256")
            },
            "scene": {"dmd_path": environment["scene"]["dmd_path"]},
            "robot": {
                "name": environment["robot"]["name"],
                "arm_groups": sorted(environment["robot"]["arm_groups"]),
                "grippers": {
                    name: {
                        "minimum_width_m": gripper["minimum_width_m"],
                        "maximum_width_m": gripper["maximum_width_m"],
                    }
                    for name, gripper in environment["robot"]["grippers"].items()
                },
            },
            "base": {
                key: environment["base"].get(key)
                for key in ("mode", "frame", "pose", "blocked")
            },
            "objects": environment["objects"],
            "navigation_geometry": environment["navigation_geometry"],
            "available_skills": environment["available_skills"],
        }
    compact_environment = json.dumps(planning_environment, separators=(",", ":"))
    compact_plan = json.dumps(task_plan, separators=(",", ":"))
    actions = {name: spec for name, spec in skill_registry("navigation").items()
               if spec["class"] == "action"}
    return (
        "Generate a SceneSmith behavior tree from these authoritative inputs. "
        f"ENV={compact_environment} TASK={compact_plan} "
        "Return only strict JSON with keys MAIN_SEQUENCE and ULTIMATE_GOAL. "
        "MAIN_SEQUENCE must be exactly one MDSL sequence with these task steps in order. "
        f"Action signatures: {json.dumps(actions, separators=(',', ':'))}. "
        "All args are quoted strings. Do not emit root, TAMP, code fences, comments, "
        "retries or extra nodes."
    )


def compile_model_response(environment: Mapping, task_plan: Mapping, raw_response: str):
    """Parse and semantically validate one raw VLM response."""
    try:
        response = json.loads(raw_response)
    except json.JSONDecodeError as error:
        raise ValueError("VLM response is not strict JSON") from error
    if not isinstance(response, dict) or set(response) != {"MAIN_SEQUENCE", "ULTIMATE_GOAL"}:
        raise ValueError("VLM response must contain exactly MAIN_SEQUENCE and ULTIMATE_GOAL")
    if not isinstance(response["ULTIMATE_GOAL"], str) or not response["ULTIMATE_GOAL"].strip():
        raise ValueError("ULTIMATE_GOAL must be nonempty text")
    subtree = MdslParser(response["MAIN_SEQUENCE"]).parse()
    expected_root = generate_tree(environment, task_plan)
    expected_sequence = expected_root.children[0].children[1]
    if subtree != expected_sequence:
        raise ValueError("VLM MAIN_SEQUENCE does not preserve the validated task plan exactly")
    root = Node(
        "root",
        children=(
            Node(
                "selector",
                children=(
                    Node("condition", task_plan["success_condition"]),
                    subtree,
                ),
            ),
        ),
    )
    return response, root


def load_generated_plan(path: Path, environment: Mapping, task_plan: Mapping):
    """Reload and revalidate a recorded VLM generation artifact."""
    artifact_path = Path(path)
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))
    response, root = compile_model_response(
        environment, task_plan, artifact["raw_response"]
    )
    mdsl = to_mdsl(root)
    if artifact.get("mdsl") != mdsl or artifact.get("tree") != to_dict(root):
        raise ValueError("Recorded VLM plan does not match its recompiled BT")
    if artifact.get("mdsl_sha256") != hashlib.sha256(mdsl.encode()).hexdigest():
        raise ValueError("Recorded VLM plan has an invalid MDSL hash")
    return artifact, root


def _number(value, name):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def load_generation_inputs(environment_path: Path, task_plan_path: Path):
    """Read the environment summary and task plan used to generate the BT."""
    environment = json.loads(Path(environment_path).read_text(encoding="utf-8"))
    task_plan = json.loads(Path(task_plan_path).read_text(encoding="utf-8"))
    if environment.get("schema") not in {
        "scenesmith.bt.environment.v1",
        "scenesmith.bt.environment.v2",
    }:
        raise ValueError("Unsupported BT environment schema")
    if environment.get("schema") == "scenesmith.bt.environment.v2":
        extraction = environment.get("extraction", {})
        if extraction.get("method") != "scenesmith_runtime_reset_and_drake_proximity_geometry":
            raise ValueError("BT environment v2 must come from the runtime extractor")
        geometry = environment.get("navigation_geometry", {})
        if not geometry.get("obstacle_aabbs_xy_m"):
            raise ValueError("Extracted BT environment has no navigation geometry")
    if task_plan.get("schema") != "scenesmith.bt.task_plan.v1":
        raise ValueError("Unsupported BT task-plan schema")
    available = set(environment.get("available_skills", ()))
    missing = set(_SIGNATURES) - available
    if missing:
        raise ValueError(f"Environment is missing BT skills: {sorted(missing)}")
    if not task_plan.get("steps"):
        raise ValueError("Task plan must contain at least one step")
    return environment, task_plan


def generate_tree(environment: Mapping, task_plan: Mapping) -> Node:
    """Compile the declarative task plan into a canonical Behavior Tree."""
    available = set(environment["available_skills"])
    actions = []
    for index, step in enumerate(task_plan["steps"]):
        skill = step.get("skill")
        if skill not in available or skill not in _SIGNATURES:
            raise ValueError(f"Unsupported skill at step {index}: {skill!r}")
        if skill == "Wait":
            duration = _number(step["duration_s"], "duration_s")
            if duration < 0:
                raise ValueError("duration_s must be nonnegative")
            args = (str(duration),)
        elif skill == "NavigateTo":
            goal = step["goal"]
            xyz = tuple(_number(v, "goal_xyz_m") for v in goal["xyz_m"])
            if len(xyz) != 3 or abs(xyz[2]) > 1e-9:
                raise ValueError("Navigation goal must be a planar xyz triple")
            yaw = _number(goal["yaw_rad"], "yaw_rad")
            frame_id = str(goal.get("frame_id", "world"))
            if not frame_id:
                raise ValueError("frame_id must be nonempty")
            args = (str(xyz[0]), str(xyz[1]), str(yaw), frame_id)
        elif skill == "SetDualJointTargets":
            widths = step["gripper_widths_m"]
            hold_s = _number(step["hold_s"], "hold_s")
            if hold_s < 0:
                raise ValueError("hold_s must be nonnegative")
            args = tuple(
                str(_number(value, name))
                for name, value in (
                    ("arm_target_rad", step["arm_target_rad"]),
                    ("left_gripper_width_m", widths["left"]),
                    ("right_gripper_width_m", widths["right"]),
                    ("hold_s", hold_s),
                )
            )
        else:
            raise ValueError(f"Conditions cannot appear as task steps: {skill}")
        actions.append(Node("action", skill, args))
    condition = task_plan.get("success_condition")
    if condition != "ParkedDualTargetsReached":
        raise ValueError("Task plan must use ParkedDualTargetsReached")
    return Node(
        "root",
        children=(
            Node(
                "selector",
                children=(
                    Node("condition", condition),
                    Node("sequence", children=tuple(actions)),
                ),
            ),
        ),
    )


class GeneratedBehaviorTreePolicy:
    """Tick a generated BT while emitting at most one action per policy step."""

    required_capabilities = {
        "arms": ("left", "right"),
        "grippers": ("left", "right"),
    }

    def __init__(self, root, navigator, task, generation_metadata):
        self.root = root
        self.navigator = navigator
        self.task = task
        self.config = generation_metadata
        self._sequence_indexes = {}
        self._leaf_state = {}
        self._tree_status = Status.RUNNING
        self._last_path = ""
        self._last_reason = ""
        self._tick_count = 0

    def reset(self, observation, info):
        self.navigator.reset()
        self._sequence_indexes.clear()
        self._leaf_state.clear()
        self._tree_status = Status.RUNNING
        self._last_path = ""
        self._last_reason = ""
        self._tick_count = 0

    @property
    def stop_reason(self):
        return self._last_reason if self._tree_status is Status.FAILURE else None

    def act(self, observation):
        self._tick_count += 1
        outcome = self._tick(self.root, "0", observation)
        self._tree_status = outcome.status
        self._last_path = outcome.path
        self._last_reason = outcome.reason
        return outcome.action if outcome.action is not None else HoldAction()

    def diagnostics(self):
        return {
            "controller": "behavior_tree",
            "tree_status": self._tree_status.value,
            "active_path": self._last_path,
            "reason": self._last_reason,
            "tick_count": self._tick_count,
            "navigation_status": self.navigator.status,
            "navigation_errors": dict(getattr(self.navigator, "errors", {})),
            "mdsl_sha256": self.config["mdsl_sha256"],
            "tamp_used": False,
        }

    def behavior_tree_visualization(self):
        """Return the static tree used by the synchronized Meshcat overlay."""
        return {"title": "Navigation Behavior Tree", "tree": to_dict(self.root)}

    def _tick(self, node, path, observation):
        return tick_tree(node, path, observation, self._sequence_indexes,
                         self._tick_condition, self._tick_action)

    def _tick_condition(self, node, path, observation):
        if node.name == "ParkedDualTargetsReached":
            success = bool(observation.task.get("success", False))
            return TickOutcome(Status.SUCCESS if success else Status.FAILURE, path=path)
        return TickOutcome(Status.FAILURE, path=path, reason=f"unknown_condition:{node.name}")

    def _tick_action(self, node, path, observation):
        state = self._leaf_state.setdefault(path, {})
        if node.name == "Wait":
            state.setdefault("start_time_s", observation.time_s)
            elapsed = observation.time_s - state["start_time_s"]
            return TickOutcome(
                Status.SUCCESS if elapsed >= float(node.args[0]) - 1e-9 else Status.RUNNING,
                HoldAction() if elapsed < float(node.args[0]) - 1e-9 else None,
                path,
            )
        if node.name == "NavigateTo":
            if not state.get("started"):
                x, y, yaw = (float(value) for value in node.args[:3])
                quaternion = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
                self.navigator.set_goal(
                    NavigationGoal(Pose((x, y, 0), quaternion), node.args[3]),
                    observation,
                )
                state["started"] = True
            action = self.navigator.act(observation)
            if self.navigator.status in ("blocked", "no_path", "timeout"):
                return TickOutcome(
                    Status.FAILURE,
                    action,
                    path,
                    f"navigation_{self.navigator.status}",
                )
            if self.navigator.status == "arrived":
                owner = self.navigator.config.control_owner
                self.navigator.release()
                release = BaseVelocityAction(0, 0, control_owner=owner, release_control=True)
                return TickOutcome(Status.SUCCESS, release, path)
            return TickOutcome(Status.RUNNING, action, path)
        if node.name == "SetDualJointTargets":
            arm_target, left_width, right_width, hold_s = map(float, node.args)
            q = dict(zip(observation.robot.joint_names, observation.robot.q, strict=True))
            joint_error = max(
                abs(q[name] - value)
                for targets in self.task.joint_targets.values()
                for name, value in targets.items()
            )
            gripper_error = max(
                abs(observation.robot.gripper_widths_m[name] - width)
                for name, width in self.task.gripper_targets.items()
            )
            pose = observation.base["pose"]
            position_error = float(
                np.linalg.norm(
                    np.asarray(pose["translation_m"][:2])
                    - np.asarray(self.task.goal.pose.translation_m[:2])
                )
            )
            w, x, y, z = pose["quaternion_wxyz"]
            yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            yaw_error = abs(math.atan2(math.sin(yaw), math.cos(yaw)))
            stable = (
                position_error <= 0.03
                and yaw_error <= math.radians(3)
                and np.linalg.norm(observation.base["linear_velocity_world_m_s"][:2]) <= 0.01
                and abs(observation.base["yaw_rate_rad_s"]) <= 0.02
                and joint_error < 0.01
                and gripper_error < 0.004
            )
            if stable:
                state.setdefault("stable_since_s", observation.time_s)
            else:
                state.pop("stable_since_s", None)
            held = (
                "stable_since_s" in state
                and observation.time_s - state["stable_since_s"] >= hold_s - 1e-9
            )
            command = RobotCommand(
                arms={
                    side: JointPositionAction(tuple(targets), tuple(targets.values()))
                    for side, targets in self.task.joint_targets.items()
                },
                grippers={
                    "left": GripperAction(left_width),
                    "right": GripperAction(right_width),
                },
            )
            if any(abs(value - arm_target) > 1e-12 for targets in self.task.joint_targets.values() for value in targets.values()):
                return TickOutcome(Status.FAILURE, command, path, "task_plan_arm_target_mismatch")
            return TickOutcome(Status.SUCCESS if held else Status.RUNNING, command, path)
        return TickOutcome(Status.FAILURE, path=path, reason=f"unknown_action:{node.name}")


def make_policy(context):
    """Factory used by scene-eval with ``--trust-factories``."""
    options = context.options
    environment_path = context.repository_root / options["environment_input"]
    task_plan_path = context.repository_root / options["task_plan_input"]
    environment, task_plan = load_generation_inputs(environment_path, task_plan_path)
    generated_plan_path = context.repository_root / options["generated_plan_input"]
    generated_artifact, root = load_generated_plan(
        generated_plan_path, environment, task_plan
    )
    environment_sha256 = hashlib.sha256(environment_path.read_bytes()).hexdigest()
    generation = generated_artifact.get("generation", {})
    recorded_environment_sha256 = generation.get(
        "environment_sha256", generation.get("full_extracted_environment_sha256")
    )
    if recorded_environment_sha256 != environment_sha256:
        raise ValueError("Generated BT does not match the extracted environment hash")
    mdsl = to_mdsl(root)
    task = context.environment_config.task_factory()

    plan_goal = next(step["goal"] for step in task_plan["steps"] if step["skill"] == "NavigateTo")
    if not np.allclose(plan_goal["xyz_m"], task.goal.pose.translation_m, atol=1e-12):
        raise ValueError("Generated navigation goal does not match task evaluator")
    target_step = next(step for step in task_plan["steps"] if step["skill"] == "SetDualJointTargets")
    if target_step["gripper_widths_m"] != task.gripper_targets:
        raise ValueError("Generated gripper targets do not match task evaluator")

    config = context.environment_config
    query = build_planning_query(
        scenario=config.scenario,
        robot_adapter=config.robot_adapter,
        timing=config.timing,
    )
    if not query.check_configuration(query.configuration()).valid:
        raise ValueError("Initial navigation posture is not collision safe")
    navigation_map = build_navigation_map(
        query,
        navigation_frame=config.robot_adapter.navigation_frame_name,
        ground_body_names=config.scenario.ground_body_names,
        ground_geometries=config.scenario.ground_geometries,
    )
    metadata = {
        "kind": "generated_behavior_tree",
        "environment_input": str(environment_path),
        "task_plan_input": str(task_plan_path),
        "generated_plan_input": str(generated_plan_path),
        "environment_sha256": environment_sha256,
        "task_plan_sha256": hashlib.sha256(task_plan_path.read_bytes()).hexdigest(),
        "generated_plan_sha256": hashlib.sha256(generated_plan_path.read_bytes()).hexdigest(),
        "mdsl": mdsl,
        "mdsl_sha256": hashlib.sha256(mdsl.encode()).hexdigest(),
        "tree": to_dict(root),
        "generation": generated_artifact["generation"],
        "tamp_used": False,
    }
    return GeneratedBehaviorTreePolicy(root, Navigator(navigation_map), task, metadata)
