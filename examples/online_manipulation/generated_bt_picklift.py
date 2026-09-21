"""Execute a validated model-generated PickLift Behavior Tree."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from examples.online_manipulation.bt_core import (
    MdslParser, Node, Status, TickOutcome, condition_names, skill_signatures,
    tick_tree, to_dict, to_mdsl, to_mermaid,
)
from src.online_manipulation import HoldAction
from src.online_manipulation.recipes.pick_policy import build_policy


Outcome = TickOutcome


SIGNATURES = skill_signatures("picklift")
CONDITIONS = condition_names("picklift")


class Parser(MdslParser):
    """PickLift skill validation using the common MDSL parser."""

    def __init__(self, text):
        super().__init__(text, signatures=SIGNATURES, conditions=CONDITIONS)


def canonical_tree(task_plan):
    actions = []
    for step in task_plan["steps"]:
        skill = step.get("skill")
        if skill == "Wait":
            duration = float(step["duration_s"])
            if not math.isfinite(duration) or duration < 0:
                raise ValueError("Wait duration must be finite and nonnegative")
            actions.append(Node("action", "Wait", (str(duration),)))
        elif skill == "ExecutePickLift":
            actions.append(Node("action", "ExecutePickLift"))
        else:
            raise ValueError(f"Unsupported PickLift task skill: {skill}")
    if task_plan.get("success_condition") != "PickLiftSucceeded":
        raise ValueError("PickLift task requires PickLiftSucceeded")
    return Node("root", children=(Node("selector", children=(
        Node("condition", "PickLiftSucceeded"),
        Node("sequence", children=tuple(actions)),
    )),))


def validate_inputs(environment, task_plan):
    """Validate already-loaded PickLift planning inputs."""
    if environment.get("schema") != "scenesmith.verigraph_pick.environment.v1":
        raise ValueError("PickLift environment must be VeriGraph-derived")
    verigraph = environment.get("verigraph", {})
    if verigraph.get("parser") != "verigraph.core.parse.parse_llm_response_to_graph":
        raise ValueError("Missing authoritative VeriGraph parser provenance")
    if task_plan.get("schema") != "scenesmith.bt.picklift_task_plan.v1":
        raise ValueError("Unsupported PickLift task plan schema")
    if set(SIGNATURES) - set(environment.get("available_skills", ())):
        raise ValueError("PickLift environment is missing executable BT skills")
    return environment, task_plan


def load_inputs(environment_path, task_plan_path):
    environment = json.loads(Path(environment_path).read_text())
    task_plan = json.loads(Path(task_plan_path).read_text())
    return validate_inputs(environment, task_plan)


def compile_response(environment, task_plan, raw_response):
    try:
        response = json.loads(raw_response)
    except json.JSONDecodeError as error:
        raise ValueError("Model response is not strict JSON") from error
    if not isinstance(response, dict) or set(response) != {"MAIN_SEQUENCE", "ULTIMATE_GOAL"}:
        raise ValueError("Response must contain MAIN_SEQUENCE and ULTIMATE_GOAL")
    subtree = Parser(response["MAIN_SEQUENCE"]).parse()
    expected = canonical_tree(task_plan).children[0].children[1]
    if subtree != expected:
        raise ValueError("Generated PickLift sequence changed the validated task plan")
    if not isinstance(response["ULTIMATE_GOAL"], str) or not response["ULTIMATE_GOAL"].strip():
        raise ValueError("ULTIMATE_GOAL must be nonempty")
    return response, canonical_tree(task_plan)


class PickLiftBehaviorTreePolicy:
    required_capabilities = {"arms": ("left",), "grippers": ("left",)}

    def __init__(self, root, expert, task, policy_dt, metadata):
        self.root = root
        self.expert = expert
        self.task = task
        self.policy_dt = policy_dt
        self.config = metadata
        self._sequence_indexes = {}
        self._leaf_state = {}
        self._status = Status.RUNNING
        self._path = ""
        self._reason = ""
        self._ticks = 0

    def reset(self, observation, info):
        self.expert.reset(observation, info)
        self._sequence_indexes.clear()
        self._leaf_state.clear()
        self._status = Status.RUNNING
        self._path = ""
        self._reason = ""
        self._ticks = 0

    @property
    def stop_reason(self):
        return self._reason if self._status is Status.FAILURE else None

    def diagnostics(self):
        return {"controller": "behavior_tree", "tree_status": self._status.value,
                "active_path": self._path, "reason": self._reason,
                "tick_count": self._ticks, "expert": self.expert.diagnostics(),
                "mdsl_sha256": self.config["mdsl_sha256"],
                "verigraph_metadata_sha256": self.config["environment_sha256"],
                "tamp_used": False}

    def behavior_tree_visualization(self):
        """Return the static tree used by the synchronized Meshcat overlay."""
        return {"title": "PickLift Behavior Tree", "tree": to_dict(self.root)}

    def act(self, observation):
        self._ticks += 1
        outcome = self._tick(self.root, "0", observation)
        self._status, self._path, self._reason = outcome.status, outcome.path, outcome.reason
        return outcome.action if outcome.action is not None else HoldAction()

    def _tick(self, node, path, observation):
        return tick_tree(node, path, observation, self._sequence_indexes,
                         self._tick_condition, self._tick_action)

    def _tick_condition(self, node, path, observation):
        success = bool(observation.task.get("success", False))
        return Outcome(Status.SUCCESS if success else Status.FAILURE, path=path)

    def _tick_action(self, node, path, observation):
        if node.name == "Wait":
            state = self._leaf_state.setdefault(path, {})
            state.setdefault("start", observation.time_s)
            running = observation.time_s - state["start"] < float(node.args[0]) - 1e-9
            return Outcome(Status.RUNNING if running else Status.SUCCESS,
                           HoldAction() if running else None, path)
        if node.name != "ExecutePickLift":
            return Outcome(Status.FAILURE, path=path, reason="unknown_action")
        action = self.expert.act(observation)
        if self.expert.stop_reason:
            return Outcome(Status.FAILURE, action, path, self.expert.stop_reason)
        state = observation.task
        task_config = self.task.config
        stable = (
            state.get("lift_m", 0.0) >= task_config.required_lift_m
            and state.get("bilateral_gripper_contact", False)
            and not state.get("support_contact", True)
            and not state.get("unexpected_target_contacts", ())
            and state.get("target_translational_speed_m_s", math.inf)
                <= task_config.maximum_target_translational_speed_m_s
            and state.get("target_rotational_speed_rad_s", math.inf)
                <= task_config.maximum_target_rotational_speed_rad_s
        )
        completes_next_step = (
            self.expert.stage == "hold" and stable
            and state.get("held_above_threshold_s", 0.0) + self.policy_dt
                >= task_config.required_hold_s - 1e-9
        )
        return Outcome(Status.SUCCESS if state.get("success") or completes_next_step
                       else Status.RUNNING, action, path)


def make_policy(context):
    options = context.options
    environment_path = context.repository_root / options["environment_input"]
    task_plan_path = context.repository_root / options["task_plan_input"]
    generated_path = context.repository_root / options["generated_plan_input"]
    environment, task_plan = load_inputs(environment_path, task_plan_path)
    artifact = json.loads(generated_path.read_text())
    _, root = compile_response(environment, task_plan, artifact["raw_response"])
    mdsl = to_mdsl(root)
    if artifact.get("tree") != to_dict(root) or artifact.get("mdsl") != mdsl:
        raise ValueError("Recorded PickLift plan does not match recompilation")
    if artifact.get("mdsl_sha256") != hashlib.sha256(mdsl.encode()).hexdigest():
        raise ValueError("Recorded PickLift MDSL hash is invalid")
    environment_sha = hashlib.sha256(environment_path.read_bytes()).hexdigest()
    if artifact["generation"].get("environment_sha256") != environment_sha:
        raise ValueError("Generated PickLift BT is bound to different metadata")
    expert = build_policy(
        context.environment_config,
        pick_artifact_root=context.repository_root / options["expert_inputs"],
        calibration_json=context.repository_root / options["expert_calibration"],
    )
    task = context.environment_config.task
    metadata = {"kind": "verigraph_microsoft_generated_picklift_bt",
                "environment_sha256": environment_sha,
                "generated_plan_sha256": hashlib.sha256(generated_path.read_bytes()).hexdigest(),
                "mdsl": mdsl, "mdsl_sha256": hashlib.sha256(mdsl.encode()).hexdigest(),
                "generation": artifact["generation"], "tamp_used": False}
    return PickLiftBehaviorTreePolicy(
        root, expert, task, context.environment_config.timing.policy_dt, metadata)
