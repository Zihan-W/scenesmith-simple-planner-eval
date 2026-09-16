"""Execute a validated model-generated PickLift Behavior Tree."""

from __future__ import annotations

import dataclasses
import enum
import hashlib
import json
import math
import re
from pathlib import Path

from src.online_manipulation import HoldAction
from src.online_manipulation.recipes.pick_policy import build_policy


class Status(enum.Enum):
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


@dataclasses.dataclass(frozen=True)
class Node:
    kind: str
    name: str = ""
    args: tuple[str, ...] = ()
    children: tuple["Node", ...] = ()


@dataclasses.dataclass(frozen=True)
class Outcome:
    status: Status
    action: object | None = None
    path: str = ""
    reason: str = ""


SIGNATURES = {
    "Wait": ("duration_s",),
    "ExecutePickLift": (),
    "PickLiftSucceeded": (),
}
CONDITIONS = {"PickLiftSucceeded"}
TOKEN = re.compile(
    r'\s*(?:(?P<word>[A-Za-z_][A-Za-z_0-9]*)|'
    r'(?P<string>"(?:[^"\\]|\\.)*")|(?P<punct>[{}\[\],]))'
)


class Parser:
    """Strict MDSL parser for the finite PickLift skill registry."""

    def __init__(self, text):
        if not isinstance(text, str) or not text.strip() or len(text) > 20_000:
            raise ValueError("MAIN_SEQUENCE must be nonempty MDSL")
        self.tokens = []
        position = 0
        while position < len(text):
            if not text[position:].strip():
                break
            match = TOKEN.match(text, position)
            if not match:
                raise ValueError(f"Invalid MDSL near {text[position:position + 30]!r}")
            self.tokens.append((match.lastgroup, match.group(match.lastgroup)))
            position = match.end()
        self.index = 0

    def pop(self, expected=None):
        if self.index >= len(self.tokens):
            raise ValueError("Unexpected end of MDSL")
        token = self.tokens[self.index]
        self.index += 1
        if expected is not None and token[1] != expected:
            raise ValueError(f"Expected {expected!r}, got {token[1]!r}")
        return token

    def peek(self):
        return self.tokens[self.index][1] if self.index < len(self.tokens) else ""

    def node(self, depth=0):
        if depth > 12:
            raise ValueError("Generated BT exceeds depth limit")
        kind = self.pop()[1]
        if kind in ("sequence", "selector"):
            self.pop("{")
            children = []
            while self.peek() != "}":
                children.append(self.node(depth + 1))
            self.pop("}")
            if not children:
                raise ValueError("Composite node must have children")
            return Node(kind, children=tuple(children))
        if kind not in ("action", "condition"):
            raise ValueError(f"Unsupported BT node kind: {kind}")
        self.pop("[")
        token_kind, name = self.pop()
        if token_kind != "word" or name not in SIGNATURES:
            raise ValueError(f"Unknown PickLift skill: {name}")
        if (kind == "condition") != (name in CONDITIONS):
            raise ValueError(f"Wrong BT node class for skill: {name}")
        args = []
        while self.peek() == ",":
            self.pop(",")
            token_kind, raw = self.pop()
            if token_kind != "string":
                raise ValueError("MDSL arguments must be JSON strings")
            args.append(json.loads(raw))
        self.pop("]")
        if len(args) != len(SIGNATURES[name]):
            raise ValueError(f"{name} has the wrong argument count")
        return Node(kind, name, tuple(args))

    def parse(self):
        result = self.node()
        if self.index != len(self.tokens):
            raise ValueError("MAIN_SEQUENCE must contain one subtree")
        return result


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


def to_dict(node):
    return {"kind": node.kind, "name": node.name, "args": list(node.args),
            "children": [to_dict(child) for child in node.children]}


def to_mdsl(root):
    def emit(node, depth):
        prefix = "    " * depth
        if node.kind in ("action", "condition"):
            args = "".join(", " + json.dumps(value) for value in node.args)
            return [f"{prefix}{node.kind} [{node.name}{args}]"]
        lines = [f"{prefix}{node.kind} {{"]
        for child in node.children:
            lines.extend(emit(child, depth + 1))
        return lines + [prefix + "}"]
    return "\n".join(emit(root, 0)) + "\n"


def to_mermaid(root):
    lines = ["flowchart TD", "    classDef root fill:#e2e8f0,stroke:#334155",
             "    classDef selector fill:#f3e8ff,stroke:#7e22ce",
             "    classDef sequence fill:#dbeafe,stroke:#2563eb",
             "    classDef condition fill:#ccfbf1,stroke:#0f766e",
             "    classDef action fill:#e0f2fe,stroke:#0369a1"]
    serial = 0
    def visit(node, parent=None, order=1):
        nonlocal serial
        serial += 1
        current = f"n{serial}"
        label = node.name or node.kind
        if node.args:
            label += "(" + ", ".join(node.args) + ")"
        lines.append(f'    {current}["{label}"]:::{node.kind}')
        if parent:
            lines.append(f"    {parent} -->|{order}| {current}")
        for index, child in enumerate(node.children, 1):
            visit(child, current, index)
    visit(root)
    return "\n".join(lines) + "\n"


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
        if node.kind == "root":
            return self._tick(node.children[0], path + ".0", observation)
        if node.kind == "selector":
            for index, child in enumerate(node.children):
                result = self._tick(child, f"{path}.{index}", observation)
                if result.status is not Status.FAILURE:
                    return result
            return Outcome(Status.FAILURE, path=path, reason="all_branches_failed")
        if node.kind == "sequence":
            index = self._sequence_indexes.get(path, 0)
            while index < len(node.children):
                result = self._tick(node.children[index], f"{path}.{index}", observation)
                if result.status is Status.FAILURE:
                    self._sequence_indexes[path] = 0
                    return result
                if result.status is Status.RUNNING:
                    self._sequence_indexes[path] = index
                    return result
                index += 1
                self._sequence_indexes[path] = index
                if result.action is not None:
                    return dataclasses.replace(result, status=(
                        Status.SUCCESS if index == len(node.children) else Status.RUNNING))
            return Outcome(Status.SUCCESS, path=path)
        if node.kind == "condition":
            success = bool(observation.task.get("success", False))
            return Outcome(Status.SUCCESS if success else Status.FAILURE, path=path)
        return self._tick_action(node, path, observation)

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
