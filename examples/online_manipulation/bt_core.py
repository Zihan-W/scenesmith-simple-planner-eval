"""Shared, strict Behavior Tree syntax, serialization, and tick semantics."""

from __future__ import annotations

import dataclasses
import enum
import json
import re
from collections.abc import Callable, Mapping


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
class TickOutcome:
    status: Status
    action: object | None = None
    path: str = ""
    reason: str = ""


# One source of truth for prompt-visible skills and strict MDSL signatures.
# Action execution remains in the profile-specific policy adapter.
SKILLS = {
    "Wait": {"class": "action", "arguments": ("duration_s",),
             "description": "Hold robot commands for a finite duration. duration_s is a decimal string."},
    "ExecutePickLift": {"class": "action", "arguments": (),
                        "description": "Run the calibrated PickLift controller until the observed target is lifted and stably held, or the controller fails."},
    "PickLiftSucceeded": {"class": "condition", "arguments": (),
                          "description": "Check whether the evaluator confirms the required lift and stable hold."},
    "NavigateTo": {"class": "action", "arguments": ("x_m", "y_m", "yaw_rad", "frame_id")},
    "SetDualJointTargets": {"class": "action", "arguments": (
        "arm_target_rad", "left_gripper_width_m", "right_gripper_width_m", "hold_s")},
    "ParkedDualTargetsReached": {"class": "condition", "arguments": ()},
}
PROFILE_SKILLS = {
    "picklift": ("Wait", "ExecutePickLift", "PickLiftSucceeded"),
    "navigation": ("Wait", "NavigateTo", "SetDualJointTargets", "ParkedDualTargetsReached"),
}


def skill_registry(profile: str) -> dict:
    return {name: SKILLS[name] for name in PROFILE_SKILLS[profile]}


def skill_signatures(profile: str) -> dict[str, tuple[str, ...]]:
    return {name: tuple(spec["arguments"]) for name, spec in skill_registry(profile).items()}


def condition_names(profile: str) -> set[str]:
    return {name for name, spec in skill_registry(profile).items()
            if spec["class"] == "condition"}


_TOKEN = re.compile(
    r'\s*(?:(?P<word>[A-Za-z_][A-Za-z_0-9]*)|'
    r'(?P<string>"(?:[^"\\]|\\.)*")|(?P<punct>[{}\[\],]))'
)


class MdslParser:
    """Parse the finite SceneSmith MDSL subset against a supplied skill registry."""

    def __init__(self, text: str, *, signatures: Mapping[str, tuple[str, ...]],
                 conditions: set[str] | frozenset[str], max_depth: int = 12,
                 max_nodes: int = 64):
        if not isinstance(text, str) or not text.strip() or len(text) > 20_000:
            raise ValueError("MAIN_SEQUENCE must be nonempty MDSL")
        self.signatures = signatures
        self.conditions = conditions
        self.max_depth = max_depth
        self.max_nodes = max_nodes
        self.tokens = []
        position = 0
        while position < len(text):
            if not text[position:].strip():
                break
            match = _TOKEN.match(text, position)
            if not match:
                raise ValueError(f"Invalid MDSL near {text[position:position + 30]!r}")
            self.tokens.append((match.lastgroup, match.group(match.lastgroup)))
            position = match.end()
        self.index = 0
        self.count = 0

    def _pop(self, expected=None):
        if self.index >= len(self.tokens):
            raise ValueError("Unexpected end of MDSL")
        token = self.tokens[self.index]
        self.index += 1
        if expected is not None and token[1] != expected:
            raise ValueError(f"Expected {expected!r}, got {token[1]!r}")
        return token

    def _peek(self):
        return self.tokens[self.index][1] if self.index < len(self.tokens) else ""

    def _node(self, depth=0):
        self.count += 1
        if depth > self.max_depth or self.count > self.max_nodes:
            raise ValueError("Generated BT exceeds depth/node limit")
        kind = self._pop()[1]
        if kind in ("sequence", "selector"):
            self._pop("{")
            children = []
            while self._peek() != "}":
                children.append(self._node(depth + 1))
            self._pop("}")
            if not children:
                raise ValueError("Composite node must contain children")
            return Node(kind, children=tuple(children))
        if kind not in ("action", "condition"):
            raise ValueError(f"Unsupported BT node kind: {kind!r}")
        self._pop("[")
        token_kind, name = self._pop()
        if token_kind != "word" or name not in self.signatures:
            raise ValueError(f"Unknown BT skill: {name!r}")
        if (kind == "condition") != (name in self.conditions):
            raise ValueError(f"Wrong BT node class for skill: {name}")
        args = []
        while self._peek() == ",":
            self._pop(",")
            arg_kind, raw = self._pop()
            if arg_kind != "string":
                raise ValueError("MDSL arguments must be JSON strings")
            args.append(json.loads(raw))
        self._pop("]")
        if len(args) != len(self.signatures[name]):
            raise ValueError(f"{name} expects {len(self.signatures[name])} arguments, got {len(args)}")
        return Node(kind, name, tuple(args))

    def parse(self):
        result = self._node()
        if self.index != len(self.tokens):
            raise ValueError("MAIN_SEQUENCE must contain exactly one subtree")
        return result


def to_dict(node: Node) -> dict:
    return {"kind": node.kind, "name": node.name, "args": list(node.args),
            "children": [to_dict(child) for child in node.children]}


def to_mdsl(root: Node) -> str:
    def emit(node, depth):
        prefix = "    " * depth
        if node.kind in ("action", "condition"):
            args = "".join(", " + json.dumps(value) for value in node.args)
            return [f"{prefix}{node.kind} [{node.name}{args}]"]
        if node.kind not in ("root", "selector", "sequence"):
            raise ValueError(f"Unsupported composite: {node.kind}")
        lines = [f"{prefix}{node.kind} {{"]
        for child in node.children:
            lines.extend(emit(child, depth + 1))
        return lines + [prefix + "}"]
    return "\n".join(emit(root, 0)) + "\n"


def to_mermaid(tree: Node | dict) -> str:
    """Render either a compiled node or its validated JSON representation."""
    lines = ["flowchart TD",
             "    classDef root fill:#e2e8f0,stroke:#334155",
             "    classDef selector fill:#f3e8ff,stroke:#7e22ce",
             "    classDef sequence fill:#dbeafe,stroke:#2563eb",
             "    classDef condition fill:#ccfbf1,stroke:#0f766e",
             "    classDef action fill:#e0f2fe,stroke:#0369a1"]
    serial = 0
    def field(node, name):
        return node[name] if isinstance(node, dict) else getattr(node, name)
    def emit(node, parent=None, order=1):
        nonlocal serial
        serial += 1
        current = f"n{serial}"
        kind = field(node, "kind")
        args = field(node, "args")
        label = field(node, "name") or kind
        if args:
            label += "(" + ", ".join(args) + ")"
        label = label.replace('"', "&quot;")
        shape = f'{{"{label}"}}' if kind == "condition" else f'["{label}"]'
        lines.append(f"    {current}{shape}:::{kind}")
        if parent:
            lines.append(f"    {parent} -->|{order}| {current}")
        for index, child in enumerate(field(node, "children"), 1):
            emit(child, current, index)
    emit(tree)
    return "\n".join(lines) + "\n"


def tick_tree(node: Node, path: str, observation, sequence_indexes: dict,
              on_condition: Callable, on_action: Callable) -> TickOutcome:
    """Advance standard composites; delegate only leaf semantics to the task adapter."""
    if node.kind == "root":
        return tick_tree(node.children[0], path + ".0", observation,
                         sequence_indexes, on_condition, on_action)
    if node.kind == "selector":
        for index, child in enumerate(node.children):
            result = tick_tree(child, f"{path}.{index}", observation,
                               sequence_indexes, on_condition, on_action)
            if result.status is not Status.FAILURE:
                return result
        return TickOutcome(Status.FAILURE, path=path, reason="all_branches_failed")
    if node.kind == "sequence":
        index = sequence_indexes.get(path, 0)
        while index < len(node.children):
            result = tick_tree(node.children[index], f"{path}.{index}", observation,
                               sequence_indexes, on_condition, on_action)
            if result.status is Status.FAILURE:
                sequence_indexes[path] = 0
                return result
            if result.status is Status.RUNNING:
                sequence_indexes[path] = index
                return result
            index += 1
            sequence_indexes[path] = index
            if result.action is not None:
                status = Status.SUCCESS if index == len(node.children) else Status.RUNNING
                return dataclasses.replace(result, status=status)
        return TickOutcome(Status.SUCCESS, path=path)
    if node.kind == "condition":
        return on_condition(node, path, observation)
    if node.kind == "action":
        return on_action(node, path, observation)
    return TickOutcome(Status.FAILURE, path=path, reason=f"unknown_kind:{node.kind}")
