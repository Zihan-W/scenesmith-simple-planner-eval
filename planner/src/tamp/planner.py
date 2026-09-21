"""VLM subgoals plus bounded constraint grounding for parameterized robot skills.

The VLM proposes *which* skills and objects matter.  A SceneSmith domain owns
continuous sampling, collision/IK checks, state prediction and execution.  No
model-generated Python is evaluated, and an infeasible proposal cannot become
an executable BT.
"""

from __future__ import annotations

import copy
import dataclasses
import json
from collections import Counter
from collections.abc import Iterable, Mapping
from typing import Any, Protocol

from planner.src.tamp.model import (
    PlannerModelError, goal_response_format, json_response_format, load_prompt,
    validated_completion,
)


PROPOSAL_SCHEMA = "scenesmith.tamp.subgoals.v1"
GOAL_SCHEMA = "scenesmith.tamp.goals.v2"


@dataclasses.dataclass(frozen=True)
class Subgoal:
    skill: str
    arguments: Mapping[str, Any]


@dataclasses.dataclass(frozen=True)
class GoalPredicate:
    name: str
    target: str


@dataclasses.dataclass(frozen=True)
class SymbolicOperator:
    """A lifted skill with preconditions and effects over one target object."""

    skill: str
    preconditions: frozenset[str]
    add_effects: frozenset[str]
    delete_effects: frozenset[str] = frozenset()


def parse_goal_proposal(content: str, allowed_predicates: frozenset[str],
                        *, max_goals: int = 12) -> tuple[GoalPredicate, ...]:
    document = json.loads(content)
    if not isinstance(document, dict) or set(document) != {"schema", "subgoals"}:
        raise ValueError("Goal proposal must contain schema and subgoals only")
    if document["schema"] != GOAL_SCHEMA:
        raise ValueError("Unsupported goal proposal schema")
    values = document["subgoals"]
    if not isinstance(values, list) or not 1 <= len(values) <= max_goals:
        raise ValueError("Goal proposal exceeds the subgoal limit")
    goals = []
    for index, value in enumerate(values):
        if not isinstance(value, dict) or set(value) != {"predicate", "target"}:
            raise ValueError(f"subgoals[{index}] has invalid shape")
        if value["predicate"] not in allowed_predicates or not isinstance(
            value["target"], str
        ) or not value["target"]:
            raise ValueError(f"subgoals[{index}] names an unknown predicate or target")
        goals.append(GoalPredicate(value["predicate"], value["target"]))
    return tuple(goals)


def refine_goals(goals: tuple[GoalPredicate, ...],
                 operators: tuple[SymbolicOperator, ...],
                 initial_facts: frozenset[GoalPredicate],
                 *, max_actions: int = 20) -> tuple[Subgoal, ...]:
    """Refine VLM predicates to a skill sequence with bounded state search."""
    from collections import deque

    facts = initial_facts
    plan: tuple[Subgoal, ...] = ()
    for goal in goals:
        if goal in facts:
            continue
        queue = deque([(facts, ())])
        seen = {facts}
        found = None
        while queue:
            current, actions = queue.popleft()
            if goal in current:
                found = (current, actions)
                break
            if len(plan) + len(actions) >= max_actions:
                continue
            for operator in operators:
                target = goal.target
                required = {GoalPredicate(name, target)
                            for name in operator.preconditions}
                if not required.issubset(current):
                    continue
                next_facts = frozenset(
                    (current - {GoalPredicate(name, target)
                                for name in operator.delete_effects})
                    | {GoalPredicate(name, target) for name in operator.add_effects}
                )
                if next_facts in seen:
                    continue
                seen.add(next_facts)
                queue.append((next_facts, actions +
                              (Subgoal(operator.skill, {"target": target}),)))
        if found is None:
            raise ValueError(f"No symbolic refinement for {goal.name}({goal.target})")
        facts, actions = found
        plan += actions
    return plan


@dataclasses.dataclass(frozen=True)
class GroundedSkill:
    skill: str
    arguments: Mapping[str, Any]
    parameters: Mapping[str, Any]
    checks: Mapping[str, Any]


@dataclasses.dataclass(frozen=True)
class GroundingResult:
    success: bool
    steps: tuple[GroundedSkill, ...]
    failures: tuple[dict, ...]
    candidates_checked: int


@dataclasses.dataclass(frozen=True)
class PlanningResult:
    success: bool
    steps: tuple[GroundedSkill, ...]
    proposals: tuple[dict, ...]
    reason: str


class Proposer(Protocol):
    def propose(self, *, goal: str, world: Mapping[str, Any],
                feedback: tuple[dict, ...]) -> tuple[Subgoal, ...]: ...


class SkillDomain(Protocol):
    """A domain provides PRoC3S-style samples and physical constraints."""

    @property
    def skill_names(self) -> frozenset[str]: ...

    def samples(self, subgoal: Subgoal, state: Any) -> Iterable[Mapping[str, Any]]: ...

    def check(self, subgoal: Subgoal, parameters: Mapping[str, Any],
              state: Any) -> tuple[bool, str, Mapping[str, Any]]: ...

    def predict(self, subgoal: Subgoal, parameters: Mapping[str, Any],
                state: Any) -> Any: ...


def parse_proposal(content: str, allowed_skills: frozenset[str],
                   *, max_steps: int = 12) -> tuple[Subgoal, ...]:
    """Parse a bounded, declarative plan; reject executable model output."""
    document = json.loads(content)
    if not isinstance(document, dict) or set(document) != {"schema", "subgoals"}:
        raise ValueError("VLM proposal must contain schema and subgoals only")
    if document["schema"] != PROPOSAL_SCHEMA:
        raise ValueError("Unsupported VLM proposal schema")
    values = document["subgoals"]
    if not isinstance(values, list) or not 1 <= len(values) <= max_steps:
        raise ValueError("VLM proposal has no skills or exceeds the step limit")
    result = []
    for index, value in enumerate(values):
        if not isinstance(value, dict) or set(value) != {"skill", "arguments"}:
            raise ValueError(f"subgoals[{index}] has invalid shape")
        name, arguments = value["skill"], value["arguments"]
        if name not in allowed_skills or not isinstance(arguments, dict):
            raise ValueError(f"subgoals[{index}] names an unavailable skill")
        if any(not isinstance(key, str) or not key for key in arguments):
            raise ValueError(f"subgoals[{index}] has invalid argument names")
        result.append(Subgoal(name, arguments))
    return tuple(result)


class JsonVlmProposer:
    """Use the existing OpenAI-compatible transport with optional camera data."""

    def __init__(self, *, client, model: str, skill_descriptions: Mapping[str, str],
                 images: tuple[Mapping[str, str], ...] = (),
                 temperature: float | None = None, max_attempts: int = 3,
                 response_format: str = "text"):
        self.client, self.model = client, model
        self.skill_descriptions = dict(skill_descriptions)
        self.images = images
        self.temperature, self.max_attempts = temperature, max_attempts
        self.response_format = response_format
        self.model_attempts = []
        if not self.model or not self.skill_descriptions:
            raise ValueError("Model and available skills are required")

    def propose(self, *, goal, world, feedback):
        instructions = load_prompt("tamp_legacy_subgoals_v1.txt")
        payload = {"goal": goal, "world": world,
                   "skills": self.skill_descriptions, "constraint_feedback": feedback}
        content = [{"type": "text", "text": json.dumps(payload, ensure_ascii=False)}]
        for item in self.images:
            content.append({"type": "image_url", "image_url": {
                "path": item["path"], "sha256": item["sha256"],
                "mime_type": "image/png"}})
        result, _ = validated_completion(client=self.client, model=self.model, messages=[
            {"role": "system", "content": instructions},
            {"role": "user", "content": content},
        ], validate=lambda content: parse_proposal(content, frozenset(self.skill_descriptions)),
            response_format=json_response_format(self.response_format),
            temperature=self.temperature, max_attempts=self.max_attempts,
            attempts=self.model_attempts)
        return result


class SymbolicVlmProposer(JsonVlmProposer):
    """VLM-TAMP-style goal proposal followed by symbolic skill refinement."""

    def __init__(self, *, operators: tuple[SymbolicOperator, ...],
                 predicates: Mapping[str, str], **kwargs):
        super().__init__(**kwargs)
        self.operators = operators
        self.predicates = dict(predicates)

    def propose(self, *, goal, world, feedback):
        instructions = load_prompt("tamp_legacy_goals_v2.txt")
        payload = {"goal": goal, "world": world,
                   "predicates": self.predicates,
                   "operators": [dataclasses.asdict(item) for item in self.operators],
                   "constraint_feedback": feedback}
        content = [{"type": "text", "text": json.dumps(
            payload, ensure_ascii=False, default=lambda v: sorted(v)
            if isinstance(v, frozenset) else str(v))}]
        for item in self.images:
            content.append({"type": "image_url", "image_url": {
                "path": item["path"], "sha256": item["sha256"],
                "mime_type": "image/png"}})
        def validate(content):
            goals = parse_goal_proposal(content, frozenset(self.predicates))
            if any(item.target not in world.get("objects", {}) for item in goals):
                raise ValueError("Semantic goal names an unobserved object")
            return goals

        goals, _ = validated_completion(client=self.client, model=self.model, messages=[
            {"role": "system", "content": instructions},
            {"role": "user", "content": content},
        ], validate=validate, temperature=self.temperature, max_attempts=self.max_attempts,
            response_format=goal_response_format(
                self.response_format, schema_name=GOAL_SCHEMA,
                predicate_arity={name: 1 for name in self.predicates},
                known_objects=frozenset(world.get("objects", {})), legacy_target=True),
            attempts=self.model_attempts)
        known = frozenset(GoalPredicate("observed", name)
                          for name in world.get("objects", {}))
        return refine_goals(goals, self.operators, known)


class ConstraintGrounder:
    """Depth-first generate-and-test over continuous skill parameters."""

    def __init__(self, domain: SkillDomain, *, max_candidates: int = 128,
                 max_failures: int = 32, rollout=None):
        if max_candidates < 1 or max_failures < 1:
            raise ValueError("Candidate and failure limits must be positive")
        self.domain, self.max_candidates = domain, max_candidates
        self.max_failures = max_failures
        self.rollout = rollout

    def solve(self, subgoals: tuple[Subgoal, ...], initial_state: Any) -> GroundingResult:
        if not subgoals or any(s.skill not in self.domain.skill_names for s in subgoals):
            raise ValueError("Plan contains no steps or an unregistered skill")
        failures: list[dict] = []
        checked = 0

        def search(index, state, prefix):
            nonlocal checked
            if index == len(subgoals):
                if self.rollout is not None:
                    valid, reason, details = self.rollout(prefix)
                    if not valid:
                        if len(failures) < self.max_failures:
                            failures.append({"step": index - 1,
                                             "skill": prefix[-1].skill,
                                             "parameters": dict(prefix[-1].parameters),
                                             "reason": reason,
                                             "details": dict(details)})
                        return None
                return prefix
            subgoal = subgoals[index]
            for candidate in self.domain.samples(subgoal, state):
                if checked >= self.max_candidates:
                    break
                checked += 1
                parameters = dict(candidate)
                valid, reason, details = self.domain.check(subgoal, parameters, state)
                if not valid:
                    if len(failures) < self.max_failures:
                        failures.append({"step": index, "skill": subgoal.skill,
                                         "parameters": parameters, "reason": reason,
                                         "details": dict(details)})
                    continue
                next_state = self.domain.predict(subgoal, parameters, copy.deepcopy(state))
                step = GroundedSkill(subgoal.skill, dict(subgoal.arguments),
                                    parameters, dict(details))
                solution = search(index + 1, next_state, prefix + (step,))
                if solution is not None:
                    return solution
            return None

        solution = search(0, copy.deepcopy(initial_state), ())
        return GroundingResult(solution is not None, solution or (),
                               tuple(failures), checked)


def plan_with_reprompting(*, proposer: Proposer, grounder: ConstraintGrounder,
                          goal: str, world: Mapping[str, Any], initial_state: Any,
                          max_proposals: int = 3) -> PlanningResult:
    """Return executable grounded skills, or all observed physical failures."""
    if not goal.strip() or max_proposals < 1:
        raise ValueError("A goal and a positive proposal limit are required")
    history: list[dict] = []
    feedback: tuple[dict, ...] = ()
    for attempt in range(max_proposals):
        try:
            subgoals = proposer.propose(goal=goal, world=world, feedback=feedback)
        except PlannerModelError as error:
            history.append({"attempt": attempt + 1, "reason": "planner_model_error",
                            "details": {"message": str(error)}})
            return PlanningResult(False, (), tuple(history), "planner_model_error")
        grounding = grounder.solve(subgoals, initial_state)
        reasons = Counter(item["reason"] for item in grounding.failures)
        frequent = []
        for reason, count in reasons.most_common(2):
            matching = [item for item in grounding.failures
                        if item["reason"] == reason]
            skill = Counter(item.get("skill", "") for item in matching).most_common(1)[0][0]
            step = Counter(item.get("step", 0) for item in matching).most_common(1)[0][0]
            frequent.append({"reason": reason, "count": count,
                             "skill": skill, "step": step})
        record = {"attempt": attempt + 1,
                  "subgoals": [dataclasses.asdict(step) for step in subgoals],
                  "candidates_checked": grounding.candidates_checked,
                  "failure_counts": dict(reasons),
                  "feedback_summary": frequent,
                  "failures": list(grounding.failures)}
        history.append(record)
        if grounding.success:
            return PlanningResult(True, grounding.steps, tuple(history), "grounded")
        feedback = tuple(history)
    return PlanningResult(False, (), tuple(history), "no_feasible_grounding")
