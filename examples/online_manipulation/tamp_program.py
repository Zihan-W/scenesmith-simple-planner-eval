"""PRoC3S-style parameter program and joint sample-and-test solver.

The language model supplies a declarative program with shared variables and
sampling domains.  This interpreter executes no model-generated Python.
Every sampled whole plan is checked skill by skill and can be replayed in a
physics simulator through the optional rollout callback.
"""

from __future__ import annotations

import dataclasses
import copy
import itertools
import json
import math
import random
from collections.abc import Callable, Mapping
from typing import Any

from examples.online_manipulation.tamp_planner import GroundedSkill, GroundingResult, Subgoal
from examples.online_manipulation.tamp_model import (
    PlannerModelError, json_response_format, load_prompt, validated_completion,
)


PROGRAM_SCHEMA = "scenesmith.tamp.program.v1"


@dataclasses.dataclass(frozen=True)
class SkillTemplate:
    skill: str
    arguments: Mapping[str, Any]
    parameters: Mapping[str, Any]


@dataclasses.dataclass(frozen=True)
class PlanSketch:
    variables: Mapping[str, Mapping[str, Any]]
    steps: tuple[SkillTemplate, ...]


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def parse_program(content: str, allowed_skills: frozenset[str],
                  *, max_variables=16, max_steps=20) -> PlanSketch:
    document = json.loads(content)
    if not isinstance(document, dict) or set(document) != {"schema", "variables", "steps"}:
        raise ValueError("Plan program requires schema, variables and steps only")
    if document["schema"] != PROGRAM_SCHEMA:
        raise ValueError("Unsupported plan program schema")
    variables, steps = document["variables"], document["steps"]
    if not isinstance(variables, dict) or len(variables) > max_variables:
        raise ValueError("Too many or invalid program variables")
    if not isinstance(steps, list) or not 1 <= len(steps) <= max_steps:
        raise ValueError("Plan program has no skills or exceeds limit")
    for name, spec in variables.items():
        if not isinstance(name, str) or not name.isidentifier() or not isinstance(spec, dict):
            raise ValueError("Invalid parameter domain")
        kind = spec.get("type")
        if kind == "discrete":
            values = spec.get("values")
            if set(spec) != {"type", "values"} or not isinstance(values, list) or not 1 <= len(values) <= 32:
                raise ValueError(f"Invalid discrete domain: {name}")
            if any(isinstance(value, (dict, list)) for value in values):
                raise ValueError(f"Discrete values must be scalars: {name}")
        elif kind in ("uniform", "normal"):
            expected = {"type", "low", "high"} if kind == "uniform" else {
                "type", "low", "high", "mean", "std"}
            if set(spec) != expected or not all(_finite(value) for key, value in spec.items() if key != "type"):
                raise ValueError(f"Invalid continuous domain: {name}")
            if spec["low"] >= spec["high"] or (kind == "normal" and spec["std"] <= 0):
                raise ValueError(f"Empty continuous domain: {name}")
        else:
            raise ValueError(f"Unknown sampler: {name}")
    templates = []
    for index, item in enumerate(steps):
        if not isinstance(item, dict) or set(item) != {"skill", "arguments", "parameters"}:
            raise ValueError(f"Invalid skill template at step {index}")
        if item["skill"] not in allowed_skills or not isinstance(item["arguments"], dict) or not isinstance(item["parameters"], dict):
            raise ValueError(f"Unknown skill or parameter shape at step {index}")
        templates.append(SkillTemplate(item["skill"], item["arguments"], item["parameters"]))
    return PlanSketch(variables, tuple(templates))


def evaluate_expression(expression: Any, assignment: Mapping[str, Any],
                        state: Mapping[str, Any], *, depth=0):
    """Evaluate a small data-only expression language with bounded depth."""
    if depth > 5 or not isinstance(expression, dict) or len(expression) != 1:
        raise ValueError("Invalid parameter expression")
    operator, value = next(iter(expression.items()))
    if operator == "const" and not isinstance(value, (dict, list)):
        return value
    if operator == "var" and isinstance(value, str) and value in assignment:
        return assignment[value]
    if operator == "state" and isinstance(value, str) and value in state:
        return state[value]
    if operator in ("add", "sub") and isinstance(value, list) and len(value) == 2:
        a = evaluate_expression(value[0], assignment, state, depth=depth + 1)
        b = evaluate_expression(value[1], assignment, state, depth=depth + 1)
        if _finite(a) and _finite(b):
            return a + b if operator == "add" else a - b
    raise ValueError("Unsupported or unbound parameter expression")


def _draw_assignment(variables, rng):
    assignment = {}
    for name, spec in variables.items():
        kind = spec["type"]
        if kind == "discrete":
            assignment[name] = rng.choice(spec["values"])
        elif kind == "uniform":
            assignment[name] = rng.uniform(spec["low"], spec["high"])
        else:
            assignment[name] = min(spec["high"], max(spec["low"], rng.gauss(
                spec["mean"], spec["std"])))
    return assignment


class ProgramGrounder:
    """Sample all shared variables, then validate and roll out the entire plan."""

    def __init__(self, domain, *, max_samples=64, seed=0,
                 rollout: Callable[[tuple[GroundedSkill, ...]],
                                   tuple[bool, str, Mapping[str, Any]]] | None = None):
        if max_samples < 1:
            raise ValueError("max_samples must be positive")
        self.domain, self.max_samples, self.seed = domain, max_samples, seed
        self.rollout = rollout

    def solve(self, sketch: PlanSketch, initial_state) -> GroundingResult:
        rng = random.Random(self.seed)
        failures = []
        all_discrete = all(spec["type"] == "discrete" for spec in sketch.variables.values())
        if all_discrete:
            names = tuple(sketch.variables)
            assignments = (dict(zip(names, values)) for values in itertools.product(
                *(sketch.variables[name]["values"] for name in names)))
        else:
            assignments = (_draw_assignment(sketch.variables, rng)
                           for _ in range(self.max_samples))
        checked = 0
        for assignment in itertools.islice(assignments, self.max_samples):
            checked += 1
            state = copy.deepcopy(initial_state)
            steps = []
            failure = None
            for index, template in enumerate(sketch.steps):
                parameters = {key: evaluate_expression(expr, assignment, state)
                              for key, expr in template.parameters.items()}
                subgoal = Subgoal(template.skill, template.arguments)
                valid, reason, details = self.domain.check(subgoal, parameters, state)
                if not valid:
                    failure = {"sample": checked, "step": index,
                               "skill": template.skill, "reason": reason,
                               "assignment": assignment, "details": dict(details)}
                    break
                steps.append(GroundedSkill(template.skill, template.arguments,
                                           parameters, dict(details)))
                state = self.domain.predict(subgoal, parameters, state)
            if failure is None and self.rollout is not None:
                valid, reason, details = self.rollout(tuple(steps))
                if not valid:
                    failure = {"sample": checked, "step": len(steps) - 1,
                               "skill": steps[-1].skill, "reason": reason,
                               "assignment": assignment, "details": dict(details)}
            if failure is None:
                return GroundingResult(True, tuple(steps), tuple(failures), checked)
            failures.append(failure)
        return GroundingResult(False, (), tuple(failures), checked)


class SketchGrounder:
    """Generate or replay a parameter program for each VLM skill refinement."""

    def __init__(self, domain, *, world: Mapping[str, Any],
                 client=None, model: str | None = None,
                 recorded_program: str | None = None,
                 images: tuple[Mapping[str, str], ...] = (),
                 max_samples: int = 64, seed: int = 0, rollout=None,
                 temperature: float | None = None, max_attempts: int = 3,
                 response_format: str = "text"):
        if (recorded_program is None) == (client is None or model is None):
            raise ValueError("Provide exactly one recorded program or live model")
        self.domain, self.world, self.client, self.model = domain, world, client, model
        self.recorded_program, self.images = recorded_program, images
        self.max_samples, self.seed, self.rollout = max_samples, seed, rollout
        self.temperature, self.max_attempts = temperature, max_attempts
        self.response_format = response_format
        self.programs: list[dict] = []

    def solve(self, subgoals: tuple[Subgoal, ...], initial_state) -> GroundingResult:
        def validate(content):
            sketch = parse_program(content, self.domain.skill_names)
            skeleton = tuple(Subgoal(step.skill, step.arguments) for step in sketch.steps)
            if skeleton != subgoals:
                raise ValueError("Program skill sequence differs from refined subgoals")
            return sketch

        attempts = []
        if self.recorded_program is not None:
            content = self.recorded_program
        else:
            prompt = {
                "world": self.world,
                "required_skill_sequence": [dataclasses.asdict(step) for step in subgoals],
                "skill_parameters": getattr(self.domain, "program_schema", {}),
                "sampler_hints": getattr(self.domain, "sampler_hints", {}),
                "previous_programs": [
                    {"program": item["raw_program"],
                     "constraint_failures": item.get("failures", [])}
                    for item in self.programs
                ],
            }
            message = [{"type": "text", "text": json.dumps(prompt, default=str)}]
            for item in self.images:
                message.append({"type": "image_url", "image_url": {
                    "path": item["path"], "sha256": item["sha256"],
                    "mime_type": "image/png"}})
            try:
                _, content = validated_completion(
                    client=self.client, model=self.model, messages=[
                        {"role": "system", "content": load_prompt("tamp_legacy_program_v1.txt")},
                        {"role": "user", "content": message},
                    ], validate=validate, temperature=self.temperature,
                    response_format=json_response_format(self.response_format),
                    max_attempts=self.max_attempts, attempts=attempts)
            except PlannerModelError as error:
                result = GroundingResult(False, (), ({
                    "step": 0, "skill": "program", "reason": "planner_model_error",
                    "details": {"message": str(error)}},), 0)
                self.programs.append({"raw_program": None, "success": False,
                                      "candidates_checked": 0, "model_attempts": attempts,
                                      "failures": list(result.failures)})
                return result
        try:
            sketch = validate(content)
            result = ProgramGrounder(self.domain, max_samples=self.max_samples,
                                     seed=self.seed + len(self.programs),
                                     rollout=self.rollout).solve(sketch, initial_state)
        except (ValueError, KeyError, TypeError) as error:
            result = GroundingResult(False, (), ({"step": 0, "skill": "program",
                                                 "reason": "invalid_program",
                                                 "details": {"message": str(error)}},), 0)
        self.programs.append({"raw_program": content, "success": result.success,
                              "model_attempts": attempts,
                              "candidates_checked": result.candidates_checked,
                              "failures": list(result.failures)})
        return result
