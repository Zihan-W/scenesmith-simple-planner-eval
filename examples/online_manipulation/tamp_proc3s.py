"""LLM open-program generation, aligned with PRoC3S's program/domain boundary.

Evidence: official vtamp/policies/ours/policy.py:144-221 and
vtamp/policies/prompt_elements/ours_role.txt. This restricted JSON language
replaces upstream Python exec. Named environment samplers replace arbitrary
model-written sampler code; neither restriction is claimed as a full replica.
"""

from __future__ import annotations

import dataclasses
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from examples.online_manipulation.bt_generation import GenerationError
from examples.online_manipulation.tamp_failures import ProgramFailure
from examples.online_manipulation.tamp_hierarchy import (
    SKILL_PROGRAM_SCHEMA, PredicateGoal, SkillProgram, SkillRegistry, SkillStep,
    WorldState, _bind, program_identity, validate_skill_program,
)
from examples.online_manipulation.tamp_semantic import ModelSettings


PROC3S_SCHEMA = "scenesmith.proc3s.program.v1"
DOMAIN_SAMPLERS = {
    "base_pose": "scene_base_pose",
    "grasp_pose": "calibrated_grasp_pose",
    "approach_pose": "calibrated_approach_pose",
}


class PRoC3SGenerationFailure(ValueError):
    """No valid LLM program was obtained; no symbolic fallback is permitted."""


@dataclasses.dataclass(frozen=True)
class PRoC3SSkillProgram(SkillProgram):
    """Common executor-facing skill program with LLM-declared open domains."""

    parameter_domains: Mapping[str, str] = dataclasses.field(default_factory=dict)


def parse_proc3s_program(content: str, *, registry: SkillRegistry,
                         world: WorldState, goals: tuple[PredicateGoal, ...]
                         ) -> PRoC3SSkillProgram:
    """Reject numeric solutions, invalid domain bindings and symbolic failures."""
    document = json.loads(content)
    if not isinstance(document, dict) or set(document) != {"schema", "steps", "domains"}:
        raise ValueError("PRoC3S JSON requires schema, steps and domains only")
    if document["schema"] != PROC3S_SCHEMA:
        raise ValueError("Unknown PRoC3S program schema")
    if not isinstance(document["steps"], list) or not isinstance(document["domains"], list):
        raise ValueError("steps and domains must be arrays")
    steps = []
    for item in document["steps"]:
        if not isinstance(item, dict) or set(item) != {
                "skill", "arguments", "continuous_variables"}:
            raise ValueError("Invalid PRoC3S step fields")
        variables = item["continuous_variables"]
        if not isinstance(variables, dict) or any(
                not isinstance(value, str) or not value.startswith("$")
                or not value[1:].isidentifier() for value in variables.values()):
            raise ValueError("All geometric arguments must be open $variable references")
        steps.append(SkillStep(item["skill"], item["arguments"],
                               {key: value[1:] for key, value in variables.items()}))
    program = SkillProgram(tuple(steps), schema=SKILL_PROGRAM_SCHEMA)
    validate_skill_program(program, registry, frozenset(world.objects))
    expected_domains = {}
    for step in steps:
        for parameter, variable in step.continuous_variables.items():
            if parameter not in DOMAIN_SAMPLERS:
                raise ValueError(f"No registered domain sampler for {parameter}")
            expected_domains[variable] = DOMAIN_SAMPLERS[parameter]
    domains = {}
    for domain in document["domains"]:
        if not isinstance(domain, dict) or set(domain) != {"variable", "sampler"}:
            raise ValueError("A domain requires variable and sampler, never numeric solutions")
        variable, sampler = domain["variable"], domain["sampler"]
        if not isinstance(variable, str) or not variable.isidentifier() or variable in domains:
            raise ValueError("Domain variables must be unique identifiers without $")
        if not isinstance(sampler, str) or expected_domains.get(variable) != sampler:
            raise ValueError("Domain sampler does not match the open parameter type")
        domains[variable] = sampler
    if set(domains) != set(expected_domains):
        raise ValueError("Every open variable needs exactly one domain")
    facts = world.facts
    for index, step in enumerate(steps):
        spec = registry[step.skill]
        if not all(_bind(fact, step.arguments) in facts for fact in spec.preconditions):
            raise ValueError(f"Symbolic precondition fails at step {index}: {step.skill}")
        facts = frozenset((facts - {_bind(fact, step.arguments) for fact in spec.delete_effects})
                          | {_bind(fact, step.arguments) for fact in spec.add_effects})
    if not set(goals).issubset(facts):
        raise ValueError("Program does not symbolically achieve the requested goals")
    return PRoC3SSkillProgram(program.steps, parameter_domains=domains)


def _response_format(mode: str, registry: SkillRegistry, objects):
    if mode == "text":
        return None
    if mode == "json_object":
        return {"type": "json_object"}

    def object_schema(properties):
        return {"type": "object", "properties": properties,
                "required": list(properties), "additionalProperties": False}

    variants = []
    for spec in registry:
        variants.append(object_schema({
            "skill": {"type": "string", "enum": [spec.name]},
            "arguments": object_schema({name: {"type": "string", "enum": sorted(objects)}
                                         for name in spec.symbolic_parameters}),
            "continuous_variables": object_schema({name: {"type": "string"}
                                                    for name in spec.geometric_parameters}),
        }))
    schema = object_schema({
        "schema": {"type": "string", "enum": [PROC3S_SCHEMA]},
        "steps": {"type": "array", "maxItems": 20, "items": {"anyOf": variants}},
        "domains": {"type": "array", "items": object_schema({
            "variable": {"type": "string"},
            "sampler": {"type": "string", "enum": sorted(set(DOMAIN_SAMPLERS.values()))},
        })},
    })
    return {"type": "json_schema", "json_schema": {
        "name": "proc3s_program", "strict": True, "schema": schema}}


class PRoC3SProgramGenerator:
    """Actually query the LLM for structure/domains and regenerate on feedback."""

    name = "proc3s"

    def __init__(self, client, settings: ModelSettings, registry: SkillRegistry, *, trace=None):
        self.client, self.settings, self.registry = client, settings, registry
        self.trace = trace
        self.calls = 0
        self.last_program = None
        self.prompt = (Path(__file__).with_name("prompts") / "proc3s_program_v1.txt").read_text()

    def generate(self, world: WorldState, goals: tuple[PredicateGoal, ...], *,
                 feedback: tuple[Mapping[str, Any], ...] = (),
                 excluded_programs: frozenset[tuple] = frozenset()
                 ) -> PRoC3SSkillProgram:
        """Return a validated open program, or raise PRoC3SGenerationFailure."""
        # Whitelist at the receiving boundary: particle/IK/collision numerics
        # and arbitrary execution failure messages are not program prompts.
        program_feedback = []
        for item in feedback:
            failure = ProgramFailure.from_feedback(item)
            sanitized = dataclasses.replace(
                failure,
                skill=failure.skill if failure.skill in {spec.name for spec in self.registry} else "",
                involved_objects=tuple(name for name in failure.involved_objects if name in world.objects),
            )
            program_feedback.append(sanitized.as_feedback())
        skills = [{key: value for key, value in dataclasses.asdict(spec).items()
                   if key in {"name", "symbolic_parameters", "geometric_parameters",
                              "preconditions", "add_effects", "delete_effects"}}
                  for spec in self.registry]
        payload = {
            "goals": [dataclasses.asdict(goal) for goal in goals],
            "world": {"objects": {name: {key: value for key, value in metadata.items()
                                          if key in {"category", "movable", "articulated", "surface"}}
                                  for name, metadata in world.objects.items()},
                      "facts": [dataclasses.asdict(fact) for fact in sorted(world.facts)]},
            "skills": skills, "domain_samplers": DOMAIN_SAMPLERS,
            "constraint_feedback": program_feedback,
            "excluded_skeletons": sorted(excluded_programs),
            "previous_program": self.last_program if feedback else None,
        }
        messages = [{"role": "system", "content": self.prompt},
                    {"role": "user", "content": json.dumps(payload, ensure_ascii=False)}]
        response_format = _response_format(self.settings.response_format, self.registry, world.objects)
        options = {} if response_format is None else {"response_format": response_format}
        error_message = ""
        for _ in range(self.settings.max_attempts):
            self.calls += 1
            response = None
            if self.trace is not None:
                self.trace({"event": "proc3s_model_request", "call": self.calls,
                            "model": self.settings.model, "messages": messages,
                            "response_format": response_format})
            try:
                response = self.client.complete(model=self.settings.model, messages=messages,
                                                temperature=self.settings.temperature, **options)
                program = parse_proc3s_program(response.content, registry=self.registry,
                                               world=world, goals=goals)
                if program_identity(program) in excluded_programs:
                    raise ValueError("The returned skeleton already failed; revise its structure")
            except (GenerationError, ValueError, TypeError, KeyError) as error:
                error_message = str(error)
                if self.trace is not None:
                    self.trace({"event": "proc3s_generation_error", "call": self.calls,
                                "reason": error_message})
                if response is not None:
                    messages.extend((
                        {"role": "assistant", "content": response.content},
                        {"role": "user", "content": "Invalid program: " + error_message
                         + ". Return corrected open-program JSON, not numeric solutions."},
                    ))
                continue
            self.last_program = json.loads(response.content)
            if self.trace is not None:
                self.trace({"event": "proc3s_model_response", "call": self.calls,
                            "response_id": getattr(response, "response_id", None),
                            "model": getattr(response, "model", self.settings.model),
                            "usage": getattr(response, "usage", None),
                            "program": self.last_program})
            return program
        raise PRoC3SGenerationFailure(
            f"PRoC3SGenerationFailure after {self.settings.max_attempts} attempts: {error_message}")
