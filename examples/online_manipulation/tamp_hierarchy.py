"""Typed semantic goals and registry-backed open-parameter skill skeletons.

This is the hierarchical TAMP path. The old model-authored sampling program
remains in ``tamp_program.py`` for the legacy ablation only.
"""

from __future__ import annotations

import dataclasses
import itertools
import json
from collections import deque
from collections.abc import Mapping
from typing import Any


GOAL_SCHEMA = "scenesmith.tamp.goals.v3"
SKILL_PROGRAM_SCHEMA = "scenesmith.tamp.skill_program.v2"


@dataclasses.dataclass(frozen=True, order=True)
class PredicateGoal:
    predicate: str
    arguments: tuple[str, ...]

    def __post_init__(self):
        if not self.predicate or any(not isinstance(arg, str) or not arg for arg in self.arguments):
            raise ValueError("Predicate and arguments must be nonempty strings")


@dataclasses.dataclass(frozen=True)
class WorldState:
    objects: Mapping[str, Mapping[str, Any]]
    facts: frozenset[PredicateGoal]
    observation_id: str = ""
    robot: Mapping[str, Any] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass(frozen=True)
class SkillSpec:
    name: str
    symbolic_parameters: tuple[str, ...]
    geometric_parameters: tuple[str, ...]
    preconditions: tuple[PredicateGoal, ...]
    add_effects: tuple[PredicateGoal, ...]
    delete_effects: tuple[PredicateGoal, ...] = ()
    constraints: tuple[str, ...] = ()
    supports_geometric_conditioning: bool = True
    runtime_action: str | None = None

    def __post_init__(self):
        if not self.name or len(set(self.symbolic_parameters)) != len(self.symbolic_parameters):
            raise ValueError("Skill name and symbolic parameters must be unique")
        if len(set(self.geometric_parameters)) != len(self.geometric_parameters):
            raise ValueError("Geometric parameter names must be unique")
        placeholders = {"$" + name for name in self.symbolic_parameters}
        for fact in (*self.preconditions, *self.add_effects, *self.delete_effects):
            if any(arg.startswith("$") and arg not in placeholders for arg in fact.arguments):
                raise ValueError(f"Unbound predicate placeholder in {self.name}")


class SkillRegistry:
    def __init__(self, specs: tuple[SkillSpec, ...]):
        self._specs = {spec.name: spec for spec in specs}
        if len(self._specs) != len(specs):
            raise ValueError("Duplicate skill registration")

    def __iter__(self):
        return iter(self._specs.values())

    def __getitem__(self, name: str) -> SkillSpec:
        return self._specs[name]

    @property
    def names(self) -> frozenset[str]:
        return frozenset(self._specs)


@dataclasses.dataclass(frozen=True)
class SkillStep:
    skill: str
    arguments: Mapping[str, str]
    continuous_variables: Mapping[str, str]


@dataclasses.dataclass(frozen=True)
class SkillProgram:
    steps: tuple[SkillStep, ...]
    schema: str = SKILL_PROGRAM_SCHEMA


@dataclasses.dataclass(frozen=True)
class ConstraintResult:
    feasible: bool
    constraint: str
    variable: str
    reason: str
    involved_objects: tuple[str, ...] = ()
    details: Mapping[str, Any] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass(frozen=True)
class ParameterizedSkillAction:
    skill_name: str
    symbolic_args: Mapping[str, str]
    geometric_parameters: Mapping[str, Any]
    expected_effects: tuple[PredicateGoal, ...]
    supports_geometric_conditioning: bool
    parameter_bindings: Mapping[str, str] = dataclasses.field(default_factory=dict)


@dataclasses.dataclass(frozen=True)
class ParameterizedSkillPlan:
    actions: tuple[ParameterizedSkillAction, ...]
    assignments: Mapping[str, Any]
    constraints: tuple[ConstraintResult, ...]


def validate_skill_program(program: SkillProgram, registry: SkillRegistry,
                           known_objects: frozenset[str], *, max_steps: int = 20):
    """Validate open variables and all symbolic arguments at the solver boundary."""
    if program.schema != SKILL_PROGRAM_SCHEMA or len(program.steps) > max_steps:
        raise ValueError("Invalid skill program schema or length")
    variable_types = {}
    for step in program.steps:
        if not isinstance(step.skill, str) or step.skill not in registry.names:
            raise ValueError("Unknown skill in program")
        spec = registry[step.skill]
        if (not isinstance(step.arguments, Mapping)
                or set(step.arguments) != set(spec.symbolic_parameters)
                or any(not isinstance(value, str) or value not in known_objects
                       for value in step.arguments.values())):
            raise ValueError("Invalid symbolic skill arguments")
        if (not isinstance(step.continuous_variables, Mapping)
                or set(step.continuous_variables) != set(spec.geometric_parameters)):
            raise ValueError("Invalid open geometric parameter names")
        for parameter, variable in step.continuous_variables.items():
            if not isinstance(variable, str) or not variable.isidentifier():
                raise ValueError("Continuous variables must be symbolic identifiers")
            if variable in variable_types and variable_types[variable] != parameter:
                raise ValueError("A shared variable cannot have different parameter types")
            variable_types[variable] = parameter


def parse_skill_program(content: str, *, registry: SkillRegistry,
                        known_objects: frozenset[str], max_steps: int = 20) -> SkillProgram:
    """Parse the versioned skeleton format without accepting numeric domains."""
    document = json.loads(content)
    if not isinstance(document, dict) or set(document) != {"schema", "steps"}:
        raise ValueError("Skill program requires schema and steps only")
    if not isinstance(document["steps"], list):
        raise ValueError("Skill program steps must be a list")
    steps = []
    for item in document["steps"]:
        if not isinstance(item, dict) or set(item) != {
                "skill", "arguments", "continuous_variables"}:
            raise ValueError("Invalid skill step shape")
        steps.append(SkillStep(**item))
    program = SkillProgram(tuple(steps), schema=document["schema"])
    validate_skill_program(program, registry, known_objects, max_steps=max_steps)
    return program


def parse_semantic_goals(content: str, *, predicate_arity: Mapping[str, int],
                         known_objects: frozenset[str], max_goals: int = 12
                         ) -> tuple[PredicateGoal, ...]:
    """Reject skill names and numeric geometry from the semantic model output."""
    document = json.loads(content)
    if not isinstance(document, dict) or set(document) != {"schema", "subgoals"}:
        raise ValueError("Semantic proposal requires schema and subgoals only")
    if document["schema"] != GOAL_SCHEMA:
        raise ValueError("Unsupported semantic goal schema")
    items = document["subgoals"]
    if not isinstance(items, list) or not 1 <= len(items) <= max_goals:
        raise ValueError("Invalid semantic subgoal count")
    goals = []
    for index, item in enumerate(items):
        if not isinstance(item, dict) or set(item) != {"predicate", "arguments"}:
            raise ValueError(f"Invalid semantic subgoal {index}")
        name, args = item["predicate"], item["arguments"]
        if name not in predicate_arity or not isinstance(args, list) or len(args) != predicate_arity[name]:
            raise ValueError(f"Unknown predicate or arity at subgoal {index}")
        if any(not isinstance(arg, str) or arg not in known_objects for arg in args):
            raise ValueError(f"Unknown object at subgoal {index}")
        goals.append(PredicateGoal(name, tuple(args)))
    return tuple(goals)


def _bind(fact: PredicateGoal, binding: Mapping[str, str]) -> PredicateGoal:
    return PredicateGoal(fact.predicate, tuple(
        binding[arg[1:]] if arg.startswith("$") else arg for arg in fact.arguments
    ))


def program_identity(program: SkillProgram) -> tuple:
    """Identify a symbolic skeleton independently of its open variable names."""
    return tuple((step.skill, tuple(sorted(step.arguments.items())))
                 for step in program.steps)


def refine_goals(world: WorldState, goals: tuple[PredicateGoal, ...],
                 registry: SkillRegistry, *, max_actions: int = 20,
                 max_expansions: int = 10000,
                 excluded_skills: frozenset[str] = frozenset(),
                 excluded_programs: frozenset[tuple] = frozenset()) -> SkillProgram:
    """Finite STRIPS search; new skills require registration, not task branches."""
    if max_actions < 1 or max_expansions < 1:
        raise ValueError("Search bounds must be positive")
    facts = world.facts
    plan: tuple[tuple[SkillSpec, dict[str, str]], ...] = ()
    names = tuple(sorted(world.objects))

    def path_identity(actions):
        return tuple((spec.name, tuple(sorted(binding.items())))
                     for spec, binding in actions)

    def search_key(current, actions):
        # Preserve paths that may still match an excluded skeleton. Once a
        # path diverges, ordinary state deduplication is sufficient again.
        prefix = path_identity(plan + actions)
        matching = tuple(sorted(identity for identity in excluded_programs
                                if identity[:len(prefix)] == prefix))
        return current, matching, len(prefix) if matching else 0

    for goal in goals:
        if goal in facts:
            continue
        queue = deque([(facts, ())])
        seen = {search_key(facts, ())}
        found = None
        expansions = 0
        while queue and expansions < max_expansions:
            current, actions = queue.popleft()
            expansions += 1
            identity = path_identity(plan + actions)
            if any(identity[:len(failed)] == failed
                   for failed in excluded_programs):
                # Appending skills cannot repair an already infeasible prefix.
                continue
            if goal in current and path_identity(plan + actions) not in excluded_programs:
                found = (current, actions)
                break
            if len(plan) + len(actions) >= max_actions:
                continue
            for spec in registry:
                if spec.name in excluded_skills:
                    continue
                assignments = itertools.product(names, repeat=len(spec.symbolic_parameters))
                for values in assignments:
                    binding = dict(zip(spec.symbolic_parameters, values, strict=True))
                    if not all(_bind(fact, binding) in current for fact in spec.preconditions):
                        continue
                    next_facts = frozenset(
                        (current - {_bind(fact, binding) for fact in spec.delete_effects})
                        | {_bind(fact, binding) for fact in spec.add_effects}
                    )
                    # No-effect actions cannot enable a prerequisite and must
                    # not be used merely to evade an excluded skeleton.
                    if next_facts == current:
                        continue
                    next_actions = actions + ((spec, binding),)
                    key = search_key(next_facts, next_actions)
                    if key in seen:
                        continue
                    seen.add(key)
                    queue.append((next_facts, next_actions))
        if found is None:
            raise ValueError(f"No skill refinement for {goal}")
        facts, actions = found
        plan += actions
    return SkillProgram(tuple(
        SkillStep(spec.name, binding, {
            parameter: f"{parameter}_{index}"
            for parameter in spec.geometric_parameters
        })
        for index, (spec, binding) in enumerate(plan)
    ))


def picklift_registry() -> SkillRegistry:
    """SceneSmith's currently implemented skills, expressed without task branches."""
    target = ("$object",)
    return SkillRegistry((
        SkillSpec(
            "NavigateToPick", ("object",), ("base_pose",),
            (PredicateGoal("observed", target),),
            (PredicateGoal("at_pick_pose", target),),
            constraints=("corridor", "base_collision", "reachability"),
            runtime_action="NavigateTo",
        ),
        SkillSpec(
            "PickLift", ("object",), ("grasp_pose", "approach_pose"),
            (PredicateGoal("observed", target),
             PredicateGoal("at_pick_pose", target),
             PredicateGoal("gripper_empty", ())),
            (PredicateGoal("holding", target),),
            (PredicateGoal("gripper_empty", ()),),
            constraints=("ik", "joint_limits", "joint_edge", "contact"),
            runtime_action="ExecutePickLift",
        ),
    ))
