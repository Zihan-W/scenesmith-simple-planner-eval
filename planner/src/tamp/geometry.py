"""Replaceable geometric solver over open skill-skeleton parameters."""

from __future__ import annotations

import copy
import time
import dataclasses
from collections.abc import Mapping
from typing import Any, Protocol

from planner.src.tamp.failures import LowLevelFailure, ProgramFailure

from planner.src.tamp.hierarchy import (
    ConstraintResult, ParameterizedSkillAction, ParameterizedSkillPlan,
    SkillProgram, SkillRegistry, WorldState, validate_skill_program,
)
from planner.src.tamp.planner import Subgoal


class GeometricUnsat(RuntimeError):
    def __init__(self, constraints: tuple[ConstraintResult, ...]):
        super().__init__("No feasible geometric assignment")
        self.constraints = constraints


class GeometryBackendError(RuntimeError):
    """A solver process failed; this is not evidence of geometric UNSAT."""

    def __init__(self, *, returncode: int, log_path: str):
        super().__init__(f"Geometry worker exited with status {returncode}")
        self.returncode = returncode
        self.log_path = log_path


class GeometryDeadlineExceeded(RuntimeError):
    """The run's monotonic wall-time budget expired during geometry work."""


def require_time_remaining(deadline_monotonic_s):
    """Stop cooperative search before starting another costly check."""
    if deadline_monotonic_s is not None and time.perf_counter() >= deadline_monotonic_s:
        raise GeometryDeadlineExceeded("Geometry wall-time budget exhausted")


@dataclasses.dataclass(frozen=True)
class SolverCapabilities:
    """Scheduler-visible guarantees, independent of backend implementation."""
    retry_after_search_failure: bool = False
    failure_context_use: str = "diagnostics_only"
    exclusion_stage: str = "postcheck"
    base_anchor_requirement: str = "none"


class GeometrySolver(Protocol):
    capabilities: SolverCapabilities

    def solve(self, world: WorldState, program: SkillProgram,
              initial_state: Mapping[str, Any], *,
              failure_context: LowLevelFailure | None = None,
              excluded_assignments: frozenset[str] = frozenset()
              ) -> ParameterizedSkillPlan: ...


# Preserve the established public protocol name while exposing the requested name.
GeometricParameterSolver = GeometrySolver


def receive_failure_context(failure_context, trace=None):
    """Keep full low-level evidence local to the solver, never a model prompt."""
    if failure_context is not None and not isinstance(failure_context, LowLevelFailure):
        raise TypeError("Geometry failure_context must be LowLevelFailure or None")
    result = copy.deepcopy(failure_context)
    if result is not None and trace is not None:
        trace({"event": "geometry_failure_context", "failure": dataclasses.asdict(result)})
    return result


def assignment_identity(values: Mapping[str, Any]) -> str:
    """Stable within-process key for excluding one failed parameter assignment."""
    return repr(tuple(sorted((key, repr(value)) for key, value in values.items())))


def abstract_constraint_feedback(constraints: tuple[ConstraintResult, ...],
                                 *, skill: str, target: str | None = None):
    """Strip low-level values and messages before program regeneration."""
    feedback = ProgramFailure.from_constraints(constraints, skill=skill, target=target).as_feedback()
    # Keep the established public tag; model input still uses the typed whitelist.
    return {"type": "GEOMETRIC_INFEASIBLE",
            "reason": feedback["failed_constraints"][0], **feedback}


def _constraint_objects(details: Mapping[str, Any], target: str | None):
    objects = {target} if target else set()

    def visit(value):
        if isinstance(value, Mapping):
            for key in ("body_a", "body_b", "object", "target"):
                body = value.get(key)
                if isinstance(body, str) and body:
                    objects.add(body.split("::", 1)[0])
            for nested in value.values():
                visit(nested)
        elif isinstance(value, (tuple, list)):
            for nested in value:
                visit(nested)

    visit(details)
    return tuple(sorted(objects))


def _parameter_value(parameter: str, candidate: Mapping[str, Any],
                     checks: Mapping[str, Any]):
    if parameter == "base_pose":
        return (candidate["base_x_m"], candidate["base_y_m"],
                candidate["base_yaw_rad"])
    if parameter == "grasp_pose":
        return {"lateral_offset_m": candidate.get("grasp_lateral_offset_m", 0.0),
                "ik": checks.get("ik", {}).get("grasp_pose_in_target")}
    if parameter == "approach_pose":
        return checks.get("ik", {}).get("staging_pose_in_target")
    return candidate.get(parameter)


class SamplingSolver:
    """Bounded generate → batch check → rank → select, using SceneSmith checks.

    ``evaluate_batch`` is the backend seam for a future vectorized/GPU
    implementation. It has no authority to change symbolic skill arguments.
    """
    capabilities = SolverCapabilities()

    def __init__(self, registry: SkillRegistry, domain, *,
                 max_candidates: int = 128, batch_size: int = 8, trace=None):
        if max_candidates < 1 or batch_size < 1:
            raise ValueError("Solver bounds must be positive")
        self.registry, self.domain = registry, domain
        self.max_candidates, self.batch_size = max_candidates, batch_size
        self.candidates_checked = 0
        self.trace = trace

    def evaluate_batch(self, skill: Subgoal,
                       candidates: tuple[Mapping[str, Any], ...],
                       state: Mapping[str, Any]):
        """Return full, inspectable constraint checks for one batch."""
        checked = []
        for candidate in candidates:
            require_time_remaining(getattr(self, "deadline_monotonic_s", None))
            checked.append((candidate, *self.domain.check(skill, candidate, state)))
        return tuple(checked)

    def set_deadline(self, deadline_monotonic_s):
        """Apply the enclosing run's absolute monotonic deadline."""
        self.deadline_monotonic_s = deadline_monotonic_s

    def solve(self, world: WorldState, program: SkillProgram,
              initial_state: Mapping[str, Any], *,
              failure_context: LowLevelFailure | None = None,
              excluded_assignments: frozenset[str] = frozenset()
              ) -> ParameterizedSkillPlan:
        if getattr(program, "parameter_subdomains", {}):
            constraint = ConstraintResult(False, "domain_restriction", "", "unsupported_subdomain", (), {})
            error = GeometricUnsat((constraint,))
            error.retryable_search = False
            error.program_feedback = ProgramFailure.from_constraints(
                (constraint,), skill="", failure_source="backend_precondition",
                attribution_scope="global").as_feedback()
            raise error
        self.failure_context = receive_failure_context(failure_context, self.trace)
        validate_skill_program(program, self.registry, frozenset(world.objects))
        if not program.steps:
            return ParameterizedSkillPlan((), {}, ())
        failures: list[ConstraintResult] = []
        self.candidates_checked = 0

        def search(index: int, state: Mapping[str, Any],
                   actions: tuple[ParameterizedSkillAction, ...],
                   assignments: dict[str, Any]):
            if index == len(program.steps):
                return ParameterizedSkillPlan(actions, assignments.copy(),
                                              tuple(failures))
            step = program.steps[index]
            spec = self.registry[step.skill]
            if set(step.arguments) != set(spec.symbolic_parameters) or (
                set(step.continuous_variables) != set(spec.geometric_parameters)
            ):
                raise ValueError(f"Skill skeleton does not match registry: {step.skill}")
            target = step.arguments.get("object")
            if target is not None and target not in world.objects:
                raise ValueError(f"Skill names an unknown object: {target}")
            arguments = dict(step.arguments)
            if target is not None:
                arguments["target"] = arguments.pop("object")
            skill = Subgoal(step.skill, arguments)
            source = iter(self.domain.samples(skill, state))
            while self.candidates_checked < self.max_candidates:
                require_time_remaining(getattr(self, "deadline_monotonic_s", None))
                batch = []
                for _ in range(min(self.batch_size,
                                   self.max_candidates - self.candidates_checked)):
                    candidate = next(source, None)
                    if candidate is None:
                        break
                    batch.append(dict(candidate))
                if not batch:
                    break
                self.candidates_checked += len(batch)
                checked = self.evaluate_batch(skill, tuple(batch), state)
                feasible_candidates = []
                for candidate, feasible, reason, details in checked:
                    values = {
                        variable: _parameter_value(parameter, candidate, details)
                        for parameter, variable in step.continuous_variables.items()
                    }
                    identity = assignment_identity(values)
                    candidate_identity = assignment_identity({
                        "skill": step.skill, "arguments": dict(step.arguments),
                        "parameters": candidate,
                    })
                    if any(variable in assignments and assignment_identity({variable: value})
                           != assignment_identity({variable: assignments[variable]})
                           for variable, value in values.items()):
                        feasible, reason = False, "shared_variable_conflict"
                    excluded = (identity in excluded_assignments
                                or candidate_identity in excluded_assignments)
                    rank = (self.domain.rank_candidate(skill, candidate, details)
                            if feasible and hasattr(self.domain, "rank_candidate")
                            else ())
                    if self.trace is not None:
                        self.trace({
                            "event": "candidate_parameters", "step": index,
                            "skill": step.skill, "arguments": dict(step.arguments),
                            "continuous_variables": dict(step.continuous_variables),
                            "parameters": candidate, "assignments": values,
                            "feasible": feasible, "excluded": excluded,
                            "constraint": reason, "details": dict(details),
                            "rank": rank,
                        })
                    if excluded:
                        continue
                    if not feasible:
                        failures.append(ConstraintResult(
                            False, reason, next(iter(values), ""), str(details.get("message", reason)),
                            _constraint_objects(details, target), dict(details),
                        ))
                        continue
                    feasible_candidates.append((rank, candidate, details, values, candidate_identity))
                # Sort only by the domain's physical quality key; Python's
                # stable sort preserves generator priority when scores tie.
                for _, candidate, details, values, candidate_identity in sorted(
                        feasible_candidates, key=lambda item: item[0]):
                    next_state = self.domain.predict(skill, candidate, copy.deepcopy(state))
                    action = ParameterizedSkillAction(
                        step.skill, dict(step.arguments),
                        {**candidate, **values, "checks": dict(details),
                         "candidate_identity": candidate_identity},
                        tuple(_bind_effect(effect, step.arguments)
                              for effect in spec.add_effects),
                        spec.supports_geometric_conditioning,
                        dict(step.continuous_variables),
                    )
                    result = search(index + 1, next_state,
                                    actions + (action,), {**assignments, **values})
                    if result is not None:
                        return result
            return None

        result = search(0, copy.deepcopy(initial_state), (), {})
        if result is None:
            raise GeometricUnsat(tuple(failures))
        return result


def _bind_effect(effect, arguments):
    return dataclasses.replace(effect, arguments=tuple(
        arguments[arg[1:]] if arg.startswith("$") else arg
        for arg in effect.arguments
    ))


class ExternalGeometrySolverAdapter:
    """Validate an external solver's output; this is not a cuTAMP optimizer."""
    capabilities = SolverCapabilities()

    def __init__(self, backend=None):
        self.backend = backend

    def solve(self, world: WorldState, program: SkillProgram,
              initial_state: Mapping[str, Any], *,
              failure_context: LowLevelFailure | None = None,
              excluded_assignments: frozenset[str] = frozenset()
              ) -> ParameterizedSkillPlan:
        if self.backend is None:
            raise RuntimeError("External geometry backend is not configured")
        # Frozen dataclasses may still contain mutable dictionaries. A backend
        # must not mutate the caller's task and then validate against that edit.
        expected_steps = copy.deepcopy(program.steps)
        result = self.backend.solve(copy.deepcopy(world), copy.deepcopy(program),
                                    copy.deepcopy(initial_state),
                                    failure_context=receive_failure_context(failure_context),
                                    excluded_assignments=excluded_assignments)
        if not isinstance(result, ParameterizedSkillPlan) or len(result.actions) != len(expected_steps):
            raise ValueError("External backend returned an invalid parameterized plan")
        for step, action in zip(expected_steps, result.actions, strict=True):
            if action.skill_name != step.skill or dict(action.symbolic_args) != dict(step.arguments):
                raise ValueError("External backend changed the symbolic skill program")
            if not set(step.continuous_variables.values()).issubset(result.assignments):
                raise ValueError("External backend left continuous variables unassigned")
        return result
