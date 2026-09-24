"""Replaceable geometric solver over open skill-skeleton parameters."""

from __future__ import annotations

import copy
import time
import dataclasses
from collections.abc import Mapping
from typing import Any, Protocol

from planner.src.tamp.failures import LowLevelFailure, ProgramFailure

from planner.src.tamp.hierarchy import (
    ConstraintResult, ParameterizedSkillPlan, SkillProgram, WorldState,
)


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
