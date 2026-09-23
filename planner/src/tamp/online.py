"""Incremental TAMP: semantic → skeleton → geometry → skill → observe → verify."""

from __future__ import annotations

import dataclasses
import json
import math
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, Protocol

from planner.src.tamp.failures import LowLevelFailure, ProgramFailure, SemanticFailure

from planner.src.tamp.geometry import (
    GeometricUnsat, GeometryBackendError, GeometryDeadlineExceeded,
    abstract_constraint_feedback,
    assignment_identity,
)
from planner.src.tamp.hierarchy import (
    ParameterizedSkillAction, PredicateGoal, SkillRegistry, WorldState,
    program_identity, validate_skill_program,
)
from planner.src.tamp.semantic import SemanticModelError
from planner.src.tamp.proc3s import PRoC3SGenerationFailure
from planner.src.tamp.skill_planning import (
    SkillProgramGenerator, StripsProgramGenerator,
)


@dataclasses.dataclass(frozen=True)
class SkillExecution:
    observation: Any
    success: bool
    reason: str
    duration_s: float = 0.0
    episode_finished: bool = False
    retryable: bool = True
    failure_details: Mapping[str, Any] = dataclasses.field(default_factory=dict)
    involved_objects: tuple[str, ...] = ()
    geometric_conditioning: Mapping[str, bool] = dataclasses.field(default_factory=dict)
    geometry_guarantee: bool = False


class SkillExecutor(Protocol):
    def execute(self, action: ParameterizedSkillAction) -> SkillExecution: ...


class WorldObserver(Protocol):
    def observe(self, observation: Any) -> WorldState: ...


@dataclasses.dataclass(frozen=True)
class RecoveryLimits:
    semantic_replans: int = 2
    skill_replans: int = 2
    geometry_retries: int = 3
    skill_retries: int = 1
    max_skill_executions: int = 20
    max_wall_time_s: float = 1200.0
    provider_stage_retries: int = 2
    provider_retry_delay_s: float = 2.0

    def __post_init__(self):
        if type(self.provider_stage_retries) is not int or self.provider_stage_retries < 0:
            raise ValueError("provider_stage_retries must be a nonnegative integer")
        if not math.isfinite(self.provider_retry_delay_s) or self.provider_retry_delay_s < 0:
            raise ValueError("provider_retry_delay_s must be finite and nonnegative")
        if not math.isfinite(self.max_wall_time_s) or self.max_wall_time_s <= 0:
            raise ValueError("max_wall_time_s must be finite and positive")


@dataclasses.dataclass(frozen=True)
class OnlineResult:
    success: bool
    reason: str
    world: WorldState
    metrics: Mapping[str, Any]


class JsonlTrace:
    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists():
            raise FileExistsError(self.path)

    def __call__(self, event: Mapping[str, Any]):
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(dict(event), ensure_ascii=False, default=str) + "\n")


def verify_expected_effects(action: ParameterizedSkillAction,
                            world: WorldState) -> bool:
    return set(action.expected_effects).issubset(world.facts)


def current_station_pick_program(program, world, registry):
    """Try the measured pick station before another model-proposed relocation."""
    if len(program.steps) != 2 or program.steps[0].skill != "NavigateToPick":
        return None
    first, pick = program.steps
    target = first.arguments.get("object")
    if (pick.skill != "PickLift" or pick.arguments.get("object") != target
            or PredicateGoal("at_pick_pose", (target,)) not in world.facts):
        return None
    spec = registry[pick.skill]
    if not all(PredicateGoal(fact.predicate, tuple(
            pick.arguments[value[1:]] if value.startswith("$") else value
            for value in fact.arguments)) in world.facts
            for fact in spec.preconditions):
        return None
    variables = set(pick.continuous_variables.values())
    changes = {"steps": (pick,)}
    if hasattr(program, "parameter_domains"):
        changes["parameter_domains"] = {
            name: sampler for name, sampler in program.parameter_domains.items()
            if name in variables
        }
    if hasattr(program, "parameter_subdomains"):
        changes["parameter_subdomains"] = {
            name: bounds for name, bounds in program.parameter_subdomains.items() if name in variables}
    direct = dataclasses.replace(program, **changes)
    try:
        validate_skill_program(direct, registry, frozenset(world.objects))
    except ValueError:
        return None
    return direct


class IncrementalTampRunner:
    """Bounded four-level recovery without resetting the robot between skills."""

    def __init__(self, *, semantic, registry: SkillRegistry,
                 solver_factory: Callable[[WorldState], Any],
                 executor: SkillExecutor, observer: WorldObserver,
                 trace: Callable[[Mapping[str, Any]], None],
                 limits: RecoveryLimits = RecoveryLimits(),
                 program_generator: SkillProgramGenerator | None = None):
        self.semantic = semantic
        self.registry = registry
        self.solver_factory = solver_factory
        self.executor = executor
        self.observer = observer
        self.trace = trace
        self.limits = limits
        self.program_generator = (program_generator if program_generator is not None
                                  else StripsProgramGenerator(registry))

    def run(self, *, task: str, task_goals: tuple[PredicateGoal, ...],
            initial_observation: Any, initial_geometry_state: Mapping[str, Any],
            predicate_arity: Mapping[str, int],
            images: tuple[Mapping[str, str], ...] = (),
            deadline_monotonic_s: float | None = None) -> OnlineResult:
        self._run_started_at = time.perf_counter()
        deadline = (self._run_started_at + self.limits.max_wall_time_s
                    if deadline_monotonic_s is None else deadline_monotonic_s)
        world = self.observer.observe(initial_observation)
        metrics: dict[str, Any] = {
            "task_success": False, "subgoal_success": 0, "skill_success": 0,
            "semantic_planning_latency_s": 0.0,
            "skill_planning_latency_s": 0.0,
            "geometry_solving_latency_s": 0.0,
            "execution_time_s": 0.0,
            "num_semantic_replans": 0, "num_skill_replans": 0,
            "num_geometry_retries": 0, "num_skill_retries": 0,
            "num_constraint_failures": 0, "skill_executions": 0,
            "vlm_calls": 0, "llm_calls": 0,
            "wall_time_budget_s": self.limits.max_wall_time_s,
            "provider_stage_retries": 0,
        }
        if hasattr(self.executor, "set_deadline"):
            self.executor.set_deadline(deadline)
        for planner in (self.semantic, self.program_generator):
            client = getattr(planner, "client", None)
            if hasattr(client, "set_deadline"):
                client.set_deadline(deadline)

        def model_call(stage, function, *args, **kwargs):
            while True:
                try:
                    return function(*args, **kwargs)
                except (SemanticModelError, PRoC3SGenerationFailure) as error:
                    failure = error.provider_failure
                    if (failure is None or not failure["retryable"]
                            or metrics["provider_stage_retries"] >= self.limits.provider_stage_retries
                            or time.perf_counter() + self.limits.provider_retry_delay_s >= deadline):
                        raise
                    metrics["provider_stage_retries"] += 1
                    self.trace({"event": "provider_stage_retry", "stage": stage,
                                "provider_failure": failure,
                                "retry": metrics["provider_stage_retries"],
                                "remaining_run_budget_s": deadline - time.perf_counter(),
                                "delay_s": self.limits.provider_retry_delay_s})
                    time.sleep(self.limits.provider_retry_delay_s)

        def expired(stage):
            if time.perf_counter() < deadline:
                return None
            self.trace({"event": "wall_time_budget_exhausted", "stage": stage,
                        "elapsed_s": time.perf_counter() - self._run_started_at})
            return self._finish(False, "wall_time_budget_exhausted", world, metrics)

        self.trace({"event": "initial_world_state", "facts": [dataclasses.asdict(f) for f in world.facts],
                    "objects": world.objects, "robot": world.robot, "task": task})
        self.trace({"event": "skill_registry", "skills": [
            dataclasses.asdict(spec) for spec in self.registry]})
        feedback: list[dict] = []
        proposed: tuple[PredicateGoal, ...] = ()
        geometry_state = dict(initial_geometry_state)
        failure_context = None
        for semantic_attempt in range(self.limits.max_skill_executions):
            result = expired("semantic_planning")
            if result is not None:
                return result
            if all(goal in world.facts for goal in task_goals):
                metrics["task_success"] = True
                return self._finish(True, "task_goal_verified", world, metrics)
            start = time.perf_counter()
            executions_before_proposal = metrics["skill_executions"]
            subgoal_failed = False
            try:
                proposed = model_call("semantic", self.semantic.propose,
                    task=task, world={"objects": world.objects,
                                      "facts": [dataclasses.asdict(f) for f in world.facts],
                                      "robot": world.robot},
                    predicate_arity=predicate_arity, feedback=tuple(feedback), images=images,
                )
            except SemanticModelError as error:
                metrics["semantic_planning_latency_s"] += time.perf_counter() - start
                metrics["vlm_calls"] = getattr(self.semantic, "calls", 0)
                result = expired("semantic_planning")
                if result is not None:
                    return result
                self.trace({"event": "semantic_planning_failure", "reason": str(error)})
                reason = ("provider_retry_exhausted" if error.provider_failure["retryable"]
                          else "provider_configuration_error") if error.provider_failure else "semantic_model_error"
                return self._finish(False, reason, world, metrics)
            metrics["semantic_planning_latency_s"] += time.perf_counter() - start
            metrics["vlm_calls"] = getattr(self.semantic, "calls", 0)
            result = expired("semantic_planning")
            if result is not None:
                return result
            self.trace({"event": "semantic_subgoal", "attempt": semantic_attempt + 1,
                        "subgoals": [dataclasses.asdict(goal) for goal in proposed],
                        "feedback": feedback[-1:]})
            for subgoal in proposed:
                if subgoal in world.facts:
                    continue
                excluded_programs: set[tuple] = set()
                abstract_failures: list[dict] = []
                subgoal_complete = False
                skill_failures = 0
                planning_cycles = 0
                pending_reposition_program = None
                current_station_attempts = set()
                recovery_blocked = None
                while planning_cycles < self.limits.max_skill_executions:
                    result = expired("skill_planning")
                    if result is not None:
                        return result
                    planning_cycles += 1
                    start = time.perf_counter()
                    if pending_reposition_program is None:
                        try:
                            program = model_call("program", self.program_generator.generate,
                                world, (subgoal,), feedback=tuple(abstract_failures),
                                excluded_programs=frozenset(excluded_programs),
                            )
                        except ValueError as error:
                            metrics["skill_planning_latency_s"] += time.perf_counter() - start
                            result = expired("skill_planning")
                            if result is not None:
                                return result
                            self.trace({"event": "skill_planning_failure",
                                        "subgoal": dataclasses.asdict(subgoal), "reason": str(error),
                                        "provider_failure": getattr(error, "provider_failure", None)})
                            provider_failure = getattr(error, "provider_failure", None)
                            if provider_failure is not None:
                                reason = ("provider_retry_exhausted" if provider_failure["retryable"]
                                          else "provider_configuration_error")
                                return self._finish(False, reason, world, metrics)
                            break
                        direct = current_station_pick_program(program, world, self.registry)
                        if direct is not None:
                            key = (world.observation_id, program_identity(direct))
                            if key not in current_station_attempts:
                                current_station_attempts.add(key)
                                pending_reposition_program = program
                                program = direct
                                self.trace({"event": "current_station_pick_check",
                                            "observation_id": world.observation_id,
                                            "deferred_program": program_identity(pending_reposition_program)})
                    else:
                        program = pending_reposition_program
                        pending_reposition_program = None
                    metrics["skill_planning_latency_s"] += time.perf_counter() - start
                    result = expired("skill_planning")
                    if result is not None:
                        return result
                    self.trace({"event": "skill_skeleton", "steps": [
                        dataclasses.asdict(step) for step in program.steps]})
                    if not program.steps:
                        subgoal_complete = True
                        break
                    if recovery_blocked is not None:
                        blockers = set(recovery_blocked.get("blockers", ()))
                        first = program.steps[0]
                        target = first.arguments.get("object")
                        safe_local_pick = (
                            first.skill == "PickLift"
                            and blockers == {"navigation_precheck_failed"}
                            and PredicateGoal("gripper_empty", ()) in world.facts
                            and PredicateGoal("at_pick_pose", (target,)) in world.facts
                        )
                        if not safe_local_pick:
                            self.trace({"event": "recovery_candidate_blocked",
                                        "skill": first.skill,
                                        "preconditions": recovery_blocked,
                                        "reason": "No verified local recovery can start from the measured state"})
                            return self._finish(False, "recovery_preconditions_failed", world, metrics)
                    excluded_assignments: set[str] = set()
                    current_skill = program.steps[0].skill
                    prerequisite_progress = False
                    for geometric_attempt in range(self.limits.geometry_retries + 1):
                        result = expired("geometry_search")
                        if result is not None:
                            return result
                        solver = self.solver_factory(world)
                        if hasattr(solver, "set_deadline"):
                            solver.set_deadline(deadline)
                        start = time.perf_counter()
                        try:
                            plan = solver.solve(
                                world, program, geometry_state,
                                failure_context=failure_context,
                                excluded_assignments=frozenset(excluded_assignments),
                            )
                        except GeometricUnsat as error:
                            metrics["geometry_solving_latency_s"] += time.perf_counter() - start
                            metrics["num_constraint_failures"] += len(error.constraints)
                            reported_failure = getattr(error, "program_feedback", None)
                            failed_skill = (reported_failure or {}).get("skill", current_skill)
                            self.trace({"event": "geometric_unsat", "skill": failed_skill,
                                        "constraints": [dataclasses.asdict(c) for c in error.constraints],
                                        "failure_feedback": getattr(error, "program_feedback", None),
                                        "solver_diagnostics": getattr(error, "solver_diagnostics", None)})
                            reported_items = getattr(error, "program_feedbacks", None)
                            if reported_items:
                                abstract_failures.extend(reported_items)
                            else:
                                abstract_failures.append(reported_failure or abstract_constraint_feedback(
                                    error.constraints, skill=current_skill,
                                    target=program.steps[0].arguments.get("object"),
                                ))
                            failure_context = LowLevelFailure(
                                failed_skill, "geometry_unsat",
                                tuple(sorted({name for item in error.constraints for name in item.involved_objects})),
                                {"observation_id": world.observation_id,
                                 "constraints": [dataclasses.asdict(item) for item in error.constraints],
                                 "solver_diagnostics": getattr(error, "solver_diagnostics", None)})
                            if (getattr(error, "retryable_search", True)
                                    and solver.capabilities.retry_after_search_failure
                                    and geometric_attempt < self.limits.geometry_retries):
                                metrics["num_geometry_retries"] += 1
                                self.trace({"event": "recovery_action",
                                            "level": "geometric_optimizer_retry",
                                            "failure_feedback": reported_failure})
                                continue
                            break
                        except GeometryBackendError as error:
                            metrics["geometry_solving_latency_s"] += time.perf_counter() - start
                            self.trace({"event": "geometry_backend_error",
                                        "returncode": error.returncode,
                                        "log_path": error.log_path})
                            return self._finish(False, "geometry_backend_error", world, metrics)
                        except GeometryDeadlineExceeded:
                            metrics["geometry_solving_latency_s"] += time.perf_counter() - start
                            result = expired("geometry_search")
                            return result or self._finish(False, "wall_time_budget_exhausted", world, metrics)
                        metrics["geometry_solving_latency_s"] += time.perf_counter() - start
                        result = expired("geometry_search")
                        if result is not None:
                            return result
                        metrics["num_constraint_failures"] += len(plan.constraints)
                        self.trace({"event": "selected_parameters", "skill": current_skill,
                                    "assignments": plan.assignments,
                                    "constraint_failures": [dataclasses.asdict(c)
                                                            for c in plan.constraints]})
                        action = plan.actions[0]
                        succeeded = False
                        for retry in range(self.limits.skill_retries + 1):
                            result = expired("skill_execution")
                            if result is not None:
                                return result
                            if metrics["skill_executions"] >= self.limits.max_skill_executions:
                                return self._finish(False, "skill_execution_budget", world, metrics)
                            previous_world = world
                            outcome = self.executor.execute(action)
                            metrics["skill_executions"] += 1
                            metrics["execution_time_s"] += outcome.duration_s
                            world = self.observer.observe(outcome.observation)
                            # Both success and failure can move the robot or an
                            # object. Every subsequent solve uses measured state.
                            if hasattr(self.observer, "geometry_state"):
                                geometry_state = self.observer.geometry_state(
                                    outcome.observation, geometry_state)
                            if hasattr(self.observer, "capture_images"):
                                images = self.observer.capture_images(outcome.observation)
                            if outcome.reason == "wall_time_budget_exhausted":
                                self.trace({"event": "wall_time_budget_exhausted",
                                            "stage": "skill_execution", "skill": action.skill_name})
                                return self._finish(False, "wall_time_budget_exhausted", world, metrics)
                            result = expired("skill_execution")
                            if result is not None:
                                return result
                            verified = verify_expected_effects(action, world)
                            self.trace({"event": "skill_execution", "skill": action.skill_name,
                                        "attempt": retry + 1, "runtime_success": outcome.success,
                                        "runtime_reason": outcome.reason,
                                        "episode_finished": outcome.episode_finished,
                                        "retryable": outcome.retryable,
                                        "failure_details": dict(outcome.failure_details),
                                        "geometric_conditioning": dict(outcome.geometric_conditioning),
                                        "geometry_guarantee": outcome.geometry_guarantee,
                                        "geometry_guarantee_not_enforced":
                                        not outcome.geometry_guarantee})
                            self.trace({"event": "verifier_result", "skill": action.skill_name,
                                        "verified": verified,
                                        "facts": [dataclasses.asdict(f) for f in world.facts],
                                        "objects": world.objects, "robot": world.robot})
                            if outcome.episode_finished:
                                if verified:
                                    metrics["skill_success"] += 1
                                if subgoal in world.facts:
                                    metrics["subgoal_success"] += 1
                                complete = all(goal in world.facts for goal in task_goals)
                                metrics["task_success"] = complete
                                return self._finish(
                                    complete, "task_goal_verified" if complete
                                    else "episode_finished", world, metrics)
                            if verified:
                                failure_context = None
                                recovery_blocked = None
                                succeeded = True
                                metrics["skill_success"] += 1
                                break
                            failure = LowLevelFailure(
                                skill=action.skill_name,
                                reason=outcome.reason if not outcome.success
                                       else "expected_effect_not_observed",
                                involved_objects=tuple(sorted(set(action.symbolic_args.values())
                                                              | set(outcome.involved_objects))),
                                details={**outcome.failure_details,
                                         "observation_id": world.observation_id,
                                         "geometric_parameters": dict(action.geometric_parameters),
                                         "assignments": dict(plan.assignments)},
                            )
                            failure_context = failure
                            abstract_failures.append(failure.abstract().as_feedback())
                            recovery = outcome.failure_details.get("recovery_preconditions")
                            if recovery is not None and not recovery["allowed"]:
                                self.trace({"event": "recovery_blocked", "skill": action.skill_name,
                                            "reason": outcome.reason, "preconditions": dict(recovery)})
                                recovery_blocked = dict(recovery)
                                # The deferred relocation predates this physical
                                # failure. Give fresh feedback to the generator
                                # before considering any recovery program.
                                pending_reposition_program = None
                                break
                            if not outcome.retryable:
                                break
                            if (world.observation_id != previous_world.observation_id
                                    or world.facts != previous_world.facts
                                    or world.robot != previous_world.robot
                                    or world.objects != previous_world.objects):
                                self.trace({"event": "recovery_action",
                                            "level": "state_changed_geometry_recheck",
                                            "reason": "Do not retry stale parameters after state changes"})
                                break
                            if retry < self.limits.skill_retries:
                                metrics["num_skill_retries"] += 1
                                self.trace({"event": "recovery_action", "level": "skill_retry"})
                        if succeeded:
                            if subgoal in world.facts:
                                subgoal_complete = True
                                metrics["subgoal_success"] += 1
                            else:
                                if not hasattr(self.observer, "geometry_state"):
                                    geometry_state = self._geometry_state_after(action, geometry_state)
                                prerequisite_progress = True
                            break
                        if recovery_blocked is not None:
                            # Feed the observed failure to the planner once;
                            # never re-execute a stale action in contact.
                            break
                        excluded_assignments.add(assignment_identity({
                            variable: plan.assignments[variable]
                            for variable in program.steps[0].continuous_variables.values()
                        }))
                        candidate_identity = action.geometric_parameters.get("candidate_identity")
                        if candidate_identity is not None:
                            excluded_assignments.add(candidate_identity)
                        if geometric_attempt < self.limits.geometry_retries:
                            metrics["num_geometry_retries"] += 1
                            self.trace({"event": "recovery_action",
                                        "level": "geometric_reparameterization"})
                    if subgoal_complete:
                        break
                    if recovery_blocked is not None and not abstract_failures:
                        raise RuntimeError("Blocked recovery lost its failure feedback")
                    if prerequisite_progress:
                        # Observe and solve the remaining symbolic goal from
                        # the measured state; never execute a stale full plan.
                        # A failed skeleton only applies to its starting world.
                        # Clearing an obstacle may make the same Pick feasible.
                        excluded_programs.clear()
                        if abstract_failures:
                            self.trace({"event": "program_feedback_reset",
                                        "reason": "verified_prerequisite_progress",
                                        "observation_id": world.observation_id,
                                        "discarded_failure_count": len(abstract_failures)})
                            abstract_failures.clear()
                        skill_failures = 0
                        continue
                    if pending_reposition_program is not None:
                        self.trace({"event": "current_station_pick_infeasible",
                                    "observation_id": world.observation_id,
                                    "reason": "Measured-station check did not produce a verified pick"})
                        continue
                    skill_failures += 1
                    if skill_failures > self.limits.skill_replans:
                        break
                    # An approximate optimizer failure is not evidence against
                    # any particular skeleton. Let the model retain it.
                    may_revise_domain = getattr(self.program_generator, "supports_domain_revision", False)
                    proved_unsat = bool(abstract_failures and abstract_failures[-1].get("program_unsat"))
                    if (not may_revise_domain or proved_unsat) and (
                            not abstract_failures or abstract_failures[-1].get("failure_source") not in {
                                "approximate_tolerance_unmet", "optimizer_timeout", "geometry_wall_time"}):
                        excluded_programs.add(program_identity(program))
                    metrics["num_skill_replans"] += 1
                    self.trace({"event": "recovery_action", "level": "skill_replan",
                                "excluded_program": (program_identity(program)
                                                     if program_identity(program) in excluded_programs else None),
                                "feedback": abstract_failures[-3:]})
                if not subgoal_complete:
                    subgoal_failed = True
                    feedback.append(SemanticFailure(
                        subgoal, tuple(ProgramFailure.from_feedback(failure)
                                       for failure in abstract_failures[-3:]),
                    ).as_feedback())
                    if recovery_blocked is not None:
                        self.trace({"event": "recovery_exhausted_at_measured_state",
                                    "preconditions": recovery_blocked,
                                    "feedback": abstract_failures[-3:]})
                        return self._finish(False, "recovery_preconditions_failed", world, metrics)
                    break
            if all(goal in world.facts for goal in task_goals):
                metrics["task_success"] = True
                return self._finish(True, "task_goal_verified", world, metrics)
            if not subgoal_failed and metrics["skill_executions"] > executions_before_proposal:
                self.trace({"event": "semantic_continuation",
                            "reason": "intermediate_subgoals_verified"})
                continue
            if not subgoal_failed:
                feedback.append({"reason": "proposal_already_satisfied_without_task_progress"})
            if metrics["num_semantic_replans"] >= self.limits.semantic_replans:
                return self._finish(False, "semantic_replan_exhausted", world, metrics)
            metrics["num_semantic_replans"] += 1
            self.trace({"event": "recovery_action", "level": "semantic_replan",
                        "feedback": feedback[-1:]})
        return self._finish(False, "semantic_cycle_budget", world, metrics)

    @staticmethod
    def _geometry_state_after(action, previous):
        next_state = dict(previous)
        if action.skill_name == "NavigateToPick":
            candidates = [value for key, value in action.geometric_parameters.items()
                          if key.startswith("base_pose_")]
            if candidates:
                next_state["base_pose"] = candidates[0]
        return next_state

    def _finish(self, success, reason, world, metrics):
        metrics["wall_time_s"] = time.perf_counter() - self._run_started_at
        metrics["llm_calls"] = self.program_generator.calls
        self.trace({"event": "final_result", "success": success, "reason": reason,
                    "metrics": metrics})
        return OnlineResult(success, reason, world, metrics)
