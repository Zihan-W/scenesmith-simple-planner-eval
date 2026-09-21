"""Incremental TAMP: semantic → skeleton → geometry → skill → observe → verify."""

from __future__ import annotations

import dataclasses
import json
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, Protocol

from examples.online_manipulation.tamp_failures import LowLevelFailure, ProgramFailure, SemanticFailure

from examples.online_manipulation.tamp_geometry import (
    GeometricUnsat, abstract_constraint_feedback, assignment_identity,
)
from examples.online_manipulation.tamp_hierarchy import (
    ParameterizedSkillAction, PredicateGoal, SkillRegistry, WorldState,
    program_identity,
)
from examples.online_manipulation.tamp_semantic import SemanticModelError
from examples.online_manipulation.tamp_skill_planning import (
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
            images: tuple[Mapping[str, str], ...] = ()) -> OnlineResult:
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
        }
        self.trace({"event": "initial_world_state", "facts": [dataclasses.asdict(f) for f in world.facts],
                    "objects": world.objects, "robot": world.robot, "task": task})
        self.trace({"event": "skill_registry", "skills": [
            dataclasses.asdict(spec) for spec in self.registry]})
        feedback: list[dict] = []
        proposed: tuple[PredicateGoal, ...] = ()
        geometry_state = dict(initial_geometry_state)
        failure_context = None
        for semantic_attempt in range(self.limits.max_skill_executions):
            if all(goal in world.facts for goal in task_goals):
                metrics["task_success"] = True
                return self._finish(True, "task_goal_verified", world, metrics)
            start = time.perf_counter()
            executions_before_proposal = metrics["skill_executions"]
            subgoal_failed = False
            try:
                proposed = self.semantic.propose(
                    task=task, world={"objects": world.objects,
                                      "facts": [dataclasses.asdict(f) for f in world.facts],
                                      "robot": world.robot},
                    predicate_arity=predicate_arity, feedback=tuple(feedback), images=images,
                )
            except SemanticModelError as error:
                metrics["semantic_planning_latency_s"] += time.perf_counter() - start
                metrics["vlm_calls"] = getattr(self.semantic, "calls", 0)
                self.trace({"event": "semantic_planning_failure", "reason": str(error)})
                return self._finish(False, "semantic_model_error", world, metrics)
            metrics["semantic_planning_latency_s"] += time.perf_counter() - start
            metrics["vlm_calls"] = getattr(self.semantic, "calls", 0)
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
                while planning_cycles < self.limits.max_skill_executions:
                    planning_cycles += 1
                    start = time.perf_counter()
                    try:
                        program = self.program_generator.generate(
                            world, (subgoal,), feedback=tuple(abstract_failures),
                            excluded_programs=frozenset(excluded_programs),
                        )
                    except ValueError as error:
                        metrics["skill_planning_latency_s"] += time.perf_counter() - start
                        self.trace({"event": "skill_planning_failure",
                                    "subgoal": dataclasses.asdict(subgoal), "reason": str(error)})
                        break
                    metrics["skill_planning_latency_s"] += time.perf_counter() - start
                    self.trace({"event": "skill_skeleton", "steps": [
                        dataclasses.asdict(step) for step in program.steps]})
                    if not program.steps:
                        subgoal_complete = True
                        break
                    excluded_assignments: set[str] = set()
                    current_skill = program.steps[0].skill
                    prerequisite_progress = False
                    for geometric_attempt in range(self.limits.geometry_retries + 1):
                        solver = self.solver_factory(world)
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
                            self.trace({"event": "geometric_unsat", "skill": current_skill,
                                        "constraints": [dataclasses.asdict(c)
                                                        for c in error.constraints]})
                            abstract_failures.append(getattr(error, "program_feedback", None) or abstract_constraint_feedback(
                                error.constraints, skill=current_skill,
                                target=program.steps[0].arguments.get("object"),
                            ))
                            failure_context = LowLevelFailure(
                                current_skill, "geometry_unsat",
                                tuple(sorted({name for item in error.constraints for name in item.involved_objects})),
                                {"observation_id": world.observation_id,
                                 "constraints": [dataclasses.asdict(item) for item in error.constraints]})
                            break
                        metrics["geometry_solving_latency_s"] += time.perf_counter() - start
                        metrics["num_constraint_failures"] += len(plan.constraints)
                        self.trace({"event": "selected_parameters", "skill": current_skill,
                                    "assignments": plan.assignments,
                                    "constraint_failures": [dataclasses.asdict(c)
                                                            for c in plan.constraints]})
                        action = plan.actions[0]
                        succeeded = False
                        for retry in range(self.limits.skill_retries + 1):
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
                                return self._finish(False, "recovery_preconditions_failed", world, metrics)
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
                    if prerequisite_progress:
                        # Observe and solve the remaining symbolic goal from
                        # the measured state; never execute a stale full plan.
                        # A failed skeleton only applies to its starting world.
                        # Clearing an obstacle may make the same Pick feasible.
                        excluded_programs.clear()
                        skill_failures = 0
                        continue
                    skill_failures += 1
                    if skill_failures > self.limits.skill_replans:
                        break
                    excluded_programs.add(program_identity(program))
                    metrics["num_skill_replans"] += 1
                    self.trace({"event": "recovery_action", "level": "skill_replan",
                                "excluded_program": program_identity(program),
                                "feedback": abstract_failures[-3:]})
                if not subgoal_complete:
                    subgoal_failed = True
                    feedback.append(SemanticFailure(
                        subgoal, tuple(ProgramFailure.from_feedback(failure)
                                       for failure in abstract_failures[-3:]),
                    ).as_feedback())
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
        metrics["llm_calls"] = self.program_generator.calls
        self.trace({"event": "final_result", "success": success, "reason": reason,
                    "metrics": metrics})
        return OnlineResult(success, reason, world, metrics)
