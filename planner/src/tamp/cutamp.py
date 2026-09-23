"""Real fixed-skeleton cuTAMP geometry backend with exact SceneSmith postchecks.

The isolated CUDA worker calls NVlabs' initializer, costs and ParticleOptimizer.
This module never generates/revises a skill skeleton or invokes a CCSP solver.
Current conversion supports NavigateToPick, PickLift and their pair; other
skeletons fail explicitly. Particle feasibility is not a collision certificate.
"""

import copy
import dataclasses
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np

from simulation.src import Pose
from simulation.src.robots.adapters.description import drake_pose

from .cutamp_problem import ContinuousProblemBuilder
from .subdomains import validate_program_subdomains, candidate_in_subdomain
from .cutamp_base_domain import base_domain_halfspaces
from .geometry import (
    SolverCapabilities,
    GeometricUnsat, _bind_effect, _constraint_objects, _parameter_value,
    assignment_identity, receive_failure_context,
)
from .hierarchy import (
    ConstraintResult, ParameterizedSkillAction, ParameterizedSkillPlan,
    validate_skill_program,
)
from .planner import Subgoal
from .cutamp_feedback import exact_failure_feedback, optimizer_failure_feedback, postcheck_budget_feedback
from .failures import ProgramFailure
from .geometry import GeometryBackendError, GeometryDeadlineExceeded, require_time_remaining


@dataclasses.dataclass(frozen=True)
class CuTAMPSettings:
    """Explicit installed backend and model artifacts; no hidden machine paths."""

    gpu_python: str
    cutamp_root: str
    kinematics_template: str
    robot_template: str
    multipliers: str
    tolerances: str
    grasp_calibration: str
    num_particles: int = 64
    num_opt_steps: int = 2000
    conf_lr: float = 0.001
    max_postchecks: int = 8
    seed: int = 500
    integrity_manifest: str | None = None
    solve_timeout_s: float = 360.0
    postcheck_timeout_s: float = 180.0
    postcheck_order: str = "cost_quantiles"
    reuse_navigation_pick_witness: bool = True
    postcheck_quality_window: int | None = None
    adaptive_postcheck: dict | None = None

    def __post_init__(self):
        if self.adaptive_postcheck is not None:
            from .postcheck_budget import AdaptivePostcheckBudget
            AdaptivePostcheckBudget.validate(self.adaptive_postcheck)
        if self.postcheck_order not in {"cost", "cost_quantiles"}:
            raise ValueError("Unknown cuTAMP postcheck order")
        if type(self.reuse_navigation_pick_witness) is not bool:
            raise ValueError("reuse_navigation_pick_witness must be boolean")
        if (self.postcheck_quality_window is not None
                and (type(self.postcheck_quality_window) is not int
                     or self.postcheck_quality_window < 0)):
            raise ValueError("postcheck_quality_window must be a nonnegative integer or null")
        for name in ("num_particles", "num_opt_steps", "max_postchecks"):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("solve_timeout_s", "postcheck_timeout_s"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError("cuTAMP seed must be an integer in [0, 2**32)")
        if not math.isfinite(self.conf_lr) or self.conf_lr <= 0:
            raise ValueError("cuTAMP configuration learning rate must be positive")

    @classmethod
    def from_file(cls, path):
        """Resolve explicit artifact paths relative to the configuration file."""
        path = Path(path).resolve()
        data = json.loads(path.read_text())
        for name in ("gpu_python", "cutamp_root", "kinematics_template", "robot_template",
                     "multipliers", "tolerances", "grasp_calibration"):
            expanded = os.path.expandvars(data[name])
            if "$" in expanded:
                raise ValueError(f"Unresolved environment variable in cuTAMP {name}")
            value = Path(expanded)
            if not value.is_absolute():
                value = path.parent / value
            # A venv Python symlink must retain its entry-point path.
            data[name] = str(value.absolute() if name == "gpu_python" else value.resolve())
            if not value.exists():
                raise FileNotFoundError(value)
        if data.get("integrity_manifest"):
            value = Path(os.path.expandvars(data["integrity_manifest"]))
            data["integrity_manifest"] = str((path.parent / value).resolve())
        return cls(**data)


def run_cutamp_worker(settings, repo, problem, output, *, optimize_base_xy,
                      deadline_monotonic_s=None):
    """Run and retain the actual CUDA optimizer without shell or fallback."""
    require_time_remaining(deadline_monotonic_s)
    from .cutamp_backend.integrity import verify_installation
    integrity = verify_installation(settings)
    repo, problem, output = Path(repo).resolve(), Path(problem).resolve(), Path(output).resolve()
    (output.parent / "verified_installation.json").write_text(json.dumps(integrity, indent=2) + "\n")
    command = [str(Path(settings.gpu_python).absolute()),
               "-m", "planner.src.tamp.cutamp_backend.worker"]
    for flag, value in (("geometry", problem / "geometry.json"),
                        ("kinematics", problem / "kinematics.json"),
                        ("robot-config", problem / "robot_config.json"),
                        ("multipliers", settings.multipliers), ("tolerances", settings.tolerances),
                        ("grasp-calibration", settings.grasp_calibration), ("output-root", output)):
        command.extend(("--" + flag, str(Path(value).resolve())))
    command.extend(("--exact-stationary-target-contact", "--num-particles", str(settings.num_particles),
                    "--num-opt-steps", str(settings.num_opt_steps), "--conf-lr", str(settings.conf_lr),
                    "--seed", str(settings.seed)))
    if optimize_base_xy:
        command.extend(("--optimize-base-xy", "--exact-planar-robot-support"))
    else:
        command.append("--exact-stationary-robot-support")
    # Resolve the installed planner package, independently of the asset repository.
    package_root = Path(__file__).resolve().parents[3]
    environment = dict(os.environ, PYTHONPATH=os.pathsep.join(
        (str(Path(settings.cutamp_root).resolve()), str(package_root))))
    (output.parent / "worker_command.json").write_text(json.dumps(command, indent=2) + "\n")
    log_path = output.parent / "optimizer.log"
    with log_path.open("w") as stream:
        try:
            remaining_s = (None if deadline_monotonic_s is None else
                           max(0.001, deadline_monotonic_s - time.perf_counter()))
            subprocess.run(command, cwd=repo, env=environment, stdout=stream,
                           stderr=subprocess.STDOUT, check=True, timeout=remaining_s)
        except subprocess.TimeoutExpired as error:
            raise GeometryDeadlineExceeded(
                f"cuTAMP worker exceeded the remaining wall-time budget; log: {log_path}"
            ) from error
        except subprocess.CalledProcessError as error:
            raise GeometryBackendError(returncode=error.returncode,
                                       log_path=str(log_path)) from error
    return json.loads((output / "result.json").read_text())


def order_postcheck_candidates(candidates, mode):
    """Interleave four cost strata; expose order, without claiming full coverage."""
    ranked = sorted(candidates, key=lambda item: (
        item["hard_constraint_cost"], abs(item["grasp_lateral_offset_m"]),
        sum(value * value for value in item["arm_joint_positions"])))
    if mode == "cost":
        return ranked
    if mode != "cost_quantiles":
        raise ValueError("Unknown cuTAMP postcheck order")
    groups = [ranked[i * len(ranked) // 4:(i + 1) * len(ranked) // 4] for i in range(4)]
    return [group[index] for index in range(max(map(len, groups), default=0))
            for group in groups if index < len(group)]


def postcheck_cutamp_candidates(registry, domain, world, program, candidates, initial_state,
                                output, *, max_postchecks=8, excluded_assignments=frozenset(),
                                trace=None, deadline_monotonic_s=None,
                                postcheck_order="cost_quantiles", reuse_navigation_pick_witness=True,
                                quality_window_after_first_pass=None, adaptive_budget=None,
                                run_deadline_monotonic_s=None):
    """Instantiate and exact-check GPU candidates; never replace their grasp q."""
    if max_postchecks <= 0:
        raise ValueError("Postcheck budget must be positive")
    if (quality_window_after_first_pass is not None
            and (type(quality_window_after_first_pass) is not int
                 or quality_window_after_first_pass < 0)):
        raise ValueError("Quality window must be a nonnegative integer or null")
    validate_skill_program(program, registry, frozenset(world.objects))
    validate_program_subdomains(program, registry)
    if tuple(step.skill for step in program.steps) not in (("NavigateToPick",), ("PickLift",), ("NavigateToPick", "PickLift")):
        raise ValueError("Unsupported fixed cuTAMP postcheck skeleton")
    pose = world.robot["base_link_pose"]
    observed_base = Pose(tuple(pose["translation_m"]), tuple(pose["quaternion_wxyz"]))
    state0 = copy.deepcopy(initial_state)
    if "base_pose" in state0 and state0["base_pose"] != observed_base:
        raise ValueError("Initial geometry state does not match the observed robot base")
    state0["base_pose"] = observed_base
    for candidate in candidates:
        values = [candidate["hard_constraint_cost"], candidate["grasp_lateral_offset_m"],
                  *candidate["arm_joint_positions"], *candidate["base_world_pose"]]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Nonfinite GPU candidate cannot enter exact postchecks")
    ordered = order_postcheck_candidates(candidates, postcheck_order)
    checks, failures, feasible_plans = [], [], []
    budget = None
    if adaptive_budget is not None:
        from .postcheck_budget import AdaptivePostcheckBudget
        budget = AdaptivePostcheckBudget(adaptive_budget, run_deadline_monotonic_s,
                                         deadline_monotonic_s)
        deadline_monotonic_s = budget.deadline
    first_pass_check_index = None
    first_pass_particle = None
    if hasattr(domain, "set_deadline"):
        domain.set_deadline(deadline_monotonic_s)
    stopped_by = "all_candidates_checked"
    for candidate in ordered:
        if budget is not None:
            reason = budget.before_candidate(time.perf_counter(), len(checks), bool(feasible_plans))
            if reason is not None:
                stopped_by = reason
                break
        if deadline_monotonic_s is not None and time.perf_counter() >= deadline_monotonic_s:
            stopped_by = "postcheck_wall_time"
            break
        candidate_started = time.perf_counter()
        incomplete = False
        base = candidate["base_world_pose"]
        base_pose = Pose(tuple(base[:3]), tuple(base[3:]))
        if program.steps[0].skill == "PickLift" and not np.allclose(
                drake_pose(base_pose).GetAsMatrix4(), drake_pose(observed_base).GetAsMatrix4(), atol=1e-5, rtol=0):
            raise ValueError("Fixed-base GPU candidate does not match the observed base frame")
        arm = domain.config.robot_adapter.spec.arm_groups["left"]
        if set(arm) != set(candidate["arm_joint_names"]):
            raise ValueError("GPU and runtime arm joint names differ")
        q = [candidate["arm_joint_positions"][candidate["arm_joint_names"].index(name)] for name in arm]
        state, assignments, actions, step_checks = copy.deepcopy(state0), {}, [], []
        rank = ()
        cached_pick = None
        joint_witness = (reuse_navigation_pick_witness and len(program.steps) == 2
                         and program.steps[0].skill == "NavigateToPick")
        for index, step in enumerate(program.steps):
            skill = Subgoal(step.skill, {"target": step.arguments["object"]})
            if step.skill == "NavigateToPick":
                parameters = {"base_x_m": base[0], "base_y_m": base[1],
                              "base_yaw_rad": drake_pose(base_pose).rotation().ToRollPitchYaw().yaw_angle()}
            elif step.skill == "PickLift":
                parameters = {"target": step.arguments["object"], "arm": "left",
                              "grasp_arm_joint_positions": q,
                              "grasp_lateral_offset_m": candidate["grasp_lateral_offset_m"],
                              "approach_segments": 8, "approach_height_offset_m": 0.0}
            else:
                raise ValueError(f"Unsupported cuTAMP skill: {step.skill}")
            outside = None
            if step.skill == "NavigateToPick":
                observed_yaw = drake_pose(observed_base).rotation().ToRollPitchYaw().yaw_angle()
                halfspaces = base_domain_halfspaces(domain.candidates, observed_yaw)
                outside = float(np.max(halfspaces[:, :2] @ np.asarray(base[:2]) + halfspaces[:, 2]))
            started = time.perf_counter()
            try:
                require_time_remaining(deadline_monotonic_s)
                if not candidate_in_subdomain(domain, program, step, parameters):
                    valid, reason, details = False, "domain_restriction", {"subdomain": True}
                elif outside is not None and outside > 1e-6:
                    valid, reason = False, "reachability"
                    details = {"base_domain_violation_m": outside,
                               "check_stage": "base_domain_halfspaces"}
                elif joint_witness and index == 0:
                    valid, reason, details = domain.check_navigation_corridor(skill, parameters, state)
                    if valid:
                        pick_step = program.steps[1]
                        pick_skill = Subgoal("PickLift", {"target": pick_step.arguments["object"]})
                        pick_parameters = {"target": pick_step.arguments["object"], "arm": "left",
                                           "grasp_arm_joint_positions": q,
                                           "grasp_lateral_offset_m": candidate["grasp_lateral_offset_m"],
                                           "approach_segments": 8, "approach_height_offset_m": 0.0}
                        pick_state = domain.predict(skill, parameters, state)
                        pick_result = (domain.check(pick_skill, pick_parameters, pick_state)
                            if candidate_in_subdomain(domain, program, pick_step, pick_parameters) else
                            (False, "domain_restriction", {"subdomain": True}))
                        cached_pick = (pick_parameters, pick_state, pick_result)
                        if pick_result[0]:
                            details = {**details, "pick_witness": {
                                "parameters": pick_parameters, "checks": pick_result[2]},
                                "navigation_witness_source": "same_particle_complete_pick_check"}
                        # A failed Pick is attributed at step 1 below. A corridor
                        # alone is never sufficient to return or execute a plan.
                elif cached_pick is not None:
                    old_parameters, old_state, pick_result = cached_pick
                    if (parameters != old_parameters or state.get("base_pose") != old_state.get("base_pose")
                            or state.get("snapshot_token") != old_state.get("snapshot_token")):
                        raise ValueError("Cannot reuse a witness across changed parameters or snapshot")
                    valid, reason, details = pick_result
                    details = {**details, "complete_pick_check_reused": True}
                else:
                    valid, reason, details = domain.check(skill, parameters, state)
                require_time_remaining(deadline_monotonic_s)
            except GeometryDeadlineExceeded:
                valid, reason = False, "postcheck_wall_time"
                details = {"incomplete": True}
                incomplete, stopped_by = True, "postcheck_wall_time"
            details = {**details, "check_wall_time_s": time.perf_counter() - started}
            if valid and step.skill == "NavigateToPick" and len(program.steps) == 1:
                # This program binds only a base pose. The registered navigation
                # check already supplies a complete exact PickLift witness. Its
                # arm configuration is not an executable GPU assignment here.
                witness = details["pick_witness"]
                rank = domain.rank_candidate(
                    Subgoal("PickLift", {"target": step.arguments["object"]}),
                    witness["parameters"], witness["checks"])
                details = {**details,
                           "gpu_grasp_configuration_is_execution_binding": False,
                           "navigation_witness_source": "registered_domain_exact_pick_witness"}
            details = {**details, "program_step": index}
            values = {variable: _parameter_value(role, parameters, details)
                      for role, variable in step.continuous_variables.items()}
            identity = assignment_identity({"skill": step.skill, "arguments": dict(step.arguments),
                                            "parameters": parameters})
            if identity in excluded_assignments or assignment_identity(values) in excluded_assignments:
                valid, reason = False, "excluded_assignment"
            if any(variable in assignments and assignments[variable] != value for variable, value in values.items()):
                valid, reason = False, "shared_variable_conflict"
            step_checks.append({"skill": step.skill, "valid": valid, "reason": reason,
                                "details": details, "assignment": parameters})
            if not valid:
                if not incomplete:
                    failures.append(ConstraintResult(False, reason, next(iter(values), ""), reason,
                                                      _constraint_objects(details, step.arguments["object"]), details))
                break
            if step.skill == "PickLift":
                if details["ik"]["grasp_pose_in_target"]["arm_joint_positions"] != q:
                    raise AssertionError("Postcheck replaced the GPU grasp configuration")
                rank = domain.rank_candidate(skill, parameters, details)
            spec = registry[step.skill]
            actions.append(ParameterizedSkillAction(
                step.skill, dict(step.arguments), {**parameters, **values, "checks": details,
                                                  "candidate_identity": identity},
                tuple(_bind_effect(effect, step.arguments) for effect in spec.add_effects),
                spec.supports_geometric_conditioning, dict(step.continuous_variables)))
            assignments.update(values)
            state = domain.predict(skill, parameters, state)
        valid = len(actions) == len(program.steps)
        record = {"particle_index": candidate["particle_index"], "valid": valid,
                  "optimizer_feasible": candidate["optimizer_feasible"],
                  "hard_constraint_cost": candidate["hard_constraint_cost"],
                  "completed": not incomplete,
                  "wall_time_s": time.perf_counter() - candidate_started,
                  "search_work": copy.deepcopy(getattr(domain, "search_work", {})),
                  "adaptive_mode": budget.mode if budget is not None else None,
                  "approximate_exact_disagreement": (not incomplete and valid != candidate["optimizer_feasible"]),
                  "reason": "valid" if valid else step_checks[-1]["reason"], "steps": step_checks}
        if len(step_checks) == 1:
            record.update(details=step_checks[0]["details"], assignment=step_checks[0]["assignment"])
        checks.append(record)
        with (Path(output) / "checks.jsonl").open("a") as stream:
            stream.write(json.dumps(record) + "\n")
        if trace is not None:
            trace({"event": "cutamp_exact_postcheck", **record})
        if valid:
            feasible_plans.append((rank, tuple(actions), assignments, candidate["particle_index"]))
            if first_pass_check_index is None:
                first_pass_check_index = len(checks) - 1
                first_pass_particle = candidate["particle_index"]
        if budget is not None:
            budget.after_candidate(time.perf_counter())
        if incomplete:
            break
        if budget is not None:
            reason = budget.before_candidate(time.perf_counter(), len(checks), bool(feasible_plans))
            if reason is not None:
                stopped_by = reason
                break
        if len(checks) == len(ordered):
            stopped_by = "all_candidates_checked"
            break
        if (first_pass_check_index is not None
                and quality_window_after_first_pass is not None
                and len(checks) - 1 >= first_pass_check_index + quality_window_after_first_pass):
            stopped_by = ("first_pass_found" if quality_window_after_first_pass == 0
                          else "quality_window_complete")
            break
        if len(checks) >= max_postchecks:
            stopped_by = "postcheck_count"
            break
    selected = min(feasible_plans, key=lambda item: item[0]) if feasible_plans else None
    # Count exhaustion is independent of the reason chosen to stop. A quality
    # window and the count limit can be reached by the same completed check.
    count_exhausted = len(checks) >= max_postchecks and len(checks) < len(ordered)
    (Path(output) / "candidate_selection.json").write_text(json.dumps({
        "max_postchecks": max_postchecks,
        "stop_reason": stopped_by,
        "count_exhausted": count_exhausted,
        "candidates_started": len(checks),
        "candidates_completed": sum(item["completed"] for item in checks),
        "candidates_available": len(ordered),
        "first_pass_check_index": first_pass_check_index,
        "first_pass_particle": first_pass_particle,
        "quality_window_after_first_pass": quality_window_after_first_pass,
        "adaptive_budget": budget.evidence(time.perf_counter()) if budget is not None else None,
        "candidate_order": postcheck_order,
        "ordered_particle_indices": [item["particle_index"] for item in ordered],
        "reuse_navigation_pick_witness": reuse_navigation_pick_witness,
        "selection_score": "existing_domain_gap_imbalance_then_lift_joint_margin_then_offset",
        "selected_particle": selected[3] if selected is not None else None,
        "selected_score": selected[0] if selected is not None else None,
        "configuration_replaced_by_ik": False}, indent=2) + "\n")
    plan = ParameterizedSkillPlan(selected[1], selected[2], tuple(failures)) if selected else None
    return plan, checks, tuple(failures)


class CuTAMPSolver:
    """Solve a supported fixed skill skeleton using the actual GPU optimizer."""
    capabilities = SolverCapabilities(
        retry_after_search_failure=True,
        base_anchor_requirement="upright_for_base_xy",
    )

    def set_deadline(self, deadline_monotonic_s):
        """Apply the enclosing run's absolute monotonic deadline."""
        self.deadline_monotonic_s = deadline_monotonic_s


    def __init__(self, registry, domain, *, settings, resolved_config, repository_root, output_root, trace=None):
        self.registry, self.domain, self.settings = registry, domain, settings
        self.resolved_config, self.repo = resolved_config, Path(repository_root).resolve()
        self.output, self.trace = Path(output_root).resolve(), trace

    def solve(self, world, program, initial_state, **kwargs):
        """Bound one solve; a local timeout is feedback, not global exhaustion."""
        parent = getattr(self, "deadline_monotonic_s", None)
        self.run_deadline_monotonic_s = parent
        started = time.perf_counter()
        self.deadline_monotonic_s = min(parent if parent is not None else math.inf,
                                        started + self.settings.solve_timeout_s)
        try:
            return self._solve(world, program, initial_state, **kwargs)
        except GeometryDeadlineExceeded:
            require_time_remaining(parent)
            error = GeometricUnsat(())
            error.program_feedback = ProgramFailure(
                "", ("no_candidates",), (self.domain.target_name,),
                search_budget_exhausted=True, failure_source="geometry_wall_time",
                attribution_scope="global",
                budget_scope="geometry_wall_time").as_feedback()
            error.solver_diagnostics = {"stop_reason": "solve_wall_time",
                                        "artifacts": str(self.output)}
            if self.output.exists():
                (self.output / "timeout_evidence.json").write_text(json.dumps({
                    **error.solver_diagnostics, "feedback": error.program_feedback}, indent=2) + "\n")
            raise error
        finally:
            elapsed = time.perf_counter() - started
            if self.output.exists():
                (self.output / "solve_timing.json").write_text(json.dumps({
                    "wall_time_s": elapsed, "solve_budget_s": self.settings.solve_timeout_s,
                    "search_work": getattr(self.domain, "search_work", {})}, indent=2) + "\n")
            self.deadline_monotonic_s = parent
            if hasattr(self.domain, "set_deadline"):
                self.domain.set_deadline(parent)

    def _solve(self, world, program, initial_state, *, failure_context=None, excluded_assignments=frozenset()):
        """Return the common plan schema; never repair or substitute the program."""
        require_time_remaining(getattr(self, "deadline_monotonic_s", None))
        self.failure_context = receive_failure_context(failure_context, self.trace)
        validate_skill_program(program, self.registry, frozenset(world.objects))
        domains = getattr(program, "parameter_domains", None)
        expected = {variable: self.registry.domain_samplers[role] for step in program.steps
                    for role, variable in step.continuous_variables.items()}
        if domains is not None and dict(domains) != expected:
            raise ValueError("cuTAMP cannot ignore or substitute program domains")
        if not program.steps:
            return ParameterizedSkillPlan((), {}, ())
        self.output.mkdir(parents=True, exist_ok=False)
        (self.output / "solver_input.json").write_text(json.dumps({
            "program": dataclasses.asdict(program), "observation_id": world.observation_id,
            "failure_context": dataclasses.asdict(self.failure_context) if self.failure_context else None,
            "excluded_assignments": sorted(excluded_assignments), "settings": dataclasses.asdict(self.settings),
        }, indent=2, default=str) + "\n")
        builder = ContinuousProblemBuilder(self.domain, self.resolved_config,
                                            self.settings.kinematics_template, self.settings.robot_template)
        manifest = builder.build(program, world, self.output / "problem")
        require_time_remaining(getattr(self, "deadline_monotonic_s", None))
        if manifest["optimize_base_xy"]:
            geometry = json.loads((self.output / "problem/geometry.json").read_text())
            rotation = np.asarray(geometry["observed_robot_base_world_matrix"], dtype=float)[:3, :3]
            if not np.allclose(rotation[:, 2], (0.0, 0.0, 1.0), atol=1e-6, rtol=0):
                constraint = ConstraintResult(
                    False, "reachability", "base_pose", "base_anchor_not_upright",
                    (self.domain.target_name,), {"program_step": 0},
                )
                error = GeometricUnsat((constraint,))
                error.program_feedback = ProgramFailure.from_constraints(
                    (constraint,), skill="NavigateToPick", target=self.domain.target_name,
                    failure_source="backend_precondition", attribution_scope="step",
                    program_step=0,
                ).as_feedback()
                error.solver_diagnostics = {
                    "backend_precondition": "upright_for_base_xy",
                    "observed_base_up_axis": rotation[:, 2].tolist(),
                    "observation_id": world.observation_id,
                }
                error.retryable_search = False
                raise error
        if self.trace is not None:
            self.trace({"event": "cutamp_optimization_started", "output_root": str(self.output),
                        "remaining_s": self.deadline_monotonic_s - time.perf_counter()})
        metrics = run_cutamp_worker(
            self.settings, self.repo, self.output / "problem", self.output / "optimizer",
            optimize_base_xy=manifest["optimize_base_xy"],
            deadline_monotonic_s=getattr(self, "deadline_monotonic_s", None),
        )
        require_time_remaining(getattr(self, "deadline_monotonic_s", None))
        if self.trace is not None:
            keys = ("num_particles", "num_opt_steps", "optimization_time", "initial_constraint_cost",
                    "final_constraint_cost", "num_satisfying_particles", "selected_particle", "optimizer_sha256")
            self.trace({"event": "cutamp_optimization", "output_root": str(self.output),
                        **{key: metrics[key] for key in keys}})
        candidates = json.loads((self.output / "optimizer/candidate_assignments.json").read_text())["candidates"]
        plan, checks, failures = postcheck_cutamp_candidates(
            self.registry, self.domain, world, program, candidates, initial_state, self.output,
            max_postchecks=self.settings.max_postchecks, excluded_assignments=excluded_assignments,
            trace=self.trace, deadline_monotonic_s=min(self.deadline_monotonic_s,
                time.perf_counter() + self.settings.postcheck_timeout_s),
            postcheck_order=self.settings.postcheck_order,
            reuse_navigation_pick_witness=self.settings.reuse_navigation_pick_witness,
            quality_window_after_first_pass=self.settings.postcheck_quality_window,
            adaptive_budget=self.settings.adaptive_postcheck,
            run_deadline_monotonic_s=self.run_deadline_monotonic_s)
        (self.output / "postcheck_result.json").write_text(json.dumps({
            "observation_id": world.observation_id,
            "plan": dataclasses.asdict(plan) if plan is not None else None, "checks": checks,
            "scope": "exact_checked_parameterized_plan_not_physical_execution"}, indent=2) + "\n")
        selection = json.loads((self.output / "candidate_selection.json").read_text())
        require_time_remaining(getattr(self, "deadline_monotonic_s", None))
        approximate_feedback = None
        if not any(item["optimizer_feasible"] for item in candidates):
            approximate_feedback = optimizer_failure_feedback(
                candidates, metrics, max_steps=self.settings.num_opt_steps, target=self.domain.target_name)
        diagnostics = {
            "approximate_failure": approximate_feedback,
            "optimizer_termination": {"timed_out": metrics["timed_out"],
                "steps_executed": metrics["num_opt_steps"], "step_budget": self.settings.num_opt_steps},
            "constraint_summary": metrics["constraint_summary"],
            "exact_checks": selection["candidates_completed"], "candidate_count": len(candidates),
            "postcheck_termination": selection,
            "search_work": getattr(self.domain, "search_work", {}),
            "disagreements": [{key: item[key] for key in (
                "particle_index", "optimizer_feasible", "valid", "reason")}
                for item in checks if item["approximate_exact_disagreement"]],
            "artifacts": str(self.output),
        }
        (self.output / "failure_evidence.json").write_text(json.dumps(diagnostics, indent=2) + "\n")
        if self.trace is not None:
            self.trace({"event": "cutamp_evidence", **diagnostics})
        if plan is None:
            error = GeometricUnsat(failures)
            exact_feedback = (exact_failure_feedback(
                program, failures, checks=checks, candidate_count=len(candidates),
                max_postchecks=self.settings.max_postchecks, target=self.domain.target_name,
                stop_reason=selection["stop_reason"],
                count_exhausted=selection["count_exhausted"])
                if failures else None)
            budget_feedback = postcheck_budget_feedback(selection["stop_reason"], self.domain.target_name)
            error.program_feedback = exact_feedback or budget_feedback or approximate_feedback
            error.program_feedbacks = tuple(item for item in
                                            (approximate_feedback, exact_feedback, budget_feedback) if item is not None)
            if error.program_feedback is None:
                raise RuntimeError("cuTAMP returned neither a plan nor failure evidence")
            error.solver_diagnostics = diagnostics
            raise error
        (self.output / "parameterized_plan.json").write_text(json.dumps(dataclasses.asdict(plan), indent=2) + "\n")
        return plan
