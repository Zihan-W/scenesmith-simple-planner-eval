"""Real fixed-skeleton cuTAMP geometry backend with exact SceneSmith postchecks.

The isolated CUDA worker calls NVlabs' initializer, costs and ParticleOptimizer.
This module never generates/revises a skill skeleton or invokes a CCSP solver.
Current conversion supports PickLift and NavigateToPick -> PickLift; other
skeletons fail explicitly. Particle feasibility is not a collision certificate.
"""

import copy
import dataclasses
import json
import math
import os
from pathlib import Path
import subprocess

import numpy as np

from src.online_manipulation import Pose
from src.online_manipulation.adapters.description import drake_pose

from .tamp_cutamp_problem import ContinuousProblemBuilder
from .tamp_geometry import (
    GeometricUnsat, _bind_effect, _constraint_objects, _parameter_value,
    assignment_identity, receive_failure_context,
)
from .tamp_hierarchy import (
    ConstraintResult, ParameterizedSkillAction, ParameterizedSkillPlan,
    validate_skill_program,
)
from .tamp_planner import Subgoal
from .tamp_proc3s import DOMAIN_SAMPLERS


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

    def __post_init__(self):
        for name in ("num_particles", "num_opt_steps", "max_postchecks"):
            if type(getattr(self, name)) is not int or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be a positive integer")
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
            value = Path(data[name])
            if not value.is_absolute():
                value = path.parent / value
            # A venv Python symlink must retain its entry-point path.
            data[name] = str(value.absolute() if name == "gpu_python" else value.resolve())
            if not value.exists():
                raise FileNotFoundError(value)
        return cls(**data)


def run_cutamp_worker(settings, repo, problem, output, *, optimize_base_xy):
    """Run and retain the actual CUDA optimizer without shell or fallback."""
    repo, problem, output = Path(repo).resolve(), Path(problem).resolve(), Path(output).resolve()
    command = [str(Path(settings.gpu_python).absolute()),
               str(repo / "scripts/check_cutamp_zerith_optimizer.py")]
    for flag, value in (("geometry", problem / "geometry.json"),
                        ("kinematics", problem / "kinematics.json"),
                        ("robot-config", problem / "robot_config.json"),
                        ("multipliers", settings.multipliers), ("tolerances", settings.tolerances),
                        ("grasp-calibration", settings.grasp_calibration), ("output-root", output)):
        command.extend(("--" + flag, str(Path(value).resolve())))
    command.extend(("--exact-stationary-target-contact", "--num-particles", str(settings.num_particles),
                    "--num-opt-steps", str(settings.num_opt_steps), "--conf-lr", str(settings.conf_lr),
                    "--seed", str(settings.seed)))
    command.append("--optimize-base-xy" if optimize_base_xy else "--exact-stationary-robot-support")
    environment = dict(os.environ, PYTHONPATH=os.pathsep.join((str(Path(settings.cutamp_root).resolve()), str(repo))))
    (output.parent / "worker_command.json").write_text(json.dumps(command, indent=2) + "\n")
    with (output.parent / "optimizer.log").open("w") as stream:
        subprocess.run(command, cwd=repo, env=environment, stdout=stream, stderr=subprocess.STDOUT, check=True)
    return json.loads((output / "result.json").read_text())


def postcheck_cutamp_candidates(registry, domain, world, program, candidates, initial_state,
                                output, *, max_postchecks=8, excluded_assignments=frozenset(), trace=None):
    """Instantiate and exact-check GPU candidates; never replace their grasp q."""
    if max_postchecks <= 0:
        raise ValueError("Postcheck budget must be positive")
    validate_skill_program(program, registry, frozenset(world.objects))
    if tuple(step.skill for step in program.steps) not in (("PickLift",), ("NavigateToPick", "PickLift")):
        raise ValueError("Unsupported fixed cuTAMP postcheck skeleton")
    pose = world.robot["base_link_pose"]
    observed_base = Pose(tuple(pose["translation_m"]), tuple(pose["quaternion_wxyz"]))
    state0 = copy.deepcopy(initial_state)
    if "base_pose" in state0 and state0["base_pose"] != observed_base:
        raise ValueError("Initial geometry state does not match the observed robot base")
    state0["base_pose"] = observed_base
    ordered = sorted(candidates, key=lambda item: (
        abs(item["grasp_lateral_offset_m"]), sum(value * value for value in item["arm_joint_positions"])))
    checks, failures, feasible_plans = [], [], []
    for candidate in ordered:
        if not candidate["optimizer_feasible"]:
            continue
        base = candidate["base_world_pose"]
        base_pose = Pose(tuple(base[:3]), tuple(base[3:]))
        if len(program.steps) == 1 and not np.allclose(
                drake_pose(base_pose).GetAsMatrix4(), drake_pose(observed_base).GetAsMatrix4(), atol=1e-5, rtol=0):
            raise ValueError("Fixed-base GPU candidate does not match the observed base frame")
        arm = domain.config.robot_adapter.spec.arm_groups["left"]
        if set(arm) != set(candidate["arm_joint_names"]):
            raise ValueError("GPU and runtime arm joint names differ")
        q = [candidate["arm_joint_positions"][candidate["arm_joint_names"].index(name)] for name in arm]
        state, assignments, actions, step_checks = copy.deepcopy(state0), {}, [], []
        rank = ()
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
            valid, reason, details = domain.check(skill, parameters, state)
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
        if len(checks) >= max_postchecks:
            break
    selected = min(feasible_plans, key=lambda item: item[0]) if feasible_plans else None
    (Path(output) / "candidate_selection.json").write_text(json.dumps({
        "max_postchecks": max_postchecks,
        "candidate_order": "absolute_calibrated_lateral_offset_then_arm_joint_norm",
        "selection_score": "existing_domain_gap_imbalance_then_lift_joint_margin_then_offset",
        "selected_particle": selected[3] if selected is not None else None,
        "selected_score": selected[0] if selected is not None else None,
        "configuration_replaced_by_ik": False}, indent=2) + "\n")
    plan = ParameterizedSkillPlan(selected[1], selected[2], tuple(failures)) if selected else None
    return plan, checks, tuple(failures)


class CuTAMPSolver:
    """Solve a supported fixed skill skeleton using the actual GPU optimizer."""

    def __init__(self, registry, domain, *, settings, resolved_config, repository_root, output_root, trace=None):
        self.registry, self.domain, self.settings = registry, domain, settings
        self.resolved_config, self.repo = resolved_config, Path(repository_root).resolve()
        self.output, self.trace = Path(output_root).resolve(), trace

    def solve(self, world, program, initial_state, *, failure_context=None, excluded_assignments=frozenset()):
        """Return the common plan schema; never repair or substitute the program."""
        self.failure_context = receive_failure_context(failure_context, self.trace)
        validate_skill_program(program, self.registry, frozenset(world.objects))
        domains = getattr(program, "parameter_domains", None)
        expected = {variable: DOMAIN_SAMPLERS[role] for step in program.steps
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
        metrics = run_cutamp_worker(self.settings, self.repo, self.output / "problem", self.output / "optimizer",
                                    optimize_base_xy=manifest["optimize_base_xy"])
        if self.trace is not None:
            keys = ("num_particles", "num_opt_steps", "optimization_time", "initial_constraint_cost",
                    "final_constraint_cost", "num_satisfying_particles", "selected_particle", "optimizer_sha256")
            self.trace({"event": "cutamp_optimization", "output_root": str(self.output),
                        **{key: metrics[key] for key in keys}})
        candidates = json.loads((self.output / "optimizer/candidate_assignments.json").read_text())["candidates"]
        plan, checks, failures = postcheck_cutamp_candidates(
            self.registry, self.domain, world, program, candidates, initial_state, self.output,
            max_postchecks=self.settings.max_postchecks, excluded_assignments=excluded_assignments, trace=self.trace)
        (self.output / "postcheck_result.json").write_text(json.dumps({
            "observation_id": world.observation_id,
            "plan": dataclasses.asdict(plan) if plan is not None else None, "checks": checks,
            "scope": "exact_checked_parameterized_plan_not_physical_execution"}, indent=2) + "\n")
        if plan is None:
            if not failures:
                failures = (ConstraintResult(False, "no_candidates", "", "No GPU-feasible particle",
                                             (self.domain.target_name,), {"optimizer_metrics": metrics}),)
            raise GeometricUnsat(failures)
        (self.output / "parameterized_plan.json").write_text(json.dumps(dataclasses.asdict(plan), indent=2) + "\n")
        return plan
