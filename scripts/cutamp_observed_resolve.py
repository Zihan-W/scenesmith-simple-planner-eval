"""Observed-state probes reuse the real GeometrySolver and its exact postchecks."""

import json
from pathlib import Path

from examples.online_manipulation.tamp_cutamp import (
    CuTAMPSolver, CuTAMPSettings, postcheck_cutamp_candidates,
)
from examples.online_manipulation.tamp_geometry import GeometricUnsat
from examples.online_manipulation.tamp_hierarchy import SkillProgram, SkillStep, picklift_registry


def _pick_program(domain):
    return SkillProgram((SkillStep("PickLift", {"object": domain.target_name},
                                   {"grasp_pose": "g0", "approach_pose": "a0"}),))


def _controls(action):
    return {name: action.geometric_parameters[name] for name in (
        "target", "arm", "grasp_arm_joint_positions", "grasp_lateral_offset_m",
        "approach_segments", "approach_height_offset_m")}


def _trace(event):
    if event["event"] == "cutamp_exact_postcheck":
        print(json.dumps({key: event[key] for key in ("particle_index", "valid", "reason")}), flush=True)


def postcheck_candidates(domain, world, candidates, output, *, max_postchecks=8):
    """Keep the offline diagnostic interface on the shared backend postchecker."""
    plan, checks, _ = postcheck_cutamp_candidates(
        picklift_registry(), domain, world, _pick_program(domain), candidates,
        {"base_height_m": domain.config.robot_adapter.base_config.base_height_m}, output,
        max_postchecks=max_postchecks, trace=_trace)
    return (_controls(plan.actions[0]) if plan is not None else None), checks


def resolve_observed_pick(domain, world, experiment, repo, output, source_probe,
                          gpu_python, cutamp_root):
    """Use CuTAMPSolver for a fresh fixed PickLift program in the paused simulator."""
    inputs = json.loads((Path(source_probe) / "problem.json").read_text())["inputs"]
    settings = CuTAMPSettings(
        gpu_python=str(gpu_python), cutamp_root=str(cutamp_root),
        kinematics_template=inputs["kinematics"]["path"], robot_template=inputs["robot_config"]["path"],
        multipliers=inputs["multipliers"]["path"], tolerances=inputs["tolerances"]["path"],
        grasp_calibration=inputs["grasp_calibration"]["path"])
    solver = CuTAMPSolver(picklift_registry(), domain, settings=settings,
                          resolved_config=experiment.resolved_config, repository_root=repo,
                          output_root=output, trace=_trace)
    try:
        plan = solver.solve(world, _pick_program(domain),
                            {"base_height_m": domain.config.robot_adapter.base_config.base_height_m})
    except GeometricUnsat:
        return None
    return _controls(plan.actions[0])
