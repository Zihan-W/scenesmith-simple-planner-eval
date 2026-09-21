"""Recheck an unchanged cuTAMP-selected endpoint in the actual Drake model.

By default this does not solve IK again. An explicit diagnostic option can
run separate IK without replacing the inspected particle. No mode advances
physics, validates an approach/lift path, or certifies a navigation corridor.
"""

import argparse
import dataclasses
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from pydrake.math import RotationMatrix

from planner.src.tamp.scenesmith import SceneSmithPickDomain
from simulation.src.robots.adapters.description import drake_pose
from simulation.src.robots.adapters.zerith_mobile import ZerithMobileRobotAdapter
from simulation.src.io.experiment import load_experiment
from simulation.src.core.observations import Pose
from simulation.src.geometry.planning import build_planning_query


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository-root", type=Path, required=True)
    parser.add_argument("--scene-root", type=Path, required=True)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--probe-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--diagnostic-candidate", type=Path,
                        help="Inspect an explicitly exported unselected particle; does not authorize execution")
    parser.add_argument("--diagnose-ik", action="store_true",
                        help="Also run separate Drake IK to investigate reachability, never replace the particle")
    args = parser.parse_args()
    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = json.loads((args.probe_root / "result.json").read_text())
    problem = json.loads((args.probe_root / "problem.json").read_text())
    selected_index = result["selected_particle"]
    inspected_index = selected_index
    if args.diagnostic_candidate is not None:
        candidate = json.loads(args.diagnostic_candidate.read_text())
        if (candidate["scope"] != "unselected_particle_diagnostic_not_execution_authorization"
                or Path(candidate["probe_root"]).resolve() != args.probe_root.resolve()
                or hashlib.sha256((args.probe_root / "particles.pt").read_bytes()).hexdigest()
                != candidate["source_particle_file_sha256"]):
            raise ValueError("Candidate does not match this retained particle artifact")
        inspected_index = candidate["particle_index"]
        result.update(selected_arm_joint_names=candidate["arm_joint_names"],
                      selected_arm_joint_positions=candidate["arm_joint_positions"],
                      selected_base_world_pose=candidate["base_world_pose"],
                      selected_grasp_lateral_offset_m=candidate["grasp_lateral_offset_m"])
    elif selected_index is None:
        raise ValueError("No particle was selected by the official feasibility checker")
    calibration_input = problem["inputs"]["grasp_calibration"]
    calibration_path = Path(calibration_input["path"])
    if hashlib.sha256(calibration_path.read_bytes()).hexdigest() != calibration_input["sha256"]:
        raise ValueError("Grasp calibration changed after optimization")
    calibration = json.loads(calibration_path.read_text())
    experiment = load_experiment(
        args.experiment, repository_root=args.repository_root, cache_root=output / "scene_cache",
        scene_root=args.scene_root, trust_factories=True)
    config = experiment.environment_config
    base = result.get("selected_base_world_pose", result["fixed_base_world_pose"])
    adapter = config.robot_adapter
    if not isinstance(adapter, ZerithMobileRobotAdapter):
        raise TypeError("This endpoint check requires the actual mobile Zerith adapter")
    placed = ZerithMobileRobotAdapter(dataclasses.replace(
        adapter.spec, base_pose=Pose(tuple(base[:3]), tuple(base[3:]))), adapter.base_config)
    query = build_planning_query(scenario=config.scenario, robot_adapter=placed, timing=config.timing)
    robot_input = problem["inputs"]["robot_config"]
    robot_path = Path(robot_input["path"])
    if hashlib.sha256(robot_path.read_bytes()).hexdigest() != robot_input["sha256"]:
        raise ValueError("cuRobo robot configuration changed after optimization")
    robot_config = json.loads(robot_path.read_text())
    positions = query.plant.GetPositions(query.context)
    locked_checks = {}
    for name, expected in robot_config["kinematics"]["lock_joints"].items():
        joint = query.plant.GetJointByName(name, query.robot_model_instance)
        if joint.num_positions() != 1:
            raise ValueError(f"Expected one position for locked joint {name}")
        actual_position = float(positions[joint.position_start()])
        locked_checks[name] = {"drake": actual_position, "curobo": expected}
        if not math.isclose(actual_position, expected, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(f"Locked joint mismatch: {name}: Drake={actual_position}, cuRobo={expected}")
    names = placed.spec.controlled_joint_names
    arm_names = result["selected_arm_joint_names"]
    if set(arm_names) != set(placed.spec.arm_groups["left"]):
        raise ValueError("Optimized joint names differ from the runtime left-arm group")
    configuration = list(query.configuration())
    for name, value in zip(arm_names, result["selected_arm_joint_positions"], strict=True):
        configuration[names.index(name)] = value
    # Reuse the existing SceneSmith grasp policy, without generating new IK or
    # constructing a new solver. Unrelated home/locked joints remain unchanged.
    domain = object.__new__(SceneSmithPickDomain)
    task = config.task_factory()
    domain.task_config = getattr(task, "task", task).config
    check = query.check_configuration(configuration, contact_policy=domain._grasp_contact_policy())
    target_instance = query.plant.GetModelInstanceByName(calibration["target_model_name"])
    target = query.plant.GetBodyByName(calibration["target_body_name"], target_instance)
    relative = calibration["grasp_pose_in_target"]
    translation = list(relative["translation_m"])
    translation[1] += result["selected_grasp_lateral_offset_m"]
    desired = target.EvalPoseInWorld(query.context) @ drake_pose(
        Pose(tuple(translation), tuple(relative["quaternion_wxyz"])))
    actual = drake_pose(query.frame_pose(placed.spec.model_instance_name,
                                        placed.spec.end_effector_frames["left"]))
    position_error_xyz = desired.translation() - actual.translation()
    pos_error = float(np.linalg.norm(position_error_xyz))
    position_box_valid = bool(np.max(np.abs(position_error_xyz)) <= 0.001)
    rot_error = float(RotationMatrix(actual.rotation().matrix().T @ desired.rotation().matrix()).ToAngleAxis().angle())
    report = {
        "scope": ("unselected_particle_endpoint_diagnostic_not_execution_authorization"
                  if args.diagnostic_candidate else "unchanged_selected_endpoint_not_approach_lift_navigation_or_physics"),
        "probe_root": str(args.probe_root.resolve()), "selected_particle": selected_index,
        "inspected_particle": inspected_index,
        "ik_resolved": False, "physical_rollout_executed": False,
        "locked_joint_checks": locked_checks,
        "arm_joint_names": arm_names, "arm_joint_positions": result["selected_arm_joint_positions"],
        "configuration_check": dataclasses.asdict(check),
        "position_error_m": pos_error, "orientation_error_rad": rot_error,
        "position_error_xyz_m": position_error_xyz.tolist(),
        "runtime_position_box_half_width_m": 0.001,
        "runtime_position_box_valid": position_box_valid,
        "runtime_orientation_tolerance_rad": math.radians(2.0),
        "endpoint_valid": bool(check.valid and position_box_valid and rot_error <= math.radians(2.0)),
    }
    if args.diagnose_ik:
        from simulation.src.robots.adapters.description import public_pose
        alternative = query.solve_ik(
            public_pose(desired), frame_name=placed.spec.end_effector_frames["left"],
            seed=configuration, position_tolerance_m=0.001,
            orientation_tolerance_rad=math.radians(2.0), contact_policy=domain._grasp_contact_policy())
        report["separate_ik_diagnostic_not_particle_replacement"] = dataclasses.asdict(alternative)
    (output / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report))


if __name__ == "__main__":
    main()
