"""Compare actual Zerith Drake and cuRobo forward kinematics across joint space.

Two explicit modes allow using the separate Drake and CUDA environments. This
probe does not claim collision validation or implement a geometry solver.
"""

import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def export_drake(args):
    import numpy as np
    from src.online_manipulation.experiment import load_experiment
    from src.online_manipulation.planning import build_planning_query

    output = args.output_root.resolve()
    output.mkdir(parents=True, exist_ok=False)
    experiment = load_experiment(args.experiment, repository_root=args.repository_root,
                                 cache_root=output / "scene_cache", scene_root=args.scene_root,
                                 trust_factories=True)
    config = experiment.environment_config
    spec = config.robot_adapter.spec
    query = build_planning_query(scenario=config.scenario, robot_adapter=config.robot_adapter,
                                 timing=config.timing)
    plant, instance = query.plant, query.robot_model_instance
    active = tuple(spec.arm_groups["left"])
    root = ET.parse(spec.model_path).getroot()
    locked = {}
    for item in root.findall("joint"):
        name = item.attrib["name"]
        if item.attrib["type"] == "fixed" or name in active:
            continue
        joint = plant.GetJointByName(name, instance)
        if joint.num_positions() != 1:
            raise ValueError(f"Cannot export non-scalar locked joint: {name}")
        locked[name] = float(plant.GetPositions(query.context)[joint.position_start()])
    parent = plant.GetFrameByName("left_wrist_pitch_link", instance)
    tcp = plant.GetFrameByName(spec.end_effector_frames["left"], instance)
    base = plant.GetFrameByName(spec.base_link_name, instance)
    x_parent_tcp = plant.CalcRelativeTransform(query.context, parent, tcp)
    limits = [[float(plant.GetJointByName(name, instance).position_lower_limits()[0]),
               float(plant.GetJointByName(name, instance).position_upper_limits()[0])]
              for name in active]
    home = [float(plant.GetPositions(query.context)[
        plant.GetJointByName(name, instance).position_start()]) for name in active]
    config_dict = {"kinematics": {
        "urdf_path": str(spec.model_path), "base_link": spec.base_link_name,
        "ee_link": tcp.name(), "link_names": [link.attrib["name"] for link in root.findall("link")],
        "lock_joints": locked, "collision_link_names": [],
        "extra_links": {tcp.name(): {
            "parent_link_name": parent.name(), "link_name": tcp.name(),
            "fixed_transform": [*x_parent_tcp.translation().tolist(),
                                *x_parent_tcp.rotation().ToQuaternion().wxyz().tolist()],
            "joint_type": "FIXED", "joint_name": "exported_left_tcp_fixed",
        }},
    }}
    rng = np.random.default_rng(500)
    configurations = [home] + [[float(rng.uniform(lo, hi)) for lo, hi in limits]
                                for _ in range(16)]
    references = []
    for values in configurations:
        for name, value in zip(active, values, strict=True):
            plant.GetJointByName(name, instance).set_angle(query.context, value)
        references.append(plant.CalcRelativeTransform(query.context, base, tcp).GetAsMatrix4().tolist())
    data = {"scope": "kinematics_only_no_collision_guarantee", "joint_names": active,
            "joint_limits": limits, "configurations": configurations,
            "base_from_tcp": references, "robot_config": config_dict,
            "urdf_sha256": hashlib.sha256(spec.model_path.read_bytes()).hexdigest()}
    (output / "drake_reference.json").write_text(json.dumps(data, indent=2) + "\n")
    print(json.dumps({"reference": str(output / "drake_reference.json"), "configurations": len(references)}))


def compare_curobo(args):
    import numpy as np
    import torch
    from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
    from curobo.types.robot import RobotConfig

    source = args.reference.resolve()
    data = json.loads(source.read_text())
    urdf = Path(data["robot_config"]["kinematics"]["urdf_path"])
    if hashlib.sha256(urdf.read_bytes()).hexdigest() != data["urdf_sha256"]:
        raise ValueError("Robot URDF changed after Drake reference export")
    robot = CudaRobotModel(RobotConfig.from_dict(data["robot_config"]).kinematics)
    if set(robot.joint_names) != set(data["joint_names"]):
        raise ValueError(f"Unexpected active joints: {robot.joint_names}")
    permutation = [data["joint_names"].index(name) for name in robot.joint_names]
    q = torch.tensor(data["configurations"], dtype=torch.float32, device="cuda")[:, permutation]
    matrices = robot.get_state(q).ee_pose.get_matrix().detach().cpu().numpy()
    reference = np.array(data["base_from_tcp"])
    position_error = np.linalg.norm(matrices[:, :3, 3] - reference[:, :3, 3], axis=1)
    rotation_error = np.linalg.norm(matrices[:, :3, :3] - reference[:, :3, :3], axis=(1, 2))
    expected_limits = np.array(data["joint_limits"])[permutation].T
    limits = robot.kinematics_config.joint_limits.position.cpu().numpy()
    limit_error = float(np.abs(expected_limits - limits).max())
    result = {"scope": data["scope"], "gpu": torch.cuda.get_device_name(),
              "configuration_count": len(q), "active_joints": robot.joint_names,
              "max_position_error_m": float(position_error.max()),
              "max_rotation_matrix_error": float(rotation_error.max()),
              "max_joint_limit_error": limit_error,
              "passed": bool(position_error.max() < 1e-5 and rotation_error.max() < 1e-5
                             and limit_error < 1e-5)}
    destination = source.parent / "curobo_comparison.json"
    if destination.exists():
        raise FileExistsError(destination)
    destination.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    if not result["passed"]:
        raise RuntimeError("cuRobo/Drake kinematics mismatch; do not use for planning")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("export-drake", "compare-curobo"))
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--experiment", type=Path)
    parser.add_argument("--repository-root", type=Path)
    parser.add_argument("--scene-root", type=Path)
    parser.add_argument("--reference", type=Path)
    args = parser.parse_args()
    if args.mode == "export-drake":
        if not all((args.output_root, args.experiment, args.repository_root, args.scene_root)):
            parser.error("export-drake requires output/experiment/repository/scene paths")
        export_drake(args)
    else:
        if args.reference is None:
            parser.error("compare-curobo requires --reference")
        compare_curobo(args)


if __name__ == "__main__":
    main()
