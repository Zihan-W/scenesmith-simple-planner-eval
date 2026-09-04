#!/usr/bin/env python3
"""Simulate the calibrated pick target at rest on its support surface."""

import argparse
import json
import sys

from pathlib import Path

import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    LoadModelDirectives,
    Parser,
    ProcessModelDirectives,
    RollPitchYaw,
    Simulator,
)

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_online_env import _register_package_xml, find_package_xml

DEFAULT_TARGET_MODEL = "living_room_box_0"
DEFAULT_TABLE_MODEL = "living_room_coffee_table_0"
EVAL_PACKAGE_XML = REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Run a free-body settling test for the calibrated Zerith pick "
            "target. This does not add or control the robot."
        )
    )
    parser.add_argument("scene_dmd", type=Path)
    parser.add_argument(
        "--scene-package-xml",
        type=Path,
        help="Scene package.xml; inferred from scene_dmd when omitted.",
    )
    parser.add_argument("--target-model-name", default=DEFAULT_TARGET_MODEL)
    parser.add_argument("--table-model-name", default=DEFAULT_TABLE_MODEL)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--sample-period", type=float, default=0.01)
    parser.add_argument("--physics-dt", type=float, default=0.001)
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Defaults to target_settle.json beside scene_dmd.",
    )
    return parser.parse_args()


def _minimum_body_distance(plant, plant_context, body_a, body_b) -> float:
    """Return the closest signed distance between two bodies."""
    query = plant.get_geometry_query_input_port().Eval(plant_context)
    distances = []
    for geometry_a in plant.GetCollisionGeometriesForBody(body_a):
        for geometry_b in plant.GetCollisionGeometriesForBody(body_b):
            pair = query.ComputeSignedDistancePairClosestPoints(
                geometry_a,
                geometry_b,
            )
            distances.append(float(pair.distance))
    if not distances:
        raise ValueError("Target and support must both have collision geometry")
    return min(distances)


def _build_simulation(
    scene_dmd: Path,
    scene_package_xml: Path,
    physics_dt: float,
):
    """Build a discrete Drake scene and return its simulation components."""
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=physics_dt)
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    _register_package_xml(parser, scene_package_xml)
    _register_package_xml(parser, EVAL_PACKAGE_XML)
    ProcessModelDirectives(LoadModelDirectives(str(scene_dmd)), parser)
    plant.Finalize()
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    plant_context = plant.GetMyMutableContextFromRoot(
        simulator.get_mutable_context()
    )
    return plant, simulator, plant_context


def main() -> None:
    """Run the target settling simulation and save raw stability metrics."""
    args = _parse_args()
    scene_dmd = args.scene_dmd.resolve()
    scene_package_xml = (
        args.scene_package_xml.resolve()
        if args.scene_package_xml is not None
        else find_package_xml(scene_dmd)
    )
    output_json = (
        args.output_json.resolve()
        if args.output_json is not None
        else scene_dmd.parent / "target_settle.json"
    )
    if not scene_dmd.is_file():
        raise FileNotFoundError(f"Scene DMD does not exist: {scene_dmd}")
    if not 3.0 <= args.duration <= 5.0:
        raise ValueError("duration must be between 3 and 5 seconds")
    if args.sample_period <= 0.0:
        raise ValueError("sample-period must be positive")
    if args.physics_dt <= 0.0:
        raise ValueError("physics-dt must be positive")

    plant, simulator, plant_context = _build_simulation(
        scene_dmd,
        scene_package_xml,
        args.physics_dt,
    )
    target_instance = plant.GetModelInstanceByName(args.target_model_name)
    table_instance = plant.GetModelInstanceByName(args.table_model_name)
    target_body = plant.GetBodyByName("base_link", target_instance)
    table_body = plant.GetBodyByName("base_link", table_instance)
    target_joints = [
        plant.get_joint(joint_index)
        for joint_index in plant.GetJointIndices(target_instance)
    ]
    target_is_free_body = (
        plant.num_positions(target_instance) == 7
        and plant.num_velocities(target_instance) == 6
        and len(target_joints) == 1
        and target_joints[0].type_name() == "quaternion_floating"
    )
    if not target_is_free_body:
        raise ValueError(
            f"Target body {args.target_model_name!r} is not a free body"
        )

    initial_pose = plant.EvalBodyPoseInWorld(plant_context, target_body)
    initial_xyz = initial_pose.translation().copy()
    initial_yaw = RollPitchYaw(initial_pose.rotation()).yaw_angle()
    samples = []
    sample_times = np.arange(
        0.0,
        args.duration + 0.5 * args.sample_period,
        args.sample_period,
    )
    for sample_time in sample_times[1:]:
        simulator.AdvanceTo(float(min(sample_time, args.duration)))
        pose = plant.EvalBodyPoseInWorld(plant_context, target_body)
        velocity = plant.EvalBodySpatialVelocityInWorld(
            plant_context,
            target_body,
        )
        rpy = RollPitchYaw(pose.rotation()).vector()
        samples.append(
            {
                "time_s": simulator.get_context().get_time(),
                "xyz_world_m": pose.translation().tolist(),
                "rpy_world_rad": rpy.tolist(),
                "linear_velocity_world_mps": velocity.translational().tolist(),
                "angular_velocity_world_radps": velocity.rotational().tolist(),
                "target_table_signed_distance_m": _minimum_body_distance(
                    plant,
                    plant_context,
                    target_body,
                    table_body,
                ),
            }
        )

    final = samples[-1]
    xyz_samples = np.asarray([sample["xyz_world_m"] for sample in samples])
    rpy_samples = np.asarray([sample["rpy_world_rad"] for sample in samples])
    linear_velocity_samples = np.asarray(
        [sample["linear_velocity_world_mps"] for sample in samples]
    )
    angular_velocity_samples = np.asarray(
        [sample["angular_velocity_world_radps"] for sample in samples]
    )
    yaw_delta = np.unwrap(
        np.r_[initial_yaw, rpy_samples[:, 2]]
    ) - initial_yaw
    result = {
        "scene_dmd": str(scene_dmd),
        "duration_s": args.duration,
        "physics_dt_s": args.physics_dt,
        "sample_period_s": args.sample_period,
        "target_model_name": args.target_model_name,
        "table_model_name": args.table_model_name,
        "target_is_free_body": target_is_free_body,
        "initial_xyz_world_m": initial_xyz.tolist(),
        "final_xyz_world_m": final["xyz_world_m"],
        "final_rpy_world_deg": np.rad2deg(
            np.asarray(final["rpy_world_rad"])
        ).tolist(),
        "final_linear_speed_mps": float(
            np.linalg.norm(linear_velocity_samples[-1])
        ),
        "final_angular_speed_radps": float(
            np.linalg.norm(angular_velocity_samples[-1])
        ),
        "maximum_xy_drift_m": float(
            np.max(np.linalg.norm(xyz_samples[:, :2] - initial_xyz[:2], axis=1))
        ),
        "minimum_z_displacement_m": float(
            np.min(xyz_samples[:, 2] - initial_xyz[2])
        ),
        "final_z_displacement_m": float(final["xyz_world_m"][2] - initial_xyz[2]),
        "maximum_absolute_roll_pitch_deg": float(
            np.max(np.abs(np.rad2deg(rpy_samples[:, :2])))
        ),
        "maximum_absolute_yaw_change_deg": float(
            np.max(np.abs(np.rad2deg(yaw_delta)))
        ),
        "minimum_target_table_signed_distance_m": float(
            min(sample["target_table_signed_distance_m"] for sample in samples)
        ),
        "final_target_table_signed_distance_m": final[
            "target_table_signed_distance_m"
        ],
        "samples": samples,
    }
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(
        json.dumps(result, indent=2) + "\n",
        encoding="utf-8",
    )
    summary = {
        key: value for key, value in result.items() if key != "samples"
    }
    print(json.dumps(summary, indent=2))
    print(f"Full settling trace: {output_json}")


if __name__ == "__main__":
    main()
