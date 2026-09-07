#!/usr/bin/env python3
"""Validate Zerith wrist and gripper collision proxies over joint ranges."""

import argparse
import itertools

from pathlib import Path

import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Parser,
    RigidTransform,
)

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_ROBOT_MODEL_DIR = REPOSITORY_ROOT / "models/zerith_drake"
EXPECTED_COLLISION_GEOMETRY_COUNT = 53
WRIST_JOINT_SUFFIXES = (
    "wrist_roll_joint",
    "wrist_yaw_joint",
    "wrist_pitch_joint",
)
GRIPPER_CONFIGURATIONS = (
    ("open", 0.0),
    ("half", 0.5),
    ("closed", 1.0),
)


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Sweep both Zerith wrists and grippers without custom collision "
            "filters, failing on any active self penetration."
        )
    )
    parser.add_argument(
        "--robot-model-dir",
        type=Path,
        default=DEFAULT_ROBOT_MODEL_DIR,
        help="Generated Drake package containing Zerith's urdf/ and meshes/.",
    )
    parser.add_argument(
        "--limit-fraction",
        type=float,
        default=0.95,
        help="Fraction of each wrist joint limit used for near-limit tests.",
    )
    return parser.parse_args()


def _near_limit_values(joint, limit_fraction: float) -> tuple[float, ...]:
    """Return lower-near-limit, midpoint, and upper-near-limit positions."""
    lower = float(joint.position_lower_limits()[0])
    upper = float(joint.position_upper_limits()[0])
    midpoint = 0.5 * (lower + upper)
    return (
        midpoint + limit_fraction * (lower - midpoint),
        midpoint,
        midpoint + limit_fraction * (upper - midpoint),
    )


def _penetration_descriptions(query_object) -> list[str]:
    """Return sorted descriptions of every active penetration pair."""
    inspector = query_object.inspector()
    descriptions = []
    pairs = sorted(
        query_object.ComputePointPairPenetration(),
        key=lambda pair: pair.depth,
        reverse=True,
    )
    for pair in pairs:
        frame_a = inspector.GetName(inspector.GetFrameId(pair.id_A))
        frame_b = inspector.GetName(inspector.GetFrameId(pair.id_B))
        descriptions.append(
            f"{pair.depth:.8f} m: {frame_a} <-> {frame_b}"
        )
    return descriptions


def main() -> None:
    """Sweep wrist and gripper configurations and reject self penetration."""
    args = _parse_args()
    if not 0.0 < args.limit_fraction <= 1.0:
        raise ValueError("--limit-fraction must be in (0, 1]")

    robot_model_dir = args.robot_model_dir.resolve()
    robot_urdf = robot_model_dir / "urdf/zerith_drake.urdf"
    if not robot_urdf.is_file():
        raise FileNotFoundError(f"Zerith URDF does not exist: {robot_urdf}")

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder,
        time_step=0.001,
    )
    parser = Parser(plant)
    parser.package_map().Add("zerith_drake", str(robot_model_dir))
    model_instances = parser.AddModels(str(robot_urdf))
    if len(model_instances) != 1:
        raise RuntimeError(
            f"Expected one Zerith model, got {len(model_instances)}"
        )
    zerith = model_instances[0]
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("dipan_link", zerith),
        RigidTransform(),
    )
    plant.Finalize()
    if plant.num_collision_geometries() != EXPECTED_COLLISION_GEOMETRY_COUNT:
        raise ValueError(
            f"Expected {EXPECTED_COLLISION_GEOMETRY_COUNT} collision "
            f"geometries, got {plant.num_collision_geometries()}"
        )

    diagram = builder.Build()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    scene_graph_context = scene_graph.GetMyContextFromRoot(root_context)
    query_port = scene_graph.get_query_output_port()
    test_count = 0

    for side in ("left", "right"):
        wrist_joints = tuple(
            plant.GetJointByName(f"{side}_{suffix}", zerith)
            for suffix in WRIST_JOINT_SUFFIXES
        )
        wrist_values = tuple(
            _near_limit_values(joint, args.limit_fraction)
            for joint in wrist_joints
        )
        left_finger = plant.GetJointByName(
            f"{side}_jaw_left_finger_joint",
            zerith,
        )
        right_finger = plant.GetJointByName(
            f"{side}_jaw_right_finger_joint",
            zerith,
        )

        for roll, yaw, pitch, gripper in itertools.product(
            *wrist_values,
            GRIPPER_CONFIGURATIONS,
        ):
            positions = np.zeros(plant.num_positions())
            for joint, value in zip(
                wrist_joints,
                (roll, yaw, pitch),
                strict=True,
            ):
                positions[joint.position_start()] = value
            gripper_name, fraction = gripper
            left_position = fraction * left_finger.position_lower_limits()[0]
            right_position = fraction * right_finger.position_upper_limits()[0]
            positions[left_finger.position_start()] = left_position
            positions[right_finger.position_start()] = right_position
            plant.SetPositions(plant_context, positions)

            query_object = query_port.Eval(scene_graph_context)
            penetrations = _penetration_descriptions(query_object)
            test_count += 1
            if penetrations:
                configuration = (
                    f"side={side}, roll={roll:.6f}, yaw={yaw:.6f}, "
                    f"pitch={pitch:.6f}, gripper={gripper_name}"
                )
                details = "\n".join(
                    f"  {description}" for description in penetrations
                )
                raise RuntimeError(
                    f"Collision proxy validation failed at {configuration}:\n"
                    f"{details}"
                )

    print(f"Validation passed: {test_count} configurations")
    print(f"Collision geometries: {plant.num_collision_geometries()}")
    print("Custom collision filters: none")


if __name__ == "__main__":
    main()
