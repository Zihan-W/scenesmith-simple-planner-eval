#!/usr/bin/env python3
"""Validate one Zerith left-arm PREGRASP pose without planning a path."""

import argparse
import json
import sys

from pathlib import Path

import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Box,
    DiagramBuilder,
    InverseKinematics,
    LoadModelDirectives,
    Parser,
    ProcessModelDirectives,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Solve,
)

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_grasp_geometry import (
    BOX_SIZE_METERS,
    PREGRASP_DISTANCE_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
    add_left_grasp_frame,
)
from src.zerith_online_env import (
    LEFT_ARM_SERVO_CONFIGS,
    ZERITH_PACKAGE_NAME,
    ZERITH_URDF_RELATIVE_PATH,
    _register_package_xml,
)

TARGET_MODEL_NAME = "living_room_box_0"
TARGET_BODY_NAME = "base_link"
EVAL_PACKAGE_XML = (
    REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"
)
ZERITH_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Solve exactly one PREGRASP IK configuration. This performs no "
            "RRT, trajectory generation, or simulation-time teleportation."
        )
    )
    parser.add_argument("scene_dmd", type=Path)
    parser.add_argument(
        "--scene-package-xml",
        type=Path,
        required=True,
        help="package.xml belonging to the original SceneSmith scene.",
    )
    parser.add_argument(
        "--eval-package-xml",
        type=Path,
        default=EVAL_PACKAGE_XML,
    )
    parser.add_argument(
        "--robot-model-dir",
        type=Path,
        default=ZERITH_MODEL_DIR,
    )
    parser.add_argument("--target-model-name", default=TARGET_MODEL_NAME)
    parser.add_argument(
        "--robot-xyz",
        type=float,
        nargs=3,
        default=ROBOT_BASE_XYZ_METERS,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--robot-yaw-deg",
        type=float,
        default=ROBOT_BASE_YAW_DEG,
    )
    parser.add_argument(
        "--pregrasp-distance",
        type=float,
        default=PREGRASP_DISTANCE_METERS,
    )
    parser.add_argument("--position-tolerance", type=float, default=0.002)
    parser.add_argument(
        "--orientation-tolerance-deg",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Defaults to pregrasp_ik.json beside scene_dmd.",
    )
    return parser.parse_args()


def _robot_geometry_ids(plant, zerith_model_instance) -> set:
    """Return all Zerith proximity geometry identifiers."""
    geometry_ids = set()
    for body_index in plant.GetBodyIndices(zerith_model_instance):
        geometry_ids.update(
            plant.GetCollisionGeometriesForBody(plant.get_body(body_index))
        )
    return geometry_ids


def _robot_penetrations(plant, context, robot_geometry_ids: set) -> list:
    """Return active penetration pairs involving the robot."""
    query = plant.get_geometry_query_input_port().Eval(context)
    return [
        pair
        for pair in query.ComputePointPairPenetration()
        if pair.id_A in robot_geometry_ids or pair.id_B in robot_geometry_ids
    ]


def _geometry_pair_details(plant, context, pairs: list) -> list[dict]:
    """Serialize penetration pairs with frame names."""
    inspector = plant.get_geometry_query_input_port().Eval(context).inspector()
    return [
        {
            "depth_m": pair.depth,
            "frame_a": inspector.GetName(inspector.GetFrameId(pair.id_A)),
            "frame_b": inspector.GetName(inspector.GetFrameId(pair.id_B)),
        }
        for pair in sorted(pairs, key=lambda item: item.depth, reverse=True)
    ]


def _target_collision_pose(plant, context, target_body):
    """Return target Box dimensions and collision-frame world pose."""
    geometry_ids = plant.GetCollisionGeometriesForBody(target_body)
    if len(geometry_ids) != 1:
        raise ValueError(
            "Expected the pick target to have exactly one collision geometry, "
            f"found {len(geometry_ids)}"
        )
    query = plant.get_geometry_query_input_port().Eval(context)
    inspector = query.inspector()
    geometry_id = geometry_ids[0]
    shape = inspector.GetShape(geometry_id)
    if not isinstance(shape, Box):
        raise TypeError(f"Expected target collision Box, got {type(shape)}")
    dimensions = np.array([shape.width(), shape.depth(), shape.height()])
    if not np.allclose(dimensions, BOX_SIZE_METERS, atol=1e-12):
        raise ValueError(
            f"Expected target dimensions {BOX_SIZE_METERS}, got {dimensions}"
        )
    X_WB = plant.EvalBodyPoseInWorld(context, target_body)
    return dimensions, X_WB, X_WB @ inspector.GetPoseInFrame(geometry_id)


def _desired_pregrasp_pose(
    X_WB: RigidTransform,
    target_center: np.ndarray,
    robot_xyz: np.ndarray,
    pregrasp_distance: float,
) -> RigidTransform:
    """Construct a horizontal side grasp aligned with the box's 4 cm axis."""
    world_up = np.array([0.0, 0.0, 1.0])
    closing_axis = X_WB.rotation().matrix()[:, 1].copy()
    closing_axis[2] = 0.0
    norm = np.linalg.norm(closing_axis)
    if norm < 1e-6:
        raise ValueError("Target box Y axis is vertical; side grasp is undefined")
    closing_axis /= norm
    approach_axis = np.cross(closing_axis, world_up)
    approach_axis /= np.linalg.norm(approach_axis)

    toward_target = target_center - robot_xyz
    toward_target[2] = 0.0
    if approach_axis @ toward_target < 0.0:
        approach_axis = -approach_axis
        closing_axis = -closing_axis

    R_WG = RotationMatrix(
        np.column_stack((approach_axis, closing_axis, world_up))
    )
    pregrasp_position = target_center - approach_axis * pregrasp_distance
    return RigidTransform(R_WG, pregrasp_position)


def main() -> None:
    """Build the task plant, solve PREGRASP IK, and write diagnostics."""
    args = _parse_args()
    scene_dmd = args.scene_dmd.resolve()
    scene_package_xml = args.scene_package_xml.resolve()
    eval_package_xml = args.eval_package_xml.resolve()
    robot_model_dir = args.robot_model_dir.resolve()
    robot_xyz = np.asarray(args.robot_xyz, dtype=float)
    if args.pregrasp_distance <= 0.0:
        raise ValueError("pregrasp-distance must be positive")
    if args.position_tolerance <= 0.0:
        raise ValueError("position-tolerance must be positive")
    if args.orientation_tolerance_deg <= 0.0:
        raise ValueError("orientation-tolerance-deg must be positive")

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    _register_package_xml(parser, scene_package_xml)
    _register_package_xml(parser, eval_package_xml)
    parser.package_map().Add(ZERITH_PACKAGE_NAME, str(robot_model_dir))
    ProcessModelDirectives(LoadModelDirectives(str(scene_dmd)), parser)
    robot_urdf = robot_model_dir / ZERITH_URDF_RELATIVE_PATH
    zerith = parser.AddModels(str(robot_urdf))[0]
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("dipan_link", zerith),
        RigidTransform(
            RollPitchYaw(0.0, 0.0, np.deg2rad(args.robot_yaw_deg)),
            robot_xyz,
        ),
    )
    grasp_frame = add_left_grasp_frame(plant, zerith)
    plant.Finalize()

    diagram = builder.Build()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    target_instance = plant.GetModelInstanceByName(args.target_model_name)
    target_body = plant.GetBodyByName(TARGET_BODY_NAME, target_instance)
    dimensions, X_WB, X_WC = _target_collision_pose(
        plant,
        plant_context,
        target_body,
    )
    X_WG_desired = _desired_pregrasp_pose(
        X_WB,
        X_WC.translation(),
        robot_xyz,
        args.pregrasp_distance,
    )

    robot_geometry_ids = _robot_geometry_ids(plant, zerith)
    initial_pairs = _robot_penetrations(
        plant,
        plant_context,
        robot_geometry_ids,
    )
    if initial_pairs:
        details = _geometry_pair_details(plant, plant_context, initial_pairs)
        raise RuntimeError(
            "Robot base pose has initial penetration; deepest pair: "
            f"{details[0]}"
        )

    ik = InverseKinematics(plant, plant_context, with_joint_limits=True)
    desired_position = X_WG_desired.translation()
    position_tolerance = args.position_tolerance
    ik.AddPositionConstraint(
        frameB=grasp_frame,
        p_BQ=np.zeros((3, 1)),
        frameA=plant.world_frame(),
        p_AQ_lower=(desired_position - position_tolerance).reshape(3, 1),
        p_AQ_upper=(desired_position + position_tolerance).reshape(3, 1),
    )
    ik.AddOrientationConstraint(
        frameAbar=plant.world_frame(),
        R_AbarA=X_WG_desired.rotation(),
        frameBbar=grasp_frame,
        R_BbarB=RotationMatrix(),
        theta_bound=np.deg2rad(args.orientation_tolerance_deg),
    )

    program = ik.prog()
    q = ik.q()
    q_initial = plant.GetPositions(plant_context).copy()
    arm_indices = np.array(
        [
            plant.GetJointByName(config.name, zerith).position_start()
            for config in LEFT_ARM_SERVO_CONFIGS
        ]
    )
    fixed_indices = np.setdiff1d(
        np.arange(plant.num_positions()),
        arm_indices,
    )
    program.AddBoundingBoxConstraint(
        q_initial[fixed_indices],
        q_initial[fixed_indices],
        q[fixed_indices],
    )
    program.AddQuadraticErrorCost(
        np.eye(len(arm_indices)),
        q_initial[arm_indices],
        q[arm_indices],
    )
    program.SetInitialGuess(q, q_initial)
    result = Solve(program)
    if not result.is_success():
        raise RuntimeError(
            f"PREGRASP IK failed: {result.get_solution_result()}"
        )

    q_solution = result.GetSolution(q)
    plant.SetPositions(plant_context, q_solution)
    X_WG_actual = plant.CalcRelativeTransform(
        plant_context,
        plant.world_frame(),
        grasp_frame,
    )
    position_error = X_WG_actual.translation() - desired_position
    orientation_error = (
        X_WG_desired.rotation().inverse() @ X_WG_actual.rotation()
    ).ToAngleAxis().angle()
    solved_pairs = _robot_penetrations(
        plant,
        plant_context,
        robot_geometry_ids,
    )

    output_path = (
        args.output_json.resolve()
        if args.output_json is not None
        else scene_dmd.parent / "pregrasp_ik.json"
    )
    output = {
        "ik_reachable": True,
        "collision_free": not solved_pairs,
        "robot_base_xyz_m": robot_xyz.tolist(),
        "robot_base_yaw_deg": args.robot_yaw_deg,
        "target_model_name": args.target_model_name,
        "target_dimensions_m": dimensions.tolist(),
        "target_collision_center_xyz_m": X_WC.translation().tolist(),
        "desired_pregrasp_pose": {
            "translation_xyz_m": desired_position.tolist(),
            "rotation_matrix": X_WG_desired.rotation().matrix().tolist(),
        },
        "q_left": q_solution[arm_indices].tolist(),
        "max_absolute_position_error_m": float(
            np.max(np.abs(position_error))
        ),
        "orientation_error_deg": float(np.rad2deg(orientation_error)),
        "penetrations": _geometry_pair_details(
            plant,
            plant_context,
            solved_pairs,
        ),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")

    print("PREGRASP IK reachable: yes")
    print(f"Collision-free IK solution: {'yes' if not solved_pairs else 'no'}")
    print(f"q_left: {q_solution[arm_indices].tolist()}")
    print(
        "Max absolute position error: "
        f"{output['max_absolute_position_error_m']:.6f} m"
    )
    print(f"Orientation error: {output['orientation_error_deg']:.3f} deg")
    if solved_pairs:
        print(
            "Deepest solved-pose penetration: "
            f"{output['penetrations'][0]}"
        )
    print(f"Diagnostics: {output_path}")


if __name__ == "__main__":
    main()
