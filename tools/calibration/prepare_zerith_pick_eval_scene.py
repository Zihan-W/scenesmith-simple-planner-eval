#!/usr/bin/env python3
"""Derive and geometrically calibrate a Zerith small-box evaluation scene."""

import argparse
import json
import sys

from pathlib import Path

import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Box,
    DiagramBuilder,
    LoadModelDirectives,
    Parser,
    ProcessModelDirectives,
    RigidTransform,
    RollPitchYaw,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.geometry_bounds import _CalcAabb, _OabbToAabb
from src.zerith_grasp_geometry import (
    APPROACH_AXIS_GRASP,
    BOX_SIZE_METERS,
    CLOSING_AXIS_GRASP,
    GRASP_WIDTH_METERS,
    LEFT_GRASP_FRAME_NAME,
    LEFT_GRASP_PARENT_FRAME_NAME,
    LIFT_DISTANCE_METERS,
    PICK_RAIL_POSITION_METERS,
    PREGRASP_DISTANCE_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
    UP_AXIS_GRASP,
    X_PARENT_GRASP_TRANSLATION_METERS,
    add_left_grasp_frame,
)
from src.zerith_online_env import (
    LEFT_ARM_SERVO_CONFIGS,
    ZERITH_PACKAGE_NAME,
    ZERITH_URDF_RELATIVE_PATH,
    _register_package_xml,
    find_package_xml,
)

DEFAULT_TARGET_MODEL = "living_room_box_0"
DEFAULT_TABLE_MODEL = "living_room_coffee_table_0"
DEFAULT_OPPOSITE_SIDE_MODEL = "living_room_vase_0"
DEFAULT_FINGERTIP_CLEARANCE_METERS = 0.01
SMALL_BOX_URI = "package://zerith_pick_eval/small_red_box.sdf"
EVAL_PACKAGE_XML = REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"
ZERITH_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Copy a SceneSmith DMD and replace exactly one target model with "
            "the tracked small red box asset. The source DMD is never edited."
        )
    )
    parser.add_argument("source_dmd", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPOSITORY_ROOT / "output" / "zerith_pick_eval",
    )
    parser.add_argument(
        "--target-model-name",
        default=DEFAULT_TARGET_MODEL,
    )
    parser.add_argument(
        "--table-model-name",
        default=DEFAULT_TABLE_MODEL,
    )
    parser.add_argument(
        "--scene-package-xml",
        type=Path,
        help="Scene package.xml; inferred from source_dmd when omitted.",
    )
    parser.add_argument(
        "--box-yaw-deg",
        type=float,
        help=(
            "Calibrated world yaw; defaults to the source target yaw."
        ),
    )
    parser.add_argument(
        "--fingertip-clearance",
        type=float,
        default=DEFAULT_FINGERTIP_CLEARANCE_METERS,
        help=(
            "Top-view gap between the box face and the open left fingertips "
            "in meters."
        ),
    )
    parser.add_argument(
        "--reference-rail-position",
        type=float,
        default=PICK_RAIL_POSITION_METERS,
        help=(
            "Rail position used with the zero left-arm posture to locate the "
            "fingertips."
        ),
    )
    parser.add_argument(
        "--opposite-side-model-name",
        default=DEFAULT_OPPOSITE_SIDE_MODEL,
        help=(
            "Move this model to the mirrored position on the other side of "
            "the robot centerline."
        ),
    )
    return parser.parse_args()


def _model_blocks(lines: list[str]) -> list[tuple[int, int]]:
    """Return half-open line ranges for top-level add_model directives."""
    starts = [
        index
        for index, line in enumerate(lines)
        if line.rstrip("\r\n") == "- add_model:"
    ]
    blocks = []
    for start in starts:
        end = start + 1
        while end < len(lines) and not lines[end].startswith("- "):
            end += 1
        blocks.append((start, end))
    return blocks


def _replace_model_file(
    source_text: str,
    model_name: str,
    replacement_uri: str,
) -> str:
    """Replace one add_model file field while preserving the DMD text."""
    lines = source_text.splitlines(keepends=True)
    matches = []
    for start, end in _model_blocks(lines):
        name_line = next(
            (
                index
                for index in range(start + 1, end)
                if lines[index].strip() == f"name: {model_name}"
            ),
            None,
        )
        if name_line is None:
            continue
        file_line = next(
            (
                index
                for index in range(start + 1, end)
                if lines[index].lstrip().startswith("file:")
            ),
            None,
        )
        if file_line is None:
            raise ValueError(f"Model {model_name!r} has no file field")
        matches.append(file_line)

    if len(matches) != 1:
        raise ValueError(
            f"Expected exactly one model named {model_name!r}, "
            f"found {len(matches)}"
        )
    index = matches[0]
    newline = "\r\n" if lines[index].endswith("\r\n") else "\n"
    indentation = lines[index][: len(lines[index]) - len(lines[index].lstrip())]
    lines[index] = f"{indentation}file: {replacement_uri}{newline}"
    return "".join(lines)


def _model_block(
    lines: list[str],
    model_name: str,
) -> tuple[int, int]:
    """Return the unique top-level add_model block for a named model."""
    matches = []
    for start, end in _model_blocks(lines):
        if any(
            lines[index].strip() == f"name: {model_name}"
            for index in range(start + 1, end)
        ):
            matches.append((start, end))
    if len(matches) != 1:
        raise ValueError(
            f"Expected exactly one model named {model_name!r}, "
            f"found {len(matches)}"
        )
    return matches[0]


def _target_base_frame(source_text: str, model_name: str) -> str:
    """Return the base frame used by a model's default free-body pose."""
    lines = source_text.splitlines()
    start, end = _model_block(lines, model_name)
    base_frames = [
        line.split(":", maxsplit=1)[1].strip()
        for line in lines[start:end]
        if line.strip().startswith("base_frame:")
    ]
    if len(base_frames) != 1:
        raise ValueError(
            f"Expected one base_frame for {model_name!r}, "
            f"found {len(base_frames)}"
        )
    return base_frames[0]


def _replace_model_pose(
    source_text: str,
    model_name: str,
    translation: np.ndarray,
    rpy_deg: np.ndarray,
) -> str:
    """Replace one model's default pose with an explicit calibrated pose."""
    lines = source_text.splitlines(keepends=True)
    start, end = _model_block(lines, model_name)
    translation_indices = [
        index
        for index in range(start, end)
        if lines[index].strip().startswith("translation:")
    ]
    rotation_indices = [
        index
        for index in range(start, end)
        if lines[index].strip().startswith("rotation:")
    ]
    if len(translation_indices) != 1 or len(rotation_indices) != 1:
        raise ValueError(
            f"Expected one translation and rotation for {model_name!r}"
        )

    translation_index = translation_indices[0]
    translation_indent = lines[translation_index][
        : len(lines[translation_index]) - len(lines[translation_index].lstrip())
    ]
    newline = "\r\n" if lines[translation_index].endswith("\r\n") else "\n"
    lines[translation_index] = (
        f"{translation_indent}translation: {translation.tolist()}{newline}"
    )

    rotation_index = rotation_indices[0]
    rotation_indent = len(lines[rotation_index]) - len(
        lines[rotation_index].lstrip()
    )
    rotation_end = rotation_index + 1
    while rotation_end < end:
        stripped = lines[rotation_end].strip()
        indentation = len(lines[rotation_end]) - len(
            lines[rotation_end].lstrip()
        )
        if stripped and indentation <= rotation_indent:
            break
        rotation_end += 1
    prefix = lines[rotation_index][:rotation_indent]
    replacement = [
        f"{prefix}rotation: !Rpy{newline}",
        f"{prefix}  deg: {rpy_deg.tolist()}{newline}",
    ]
    lines[rotation_index:rotation_end] = replacement
    return "".join(lines)


def _build_scene(dmd: Path, package_xmls: tuple[Path, ...]):
    """Load one DMD into a finalized Drake plant for geometry queries."""
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    for package_xml in package_xmls:
        _register_package_xml(parser, package_xml)
    ProcessModelDirectives(LoadModelDirectives(str(dmd)), parser)
    plant.Finalize()
    diagram = builder.Build()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    return diagram, plant, plant_context


def _reference_gripper_geometry(rail_position: float) -> dict:
    """Measure the open left fingertips at the requested reference posture."""
    robot_urdf = ZERITH_MODEL_DIR / ZERITH_URDF_RELATIVE_PATH
    if not robot_urdf.is_file():
        raise FileNotFoundError(f"Zerith URDF does not exist: {robot_urdf}")

    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    parser = Parser(plant)
    parser.package_map().Add(ZERITH_PACKAGE_NAME, str(ZERITH_MODEL_DIR))
    zerith = parser.AddModels(str(robot_urdf))[0]
    grasp_frame = add_left_grasp_frame(plant, zerith)
    plant.WeldFrames(
        plant.world_frame(),
        plant.GetFrameByName("dipan_link", zerith),
        RigidTransform(
            RollPitchYaw(
                0.0,
                0.0,
                np.deg2rad(ROBOT_BASE_YAW_DEG),
            ),
            ROBOT_BASE_XYZ_METERS,
        ),
    )
    plant.Finalize()
    diagram = builder.Build()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    positions = plant.GetPositions(plant_context).copy()
    rail_joint = plant.GetJointByName("daogui_joint", zerith)
    if not (
        rail_joint.position_lower_limits()[0]
        <= rail_position
        <= rail_joint.position_upper_limits()[0]
    ):
        raise ValueError("reference-rail-position is outside its joint limits")
    positions[rail_joint.position_start()] = rail_position
    for config in LEFT_ARM_SERVO_CONFIGS:
        joint = plant.GetJointByName(config.name, zerith)
        positions[joint.position_start()] = 0.0
    plant.SetPositions(plant_context, positions)

    X_WG = plant.CalcRelativeTransform(
        plant_context,
        plant.world_frame(),
        grasp_frame,
    )
    approach_world = X_WG.rotation().multiply(APPROACH_AXIS_GRASP)
    closing_world = X_WG.rotation().multiply(CLOSING_AXIS_GRASP)
    if abs(approach_world[2]) > 1e-8 or abs(closing_world[2]) > 1e-8:
        raise ValueError(
            "Reference grasp approach and closing axes must be horizontal"
        )
    approach_xy = approach_world[:2] / np.linalg.norm(approach_world[:2])
    closing_xy = closing_world[:2] / np.linalg.norm(closing_world[:2])

    query = plant.get_geometry_query_input_port().Eval(plant_context)
    inspector = query.inspector()
    fingertip_points = []
    fingertip_geometry_names = []
    for body_name in (
        "left_jaw_left_finger_link",
        "left_jaw_right_finger_link",
    ):
        body = plant.GetBodyByName(body_name, zerith)
        X_WB = plant.EvalBodyPoseInWorld(plant_context, body)
        for geometry_id in plant.GetCollisionGeometriesForBody(body):
            geometry_name = inspector.GetName(geometry_id)
            if "finger_tip" not in geometry_name:
                continue
            shape = inspector.GetShape(geometry_id)
            if not isinstance(shape, Box):
                raise TypeError(
                    f"Fingertip geometry must be a Box: {geometry_name}"
                )
            X_WF = X_WB.multiply(inspector.GetPoseInFrame(geometry_id))
            half_size = 0.5 * np.array(
                [shape.width(), shape.depth(), shape.height()]
            )
            corners = np.array(
                [
                    [x, y, z]
                    for x in (-half_size[0], half_size[0])
                    for y in (-half_size[1], half_size[1])
                    for z in (-half_size[2], half_size[2])
                ]
            )
            fingertip_points.extend(X_WF.multiply(corner) for corner in corners)
            fingertip_geometry_names.append(geometry_name)
    if len(fingertip_geometry_names) != 2:
        raise ValueError(
            "Expected exactly two left fingertip collision geometries, found "
            f"{len(fingertip_geometry_names)}"
        )
    fingertip_points = np.asarray(fingertip_points)
    return {
        "rail_position_m": rail_position,
        "left_arm_joint_positions_rad": [0.0] * len(LEFT_ARM_SERVO_CONFIGS),
        "grasp_origin_world_m": X_WG.translation(),
        "approach_axis_world": approach_world,
        "closing_axis_world": closing_world,
        "approach_axis_xy_world": approach_xy,
        "closing_axis_xy_world": closing_xy,
        "fingertip_front_projection_m": float(
            np.max(fingertip_points[:, :2] @ approach_xy)
        ),
        "fingertip_aabb_min_world_m": fingertip_points.min(axis=0),
        "fingertip_aabb_max_world_m": fingertip_points.max(axis=0),
        "fingertip_geometry_names": fingertip_geometry_names,
    }


def _world_collision_bounds(plant, plant_context, body):
    """Return minimum and maximum world collision-AABB corners."""
    query = plant.get_geometry_query_input_port().Eval(plant_context)
    body_aabb = _CalcAabb(query.inspector(), body)
    world_aabb = _OabbToAabb(
        body_aabb,
        plant.EvalBodyPoseInWorld(plant_context, body),
    )
    half_size = 0.5 * world_aabb.size
    return world_aabb.center - half_size, world_aabb.center + half_size


def _calibrated_poses(
    *,
    source_dmd: Path,
    scene_package_xml: Path,
    source_text: str,
    target_model_name: str,
    table_model_name: str,
    opposite_side_model_name: str,
    fingertip_clearance: float,
    reference_rail_position: float,
    box_yaw_deg: float | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
    """Compute initial red-box and opposite-side-object support poses."""
    if fingertip_clearance < 0.0:
        raise ValueError("fingertip-clearance must be nonnegative")
    gripper = _reference_gripper_geometry(reference_rail_position)
    _, plant, plant_context = _build_scene(
        source_dmd,
        (scene_package_xml,),
    )
    target_instance = plant.GetModelInstanceByName(target_model_name)
    table_instance = plant.GetModelInstanceByName(table_model_name)
    opposite_instance = plant.GetModelInstanceByName(
        opposite_side_model_name
    )
    target_body = plant.GetBodyByName("base_link", target_instance)
    table_body = plant.GetBodyByName("base_link", table_instance)
    opposite_body = plant.GetBodyByName("base_link", opposite_instance)
    X_WT_source = plant.EvalBodyPoseInWorld(plant_context, target_body)
    X_WS = plant.EvalBodyPoseInWorld(plant_context, table_body)
    X_WO = plant.EvalBodyPoseInWorld(plant_context, opposite_body)
    table_min, table_max = _world_collision_bounds(
        plant,
        plant_context,
        table_body,
    )
    source_rpy = RollPitchYaw(X_WT_source.rotation())
    table_rpy = RollPitchYaw(X_WS.rotation())
    yaw_rad = (
        np.deg2rad(box_yaw_deg)
        if box_yaw_deg is not None
        else source_rpy.yaw_angle()
    )
    source_xyz_world = X_WT_source.translation().copy()
    box_rotation_world = RollPitchYaw(
        table_rpy.roll_angle(),
        table_rpy.pitch_angle(),
        yaw_rad,
    ).ToRotationMatrix()
    approach_xy = gripper["approach_axis_xy_world"]
    closing_xy = gripper["closing_axis_xy_world"]
    box_half_extent_along_approach = float(
        np.abs(
            box_rotation_world.matrix().T
            @ np.r_[approach_xy, 0.0]
        )
        @ (0.5 * BOX_SIZE_METERS)
    )
    target_approach_projection = (
        gripper["fingertip_front_projection_m"]
        + fingertip_clearance
        + box_half_extent_along_approach
    )
    target_closing_projection = float(
        closing_xy @ gripper["grasp_origin_world_m"][:2]
    )
    target_xyz_world = source_xyz_world.copy()
    target_xyz_world[:2] = np.linalg.solve(
        np.vstack([approach_xy, closing_xy]),
        np.array(
            [target_approach_projection, target_closing_projection]
        ),
    )
    target_xyz_world[2] = table_max[2] + 0.5 * BOX_SIZE_METERS[2]
    X_WT_calibrated = RigidTransform(
        box_rotation_world,
        target_xyz_world,
    )

    robot_xy_world = ROBOT_BASE_XYZ_METERS[:2]
    robot_yaw_rad = np.deg2rad(ROBOT_BASE_YAW_DEG)
    R_WB_xy = np.array(
        [
            [np.cos(robot_yaw_rad), -np.sin(robot_yaw_rad)],
            [np.sin(robot_yaw_rad), np.cos(robot_yaw_rad)],
        ]
    )
    target_xy_robot = R_WB_xy.T @ (
        target_xyz_world[:2] - robot_xy_world
    )
    opposite_xy_robot = target_xy_robot.copy()
    opposite_xy_robot[1] *= -1.0
    opposite_xyz_world = X_WO.translation().copy()
    opposite_xyz_world[:2] = robot_xy_world + R_WB_xy @ opposite_xy_robot
    X_WO_calibrated = RigidTransform(
        X_WO.rotation(),
        opposite_xyz_world,
    )

    target_base_frame_name = _target_base_frame(
        source_text,
        target_model_name,
    )
    target_base_frame = plant.GetFrameByName(target_base_frame_name)
    X_WB_target = plant.CalcRelativeTransform(
        plant_context,
        plant.world_frame(),
        target_base_frame,
    )
    X_BT_calibrated = X_WB_target.inverse().multiply(X_WT_calibrated)
    target_relative_rpy_deg = np.rad2deg(
        RollPitchYaw(X_BT_calibrated.rotation()).vector()
    )
    opposite_base_frame_name = _target_base_frame(
        source_text,
        opposite_side_model_name,
    )
    opposite_base_frame = plant.GetFrameByName(opposite_base_frame_name)
    X_WB_opposite = plant.CalcRelativeTransform(
        plant_context,
        plant.world_frame(),
        opposite_base_frame,
    )
    X_BO_calibrated = X_WB_opposite.inverse().multiply(X_WO_calibrated)
    opposite_relative_rpy_deg = np.rad2deg(
        RollPitchYaw(X_BO_calibrated.rotation()).vector()
    )
    direction_to_robot = robot_xy_world - source_xyz_world[:2]
    direction_to_robot /= np.linalg.norm(direction_to_robot)
    target_displacement = target_xyz_world[:2] - source_xyz_world[:2]
    calibration = {
        "coordinate_convention": (
            "small-box base_link is the geometric and inertial center"
        ),
        "support_formula": "box_center_z = table_top_z + box_height / 2",
        "table_model_name": table_model_name,
        "table_collision_aabb_min_world_m": table_min.tolist(),
        "table_collision_aabb_max_world_m": table_max.tolist(),
        "table_top_z_world_m": float(table_max[2]),
        "table_body_rpy_world_deg": np.rad2deg(
            table_rpy.vector()
        ).tolist(),
        "source_target_body_xyz_world_m": source_xyz_world.tolist(),
        "source_target_rpy_world_deg": np.rad2deg(
            source_rpy.vector()
        ).tolist(),
        "calibrated_target_body_xyz_world_m": target_xyz_world.tolist(),
        "target_xy_displacement_world_m": target_displacement.tolist(),
        "target_xy_displacement_norm_m": float(
            np.linalg.norm(target_displacement)
        ),
        "target_displacement_toward_robot_m": float(
            target_displacement @ direction_to_robot
        ),
        "fingertip_clearance_xy_m": fingertip_clearance,
        "fingertip_clearance_definition": (
            "top-view gap along the open left gripper approach axis from "
            "the fingertip front plane to the nearest red-box face"
        ),
        "box_half_extent_along_approach_m": (
            box_half_extent_along_approach
        ),
        "reference_gripper": {
            key: value.tolist() if isinstance(value, np.ndarray) else value
            for key, value in gripper.items()
        },
        "opposite_side_model_name": opposite_side_model_name,
        "source_opposite_side_model_xyz_world_m": (
            X_WO.translation().tolist()
        ),
        "calibrated_opposite_side_model_xyz_world_m": (
            opposite_xyz_world.tolist()
        ),
        "calibrated_opposite_side_model_xy_in_robot_frame_m": (
            opposite_xy_robot.tolist()
        ),
        "calibrated_target_xy_in_robot_frame_m": target_xy_robot.tolist(),
        "lateral_layout": (
            "red box aligned with the left gripper; opposite-side model "
            "mirrored across the robot x-axis"
        ),
        "calibrated_target_rpy_world_deg": [
            float(np.rad2deg(table_rpy.roll_angle())),
            float(np.rad2deg(table_rpy.pitch_angle())),
            float(np.rad2deg(yaw_rad)),
        ],
        "calibrated_target_yaw_world_deg": float(np.rad2deg(yaw_rad)),
        "calibrated_target_yaw_world_rad": float(yaw_rad),
        "dmd_rotation_serialization": "!Rpy deg",
        "calibrated_base_frame": target_base_frame_name,
        "calibrated_translation_in_base_frame_m": (
            X_BT_calibrated.translation().tolist()
        ),
        "calibrated_rpy_in_base_frame_deg": (
            target_relative_rpy_deg.tolist()
        ),
        "opposite_side_calibrated_base_frame": opposite_base_frame_name,
        "opposite_side_calibrated_translation_in_base_frame_m": (
            X_BO_calibrated.translation().tolist()
        ),
        "opposite_side_calibrated_rpy_in_base_frame_deg": (
            opposite_relative_rpy_deg.tolist()
        ),
        "xy_position_source_preserved": False,
        "roll_pitch_leveled": False,
        "roll_pitch_aligned_to_table_body": True,
    }
    return (
        X_BT_calibrated.translation(),
        target_relative_rpy_deg,
        X_BO_calibrated.translation(),
        opposite_relative_rpy_deg,
        calibration,
    )


def _minimum_body_signed_distance(
    plant,
    plant_context,
    body_a,
    body_b,
) -> float:
    """Return the minimum signed distance between two bodies."""
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
        raise ValueError(
            f"Bodies {body_a.name()!r} and {body_b.name()!r} must both "
            "have collision geometry"
        )
    return min(distances)


def _refine_support_pose(
    *,
    derived_dmd: Path,
    scene_package_xml: Path,
    model_name: str,
    support_model_name: str,
    base_frame_name: str,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Refine one free body's Z until it touches its support geometry."""
    _, plant, plant_context = _build_scene(
        derived_dmd,
        (scene_package_xml, EVAL_PACKAGE_XML),
    )
    model_instance = plant.GetModelInstanceByName(model_name)
    support_instance = plant.GetModelInstanceByName(support_model_name)
    body = plant.GetBodyByName("base_link", model_instance)
    support_body = plant.GetBodyByName("base_link", support_instance)
    X_WM = plant.EvalBodyPoseInWorld(plant_context, body)
    base_frame = plant.GetFrameByName(base_frame_name)
    X_WB = plant.CalcRelativeTransform(
        plant_context,
        plant.world_frame(),
        base_frame,
    )

    def distance_at_z(z_world: float) -> float:
        X_WM_test = RigidTransform(
            X_WM.rotation(),
            np.r_[X_WM.translation()[:2], z_world],
        )
        plant.SetFreeBodyPose(
            plant_context,
            body,
            X_WB.inverse().multiply(X_WM_test),
        )
        return _minimum_body_signed_distance(
            plant,
            plant_context,
            body,
            support_body,
        )

    initial_z = float(X_WM.translation()[2])
    initial_distance = distance_at_z(initial_z)
    bracket_step = 0.001
    maximum_adjustment = 0.05
    if initial_distance >= 0.0:
        upper_z = initial_z
        upper_distance = initial_distance
        lower_z = initial_z - bracket_step
        lower_distance = distance_at_z(lower_z)
        while (
            lower_distance > 0.0
            and initial_z - lower_z < maximum_adjustment
        ):
            lower_z -= bracket_step
            lower_distance = distance_at_z(lower_z)
    else:
        lower_z = initial_z
        lower_distance = initial_distance
        upper_z = initial_z + bracket_step
        upper_distance = distance_at_z(upper_z)
        while (
            upper_distance < 0.0
            and upper_z - initial_z < maximum_adjustment
        ):
            upper_z += bracket_step
            upper_distance = distance_at_z(upper_z)
    if lower_distance > 0.0 or upper_distance < 0.0:
        raise ValueError(
            f"Could not bracket {model_name!r} against "
            f"{support_model_name!r} within {maximum_adjustment} m"
        )
    for _ in range(60):
        middle_z = 0.5 * (lower_z + upper_z)
        middle_distance = distance_at_z(middle_z)
        if middle_distance < 0.0:
            lower_z = middle_z
            lower_distance = middle_distance
        else:
            upper_z = middle_z
            upper_distance = middle_distance
    final_xyz_world = X_WM.translation().copy()
    final_xyz_world[2] = upper_z
    X_WM_refined = RigidTransform(X_WM.rotation(), final_xyz_world)

    X_BM_refined = X_WB.inverse().multiply(X_WM_refined)
    rpy_deg = np.rad2deg(
        RollPitchYaw(X_BM_refined.rotation()).vector()
    )
    return (
        X_BM_refined.translation(),
        rpy_deg,
        {
            "model_name": model_name,
            "support_model_name": support_model_name,
            "initial_center_z_world_m": initial_z,
            "initial_support_distance_m": initial_distance,
            "refined_center_z_world_m": upper_z,
            "refined_support_signed_distance_m": upper_distance,
            "penetrating_bracket_distance_m": lower_distance,
            "method": (
                "vertical signed-distance bracketing and bisection at "
                "fixed XY and orientation"
            ),
        },
    )


def _derived_geometry_diagnostics(
    *,
    derived_dmd: Path,
    scene_package_xml: Path,
    target_model_name: str,
    table_model_name: str,
    opposite_side_model_name: str,
) -> dict:
    """Measure support alignment and nearby geometry in the derived scene."""
    _, plant, plant_context = _build_scene(
        derived_dmd,
        (scene_package_xml, EVAL_PACKAGE_XML),
    )
    target_instance = plant.GetModelInstanceByName(target_model_name)
    table_instance = plant.GetModelInstanceByName(table_model_name)
    opposite_instance = plant.GetModelInstanceByName(
        opposite_side_model_name
    )
    target_body = plant.GetBodyByName("base_link", target_instance)
    table_body = plant.GetBodyByName("base_link", table_instance)
    opposite_body = plant.GetBodyByName("base_link", opposite_instance)
    target_min, target_max = _world_collision_bounds(
        plant,
        plant_context,
        target_body,
    )
    table_min, table_max = _world_collision_bounds(
        plant,
        plant_context,
        table_body,
    )

    query = plant.get_geometry_query_input_port().Eval(plant_context)
    inspector = query.inspector()
    target_geometry_ids = set(
        plant.GetCollisionGeometriesForBody(target_body)
    )
    nearby = []
    for pair in query.ComputeSignedDistancePairwiseClosestPoints(
        max_distance=1.0
    ):
        if (
            pair.id_A not in target_geometry_ids
            and pair.id_B not in target_geometry_ids
        ):
            continue
        other_id = pair.id_B if pair.id_A in target_geometry_ids else pair.id_A
        other_body = plant.GetBodyFromFrameId(inspector.GetFrameId(other_id))
        nearby.append(
            {
                "distance_m": float(pair.distance),
                "other_model": plant.GetModelInstanceName(
                    other_body.model_instance()
                ),
                "other_body": other_body.name(),
                "other_geometry": inspector.GetName(other_id),
            }
        )
    nearby.sort(key=lambda item: item["distance_m"])
    nearest_by_model = {}
    for item in nearby:
        nearest_by_model.setdefault(item["other_model"], item)
    nearest_models = sorted(
        nearest_by_model.values(),
        key=lambda item: item["distance_m"],
    )
    target_table_distance = nearest_by_model[table_model_name]["distance_m"]
    X_WT = plant.EvalBodyPoseInWorld(plant_context, target_body)
    X_WO = plant.EvalBodyPoseInWorld(plant_context, opposite_body)
    return {
        "target_body_xyz_world_m": X_WT.translation().tolist(),
        "target_body_rpy_world_deg": np.rad2deg(
            RollPitchYaw(X_WT.rotation()).vector()
        ).tolist(),
        "target_collision_aabb_min_world_m": target_min.tolist(),
        "target_collision_aabb_max_world_m": target_max.tolist(),
        "table_collision_aabb_min_world_m": table_min.tolist(),
        "table_collision_aabb_max_world_m": table_max.tolist(),
        "support_gap_m": target_table_distance,
        "minimum_target_table_signed_distance_m": target_table_distance,
        "opposite_side_model_name": opposite_side_model_name,
        "opposite_side_body_xyz_world_m": X_WO.translation().tolist(),
        "opposite_side_body_rpy_world_deg": np.rad2deg(
            RollPitchYaw(X_WO.rotation()).vector()
        ).tolist(),
        "minimum_target_opposite_side_signed_distance_m": (
            nearest_by_model[opposite_side_model_name]["distance_m"]
        ),
        "global_aabb_vertical_gap_m": float(
            target_min[2] - table_max[2]
        ),
        "nearby_target_models": nearest_models,
        "nearby_target_collision_pairs": nearby,
        "requires_manual_visual_confirmation": True,
    }


def _task_metadata(
    source_dmd: Path,
    target_model_name: str,
    calibration: dict,
    derived_diagnostics: dict,
) -> dict:
    """Return synchronized task and grasp metadata."""
    return {
        "task": (
            "Pick up the only small red rectangular box from the coffee "
            "table and lift it at least 5 cm."
        ),
        "source_dmd": str(source_dmd),
        "target": {
            "model_name": target_model_name,
            "body_name": "base_link",
            "asset_uri": SMALL_BOX_URI,
            "dimensions_m": BOX_SIZE_METERS.tolist(),
            "grasp_width_m": GRASP_WIDTH_METERS,
        },
        "robot": {
            "model": "zerith",
            "arm": "left",
            "base_xyz_m": ROBOT_BASE_XYZ_METERS.tolist(),
            "base_yaw_deg": ROBOT_BASE_YAW_DEG,
            "placement_status": "authoritative_pick_task_baseline",
            "placement_reason": "intentionally positioned beside coffee table",
        },
        "rail_calibration": {
            "status": "pending_manual_and_vendor_parameter_confirmation",
            "joint_name": "daogui_joint",
            "axis": [0.0, 0.0, 1.0],
            "urdf_position_range_m": [0.0, 0.8],
            "upstream_effort_limit": 0.0,
            "upstream_velocity_limit": 0.0,
            "effort_velocity_interpretation": (
                "missing_model_parameters; not physical zero capability"
            ),
        },
        "locked_rail_pick_results_status": (
            "pregrasp_validated_open_gripper_only"
        ),
        "fixed_rail_pregrasp": {
            "joint_position_m": calibration["reference_gripper"][
                "rail_position_m"
            ],
            "validated_scope": "pick_home_to_pregrasp",
            "gripper_state": "open",
            "approach_and_grasp_approved": False,
        },
        "grasp": {
            "frame_name": LEFT_GRASP_FRAME_NAME,
            "parent_frame": LEFT_GRASP_PARENT_FRAME_NAME,
            "translation_xyz_m": (
                X_PARENT_GRASP_TRANSLATION_METERS.tolist()
            ),
            "approach_axis": APPROACH_AXIS_GRASP.tolist(),
            "closing_axis": CLOSING_AXIS_GRASP.tolist(),
            "up_axis": UP_AXIS_GRASP.tolist(),
            "pregrasp_distance_m": PREGRASP_DISTANCE_METERS,
            "lift_distance_m": LIFT_DISTANCE_METERS,
        },
        "source_target_pose_preserved": False,
        "target_calibration": calibration,
        "derived_geometry_diagnostics": derived_diagnostics,
        "success": {
            "minimum_lift_m": 0.05,
            "stable_duration_s": 1.0,
            "requires_both_finger_contacts": True,
        },
    }


def main() -> None:
    """Write a derived DMD and matching task metadata."""
    args = _parse_args()
    source_dmd = args.source_dmd.resolve()
    output_dir = args.output_dir.resolve()
    output_dmd = output_dir / "zerith_pick_eval.dmd.yaml"
    output_metadata = output_dir / "task_metadata.yaml"
    scene_package_xml = (
        args.scene_package_xml.resolve()
        if args.scene_package_xml is not None
        else find_package_xml(source_dmd)
    )

    if not source_dmd.is_file():
        raise FileNotFoundError(f"Source DMD does not exist: {source_dmd}")
    if output_dmd == source_dmd:
        raise ValueError("Output DMD must not overwrite the source DMD")
    if not scene_package_xml.is_file():
        raise FileNotFoundError(
            f"Scene package.xml does not exist: {scene_package_xml}"
        )

    source_text = source_dmd.read_text(encoding="utf-8")
    (
        target_translation,
        target_rpy_deg,
        opposite_translation,
        opposite_rpy_deg,
        calibration,
    ) = _calibrated_poses(
        source_dmd=source_dmd,
        scene_package_xml=scene_package_xml,
        source_text=source_text,
        target_model_name=args.target_model_name,
        table_model_name=args.table_model_name,
        opposite_side_model_name=args.opposite_side_model_name,
        fingertip_clearance=args.fingertip_clearance,
        reference_rail_position=args.reference_rail_position,
        box_yaw_deg=args.box_yaw_deg,
    )
    derived_text = _replace_model_file(
        source_text,
        args.target_model_name,
        SMALL_BOX_URI,
    )
    derived_text = _replace_model_pose(
        derived_text,
        args.target_model_name,
        target_translation,
        target_rpy_deg,
    )
    derived_text = _replace_model_pose(
        derived_text,
        args.opposite_side_model_name,
        opposite_translation,
        opposite_rpy_deg,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    output_dmd.write_text(derived_text, encoding="utf-8")
    target_base_frame_name = _target_base_frame(
        derived_text,
        args.target_model_name,
    )
    target_translation, target_rpy_deg, target_support_refinement = (
        _refine_support_pose(
            derived_dmd=output_dmd,
            scene_package_xml=scene_package_xml,
            model_name=args.target_model_name,
            support_model_name=args.table_model_name,
            base_frame_name=target_base_frame_name,
        )
    )
    derived_text = _replace_model_pose(
        derived_text,
        args.target_model_name,
        target_translation,
        target_rpy_deg,
    )
    output_dmd.write_text(derived_text, encoding="utf-8")
    opposite_base_frame_name = _target_base_frame(
        derived_text,
        args.opposite_side_model_name,
    )
    opposite_translation, opposite_rpy_deg, opposite_support_refinement = (
        _refine_support_pose(
            derived_dmd=output_dmd,
            scene_package_xml=scene_package_xml,
            model_name=args.opposite_side_model_name,
            support_model_name=args.table_model_name,
            base_frame_name=opposite_base_frame_name,
        )
    )
    derived_text = _replace_model_pose(
        derived_text,
        args.opposite_side_model_name,
        opposite_translation,
        opposite_rpy_deg,
    )
    output_dmd.write_text(derived_text, encoding="utf-8")
    calibration["target_support_surface_refinement"] = (
        target_support_refinement
    )
    calibration["opposite_side_support_surface_refinement"] = (
        opposite_support_refinement
    )
    derived_diagnostics = _derived_geometry_diagnostics(
        derived_dmd=output_dmd,
        scene_package_xml=scene_package_xml,
        target_model_name=args.target_model_name,
        table_model_name=args.table_model_name,
        opposite_side_model_name=args.opposite_side_model_name,
    )
    calibration["calibrated_target_body_xyz_world_m"] = (
        derived_diagnostics["target_body_xyz_world_m"]
    )
    calibration["calibrated_target_rpy_world_deg"] = (
        derived_diagnostics["target_body_rpy_world_deg"]
    )
    calibration["calibrated_translation_in_base_frame_m"] = (
        target_translation.tolist()
    )
    calibration["calibrated_rpy_in_base_frame_deg"] = (
        target_rpy_deg.tolist()
    )
    calibration["calibrated_opposite_side_model_xyz_world_m"] = (
        derived_diagnostics["opposite_side_body_xyz_world_m"]
    )
    calibration["calibrated_opposite_side_model_rpy_world_deg"] = (
        derived_diagnostics["opposite_side_body_rpy_world_deg"]
    )
    calibration["opposite_side_calibrated_translation_in_base_frame_m"] = (
        opposite_translation.tolist()
    )
    calibration["opposite_side_calibrated_rpy_in_base_frame_deg"] = (
        opposite_rpy_deg.tolist()
    )
    output_metadata.write_text(
        json.dumps(
            _task_metadata(
                source_dmd,
                args.target_model_name,
                calibration,
                derived_diagnostics,
            ),
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"Derived DMD: {output_dmd}")
    print(f"Task metadata: {output_metadata}")
    print(f"Source DMD left unchanged: {source_dmd}")
    print(
        "Calibrated box support gap: "
        f"{derived_diagnostics['support_gap_m']:.9f} m"
    )
    print("Manual multi-view confirmation is still required")


if __name__ == "__main__":
    main()
