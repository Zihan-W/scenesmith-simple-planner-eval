"""Collision-checking utilities for Zerith PREGRASP validation."""

import dataclasses

from pathlib import Path
from typing import Any

import numpy as np

from pydrake.all import (
    LoadModelDirectives,
    ProcessModelDirectives,
    RigidTransform,
    RollPitchYaw,
)
from pydrake.common import Parallelism
from pydrake.planning import RobotDiagramBuilder, SceneGraphCollisionChecker

from src.zerith_grasp_geometry import add_left_grasp_frame
from src.zerith_online_env import (
    LEFT_ARM_SERVO_CONFIGS,
    ZERITH_PACKAGE_NAME,
    ZERITH_URDF_RELATIVE_PATH,
    _register_package_xml,
)

ROOM_GEOMETRY_MODEL_NAME = "room_geometry_living_room"
ROOM_GEOMETRY_BODY_NAME = "room_geometry_body_link"


@dataclasses.dataclass(frozen=True)
class PlanningCollisionFilter:
    """One explicitly filtered body pair and its planning rationale."""

    body_a: str
    body_b: str
    reason: str


@dataclasses.dataclass
class PregraspPlanningModel:
    """Drake model, contexts, and indices used for PREGRASP validation."""

    diagram: Any
    plant: Any
    plant_context: Any
    zerith: Any
    grasp_frame: Any
    collision_checker: Any
    collision_checker_context: Any
    q_scene: np.ndarray
    arm_indices: np.ndarray
    arm_lower_limits: np.ndarray
    arm_upper_limits: np.ndarray
    moving_left_arm_body_indices: set


def _moving_left_arm_body_indices(plant, robot_model_instance) -> set:
    """Return the body subtree moved by the first left-arm joint."""
    root_joint = plant.GetJointByName(
        LEFT_ARM_SERVO_CONFIGS[0].name,
        robot_model_instance,
    )
    moving_body_indices = {root_joint.child_body().index()}
    joints = [
        plant.get_joint(index)
        for index in plant.GetJointIndices(robot_model_instance)
    ]
    changed = True
    while changed:
        changed = False
        for joint in joints:
            if joint.parent_body().index() not in moving_body_indices:
                continue
            child_index = joint.child_body().index()
            if child_index in moving_body_indices:
                continue
            moving_body_indices.add(child_index)
            changed = True
    return moving_body_indices


def _arm_position_metadata(plant, zerith_model_instance):
    """Return left-arm position indices and joint limits in action order."""
    joints = [
        plant.GetJointByName(config.name, zerith_model_instance)
        for config in LEFT_ARM_SERVO_CONFIGS
    ]
    indices = np.array([joint.position_start() for joint in joints])
    lower = np.array([joint.position_lower_limits()[0] for joint in joints])
    upper = np.array([joint.position_upper_limits()[0] for joint in joints])
    return indices, lower, upper


def build_pregrasp_planning_model(
    *,
    scene_dmd: Path,
    scene_package_xml: Path,
    eval_package_xml: Path,
    robot_model_dir: Path,
    robot_xyz: np.ndarray,
    robot_yaw_deg: float,
) -> PregraspPlanningModel:
    """Build a RobotDiagram and robot-scoped SceneGraph collision checker."""
    builder = RobotDiagramBuilder(time_step=0.001)
    plant = builder.plant()
    parser = builder.parser()
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
            RollPitchYaw(0.0, 0.0, np.deg2rad(robot_yaw_deg)),
            robot_xyz,
        ),
    )
    grasp_frame = add_left_grasp_frame(plant, zerith)
    plant.Finalize()

    diagram = builder.Build()
    root_context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(root_context)
    collision_checker = SceneGraphCollisionChecker(
        model=diagram,
        robot_model_instances=[zerith],
        edge_step_size=0.05,
        implicit_context_parallelism=Parallelism(1),
    )
    collision_checker_context = (
        collision_checker.MakeStandaloneModelContext()
    )
    arm_indices, lower_limits, upper_limits = _arm_position_metadata(
        plant,
        zerith,
    )
    return PregraspPlanningModel(
        diagram=diagram,
        plant=plant,
        plant_context=plant_context,
        zerith=zerith,
        grasp_frame=grasp_frame,
        collision_checker=collision_checker,
        collision_checker_context=collision_checker_context,
        q_scene=plant.GetPositions(plant_context).copy(),
        arm_indices=arm_indices,
        arm_lower_limits=lower_limits,
        arm_upper_limits=upper_limits,
        moving_left_arm_body_indices=_moving_left_arm_body_indices(
            plant,
            zerith,
        ),
    )


def apply_pregrasp_planning_filters(
    model: PregraspPlanningModel,
) -> list[PlanningCollisionFilter]:
    """Filter only named invariant support and gripper assembly pairs."""
    plant = model.plant
    checker = model.collision_checker
    zerith = model.zerith
    floor_instance = plant.GetModelInstanceByName(ROOM_GEOMETRY_MODEL_NAME)
    floor = plant.GetBodyByName(
        ROOM_GEOMETRY_BODY_NAME,
        floor_instance,
    )

    applied = []
    for wheel_name in ("left_middle_wheel_link", "right_middle_wheel_link"):
        wheel = plant.GetBodyByName(wheel_name, zerith)
        checker.SetCollisionFilteredBetween(wheel, floor, True)
        applied.append(
            PlanningCollisionFilter(
                body_a=qualified_body_name(plant, wheel),
                body_b=qualified_body_name(plant, floor),
                reason="invariant fixed-base wheel-to-floor support",
            )
        )

    for side in ("left", "right"):
        palm = plant.GetBodyByName(f"{side}_end_effector_link", zerith)
        for finger_side in ("left", "right"):
            finger = plant.GetBodyByName(
                f"{side}_jaw_{finger_side}_finger_link",
                zerith,
            )
            checker.SetCollisionFilteredBetween(finger, palm, True)
            applied.append(
                PlanningCollisionFilter(
                    body_a=qualified_body_name(plant, finger),
                    body_b=qualified_body_name(plant, palm),
                    reason=(
                        "invariant sibling gripper assembly clearance below "
                        "the 2 mm planning margin"
                    ),
                )
            )
    return applied


def qualified_body_name(plant, body) -> str:
    """Return ``model_instance::body`` for a Drake body."""
    return (
        f"{plant.GetModelInstanceName(body.model_instance())}::{body.name()}"
    )


def _pair_category(
    model: PregraspPlanningModel,
    body_a,
    body_b,
) -> str:
    """Classify a body pair by robot and moving-left-arm membership."""
    robot_indices = set(model.plant.GetBodyIndices(model.zerith))
    moving_indices = model.moving_left_arm_body_indices
    a_robot = body_a.index() in robot_indices
    b_robot = body_b.index() in robot_indices
    a_moving = body_a.index() in moving_indices
    b_moving = body_b.index() in moving_indices
    if not a_robot and not b_robot:
        return "environment_environment"
    if a_moving or b_moving:
        if a_robot and b_robot:
            return "left_arm_robot"
        return "left_arm_environment"
    if a_robot and b_robot:
        return "fixed_robot_robot"
    return "fixed_robot_environment"


def raw_scene_distance_records(
    model: PregraspPlanningModel,
    q: np.ndarray,
    max_distance: float,
) -> list[dict]:
    """Return active SceneGraph distance pairs and requested categories."""
    plant = model.plant
    plant.SetPositions(model.plant_context, q)
    query = plant.get_geometry_query_input_port().Eval(model.plant_context)
    inspector = query.inspector()
    records = []
    pairs = query.ComputeSignedDistancePairwiseClosestPoints(
        max_distance=max_distance
    )
    for pair in sorted(pairs, key=lambda item: item.distance):
        body_a = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_A))
        body_b = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_B))
        records.append(
            {
                "distance_m": float(pair.distance),
                "category": _pair_category(model, body_a, body_b),
                "body_a": qualified_body_name(plant, body_a),
                "body_b": qualified_body_name(plant, body_b),
                "geometry_a": inspector.GetName(pair.id_A),
                "geometry_b": inspector.GetName(pair.id_B),
            }
        )
    return records


def _nearest_geometry_pair(model, body_a, body_b) -> dict:
    """Return the closest unfiltered geometry pair for two selected bodies."""
    plant = model.plant
    query = plant.get_geometry_query_input_port().Eval(model.plant_context)
    inspector = query.inspector()
    nearest = None
    for geometry_a in plant.GetCollisionGeometriesForBody(body_a):
        for geometry_b in plant.GetCollisionGeometriesForBody(body_b):
            pair = query.ComputeSignedDistancePairClosestPoints(
                geometry_a,
                geometry_b,
            )
            if nearest is None or pair.distance < nearest[0]:
                nearest = (pair.distance, geometry_a, geometry_b)
    if nearest is None:
        raise ValueError(
            "CollisionChecker returned bodies without collision geometries: "
            f"{qualified_body_name(plant, body_a)}, "
            f"{qualified_body_name(plant, body_b)}"
        )
    return {
        "geometry_a": inspector.GetName(nearest[1]),
        "geometry_b": inspector.GetName(nearest[2]),
        "raw_geometry_distance_m": float(nearest[0]),
    }


def robot_clearance_records(
    model: PregraspPlanningModel,
    q: np.ndarray,
    influence_distance: float,
) -> list[dict]:
    """Return robot-scoped CollisionChecker clearance records."""
    model.plant.SetPositions(model.plant_context, q)
    clearance = model.collision_checker.CalcContextRobotClearance(
        model.collision_checker_context,
        q,
        influence_distance,
    )
    records = []
    rows = zip(
        clearance.distances(),
        clearance.robot_indices(),
        clearance.other_indices(),
        clearance.collision_types(),
        strict=True,
    )
    for distance, robot_index, other_index, collision_type in sorted(
        rows,
        key=lambda item: item[0],
    ):
        robot_body = model.plant.get_body(robot_index)
        other_body = model.plant.get_body(other_index)
        record = {
            "distance_m": float(distance),
            "category": _pair_category(model, robot_body, other_body),
            "collision_type": str(collision_type),
            "body_a": qualified_body_name(model.plant, robot_body),
            "body_b": qualified_body_name(model.plant, other_body),
        }
        record.update(_nearest_geometry_pair(model, robot_body, other_body))
        records.append(record)
    return records
