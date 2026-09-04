"""Collision-checking utilities for Zerith PREGRASP validation."""

import dataclasses

from pathlib import Path
from typing import Any

import numpy as np

from pydrake.all import (
    BodyIndex,
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
    nonpenetration_checker: Any
    nonpenetration_checker_context: Any
    safety_checker: Any
    safety_checker_context: Any
    q_scene: np.ndarray
    arm_indices: np.ndarray
    arm_velocity_indices: np.ndarray
    arm_lower_limits: np.ndarray
    arm_upper_limits: np.ndarray
    rail_index: int
    rail_velocity_index: int
    rail_lower_limit: float
    rail_upper_limit: float
    moving_left_arm_body_indices: set
    active_joint_influences: dict


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
    """Return left-arm indices and joint limits in action order."""
    joints = [
        plant.GetJointByName(config.name, zerith_model_instance)
        for config in LEFT_ARM_SERVO_CONFIGS
    ]
    indices = np.array([joint.position_start() for joint in joints])
    velocity_indices = np.array([joint.velocity_start() for joint in joints])
    lower = np.array([joint.position_lower_limits()[0] for joint in joints])
    upper = np.array([joint.position_upper_limits()[0] for joint in joints])
    return indices, velocity_indices, lower, upper


def _active_joint_influences(
    plant,
    zerith_model_instance,
    active_joint_names: set[str],
) -> dict:
    """Map each body to active joints affecting its world pose."""
    influences = {plant.world_body().index(): frozenset()}
    pending = [
        plant.get_joint(index)
        for index in plant.GetJointIndices(zerith_model_instance)
    ]
    while pending:
        remaining = []
        for joint in pending:
            parent_index = joint.parent_body().index()
            if parent_index not in influences:
                remaining.append(joint)
                continue
            child_influences = set(influences[parent_index])
            if joint.name() in active_joint_names:
                child_influences.add(joint.name())
            influences[joint.child_body().index()] = frozenset(
                child_influences
            )
        if len(remaining) == len(pending):
            unresolved = [joint.name() for joint in remaining]
            raise RuntimeError(
                "Could not resolve Zerith kinematic influences for joints: "
                f"{unresolved}"
            )
        pending = remaining

    for body_index_value in range(plant.num_bodies()):
        body_index = BodyIndex(body_index_value)
        influences.setdefault(body_index, frozenset())
    return influences


def build_pregrasp_planning_model(
    *,
    scene_dmd: Path,
    scene_package_xml: Path,
    eval_package_xml: Path,
    robot_model_dir: Path,
    robot_xyz: np.ndarray,
    robot_yaw_deg: float,
    include_rail: bool = False,
    rail_position: float = 0.0,
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
    nonpenetration_checker = SceneGraphCollisionChecker(
        model=diagram,
        robot_model_instances=[zerith],
        edge_step_size=0.005,
        implicit_context_parallelism=Parallelism(1),
    )
    nonpenetration_checker_context = (
        nonpenetration_checker.MakeStandaloneModelContext()
    )
    safety_checker = SceneGraphCollisionChecker(
        model=diagram,
        robot_model_instances=[zerith],
        edge_step_size=0.005,
        implicit_context_parallelism=Parallelism(1),
    )
    safety_checker_context = (
        safety_checker.MakeStandaloneModelContext()
    )
    (
        arm_indices,
        arm_velocity_indices,
        lower_limits,
        upper_limits,
    ) = _arm_position_metadata(
        plant,
        zerith,
    )
    rail_joint = plant.GetJointByName("daogui_joint", zerith)
    if not (
        rail_joint.position_lower_limits()[0]
        <= rail_position
        <= rail_joint.position_upper_limits()[0]
    ):
        raise ValueError("rail_position is outside daogui_joint limits")
    q_scene = plant.GetPositions(plant_context).copy()
    q_scene[rail_joint.position_start()] = rail_position
    plant.SetPositions(plant_context, q_scene)
    active_joint_names = {
        config.name for config in LEFT_ARM_SERVO_CONFIGS
    }
    if include_rail:
        active_joint_names.add(rail_joint.name())
    return PregraspPlanningModel(
        diagram=diagram,
        plant=plant,
        plant_context=plant_context,
        zerith=zerith,
        grasp_frame=grasp_frame,
        nonpenetration_checker=nonpenetration_checker,
        nonpenetration_checker_context=nonpenetration_checker_context,
        safety_checker=safety_checker,
        safety_checker_context=safety_checker_context,
        q_scene=q_scene,
        arm_indices=arm_indices,
        arm_velocity_indices=arm_velocity_indices,
        arm_lower_limits=lower_limits,
        arm_upper_limits=upper_limits,
        rail_index=rail_joint.position_start(),
        rail_velocity_index=rail_joint.velocity_start(),
        rail_lower_limit=float(rail_joint.position_lower_limits()[0]),
        rail_upper_limit=float(rail_joint.position_upper_limits()[0]),
        moving_left_arm_body_indices=_moving_left_arm_body_indices(
            plant,
            zerith,
        ),
        active_joint_influences=_active_joint_influences(
            plant,
            zerith,
            active_joint_names,
        ),
    )


def apply_safety_clearance_filters(
    model: PregraspPlanningModel,
    influence_distance: float,
) -> list[PlanningCollisionFilter]:
    """Configure only the safety-clearance checker.

    Pairs invariant to the active left-arm joints do not participate in the
    arm's safety-margin optimization. The mechanically close left shoulder to
    torso pair is the sole explicit assembly whitelist entry. It remains in
    the separate nonpenetration checker. The dynamics plant is never changed.
    """
    if influence_distance <= 0.0:
        raise ValueError("influence_distance must be positive")
    plant = model.plant
    checker = model.safety_checker
    robot_body_indices = set(plant.GetBodyIndices(model.zerith))
    plant.SetPositions(model.plant_context, model.q_scene)
    query = plant.get_geometry_query_input_port().Eval(model.plant_context)
    inspector = query.inspector()
    nearby_body_pairs = set()
    for pair in query.ComputeSignedDistancePairwiseClosestPoints(
        max_distance=influence_distance
    ):
        body_a = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_A))
        body_b = plant.GetBodyFromFrameId(inspector.GetFrameId(pair.id_B))
        body_indices = tuple(sorted((body_a.index(), body_b.index())))
        nearby_body_pairs.add(body_indices)

    applied = []
    for body_a_index, body_b_index in sorted(nearby_body_pairs):
        if (
            body_a_index not in robot_body_indices
            and body_b_index not in robot_body_indices
        ):
            continue
        influences_a = model.active_joint_influences[body_a_index]
        influences_b = model.active_joint_influences[body_b_index]
        if influences_a != influences_b:
            continue
        if checker.IsCollisionFilteredBetween(body_a_index, body_b_index):
            continue
        body_a = plant.get_body(body_a_index)
        body_b = plant.get_body(body_b_index)
        checker.SetCollisionFilteredBetween(body_a, body_b, True)
        applied.append(
            PlanningCollisionFilter(
                body_a=qualified_body_name(plant, body_a),
                body_b=qualified_body_name(plant, body_b),
                reason=(
                    "relative geometry is invariant to the active left-arm "
                    "seven-joint configuration and lies within the safety "
                    "constraint influence distance"
                ),
            )
        )

    torso = plant.GetBodyByName("body_yaw_link", model.zerith)
    left_shoulder = plant.GetBodyByName(
        "left_shoulder_roll_link",
        model.zerith,
    )
    checker.SetCollisionFilteredBetween(torso, left_shoulder, True)
    applied.append(
        PlanningCollisionFilter(
            body_a=qualified_body_name(plant, torso),
            body_b=qualified_body_name(plant, left_shoulder),
            reason=(
                "explicit safety-margin whitelist for the mechanically "
                "close left-shoulder assembly; nonpenetration remains active"
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
    *,
    layer: str,
) -> list[dict]:
    """Return robot-scoped clearance records for one constraint layer."""
    if layer == "nonpenetration":
        checker = model.nonpenetration_checker
        checker_context = model.nonpenetration_checker_context
    elif layer == "safety":
        checker = model.safety_checker
        checker_context = model.safety_checker_context
    else:
        raise ValueError(f"Unknown collision layer: {layer!r}")
    model.plant.SetPositions(model.plant_context, q)
    clearance = checker.CalcContextRobotClearance(
        checker_context,
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


def configuration_clearance_metrics(
    model: PregraspPlanningModel,
    q: np.ndarray,
    influence_distance: float,
) -> dict:
    """Measure both collision layers at one full-plant configuration."""
    if influence_distance <= 0.0:
        raise ValueError("influence_distance must be positive")
    nonpenetration_records = robot_clearance_records(
        model,
        q,
        influence_distance,
        layer="nonpenetration",
    )
    safety_records = robot_clearance_records(
        model,
        q,
        influence_distance,
        layer="safety",
    )
    nonpenetration_nearest = (
        nonpenetration_records[0] if nonpenetration_records else None
    )
    safety_nearest = safety_records[0] if safety_records else None
    return {
        "minimum_nonpenetration_distance": (
            nonpenetration_nearest["distance_m"]
            if nonpenetration_nearest is not None
            else influence_distance
        ),
        "minimum_nonpenetration_distance_is_lower_bound": (
            nonpenetration_nearest is None
        ),
        "nearest_nonpenetration_pair": nonpenetration_nearest,
        "minimum_safety_clearance": (
            safety_nearest["distance_m"]
            if safety_nearest is not None
            else influence_distance
        ),
        "minimum_safety_clearance_is_lower_bound": safety_nearest is None,
        "nearest_safety_pair": safety_nearest,
    }


def _minimum_clearance_record(
    model: PregraspPlanningModel,
    q: np.ndarray,
    influence_distance: float,
    layer: str,
) -> tuple[float, dict | None]:
    """Return one layer's minimum without expensive geometry-pair detail."""
    if layer == "nonpenetration":
        checker = model.nonpenetration_checker
        checker_context = model.nonpenetration_checker_context
    elif layer == "safety":
        checker = model.safety_checker
        checker_context = model.safety_checker_context
    else:
        raise ValueError(f"Unknown collision layer: {layer!r}")
    clearance = checker.CalcContextRobotClearance(
        checker_context,
        q,
        influence_distance,
    )
    distances = clearance.distances()
    if len(distances) == 0:
        return influence_distance, None
    index = int(np.argmin(distances))
    robot_body = model.plant.get_body(clearance.robot_indices()[index])
    other_body = model.plant.get_body(clearance.other_indices()[index])
    return float(distances[index]), {
        "distance_m": float(distances[index]),
        "category": _pair_category(model, robot_body, other_body),
        "collision_type": str(clearance.collision_types()[index]),
        "body_a": qualified_body_name(model.plant, robot_body),
        "body_b": qualified_body_name(model.plant, other_body),
    }


def edge_clearance_metrics(
    model: PregraspPlanningModel,
    q_start: np.ndarray,
    q_end: np.ndarray,
    influence_distance: float,
    max_joint_step: float = 0.002,
) -> dict:
    """Numerically validate both layers along a joint-space line segment.

    The edge is sampled such that no position changes by more than
    ``max_joint_step`` between samples. This is a high-density numerical
    validation, not a mathematical proof over the continuous segment.
    """
    if max_joint_step <= 0.0:
        raise ValueError("max_joint_step must be positive")
    q_start = np.asarray(q_start, dtype=float)
    q_end = np.asarray(q_end, dtype=float)
    if q_start.shape != q_end.shape:
        raise ValueError("q_start and q_end must have matching shapes")
    sample_count = max(
        2,
        int(np.ceil(np.max(np.abs(q_end - q_start)) / max_joint_step)) + 1,
    )
    minimum_nonpenetration = float("inf")
    minimum_safety = float("inf")
    nearest_nonpenetration = None
    nearest_safety = None
    minimum_nonpenetration_alpha = None
    minimum_safety_alpha = None
    for alpha in np.linspace(0.0, 1.0, sample_count):
        q = q_start + alpha * (q_end - q_start)
        nonpenetration_distance, nonpenetration_pair = (
            _minimum_clearance_record(
                model,
                q,
                influence_distance,
                "nonpenetration",
            )
        )
        safety_distance, safety_pair = _minimum_clearance_record(
            model,
            q,
            influence_distance,
            "safety",
        )
        if nonpenetration_distance < minimum_nonpenetration:
            minimum_nonpenetration = nonpenetration_distance
            nearest_nonpenetration = nonpenetration_pair
            minimum_nonpenetration_alpha = float(alpha)
        if safety_distance < minimum_safety:
            minimum_safety = safety_distance
            nearest_safety = safety_pair
            minimum_safety_alpha = float(alpha)
    return {
        "minimum_nonpenetration_distance": minimum_nonpenetration,
        "minimum_safety_clearance": minimum_safety,
        "nearest_nonpenetration_pair": nearest_nonpenetration,
        "nearest_safety_pair": nearest_safety,
        "minimum_nonpenetration_alpha": minimum_nonpenetration_alpha,
        "minimum_safety_alpha": minimum_safety_alpha,
        "sample_count": sample_count,
        "maximum_joint_sample_step_rad": max_joint_step,
        "validation_kind": "high_density_numerical_joint_edge_sampling",
    }
