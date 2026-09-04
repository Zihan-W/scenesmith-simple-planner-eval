#!/usr/bin/env python3
"""Search collision-constrained Zerith left-arm PREGRASP poses."""

import argparse
import json
import sys

from pathlib import Path

import numpy as np

from pydrake.all import (
    Box,
    InverseKinematics,
    RigidTransform,
    RotationMatrix,
    Solve,
)
from pydrake.multibody.inverse_kinematics import (
    MinimumDistanceLowerBoundConstraint,
)

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_grasp_geometry import (
    BOX_SIZE_METERS,
    PICK_RAIL_POSITION_METERS,
    PREGRASP_DISTANCE_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)
from src.zerith_pick_workspace import LEFT_WRIST_GRIPPER_BODY_NAMES
from src.zerith_pregrasp_collision import (
    apply_safety_clearance_filters,
    build_pregrasp_planning_model,
    configuration_clearance_metrics,
    edge_clearance_metrics,
    raw_scene_distance_records,
    robot_clearance_records,
)

TARGET_MODEL_NAME = "living_room_box_0"
TARGET_BODY_NAME = "base_link"
EVAL_PACKAGE_XML = (
    REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"
)
ZERITH_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"
DEFAULT_APPROACH_TILTS_DEG = (0.0, 15.0, 30.0, 60.0, 90.0)
DEFAULT_YAW_OFFSETS_DEG = (0.0, -20.0, 20.0, -45.0, 45.0)
PREFERRED_ELBOW_BEND_DEG = 60.0


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Search collision-constrained PREGRASP IK configurations over "
            "multiple approach poses and initial guesses. This performs no "
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
        "--rail-position",
        type=float,
        default=PICK_RAIL_POSITION_METERS,
        help="Fixed daogui_joint position in meters.",
    )
    parser.add_argument(
        "--pregrasp-distance",
        type=float,
        default=PREGRASP_DISTANCE_METERS,
    )
    parser.add_argument("--position-tolerance", type=float, default=0.01)
    parser.add_argument(
        "--position-cost-weight",
        type=float,
        default=10000.0,
        help="Quadratic cost weight for centering within the pose tolerance.",
    )
    parser.add_argument(
        "--orientation-tolerance-deg",
        type=float,
        default=5.0,
    )
    parser.add_argument(
        "--orientation-cost-weight",
        type=float,
        default=100.0,
        help="Cost weight for aligning within the orientation tolerance.",
    )
    parser.add_argument(
        "--safety-objective-clearance",
        type=float,
        default=0.007,
        help="IK safety-layer target in meters (default: 0.007).",
    )
    parser.add_argument(
        "--minimum-target-gap",
        type=float,
        default=0.05,
        help="Minimum collision-surface gap to the target at PREGRASP.",
    )
    parser.add_argument(
        "--maximum-target-gap",
        type=float,
        default=0.10,
        help="Maximum collision-surface gap to the target at PREGRASP.",
    )
    parser.add_argument(
        "--safety-acceptance-clearance",
        type=float,
        default=0.005,
        help="Independent safety-layer threshold (default: 0.005).",
    )
    parser.add_argument(
        "--influence-distance-offset",
        type=float,
        default=0.01,
    )
    parser.add_argument(
        "--approach-tilt-deg",
        type=float,
        nargs="+",
        default=DEFAULT_APPROACH_TILTS_DEG,
        help="Downward approach tilts; 0 is horizontal and 90 is top-down.",
    )
    parser.add_argument(
        "--yaw-offset-deg",
        type=float,
        nargs="+",
        default=DEFAULT_YAW_OFFSETS_DEG,
        help="Yaw offsets around the box vertical axis.",
    )
    parser.add_argument(
        "--q-home",
        type=float,
        nargs=7,
        default=(0.0,) * 7,
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
        help=(
            "Compatibility fallback for q_safe_home when "
            "--safe-home-json is omitted."
        ),
    )
    parser.add_argument(
        "--safe-home-json",
        type=Path,
        help="Output from search_zerith_safe_home.py.",
    )
    parser.add_argument(
        "--q-current",
        type=float,
        nargs=7,
        help="Current left-arm posture; defaults to q_safe_home.",
    )
    parser.add_argument("--num-random-seeds", type=int, default=4)
    parser.add_argument("--random-seed", type=int, default=4)
    parser.add_argument(
        "--exhaustive",
        action="store_true",
        help="Evaluate every candidate/seed pair after finding a solution.",
    )
    parser.add_argument(
        "--edge-sample-step",
        type=float,
        default=0.005,
        help=(
            "Maximum joint change between q_safe_home-to-PREGRASP edge "
            "samples in radians (default: 0.005)."
        ),
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Defaults to pregrasp_ik.json beside scene_dmd.",
    )
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    """Validate scalar and candidate-search arguments."""
    if args.pregrasp_distance <= 0.0:
        raise ValueError("pregrasp-distance must be positive")
    if args.position_tolerance <= 0.0:
        raise ValueError("position-tolerance must be positive")
    if args.position_cost_weight <= 0.0:
        raise ValueError("position-cost-weight must be positive")
    if args.orientation_tolerance_deg <= 0.0:
        raise ValueError("orientation-tolerance-deg must be positive")
    if args.orientation_cost_weight <= 0.0:
        raise ValueError("orientation-cost-weight must be positive")
    if args.safety_acceptance_clearance <= 0.0:
        raise ValueError("safety-acceptance-clearance must be positive")
    if args.minimum_target_gap < 0.0:
        raise ValueError("minimum-target-gap must be nonnegative")
    if args.maximum_target_gap <= args.minimum_target_gap:
        raise ValueError(
            "maximum-target-gap must exceed minimum-target-gap"
        )
    if (
        args.safety_objective_clearance
        < args.safety_acceptance_clearance
    ):
        raise ValueError(
            "safety-objective-clearance must be at least "
            "safety-acceptance-clearance"
        )
    if args.influence_distance_offset <= 0.0:
        raise ValueError("influence-distance-offset must be positive")
    if args.num_random_seeds < 0:
        raise ValueError("num-random-seeds must be nonnegative")
    if args.edge_sample_step <= 0.0:
        raise ValueError("edge-sample-step must be positive")
    if not args.approach_tilt_deg or not args.yaw_offset_deg:
        raise ValueError("At least one approach tilt and yaw offset is required")
    for tilt in args.approach_tilt_deg:
        if tilt < 0.0 or tilt > 90.0:
            raise ValueError("approach tilts must be within [0, 90] degrees")


def _target_collision_pose(plant, context, target_body):
    """Return the tiled target Box bounds and their world-center pose."""
    geometry_ids = plant.GetCollisionGeometriesForBody(target_body)
    if not geometry_ids:
        raise ValueError("Pick target has no collision geometry")
    query = plant.get_geometry_query_input_port().Eval(context)
    inspector = query.inspector()
    body_points = []
    for geometry_id in geometry_ids:
        shape = inspector.GetShape(geometry_id)
        if not isinstance(shape, Box):
            raise TypeError(
                f"Expected target collision Box, got {type(shape)}"
            )
        half_size = 0.5 * np.array(
            [shape.width(), shape.depth(), shape.height()]
        )
        X_BG = inspector.GetPoseInFrame(geometry_id)
        body_points.extend(
            X_BG.multiply(np.array([x, y, z]))
            for x in (-half_size[0], half_size[0])
            for y in (-half_size[1], half_size[1])
            for z in (-half_size[2], half_size[2])
        )
    body_points = np.asarray(body_points)
    bounds_min = body_points.min(axis=0)
    bounds_max = body_points.max(axis=0)
    dimensions = bounds_max - bounds_min
    if not np.allclose(dimensions, BOX_SIZE_METERS, atol=1e-12):
        raise ValueError(
            f"Expected target dimensions {BOX_SIZE_METERS}, got {dimensions}"
        )
    center_body = 0.5 * (bounds_min + bounds_max)
    if not np.allclose(center_body, np.zeros(3), atol=1e-12):
        raise ValueError(
            "Expected tiled target collision geometry centered on base_link"
        )
    X_WB = plant.EvalBodyPoseInWorld(context, target_body)
    return dimensions, X_WB, X_WB @ RigidTransform(center_body)


def _minimum_wrist_gripper_target_distance(
    plant,
    context,
    zerith,
    target_body,
) -> float:
    """Return the unfiltered collision-surface gap to the pick target."""
    query = plant.get_geometry_query_input_port().Eval(context)
    target_geometry_ids = plant.GetCollisionGeometriesForBody(target_body)
    distances = []
    for body_name in LEFT_WRIST_GRIPPER_BODY_NAMES:
        body = plant.GetBodyByName(body_name, zerith)
        for robot_geometry_id in plant.GetCollisionGeometriesForBody(body):
            for target_geometry_id in target_geometry_ids:
                distances.append(
                    query.ComputeSignedDistancePairClosestPoints(
                        robot_geometry_id,
                        target_geometry_id,
                    ).distance
                )
    if not distances:
        raise ValueError(
            "Wrist, gripper, and target must provide collision geometry"
        )
    return float(min(distances))


def _candidate_pregrasp_poses(
    X_WB: RigidTransform,
    target_center: np.ndarray,
    robot_xyz: np.ndarray,
    pregrasp_distance: float,
    tilts_deg: list[float],
    yaw_offsets_deg: list[float],
) -> list[dict]:
    """Construct side, oblique, and top-down PREGRASP candidates."""
    world_up = np.array([0.0, 0.0, 1.0])
    base_closing = X_WB.rotation().matrix()[:, 1].copy()
    base_closing[2] = 0.0
    norm = np.linalg.norm(base_closing)
    if norm < 1e-6:
        raise ValueError("Target box Y axis is vertical; grasp is undefined")
    base_closing /= norm
    base_approach = np.cross(base_closing, world_up)
    base_approach /= np.linalg.norm(base_approach)
    toward_target = target_center - robot_xyz
    toward_target[2] = 0.0
    if base_approach @ toward_target < 0.0:
        base_approach = -base_approach
        base_closing = -base_closing

    candidates = []
    for tilt_deg in tilts_deg:
        tilt = np.deg2rad(tilt_deg)
        for yaw_offset_deg in yaw_offsets_deg:
            yaw = np.deg2rad(yaw_offset_deg)
            yaw_rotation = np.array(
                [
                    [np.cos(yaw), -np.sin(yaw), 0.0],
                    [np.sin(yaw), np.cos(yaw), 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )
            horizontal_approach = yaw_rotation @ base_approach
            closing_axis = yaw_rotation @ base_closing
            approach_axis = (
                np.cos(tilt) * horizontal_approach
                - np.sin(tilt) * world_up
            )
            up_axis = np.cross(approach_axis, closing_axis)
            up_axis /= np.linalg.norm(up_axis)
            rotation = RotationMatrix(
                np.column_stack((approach_axis, closing_axis, up_axis))
            )
            position = target_center - approach_axis * pregrasp_distance
            candidates.append(
                {
                    "name": (
                        f"tilt_{tilt_deg:g}_yaw_{yaw_offset_deg:+g}"
                    ),
                    "tilt_deg": tilt_deg,
                    "yaw_offset_deg": yaw_offset_deg,
                    "pose": RigidTransform(rotation, position),
                }
            )
    return candidates


def _initial_guesses(
    q_safe_home: np.ndarray,
    q_current: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    num_random_seeds: int,
    random_seed: int,
) -> list[tuple[str, np.ndarray]]:
    """Construct deterministic home, current, and random arm seeds."""
    guesses = [("q_safe_home", q_safe_home)]
    if not np.allclose(q_current, q_safe_home, atol=1e-12):
        guesses.append(("q_current", q_current))
    rng = np.random.default_rng(random_seed)
    for index in range(num_random_seeds):
        guesses.append((f"random_{index:02d}", rng.uniform(lower, upper)))
    return guesses


def _solve_candidate(
    *,
    plant,
    context,
    nonpenetration_checker,
    nonpenetration_checker_context,
    safety_checker,
    safety_checker_context,
    grasp_frame,
    desired_pose: RigidTransform,
    q_scene: np.ndarray,
    arm_indices: np.ndarray,
    q_safe_home: np.ndarray,
    q_seed: np.ndarray,
    position_tolerance: float,
    position_cost_weight: float,
    orientation_tolerance_rad: float,
    orientation_cost_weight: float,
    safety_objective_clearance: float,
    influence_distance_offset: float,
):
    """Solve one collision-constrained IK candidate and initial guess."""
    plant.SetPositions(context, q_scene)
    ik = InverseKinematics(plant, context, with_joint_limits=True)
    desired_position = desired_pose.translation()
    ik.AddPositionConstraint(
        frameB=grasp_frame,
        p_BQ=np.zeros((3, 1)),
        frameA=plant.world_frame(),
        p_AQ_lower=(desired_position - position_tolerance).reshape(3, 1),
        p_AQ_upper=(desired_position + position_tolerance).reshape(3, 1),
    )
    ik.AddOrientationConstraint(
        frameAbar=plant.world_frame(),
        R_AbarA=desired_pose.rotation(),
        frameBbar=grasp_frame,
        R_BbarB=RotationMatrix(),
        theta_bound=orientation_tolerance_rad,
    )
    ik.AddPositionCost(
        frameA=plant.world_frame(),
        p_AP=desired_position,
        frameB=grasp_frame,
        p_BQ=np.zeros(3),
        C=np.asfortranarray(position_cost_weight * np.eye(3)),
    )
    ik.AddOrientationCost(
        frameAbar=plant.world_frame(),
        R_AbarA=desired_pose.rotation(),
        frameBbar=grasp_frame,
        R_BbarB=RotationMatrix(),
        c=orientation_cost_weight,
    )
    nonpenetration_constraint = MinimumDistanceLowerBoundConstraint(
        collision_checker=nonpenetration_checker,
        collision_checker_context=nonpenetration_checker_context,
        bound=0.0,
        influence_distance_offset=influence_distance_offset,
    )
    safety_constraint = MinimumDistanceLowerBoundConstraint(
        collision_checker=safety_checker,
        collision_checker_context=safety_checker_context,
        bound=safety_objective_clearance,
        influence_distance_offset=influence_distance_offset,
    )

    program = ik.prog()
    q = ik.q()
    program.AddConstraint(nonpenetration_constraint, q)
    program.AddConstraint(safety_constraint, q)
    fixed_indices = np.setdiff1d(
        np.arange(plant.num_positions()),
        arm_indices,
    )
    program.AddBoundingBoxConstraint(
        q_scene[fixed_indices],
        q_scene[fixed_indices],
        q[fixed_indices],
    )
    program.AddQuadraticErrorCost(
        np.eye(len(arm_indices)),
        q_safe_home,
        q[arm_indices],
    )
    q_initial = q_scene.copy()
    q_initial[arm_indices] = q_seed
    program.SetInitialGuess(q, q_initial)
    result = Solve(program)
    return result, q


def _configuration_diagnostics(
    *,
    model,
    q: np.ndarray,
    desired_pose: RigidTransform,
    safety_acceptance_clearance: float,
    diagnostic_distance: float,
) -> tuple[dict, RigidTransform | None]:
    """Measure pose, limits, and nearest robot collision at one result."""
    if not np.all(np.isfinite(q)):
        return {"configuration_finite": False}, None

    plant = model.plant
    plant.SetPositions(model.plant_context, q)
    actual_pose = plant.CalcRelativeTransform(
        model.plant_context,
        plant.world_frame(),
        model.grasp_frame,
    )
    position_error = actual_pose.translation() - desired_pose.translation()
    orientation_error = (
        desired_pose.rotation().inverse() @ actual_pose.rotation()
    ).ToAngleAxis().angle()
    clearance_metrics = configuration_clearance_metrics(
        model,
        q,
        diagnostic_distance,
    )
    q_left = q[model.arm_indices]
    joint_limit_margins = np.minimum(
        q_left - model.arm_lower_limits,
        model.arm_upper_limits - q_left,
    )
    collision_free = (
        model.nonpenetration_checker.CheckContextConfigCollisionFree(
            model.nonpenetration_checker_context,
            q,
        )
    )
    return {
        "configuration_finite": True,
        **clearance_metrics,
        "position_error_norm_m": float(np.linalg.norm(position_error)),
        "max_absolute_position_error_m": float(
            np.max(np.abs(position_error))
        ),
        "orientation_error_deg": float(np.rad2deg(orientation_error)),
        "minimum_joint_limit_margin": float(np.min(joint_limit_margins)),
        "joint_limit_margins": joint_limit_margins.tolist(),
        "nonpenetration_collision_free": bool(collision_free),
        "nonpenetration_satisfied": bool(
            clearance_metrics["minimum_nonpenetration_distance"] >= 0.0
        ),
        "safety_acceptance_satisfied": bool(
            clearance_metrics["minimum_safety_clearance"]
            >= safety_acceptance_clearance
        ),
    }, actual_pose


def _load_q_safe_home(args: argparse.Namespace) -> np.ndarray:
    """Load q_safe_home, preserving --q-home as a compatibility fallback."""
    if args.safe_home_json is None:
        return np.asarray(args.q_home, dtype=float)
    payload = json.loads(
        args.safe_home_json.resolve().read_text(encoding="utf-8")
    )
    q_safe_home = np.asarray(payload["q_safe_home"], dtype=float)
    if q_safe_home.shape != (7,) or not np.all(np.isfinite(q_safe_home)):
        raise ValueError("safe-home-json contains an invalid q_safe_home")
    return q_safe_home


def main() -> None:
    """Search collision-constrained PREGRASP IK and write diagnostics."""
    args = _parse_args()
    _validate_args(args)
    scene_dmd = args.scene_dmd.resolve()
    scene_package_xml = args.scene_package_xml.resolve()
    eval_package_xml = args.eval_package_xml.resolve()
    robot_model_dir = args.robot_model_dir.resolve()
    robot_xyz = np.asarray(args.robot_xyz, dtype=float)

    model = build_pregrasp_planning_model(
        scene_dmd=scene_dmd,
        scene_package_xml=scene_package_xml,
        eval_package_xml=eval_package_xml,
        robot_model_dir=robot_model_dir,
        robot_xyz=robot_xyz,
        robot_yaw_deg=args.robot_yaw_deg,
        rail_position=args.rail_position,
    )
    plant = model.plant
    plant_context = model.plant_context
    grasp_frame = model.grasp_frame
    arm_indices = model.arm_indices
    lower_limits = model.arm_lower_limits
    upper_limits = model.arm_upper_limits
    q_scene = model.q_scene.copy()
    q_safe_home = _load_q_safe_home(args)
    q_current = (
        np.asarray(args.q_current, dtype=float)
        if args.q_current is not None
        else q_safe_home.copy()
    )
    for name, values in (
        ("q_safe_home", q_safe_home),
        ("q_current", q_current),
    ):
        if np.any(values < lower_limits) or np.any(values > upper_limits):
            raise ValueError(f"{name} violates left-arm joint limits")
    q_scene[arm_indices] = q_current
    plant.SetPositions(plant_context, q_scene)
    q_safe_home_scene = q_scene.copy()
    q_safe_home_scene[arm_indices] = q_safe_home
    raw_q_safe_home_distances = raw_scene_distance_records(
        model,
        q_safe_home_scene,
        args.safety_objective_clearance,
    )
    unfiltered_q_safe_home_safety_distances = robot_clearance_records(
        model,
        q_safe_home_scene,
        args.safety_objective_clearance,
        layer="safety",
    )
    planning_filters = apply_safety_clearance_filters(
        model,
        influence_distance=(
            args.safety_objective_clearance
            + args.influence_distance_offset
        ),
    )
    diagnostic_distance = max(
        0.05,
        args.safety_objective_clearance
        + args.influence_distance_offset,
    )
    q_safe_home_metrics = configuration_clearance_metrics(
        model,
        q_safe_home_scene,
        diagnostic_distance,
    )
    print(
        "q_safe_home clearances: "
        "nonpenetration="
        f"{q_safe_home_metrics['minimum_nonpenetration_distance']:.6f} m, "
        f"safety={q_safe_home_metrics['minimum_safety_clearance']:.6f} m"
    )
    if (
        q_safe_home_metrics["minimum_nonpenetration_distance"] < 0.0
        or q_safe_home_metrics["minimum_safety_clearance"]
        < args.safety_acceptance_clearance
    ):
        raise ValueError("q_safe_home does not satisfy both collision layers")

    target_instance = plant.GetModelInstanceByName(args.target_model_name)
    target_body = plant.GetBodyByName(TARGET_BODY_NAME, target_instance)
    dimensions, X_WB, X_WC = _target_collision_pose(
        plant,
        plant_context,
        target_body,
    )
    candidates = _candidate_pregrasp_poses(
        X_WB,
        X_WC.translation(),
        robot_xyz,
        args.pregrasp_distance,
        args.approach_tilt_deg,
        args.yaw_offset_deg,
    )
    seeds = _initial_guesses(
        q_safe_home,
        q_current,
        lower_limits,
        upper_limits,
        args.num_random_seeds,
        args.random_seed,
    )

    attempts = []
    valid_solutions = []
    orientation_tolerance_rad = np.deg2rad(
        args.orientation_tolerance_deg
    )
    validation_distance = diagnostic_distance
    for candidate in candidates:
        for seed_name, q_seed in seeds:
            result, q_variables = _solve_candidate(
                plant=plant,
                context=plant_context,
                nonpenetration_checker=model.nonpenetration_checker,
                nonpenetration_checker_context=(
                    model.nonpenetration_checker_context
                ),
                safety_checker=model.safety_checker,
                safety_checker_context=model.safety_checker_context,
                grasp_frame=grasp_frame,
                desired_pose=candidate["pose"],
                q_scene=q_scene,
                arm_indices=arm_indices,
                q_safe_home=q_safe_home,
                q_seed=q_seed,
                position_tolerance=args.position_tolerance,
                position_cost_weight=args.position_cost_weight,
                orientation_tolerance_rad=orientation_tolerance_rad,
                orientation_cost_weight=args.orientation_cost_weight,
                safety_objective_clearance=(
                    args.safety_objective_clearance
                ),
                influence_distance_offset=args.influence_distance_offset,
            )
            attempt = {
                "candidate": candidate["name"],
                "seed": seed_name,
                "solver_result": str(result.get_solution_result()),
                "solver_success": result.is_success(),
            }
            q_result = result.GetSolution(q_variables)
            diagnostics, actual_pose = _configuration_diagnostics(
                model=model,
                q=q_result,
                desired_pose=candidate["pose"],
                safety_acceptance_clearance=(
                    args.safety_acceptance_clearance
                ),
                diagnostic_distance=max(validation_distance, 0.05),
            )
            attempt.update(diagnostics)
            if diagnostics.get("configuration_finite", False):
                target_gap = _minimum_wrist_gripper_target_distance(
                    plant,
                    plant_context,
                    model.zerith,
                    target_body,
                )
                attempt["minimum_wrist_gripper_target_distance_m"] = (
                    target_gap
                )
                attempt["target_gap_satisfied"] = bool(
                    args.minimum_target_gap
                    <= target_gap
                    <= args.maximum_target_gap
                )
            if not result.is_success():
                attempts.append(attempt)
                nearest = attempt.get("nearest_safety_pair")
                nearest_text = (
                    f"{nearest['body_a']} <-> {nearest['body_b']}"
                    if nearest is not None
                    else "none within diagnostic distance"
                )
                print(
                    f"{candidate['name']} / {seed_name}: "
                    f"{attempt['solver_result']}, "
                    "nonpenetration="
                    f"{attempt.get('minimum_nonpenetration_distance')}, "
                    f"safety={attempt.get('minimum_safety_clearance')}, "
                    f"nearest={nearest_text}, "
                    f"position_error="
                    f"{attempt.get('position_error_norm_m')}, "
                    f"joint_margin="
                    f"{attempt.get('minimum_joint_limit_margin')}"
                )
                continue

            q_solution = q_result
            endpoint_valid = bool(
                diagnostics["nonpenetration_collision_free"]
                and diagnostics["nonpenetration_satisfied"]
                and diagnostics["safety_acceptance_satisfied"]
                and attempt["target_gap_satisfied"]
            )
            edge_metrics = edge_clearance_metrics(
                model,
                q_safe_home_scene,
                q_solution,
                validation_distance,
                max_joint_step=args.edge_sample_step,
            )
            edge_valid = bool(
                edge_metrics["minimum_nonpenetration_distance"] >= 0.0
                and edge_metrics["minimum_safety_clearance"]
                >= args.safety_acceptance_clearance
            )
            endpoint_and_direct_edge_valid = endpoint_valid and edge_valid
            attempt["q_safe_home_to_pregrasp_edge"] = edge_metrics
            attempt["endpoint_valid"] = endpoint_valid
            attempt["direct_interpolation_valid"] = edge_valid
            attempt["endpoint_and_direct_edge_valid"] = (
                endpoint_and_direct_edge_valid
            )
            attempts.append(attempt)
            print(
                f"{candidate['name']} / {seed_name}: solved, "
                "endpoint_safety="
                f"{diagnostics['minimum_safety_clearance']:.6f} m, "
                "edge_safety="
                f"{edge_metrics['minimum_safety_clearance']:.6f} m, "
                f"endpoint_valid={endpoint_valid}, "
                f"direct_edge_valid={edge_valid}"
            )
            if endpoint_valid:
                q_left = q_solution[arm_indices]
                joint_spans = upper_limits - lower_limits
                joint_limit_margins = np.minimum(
                    q_left - lower_limits,
                    upper_limits - q_left,
                )
                normalized_joint_limit_margins = (
                    joint_limit_margins / joint_spans
                )
                elbow_bend_deg = float(np.rad2deg(abs(q_left[3])))
                wrist_midpoints = 0.5 * (
                    lower_limits[4:] + upper_limits[4:]
                )
                candidate_solution = {
                    "candidate": candidate["name"],
                    "approach_tilt_deg": candidate["tilt_deg"],
                    "yaw_offset_deg": candidate["yaw_offset_deg"],
                    "seed": seed_name,
                    "q_left": q_left.tolist(),
                    "distance_from_q_safe_home": float(
                        np.linalg.norm(q_left - q_safe_home)
                    ),
                    "minimum_joint_limit_margin": diagnostics[
                        "minimum_joint_limit_margin"
                    ],
                    "minimum_normalized_joint_limit_margin": float(
                        np.min(normalized_joint_limit_margins)
                    ),
                    "elbow_bend_abs_deg": elbow_bend_deg,
                    "preferred_elbow_bend_deg": (
                        PREFERRED_ELBOW_BEND_DEG
                    ),
                    "elbow_preference_error_deg": abs(
                        elbow_bend_deg - PREFERRED_ELBOW_BEND_DEG
                    ),
                    "wrist_deviation_from_midpoint_rad": float(
                        np.linalg.norm(q_left[4:] - wrist_midpoints)
                    ),
                    "desired_pregrasp_pose": {
                        "translation_xyz_m": (
                            candidate["pose"].translation().tolist()
                        ),
                        "rotation_matrix": (
                            candidate["pose"].rotation().matrix().tolist()
                        ),
                    },
                    "actual_pregrasp_pose": {
                        "translation_xyz_m": actual_pose.translation().tolist(),
                        "rotation_matrix": (
                            actual_pose.rotation().matrix().tolist()
                        ),
                    },
                    "position_error_norm_m": diagnostics[
                        "position_error_norm_m"
                    ],
                    "max_absolute_position_error_m": diagnostics[
                        "max_absolute_position_error_m"
                    ],
                    "orientation_error_deg": diagnostics[
                        "orientation_error_deg"
                    ],
                    "grasp_axes_world": {
                        "approach": candidate["pose"]
                        .rotation()
                        .matrix()[:, 0]
                        .tolist(),
                        "closing": candidate["pose"]
                        .rotation()
                        .matrix()[:, 1]
                        .tolist(),
                        "up": candidate["pose"]
                        .rotation()
                        .matrix()[:, 2]
                        .tolist(),
                    },
                    "minimum_wrist_gripper_target_distance_m": (
                        target_gap
                    ),
                    "minimum_nonpenetration_distance": diagnostics[
                        "minimum_nonpenetration_distance"
                    ],
                    "minimum_safety_clearance": diagnostics[
                        "minimum_safety_clearance"
                    ],
                    "q_safe_home_to_pregrasp_edge": edge_metrics,
                    "direct_interpolation_valid": edge_valid,
                    "nearby_nonpenetration_clearances": (
                        robot_clearance_records(
                            model,
                            q_solution,
                            validation_distance,
                            layer="nonpenetration",
                        )
                    ),
                    "nearby_safety_clearances": robot_clearance_records(
                        model,
                        q_solution,
                        validation_distance,
                        layer="safety",
                    ),
                }
                valid_solutions.append(candidate_solution)
                if not args.exhaustive:
                    break
        if valid_solutions and not args.exhaustive:
            break

    valid_solutions.sort(
        key=lambda item: (
            -int(item["direct_interpolation_valid"]),
            item["elbow_preference_error_deg"],
            item["wrist_deviation_from_midpoint_rad"],
            -item["minimum_normalized_joint_limit_margin"],
            -item["q_safe_home_to_pregrasp_edge"][
                "minimum_safety_clearance"
            ],
            -item["minimum_safety_clearance"],
            item["distance_from_q_safe_home"],
        )
    )
    solution = valid_solutions[0] if valid_solutions else None

    output_path = (
        args.output_json.resolve()
        if args.output_json is not None
        else scene_dmd.parent / "pregrasp_ik.json"
    )
    output = {
        "search_succeeded": solution is not None,
        "calibration_status": (
            "fixed_rail_pregrasp_candidate_validated"
        ),
        "daogui_joint_position_m": float(
            q_scene[
                plant.GetJointByName(
                    "daogui_joint",
                    model.zerith,
                ).position_start()
            ]
        ),
        "robot_base_xyz_m": robot_xyz.tolist(),
        "robot_base_yaw_deg": args.robot_yaw_deg,
        "target_model_name": args.target_model_name,
        "target_dimensions_m": dimensions.tolist(),
        "target_collision_center_xyz_m": X_WC.translation().tolist(),
        "position_tolerance_m": args.position_tolerance,
        "position_cost_weight": args.position_cost_weight,
        "orientation_tolerance_deg": args.orientation_tolerance_deg,
        "orientation_cost_weight": args.orientation_cost_weight,
        "minimum_target_gap_m": args.minimum_target_gap,
        "maximum_target_gap_m": args.maximum_target_gap,
        "nonpenetration_bound_m": 0.0,
        "safety_objective_clearance_m": (
            args.safety_objective_clearance
        ),
        "safety_acceptance_clearance_m": (
            args.safety_acceptance_clearance
        ),
        "influence_distance_offset_m": args.influence_distance_offset,
        "edge_sample_step_rad": args.edge_sample_step,
        "solution_ranking_order": [
            "direct_interpolation_valid",
            "elbow_bend_closest_to_60_deg",
            "minimum_wrist_midpoint_deviation",
            "maximum_normalized_joint_limit_margin",
            "maximum_edge_safety_clearance",
            "maximum_endpoint_safety_clearance",
            "minimum_distance_from_q_safe_home",
        ],
        "non_arm_positions_fixed": int(
            plant.num_positions() - len(arm_indices)
        ),
        "collision_constraint_scope": {
            "nonpenetration": (
                "All real Zerith collision candidates retained by the "
                "dynamics model, with no added planning filters"
            ),
            "safety": (
                "Zerith-environment and non-assembly Zerith self-collision "
                "pairs; active-arm-invariant pairs and the explicit "
                "left-shoulder-to-torso whitelist are excluded"
            ),
        },
        "safety_clearance_filters": [
            {
                "body_a": item.body_a,
                "body_b": item.body_b,
                "reason": item.reason,
            }
            for item in planning_filters
        ],
        "q_safe_home": q_safe_home.tolist(),
        "q_safe_home_distance_diagnostics": {
            "clearance_metrics": q_safe_home_metrics,
            "raw_scene_pairs_below_safety_objective": (
                raw_q_safe_home_distances
            ),
            "robot_pairs_before_safety_filters": (
                unfiltered_q_safe_home_safety_distances
            ),
        },
        "moving_left_arm_body_count": len(
            model.moving_left_arm_body_indices
        ),
        "candidate_count": len(candidates),
        "seed_count": len(seeds),
        "maximum_attempt_count": len(candidates) * len(seeds),
        "attempt_count": len(attempts),
        "valid_solution_count": len(valid_solutions),
        "valid_solutions": valid_solutions,
        "exhaustive": args.exhaustive,
        "attempts": attempts,
        "solution": solution,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")

    if solution is None:
        print(f"Diagnostics: {output_path}")
        raise RuntimeError(
            "No collision-constrained PREGRASP solution was found across "
            f"{len(attempts)} candidate/seed attempts"
        )
    print("Collision-constrained PREGRASP found")
    print(f"Candidate: {solution['candidate']}")
    print(f"Seed: {solution['seed']}")
    print(f"q_left: {solution['q_left']}")
    print(
        "Selected clearances: "
        "nonpenetration="
        f"{solution['minimum_nonpenetration_distance']:.6f} m, "
        f"safety={solution['minimum_safety_clearance']:.6f} m, "
        "edge_safety="
        f"{solution['q_safe_home_to_pregrasp_edge']['minimum_safety_clearance']:.6f} m"
    )
    print(f"Diagnostics: {output_path}")


if __name__ == "__main__":
    main()
