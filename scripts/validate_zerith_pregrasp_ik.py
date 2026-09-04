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
    PREGRASP_DISTANCE_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)
from src.zerith_pregrasp_collision import (
    apply_pregrasp_planning_filters,
    build_pregrasp_planning_model,
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
        "--pregrasp-distance",
        type=float,
        default=PREGRASP_DISTANCE_METERS,
    )
    parser.add_argument("--position-tolerance", type=float, default=0.015)
    parser.add_argument(
        "--orientation-tolerance-deg",
        type=float,
        default=8.0,
    )
    parser.add_argument(
        "--minimum-distance",
        type=float,
        default=0.002,
        help="Required robot clearance in meters (default: 0.002).",
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
    )
    parser.add_argument(
        "--q-current",
        type=float,
        nargs=7,
        help="Current left-arm posture; defaults to q_home.",
    )
    parser.add_argument("--num-random-seeds", type=int, default=4)
    parser.add_argument("--random-seed", type=int, default=4)
    parser.add_argument(
        "--exhaustive",
        action="store_true",
        help="Evaluate every candidate/seed pair after finding a solution.",
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
    if args.orientation_tolerance_deg <= 0.0:
        raise ValueError("orientation-tolerance-deg must be positive")
    if args.minimum_distance < 0.0:
        raise ValueError("minimum-distance must be nonnegative")
    if args.influence_distance_offset <= 0.0:
        raise ValueError("influence-distance-offset must be positive")
    if args.num_random_seeds < 0:
        raise ValueError("num-random-seeds must be nonnegative")
    if not args.approach_tilt_deg or not args.yaw_offset_deg:
        raise ValueError("At least one approach tilt and yaw offset is required")
    for tilt in args.approach_tilt_deg:
        if tilt < 0.0 or tilt > 90.0:
            raise ValueError("approach tilts must be within [0, 90] degrees")


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
    q_home: np.ndarray,
    q_current: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    num_random_seeds: int,
    random_seed: int,
) -> list[tuple[str, np.ndarray]]:
    """Construct deterministic home, current, and random arm seeds."""
    guesses = [("q_home", q_home)]
    if not np.allclose(q_current, q_home, atol=1e-12):
        guesses.append(("q_current", q_current))
    rng = np.random.default_rng(random_seed)
    for index in range(num_random_seeds):
        guesses.append((f"random_{index:02d}", rng.uniform(lower, upper)))
    return guesses


def _solve_candidate(
    *,
    plant,
    context,
    collision_checker,
    collision_checker_context,
    grasp_frame,
    desired_pose: RigidTransform,
    q_scene: np.ndarray,
    arm_indices: np.ndarray,
    q_home: np.ndarray,
    q_seed: np.ndarray,
    position_tolerance: float,
    orientation_tolerance_rad: float,
    minimum_distance: float,
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
    collision_constraint = MinimumDistanceLowerBoundConstraint(
        collision_checker=collision_checker,
        collision_checker_context=collision_checker_context,
        bound=minimum_distance,
        influence_distance_offset=influence_distance_offset,
    )

    program = ik.prog()
    q = ik.q()
    program.AddConstraint(collision_constraint, q)
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
        q_home,
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
    minimum_distance: float,
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
    clearances = robot_clearance_records(
        model,
        q,
        diagnostic_distance,
    )
    nearest = clearances[0] if clearances else None
    minimum_measured_distance = (
        nearest["distance_m"] if nearest is not None else diagnostic_distance
    )
    q_left = q[model.arm_indices]
    joint_limit_margins = np.minimum(
        q_left - model.arm_lower_limits,
        model.arm_upper_limits - q_left,
    )
    collision_free = model.collision_checker.CheckContextConfigCollisionFree(
        model.collision_checker_context,
        q,
    )
    return {
        "configuration_finite": True,
        "minimum_robot_distance_m": minimum_measured_distance,
        "minimum_robot_distance_is_lower_bound": nearest is None,
        "nearest_robot_pair": nearest,
        "position_error_norm_m": float(np.linalg.norm(position_error)),
        "max_absolute_position_error_m": float(
            np.max(np.abs(position_error))
        ),
        "orientation_error_deg": float(np.rad2deg(orientation_error)),
        "minimum_joint_limit_margin": float(np.min(joint_limit_margins)),
        "joint_limit_margins": joint_limit_margins.tolist(),
        "collision_free": bool(collision_free),
        "minimum_distance_satisfied": bool(
            minimum_measured_distance >= minimum_distance - 1e-5
        ),
    }, actual_pose


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
    )
    plant = model.plant
    plant_context = model.plant_context
    grasp_frame = model.grasp_frame
    arm_indices = model.arm_indices
    lower_limits = model.arm_lower_limits
    upper_limits = model.arm_upper_limits
    q_scene = model.q_scene.copy()
    q_home = np.asarray(args.q_home, dtype=float)
    q_current = (
        np.asarray(args.q_current, dtype=float)
        if args.q_current is not None
        else q_home.copy()
    )
    for name, values in (("q_home", q_home), ("q_current", q_current)):
        if np.any(values < lower_limits) or np.any(values > upper_limits):
            raise ValueError(f"{name} violates left-arm joint limits")
    q_scene[arm_indices] = q_current
    plant.SetPositions(plant_context, q_scene)
    q_home_scene = q_scene.copy()
    q_home_scene[arm_indices] = q_home
    raw_q_home_distances = raw_scene_distance_records(
        model,
        q_home_scene,
        args.minimum_distance,
    )
    unfiltered_q_home_robot_distances = robot_clearance_records(
        model,
        q_home_scene,
        args.minimum_distance,
    )
    planning_filters = apply_pregrasp_planning_filters(model)
    filtered_q_home_robot_distances = robot_clearance_records(
        model,
        q_home_scene,
        args.minimum_distance,
    )
    print(
        "q_home distances below minimum: "
        f"scene={len(raw_q_home_distances)}, "
        f"robot_before_filters={len(unfiltered_q_home_robot_distances)}, "
        f"robot_after_filters={len(filtered_q_home_robot_distances)}"
    )

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
        q_home,
        q_current,
        lower_limits,
        upper_limits,
        args.num_random_seeds,
        args.random_seed,
    )

    attempts = []
    solution = None
    orientation_tolerance_rad = np.deg2rad(
        args.orientation_tolerance_deg
    )
    validation_distance = (
        args.minimum_distance + args.influence_distance_offset
    )
    for candidate in candidates:
        for seed_name, q_seed in seeds:
            result, q_variables = _solve_candidate(
                plant=plant,
                context=plant_context,
                collision_checker=model.collision_checker,
                collision_checker_context=(
                    model.collision_checker_context
                ),
                grasp_frame=grasp_frame,
                desired_pose=candidate["pose"],
                q_scene=q_scene,
                arm_indices=arm_indices,
                q_home=q_home,
                q_seed=q_seed,
                position_tolerance=args.position_tolerance,
                orientation_tolerance_rad=orientation_tolerance_rad,
                minimum_distance=args.minimum_distance,
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
                minimum_distance=args.minimum_distance,
                diagnostic_distance=max(validation_distance, 0.05),
            )
            attempt.update(diagnostics)
            attempts.append(attempt)
            if not result.is_success():
                nearest = attempt.get("nearest_robot_pair")
                nearest_text = (
                    f"{nearest['body_a']} <-> {nearest['body_b']}"
                    if nearest is not None
                    else "none within diagnostic distance"
                )
                print(
                    f"{candidate['name']} / {seed_name}: "
                    f"{attempt['solver_result']}, "
                    f"min_robot_distance="
                    f"{attempt.get('minimum_robot_distance_m')}, "
                    f"nearest={nearest_text}, "
                    f"position_error="
                    f"{attempt.get('position_error_norm_m')}, "
                    f"joint_margin="
                    f"{attempt.get('minimum_joint_limit_margin')}"
                )
                continue

            q_solution = q_result
            independently_valid = bool(
                diagnostics["collision_free"]
                and diagnostics["minimum_distance_satisfied"]
            )
            attempt["independent_collision_check"] = independently_valid
            print(
                f"{candidate['name']} / {seed_name}: solved, "
                "minimum_distance="
                f"{diagnostics['minimum_robot_distance_m']:.6f} m, "
                f"independent_check={independently_valid}"
            )
            if independently_valid:
                candidate_solution = {
                    "candidate": candidate["name"],
                    "approach_tilt_deg": candidate["tilt_deg"],
                    "yaw_offset_deg": candidate["yaw_offset_deg"],
                    "seed": seed_name,
                    "q_left": q_solution[arm_indices].tolist(),
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
                    "minimum_measured_distance_m": diagnostics[
                        "minimum_robot_distance_m"
                    ],
                    "nearby_robot_clearances": robot_clearance_records(
                        model,
                        q_solution,
                        validation_distance,
                    ),
                }
                if solution is None:
                    solution = candidate_solution
                if not args.exhaustive:
                    break
        if solution is not None and not args.exhaustive:
            break

    output_path = (
        args.output_json.resolve()
        if args.output_json is not None
        else scene_dmd.parent / "pregrasp_ik.json"
    )
    output = {
        "search_succeeded": solution is not None,
        "robot_base_xyz_m": robot_xyz.tolist(),
        "robot_base_yaw_deg": args.robot_yaw_deg,
        "target_model_name": args.target_model_name,
        "target_dimensions_m": dimensions.tolist(),
        "target_collision_center_xyz_m": X_WC.translation().tolist(),
        "position_tolerance_m": args.position_tolerance,
        "orientation_tolerance_deg": args.orientation_tolerance_deg,
        "minimum_distance_m": args.minimum_distance,
        "influence_distance_offset_m": args.influence_distance_offset,
        "non_arm_positions_fixed": int(
            plant.num_positions() - len(arm_indices)
        ),
        "collision_constraint_scope": (
            "SceneGraphCollisionChecker with Zerith as the only robot model "
            "instance"
        ),
        "planning_collision_filters": [
            {
                "body_a": item.body_a,
                "body_b": item.body_b,
                "reason": item.reason,
            }
            for item in planning_filters
        ],
        "q_home_distance_diagnostics": {
            "raw_scene_pairs_below_minimum": raw_q_home_distances,
            "robot_pairs_before_planning_filters": (
                unfiltered_q_home_robot_distances
            ),
            "robot_pairs_after_planning_filters": (
                filtered_q_home_robot_distances
            ),
        },
        "moving_left_arm_body_count": len(
            model.moving_left_arm_body_indices
        ),
        "candidate_count": len(candidates),
        "seed_count": len(seeds),
        "maximum_attempt_count": len(candidates) * len(seeds),
        "attempt_count": len(attempts),
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
        "Independent minimum distance: "
        f"{solution['minimum_measured_distance_m']:.6f} m"
    )
    print(f"Diagnostics: {output_path}")


if __name__ == "__main__":
    main()
