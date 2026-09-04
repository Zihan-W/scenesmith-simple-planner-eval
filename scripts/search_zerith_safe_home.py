#!/usr/bin/env python3
"""Search a collision-clear Zerith left-arm simulation home posture."""

import argparse
import json
import sys

from pathlib import Path

import numpy as np

from pydrake.all import InverseKinematics, Solve
from pydrake.multibody.inverse_kinematics import (
    MinimumDistanceLowerBoundConstraint,
)

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_grasp_geometry import (
    PICK_RAIL_POSITION_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)
from src.zerith_pregrasp_collision import (
    apply_safety_clearance_filters,
    build_pregrasp_planning_model,
    configuration_clearance_metrics,
)

EVAL_PACKAGE_XML = (
    REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"
)
ZERITH_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"
Q_ZERO = np.zeros(7)


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Find q_safe_home near the all-zero arm reference without "
            "changing the robot base or scene objects."
        )
    )
    parser.add_argument("scene_dmd", type=Path)
    parser.add_argument("--scene-package-xml", type=Path, required=True)
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
    parser.add_argument(
        "--rail-position",
        type=float,
        default=PICK_RAIL_POSITION_METERS,
    )
    parser.add_argument(
        "--objective-clearance",
        type=float,
        default=0.007,
        help="Optimization clearance in meters (default: 0.007).",
    )
    parser.add_argument(
        "--acceptance-clearance",
        type=float,
        default=0.005,
        help="Independent acceptance threshold in meters (default: 0.005).",
    )
    parser.add_argument(
        "--influence-distance-offset",
        type=float,
        default=0.01,
    )
    parser.add_argument("--num-random-seeds", type=int, default=24)
    parser.add_argument("--random-seed", type=int, default=4)
    parser.add_argument(
        "--seed-standard-deviation",
        type=float,
        default=0.35,
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Defaults to safe_home.json beside scene_dmd.",
    )
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    """Validate safe-home search arguments."""
    if args.acceptance_clearance <= 0.0:
        raise ValueError("acceptance-clearance must be positive")
    if args.objective_clearance < args.acceptance_clearance:
        raise ValueError(
            "objective-clearance must be at least acceptance-clearance"
        )
    if args.influence_distance_offset <= 0.0:
        raise ValueError("influence-distance-offset must be positive")
    if args.num_random_seeds < 0:
        raise ValueError("num-random-seeds must be nonnegative")
    if args.seed_standard_deviation <= 0.0:
        raise ValueError("seed-standard-deviation must be positive")


def _seeds(
    lower: np.ndarray,
    upper: np.ndarray,
    count: int,
    standard_deviation: float,
    random_seed: int,
) -> list[tuple[str, np.ndarray]]:
    """Return the zero reference and deterministic nearby random seeds."""
    seeds = [("q_zero", Q_ZERO.copy())]
    rng = np.random.default_rng(random_seed)
    for index in range(count):
        sample = rng.normal(0.0, standard_deviation, size=7)
        seeds.append(
            (
                f"near_zero_{index:02d}",
                np.clip(sample, lower, upper),
            )
        )
    return seeds


def _solve_seed(
    *,
    model,
    q_zero_scene: np.ndarray,
    q_seed: np.ndarray,
    objective_clearance: float,
    influence_distance_offset: float,
):
    """Solve one minimum-displacement collision-clear home problem."""
    model.plant.SetPositions(model.plant_context, q_zero_scene)
    ik = InverseKinematics(
        model.plant,
        model.plant_context,
        with_joint_limits=True,
    )
    q = ik.q()
    program = ik.prog()
    nonpenetration_constraint = MinimumDistanceLowerBoundConstraint(
        collision_checker=model.nonpenetration_checker,
        collision_checker_context=model.nonpenetration_checker_context,
        bound=0.0,
        influence_distance_offset=influence_distance_offset,
    )
    safety_constraint = MinimumDistanceLowerBoundConstraint(
        collision_checker=model.safety_checker,
        collision_checker_context=model.safety_checker_context,
        bound=objective_clearance,
        influence_distance_offset=influence_distance_offset,
    )
    program.AddConstraint(nonpenetration_constraint, q)
    program.AddConstraint(safety_constraint, q)
    fixed_indices = np.setdiff1d(
        np.arange(model.plant.num_positions()),
        model.arm_indices,
    )
    program.AddBoundingBoxConstraint(
        q_zero_scene[fixed_indices],
        q_zero_scene[fixed_indices],
        q[fixed_indices],
    )
    program.AddQuadraticErrorCost(
        np.eye(7),
        Q_ZERO,
        q[model.arm_indices],
    )
    initial = q_zero_scene.copy()
    initial[model.arm_indices] = q_seed
    program.SetInitialGuess(q, initial)
    result = Solve(program)
    return result, q


def main() -> None:
    """Search, rank, and save q_safe_home candidates."""
    args = _parse_args()
    _validate_args(args)
    scene_dmd = args.scene_dmd.resolve()
    model = build_pregrasp_planning_model(
        scene_dmd=scene_dmd,
        scene_package_xml=args.scene_package_xml.resolve(),
        eval_package_xml=args.eval_package_xml.resolve(),
        robot_model_dir=args.robot_model_dir.resolve(),
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
        rail_position=args.rail_position,
    )
    planning_filters = apply_safety_clearance_filters(
        model,
        influence_distance=(
            args.objective_clearance + args.influence_distance_offset
        ),
    )
    q_zero_scene = model.q_scene.copy()
    q_zero_scene[model.arm_indices] = Q_ZERO
    diagnostic_distance = max(
        0.05,
        args.objective_clearance + args.influence_distance_offset,
    )
    q_zero_metrics = configuration_clearance_metrics(
        model,
        q_zero_scene,
        diagnostic_distance,
    )

    attempts = []
    candidates = []
    for seed_name, q_seed in _seeds(
        model.arm_lower_limits,
        model.arm_upper_limits,
        args.num_random_seeds,
        args.seed_standard_deviation,
        args.random_seed,
    ):
        result, q_variables = _solve_seed(
            model=model,
            q_zero_scene=q_zero_scene,
            q_seed=q_seed,
            objective_clearance=args.objective_clearance,
            influence_distance_offset=args.influence_distance_offset,
        )
        q_result = result.GetSolution(q_variables)
        q_left = q_result[model.arm_indices]
        clearance_metrics = configuration_clearance_metrics(
            model,
            q_result,
            diagnostic_distance,
        )
        distance_from_zero = float(np.linalg.norm(q_left))
        joint_margins = np.minimum(
            q_left - model.arm_lower_limits,
            model.arm_upper_limits - q_left,
        )
        independently_valid = bool(
            result.is_success()
            and model.nonpenetration_checker.CheckContextConfigCollisionFree(
                model.nonpenetration_checker_context,
                q_result,
            )
            and clearance_metrics["minimum_nonpenetration_distance"] >= 0.0
            and clearance_metrics["minimum_safety_clearance"]
            >= args.acceptance_clearance
        )
        attempt = {
            "seed": seed_name,
            "solver_result": str(result.get_solution_result()),
            "solver_success": result.is_success(),
            "independently_valid": independently_valid,
            "q_left": q_left.tolist(),
            "distance_from_q_zero": distance_from_zero,
            "minimum_nonpenetration_distance": clearance_metrics[
                "minimum_nonpenetration_distance"
            ],
            "minimum_safety_clearance": clearance_metrics[
                "minimum_safety_clearance"
            ],
            "minimum_joint_limit_margin": float(np.min(joint_margins)),
            "nearest_nonpenetration_pair": clearance_metrics[
                "nearest_nonpenetration_pair"
            ],
            "nearest_safety_pair": clearance_metrics[
                "nearest_safety_pair"
            ],
        }
        attempts.append(attempt)
        print(
            f"{seed_name}: {attempt['solver_result']}, "
            f"distance_from_zero={distance_from_zero:.6f}, "
            "nonpenetration="
            f"{attempt['minimum_nonpenetration_distance']:.6f}, "
            f"safety={attempt['minimum_safety_clearance']:.6f}, "
            f"valid={independently_valid}"
        )
        if independently_valid:
            candidates.append(attempt)

    candidates.sort(
        key=lambda item: (
            item["distance_from_q_zero"],
            -item["minimum_safety_clearance"],
            -item["minimum_joint_limit_margin"],
        )
    )
    selected = candidates[0] if candidates else None
    output_path = (
        args.output_json.resolve()
        if args.output_json is not None
        else scene_dmd.parent / "safe_home.json"
    )
    output = {
        "search_succeeded": selected is not None,
        "calibration_status": (
            "fixed_rail_collision_regression_home"
        ),
        "daogui_joint_position_m": float(
            q_zero_scene[
                model.plant.GetJointByName(
                    "daogui_joint",
                    model.zerith,
                ).position_start()
            ]
        ),
        "q_zero": Q_ZERO.tolist(),
        "q_safe_home": selected["q_left"] if selected else None,
        "objective_clearance_m": args.objective_clearance,
        "acceptance_clearance_m": args.acceptance_clearance,
        "robot_base_xyz_m": ROBOT_BASE_XYZ_METERS.tolist(),
        "robot_base_yaw_deg": ROBOT_BASE_YAW_DEG,
        "distance_unit": "meters",
        "q_zero_clearance_metrics": q_zero_metrics,
        "planning_collision_filters": [
            {
                "body_a": item.body_a,
                "body_b": item.body_b,
                "reason": item.reason,
            }
            for item in planning_filters
        ],
        "candidate_count": len(candidates),
        "selected": selected,
        "attempts": attempts,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    if selected is None:
        print(f"Diagnostics: {output_path}")
        raise RuntimeError("No valid q_safe_home candidate was found")
    print(f"q_safe_home: {selected['q_left']}")
    print(f"Diagnostics: {output_path}")


if __name__ == "__main__":
    main()
