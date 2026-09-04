#!/usr/bin/env python3
"""Search static collision-safe left-arm postures across rail heights."""

import argparse
import json
import sys

from pathlib import Path

import numpy as np

from pydrake.all import InverseKinematics, JacobianWrtVariable, Solve
from pydrake.multibody.inverse_kinematics import (
    MinimumDistanceLowerBoundConstraint,
)

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_grasp_geometry import (
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)
from src.zerith_pick_workspace import PickWorkspaceEvaluator
from src.zerith_pregrasp_collision import (
    apply_safety_clearance_filters,
    build_pregrasp_planning_model,
    configuration_clearance_metrics,
)

EVAL_PACKAGE_XML = (
    REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"
)
ZERITH_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"
DEFAULT_RAIL_HEIGHTS_METERS = tuple(np.linspace(0.0, 0.8, 9))


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "At each rail height, jointly solve collision-constrained "
            "left-arm postures and report unweighted naturalness metrics."
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
        "--table-model-name",
        default="living_room_coffee_table_0",
    )
    parser.add_argument(
        "--rail-heights",
        type=float,
        nargs="+",
        default=DEFAULT_RAIL_HEIGHTS_METERS,
    )
    parser.add_argument(
        "--objective-clearance",
        type=float,
        default=0.007,
    )
    parser.add_argument(
        "--acceptance-clearance",
        type=float,
        default=0.005,
    )
    parser.add_argument(
        "--workspace-safety-margin",
        type=float,
        default=0.005,
    )
    parser.add_argument(
        "--influence-distance-offset",
        type=float,
        default=0.01,
    )
    parser.add_argument("--num-random-references", type=int, default=8)
    parser.add_argument("--random-seed", type=int, default=4)
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Defaults to rail_postures.json beside scene_dmd.",
    )
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    """Validate search arguments that do not depend on the loaded model."""
    if not args.rail_heights:
        raise ValueError("rail-heights must not be empty")
    if len(set(args.rail_heights)) != len(args.rail_heights):
        raise ValueError("rail-heights must not contain duplicates")
    if args.objective_clearance < args.acceptance_clearance:
        raise ValueError(
            "objective-clearance must be at least acceptance-clearance"
        )
    if args.acceptance_clearance <= 0.0:
        raise ValueError("acceptance-clearance must be positive")
    if args.workspace_safety_margin <= 0.0:
        raise ValueError("workspace-safety-margin must be positive")
    if args.influence_distance_offset <= 0.0:
        raise ValueError("influence-distance-offset must be positive")
    if args.num_random_references < 0:
        raise ValueError("num-random-references must be nonnegative")


def _posture_references(
    lower: np.ndarray,
    upper: np.ndarray,
    random_count: int,
    random_seed: int,
) -> list[tuple[str, np.ndarray]]:
    """Return transparent deterministic references for constrained solves."""
    midpoint = 0.5 * (lower + upper)
    q_zero = np.clip(np.zeros(7), lower, upper)
    references = [
        ("q_zero", q_zero),
        ("joint_midpoints", midpoint),
    ]
    for elbow_radians in (-0.8, 0.8):
        reference = midpoint.copy()
        reference[3] = np.clip(elbow_radians, lower[3], upper[3])
        reference[4:] = np.clip(0.0, lower[4:], upper[4:])
        references.append(
            (f"bent_elbow_{elbow_radians:+.1f}", reference)
        )

    rng = np.random.default_rng(random_seed)
    span = upper - lower
    for index in range(random_count):
        reference = rng.normal(midpoint, 0.22 * span)
        reference[4:] = rng.normal(0.0, 0.12 * span[4:])
        references.append(
            (f"random_natural_{index:02d}", np.clip(reference, lower, upper))
        )
    return references


def _solve_reference(
    *,
    model,
    rail_height: float,
    q_reference: np.ndarray,
    objective_clearance: float,
    influence_distance_offset: float,
):
    """Solve one fixed-rail, free-left-arm collision-constrained posture."""
    q_nominal = model.q_scene.copy()
    q_nominal[model.rail_index] = rail_height
    q_nominal[model.arm_indices] = q_reference
    model.plant.SetPositions(model.plant_context, q_nominal)
    ik = InverseKinematics(
        model.plant,
        model.plant_context,
        with_joint_limits=True,
    )
    q = ik.q()
    program = ik.prog()
    program.AddConstraint(
        MinimumDistanceLowerBoundConstraint(
            collision_checker=model.nonpenetration_checker,
            collision_checker_context=model.nonpenetration_checker_context,
            bound=0.0,
            influence_distance_offset=influence_distance_offset,
        ),
        q,
    )
    program.AddConstraint(
        MinimumDistanceLowerBoundConstraint(
            collision_checker=model.safety_checker,
            collision_checker_context=model.safety_checker_context,
            bound=objective_clearance,
            influence_distance_offset=influence_distance_offset,
        ),
        q,
    )
    active_indices = np.r_[model.rail_index, model.arm_indices]
    fixed_indices = np.setdiff1d(
        np.arange(model.plant.num_positions()),
        active_indices,
    )
    program.AddBoundingBoxConstraint(
        q_nominal[fixed_indices],
        q_nominal[fixed_indices],
        q[fixed_indices],
    )
    program.AddBoundingBoxConstraint(
        [rail_height],
        [rail_height],
        q[[model.rail_index]],
    )
    joint_span = model.arm_upper_limits - model.arm_lower_limits
    program.AddQuadraticErrorCost(
        np.diag(1.0 / np.square(joint_span)),
        q_reference,
        q[model.arm_indices],
    )
    program.SetInitialGuess(q, q_nominal)
    result = Solve(program)
    return result, q


def _manipulability_metrics(model, q: np.ndarray) -> dict:
    """Measure grasp-frame arm and rail-plus-arm Jacobian conditioning."""
    model.plant.SetPositions(model.plant_context, q)
    jacobian = model.plant.CalcJacobianSpatialVelocity(
        model.plant_context,
        JacobianWrtVariable.kV,
        model.grasp_frame,
        np.zeros(3),
        model.plant.world_frame(),
        model.plant.world_frame(),
    )
    arm_jacobian = jacobian[:, model.arm_velocity_indices]
    rail_arm_velocity_indices = np.r_[
        model.rail_velocity_index,
        model.arm_velocity_indices,
    ]
    rail_arm_jacobian = jacobian[:, rail_arm_velocity_indices]

    def singular_values(block: np.ndarray) -> list[float]:
        return np.linalg.svd(block, compute_uv=False).tolist()

    arm_translation = singular_values(arm_jacobian[3:, :])
    arm_rotation = singular_values(arm_jacobian[:3, :])
    rail_arm_translation = singular_values(rail_arm_jacobian[3:, :])
    return {
        "left_arm_translational_singular_values": arm_translation,
        "left_arm_rotational_singular_values": arm_rotation,
        "rail_plus_arm_translational_singular_values": (
            rail_arm_translation
        ),
        "left_arm_minimum_translational_singular_value": min(
            arm_translation
        ),
        "left_arm_translational_manipulability": float(
            np.prod(arm_translation)
        ),
        "left_arm_minimum_rotational_singular_value": min(arm_rotation),
        "rail_plus_arm_minimum_translational_singular_value": min(
            rail_arm_translation
        ),
    }


def _candidate_metrics(
    *,
    model,
    workspace: PickWorkspaceEvaluator,
    q: np.ndarray,
    reference_name: str,
    solver_result,
    acceptance_clearance: float,
    workspace_safety_margin: float,
    diagnostic_distance: float,
) -> dict:
    """Return raw collision, naturalness, and manipulability metrics."""
    q_left = q[model.arm_indices]
    joint_midpoints = 0.5 * (
        model.arm_lower_limits + model.arm_upper_limits
    )
    joint_spans = model.arm_upper_limits - model.arm_lower_limits
    joint_margins = np.minimum(
        q_left - model.arm_lower_limits,
        model.arm_upper_limits - q_left,
    )
    normalized_joint_margins = joint_margins / joint_spans
    clearance = configuration_clearance_metrics(
        model,
        q,
        diagnostic_distance,
    )
    workspace_metrics = workspace.evaluate(q, workspace_safety_margin)
    X_WG = model.plant.CalcRelativeTransform(
        model.plant_context,
        model.plant.world_frame(),
        model.grasp_frame,
    )
    valid = bool(
        solver_result.is_success()
        and clearance["minimum_nonpenetration_distance"] >= 0.0
        and clearance["minimum_safety_clearance"] >= acceptance_clearance
        and workspace_metrics["satisfied"]
    )
    return {
        "reference": reference_name,
        "solver_result": str(solver_result.get_solution_result()),
        "solver_success": solver_result.is_success(),
        "valid": valid,
        "rail_height_m": float(q[model.rail_index]),
        "q_left_rad": q_left.tolist(),
        "minimum_nonpenetration_distance_m": clearance[
            "minimum_nonpenetration_distance"
        ],
        "minimum_safety_clearance_m": clearance[
            "minimum_safety_clearance"
        ],
        "nearest_nonpenetration_pair": clearance[
            "nearest_nonpenetration_pair"
        ],
        "nearest_safety_pair": clearance["nearest_safety_pair"],
        "minimum_joint_limit_margin_rad": float(np.min(joint_margins)),
        "minimum_normalized_joint_limit_margin": float(
            np.min(normalized_joint_margins)
        ),
        "joint_limit_margins_rad": joint_margins.tolist(),
        "elbow_bend_abs_deg": float(np.rad2deg(abs(q_left[3]))),
        "elbow_joint_rad": float(q_left[3]),
        "wrist_deviation_from_midpoint_rad": float(
            np.linalg.norm(q_left[4:] - joint_midpoints[4:])
        ),
        "workspace": workspace_metrics,
        "grasp_frame_xyz_world_m": X_WG.translation().tolist(),
        "grasp_frame_rpy_world_deg": np.rad2deg(
            X_WG.rotation().ToRollPitchYaw().vector()
        ).tolist(),
        "manipulability": _manipulability_metrics(model, q),
    }


def _shortlist(candidates: list[dict]) -> list[dict]:
    """Return unique candidates that lead one transparent raw metric."""
    if not candidates:
        return []
    natural_candidates = [
        candidate
        for candidate in candidates
        if candidate["reference"] != "q_zero"
    ]
    if not natural_candidates:
        natural_candidates = candidates
    selectors = (
        (
            "maximum_safety_clearance",
            lambda candidate: candidate["minimum_safety_clearance_m"],
            max,
        ),
        (
            "maximum_joint_limit_margin",
            lambda candidate: candidate[
                "minimum_normalized_joint_limit_margin"
            ],
            max,
        ),
        (
            "minimum_wrist_midpoint_deviation",
            lambda candidate: candidate[
                "wrist_deviation_from_midpoint_rad"
            ],
            min,
        ),
        (
            "maximum_translational_manipulability",
            lambda candidate: candidate["manipulability"][
                "left_arm_translational_manipulability"
            ],
            max,
        ),
        (
            "maximum_observed_elbow_bend",
            lambda candidate: candidate["elbow_bend_abs_deg"],
            max,
        ),
    )
    shortlisted = {}
    for basis, key, selector in selectors:
        selected = selector(natural_candidates, key=key)
        identity = tuple(selected["q_left_rad"])
        if identity not in shortlisted:
            shortlisted[identity] = {
                "selection_bases": [],
                **selected,
            }
        shortlisted[identity]["selection_bases"].append(basis)
    for candidate in natural_candidates:
        if not candidate["reference"].startswith("bent_elbow_"):
            continue
        identity = tuple(candidate["q_left_rad"])
        if identity not in shortlisted:
            shortlisted[identity] = {
                "selection_bases": [],
                **candidate,
            }
        shortlisted[identity]["selection_bases"].append(
            f"explicit_{candidate['reference']}_reference"
        )
    return list(shortlisted.values())


def main() -> None:
    """Search rail-height postures and save an unweighted visual shortlist."""
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
        include_rail=True,
    )
    for rail_height in args.rail_heights:
        if not model.rail_lower_limit <= rail_height <= model.rail_upper_limit:
            raise ValueError(
                f"Rail height {rail_height} is outside "
                f"[{model.rail_lower_limit}, {model.rail_upper_limit}]"
            )
    planning_filters = apply_safety_clearance_filters(
        model,
        influence_distance=(
            args.objective_clearance + args.influence_distance_offset
        ),
    )
    workspace = PickWorkspaceEvaluator(
        model=model,
        table_model_name=args.table_model_name,
    )
    references = _posture_references(
        model.arm_lower_limits,
        model.arm_upper_limits,
        args.num_random_references,
        args.random_seed,
    )
    diagnostic_distance = max(
        0.05,
        args.objective_clearance + args.influence_distance_offset,
    )
    height_results = []
    for rail_height in sorted(args.rail_heights):
        candidates = []
        for reference_name, q_reference in references:
            result, q_variables = _solve_reference(
                model=model,
                rail_height=rail_height,
                q_reference=q_reference,
                objective_clearance=args.objective_clearance,
                influence_distance_offset=args.influence_distance_offset,
            )
            q_solution = result.GetSolution(q_variables)
            candidate = _candidate_metrics(
                model=model,
                workspace=workspace,
                q=q_solution,
                reference_name=reference_name,
                solver_result=result,
                acceptance_clearance=args.acceptance_clearance,
                workspace_safety_margin=args.workspace_safety_margin,
                diagnostic_distance=diagnostic_distance,
            )
            candidates.append(candidate)
            print(
                f"rail={rail_height:.3f} reference={reference_name} "
                f"valid={candidate['valid']} "
                "clearance="
                f"{candidate['minimum_safety_clearance_m']:.6f}",
                flush=True,
            )
        valid_candidates = [
            candidate for candidate in candidates if candidate["valid"]
        ]
        height_results.append(
            {
                "rail_height_m": rail_height,
                "valid_candidate_count": len(valid_candidates),
                "shortlist": _shortlist(valid_candidates),
                "candidates": candidates,
            }
        )

    output_path = (
        args.output_json.resolve()
        if args.output_json is not None
        else scene_dmd.parent / "rail_postures.json"
    )
    output = {
        "search_kind": "static_joint_rail_and_left_arm_posture_search",
        "automatic_best_posture_selected": False,
        "requires_manual_multiview_selection": True,
        "robot_base_xyz_m": ROBOT_BASE_XYZ_METERS.tolist(),
        "robot_base_yaw_deg": ROBOT_BASE_YAW_DEG,
        "rail_limits_m": [model.rail_lower_limit, model.rail_upper_limit],
        "objective_clearance_m": args.objective_clearance,
        "acceptance_clearance_m": args.acceptance_clearance,
        "workspace_safety_margin_m": args.workspace_safety_margin,
        "naturalness_policy": (
            "No weighted naturalness score is used. Per-height shortlists "
            "lead transparent raw metrics and require visual selection."
        ),
        "q_zero_policy": (
            "q_zero is retained as a diagnostic solve but cannot lead a "
            "shortlist metric while any valid nonzero reference exists."
        ),
        "planning_collision_filters": [
            {
                "body_a": item.body_a,
                "body_b": item.body_b,
                "reason": item.reason,
            }
            for item in planning_filters
        ],
        "heights": height_results,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(output, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"Static rail/posture diagnostics: {output_path}")
    print("No posture was automatically approved; inspect the shortlists")


if __name__ == "__main__":
    main()
