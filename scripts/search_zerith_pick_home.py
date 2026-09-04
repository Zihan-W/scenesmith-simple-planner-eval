#!/usr/bin/env python3
"""Search a task-specific Zerith home connected to PREGRASP."""

import argparse
import dataclasses
import json
import sys

from pathlib import Path

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_grasp_geometry import (
    PICK_RAIL_POSITION_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)
from src.zerith_pick_workspace import (
    PickWorkspaceEvaluator,
    edge_workspace_metrics,
)
from src.zerith_pregrasp_collision import (
    apply_safety_clearance_filters,
    build_pregrasp_planning_model,
    configuration_clearance_metrics,
    edge_clearance_metrics,
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
            "Search q_pick_home inside the collision-safe visibility region "
            "of a validated PREGRASP configuration."
        )
    )
    parser.add_argument("scene_dmd", type=Path)
    parser.add_argument("--scene-package-xml", type=Path, required=True)
    parser.add_argument("--pregrasp-json", type=Path, required=True)
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
        "--rail-position",
        type=float,
        default=PICK_RAIL_POSITION_METERS,
    )
    parser.add_argument(
        "--safety-objective-clearance",
        type=float,
        default=0.007,
    )
    parser.add_argument(
        "--safety-acceptance-clearance",
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
    parser.add_argument(
        "--search-step-sizes",
        type=float,
        nargs="+",
        default=(0.04, 0.02, 0.01, 0.005),
    )
    parser.add_argument("--maximum-passes-per-step-size", type=int, default=20)
    parser.add_argument(
        "--search-edge-sample-step",
        type=float,
        default=0.01,
    )
    parser.add_argument(
        "--validation-edge-sample-step",
        type=float,
        default=0.002,
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Defaults to pick_home.json beside pregrasp-json.",
    )
    return parser.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    """Validate collision, workspace, and search parameters."""
    positive_fields = (
        "safety_objective_clearance",
        "safety_acceptance_clearance",
        "workspace_safety_margin",
        "influence_distance_offset",
        "search_edge_sample_step",
        "validation_edge_sample_step",
    )
    for name in positive_fields:
        if getattr(args, name) <= 0.0:
            raise ValueError(f"{name.replace('_', '-')} must be positive")
    if (
        args.safety_objective_clearance
        < args.safety_acceptance_clearance
    ):
        raise ValueError(
            "safety-objective-clearance must be at least "
            "safety-acceptance-clearance"
        )
    if args.maximum_passes_per_step_size < 1:
        raise ValueError("maximum-passes-per-step-size must be positive")
    if not args.search_step_sizes or any(
        step <= 0.0 for step in args.search_step_sizes
    ):
        raise ValueError("search-step-sizes must contain positive values")
    if any(
        later >= earlier
        for earlier, later in zip(
            args.search_step_sizes,
            args.search_step_sizes[1:],
        )
    ):
        raise ValueError("search-step-sizes must be strictly decreasing")


def _load_pregrasp(path: Path) -> np.ndarray:
    """Load the selected seven-joint PREGRASP configuration."""
    payload = json.loads(path.read_text(encoding="utf-8"))
    q_pregrasp = np.asarray(payload["solution"]["q_left"], dtype=float)
    if q_pregrasp.shape != (7,) or not np.all(np.isfinite(q_pregrasp)):
        raise ValueError("pregrasp-json contains an invalid solution.q_left")
    return q_pregrasp


def _full_configuration(model, q_left: np.ndarray) -> np.ndarray:
    """Insert a left-arm posture into the fixed planning scene."""
    q = model.q_scene.copy()
    q[model.arm_indices] = q_left
    return q


def _candidate_directions(q_current: np.ndarray, step_size: float):
    """Return deterministic coordinate-descent candidates toward q_zero."""
    delta = np.clip(-q_current, -step_size, step_size)
    candidates = []
    for joint_index in range(7):
        q_candidate = q_current.copy()
        q_candidate[joint_index] += delta[joint_index]
        candidates.append((f"joint_{joint_index}", q_candidate))
    candidates.append(("all_joints", q_current + delta))
    largest = np.argsort(-np.abs(q_current))[:2]
    q_candidate = q_current.copy()
    q_candidate[largest] += delta[largest]
    candidates.append(("two_largest_joints", q_candidate))
    return candidates


def _evaluate_candidate(
    *,
    model,
    workspace: PickWorkspaceEvaluator,
    q_left: np.ndarray,
    q_pregrasp_scene: np.ndarray,
    args: argparse.Namespace,
    edge_sample_step: float,
) -> dict:
    """Evaluate the ordered pick-home acceptance conditions."""
    q_scene = _full_configuration(model, q_left)
    diagnostic_distance = max(
        0.05,
        args.safety_objective_clearance
        + args.influence_distance_offset,
    )
    clearance = configuration_clearance_metrics(
        model,
        q_scene,
        diagnostic_distance,
    )
    collision_constraints_satisfied = bool(
        clearance["minimum_nonpenetration_distance"] >= 0.0
        and clearance["minimum_safety_clearance"]
        >= args.safety_objective_clearance
    )
    workspace_constraint = workspace.evaluate(
        q_scene,
        args.workspace_safety_margin,
    )
    edge = None
    edge_workspace = None
    connected_to_pregrasp = False
    if collision_constraints_satisfied and workspace_constraint["satisfied"]:
        edge = edge_clearance_metrics(
            model,
            q_scene,
            q_pregrasp_scene,
            diagnostic_distance,
            max_joint_step=edge_sample_step,
        )
        edge_workspace = edge_workspace_metrics(
            workspace,
            q_scene,
            q_pregrasp_scene,
            args.workspace_safety_margin,
            edge_sample_step,
        )
        connected_to_pregrasp = bool(
            edge["minimum_nonpenetration_distance"] >= 0.0
            and edge["minimum_safety_clearance"]
            >= args.safety_objective_clearance
            and edge_workspace["workspace_constraint_satisfied"]
        )
    return {
        "q_left": q_left.tolist(),
        "distance_from_q_zero": float(np.linalg.norm(q_left - Q_ZERO)),
        "collision_constraints_satisfied": collision_constraints_satisfied,
        "workspace_constraint_satisfied": workspace_constraint["satisfied"],
        "connected_to_pregrasp": connected_to_pregrasp,
        "clearance_metrics": clearance,
        "workspace_constraint": workspace_constraint,
        "q_pick_home_to_pregrasp_edge": edge,
        "q_pick_home_to_pregrasp_workspace_edge": edge_workspace,
    }


def _accepted(candidate: dict) -> bool:
    """Return whether a candidate satisfies all ordered requirements."""
    return bool(
        candidate["collision_constraints_satisfied"]
        and candidate["workspace_constraint_satisfied"]
        and candidate["connected_to_pregrasp"]
    )


def main() -> None:
    """Search, densely validate, and save q_pick_home."""
    args = _parse_args()
    _validate_args(args)
    scene_dmd = args.scene_dmd.resolve()
    pregrasp_json = args.pregrasp_json.resolve()
    q_pregrasp = _load_pregrasp(pregrasp_json)
    model = build_pregrasp_planning_model(
        scene_dmd=scene_dmd,
        scene_package_xml=args.scene_package_xml.resolve(),
        eval_package_xml=args.eval_package_xml.resolve(),
        robot_model_dir=args.robot_model_dir.resolve(),
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
        rail_position=args.rail_position,
    )
    filters = apply_safety_clearance_filters(
        model,
        influence_distance=(
            args.safety_objective_clearance
            + args.influence_distance_offset
        ),
    )
    workspace = PickWorkspaceEvaluator(
        model=model,
        table_model_name=args.table_model_name,
    )
    q_pregrasp_scene = _full_configuration(model, q_pregrasp)
    pregrasp = _evaluate_candidate(
        model=model,
        workspace=workspace,
        q_left=q_pregrasp,
        q_pregrasp_scene=q_pregrasp_scene,
        args=args,
        edge_sample_step=args.validation_edge_sample_step,
    )
    if not _accepted(pregrasp):
        raise ValueError(
            "The supplied PREGRASP does not satisfy the pick-home collision "
            "and whole-wrist workspace requirements"
        )

    current = pregrasp
    accepted_history = []
    evaluated_candidate_count = 1
    for step_size in args.search_step_sizes:
        for pass_index in range(args.maximum_passes_per_step_size):
            improvements = []
            for source, q_candidate in _candidate_directions(
                np.asarray(current["q_left"]),
                step_size,
            ):
                candidate = _evaluate_candidate(
                    model=model,
                    workspace=workspace,
                    q_left=q_candidate,
                    q_pregrasp_scene=q_pregrasp_scene,
                    args=args,
                    edge_sample_step=args.search_edge_sample_step,
                )
                evaluated_candidate_count += 1
                if (
                    _accepted(candidate)
                    and candidate["distance_from_q_zero"]
                    < current["distance_from_q_zero"] - 1e-9
                ):
                    candidate["source"] = source
                    improvements.append(candidate)
            if not improvements:
                break
            current = min(
                improvements,
                key=lambda candidate: candidate["distance_from_q_zero"],
            )
            accepted_history.append(
                {
                    "step_size_rad": step_size,
                    "pass_index": pass_index,
                    **current,
                }
            )
            edge_safety = current["q_pick_home_to_pregrasp_edge"][
                "minimum_safety_clearance"
            ]
            print(
                f"step={step_size:.4f}, pass={pass_index}, "
                f"source={current['source']}, "
                "distance_from_q_zero="
                f"{current['distance_from_q_zero']:.6f}, "
                f"edge_safety={edge_safety:.6f}",
                flush=True,
            )

    dense_candidates = list(reversed(accepted_history)) + [pregrasp]
    selected = None
    for candidate in dense_candidates:
        dense_candidate = _evaluate_candidate(
            model=model,
            workspace=workspace,
            q_left=np.asarray(candidate["q_left"]),
            q_pregrasp_scene=q_pregrasp_scene,
            args=args,
            edge_sample_step=args.validation_edge_sample_step,
        )
        if _accepted(dense_candidate):
            selected = dense_candidate
            break
    assert selected is not None

    output_path = (
        args.output_json.resolve()
        if args.output_json is not None
        else pregrasp_json.parent / "pick_home.json"
    )
    output = {
        "search_succeeded": True,
        "calibration_status": (
            "fixed_rail_pick_home_to_pregrasp_validated"
        ),
        "daogui_joint_position_m": float(
            model.q_scene[
                model.plant.GetJointByName(
                    "daogui_joint",
                    model.zerith,
                ).position_start()
            ]
        ),
        "q_zero": Q_ZERO.tolist(),
        "q_pick_home": selected["q_left"],
        "q_pregrasp": q_pregrasp.tolist(),
        "ranking_order": [
            "double_layer_collision_constraints",
            "whole_wrist_gripper_table_workspace_constraint",
            "direct_edge_connected_to_pregrasp",
            "minimum_distance_from_q_zero",
        ],
        "safety_objective_clearance_m": (
            args.safety_objective_clearance
        ),
        "safety_acceptance_clearance_m": (
            args.safety_acceptance_clearance
        ),
        "workspace_safety_margin_m": args.workspace_safety_margin,
        "search_edge_sample_step_rad": args.search_edge_sample_step,
        "validation_edge_sample_step_rad": (
            args.validation_edge_sample_step
        ),
        "selected": selected,
        "pregrasp_endpoint": pregrasp,
        "accepted_search_step_count": len(accepted_history),
        "evaluated_candidate_count": evaluated_candidate_count,
        "safety_clearance_filters": [
            dataclasses.asdict(item) for item in filters
        ],
        "connectivity_validation": (
            "high-density numerical joint-edge sampling, not a "
            "mathematical proof"
        ),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(f"q_pick_home: {selected['q_left']}")
    print(
        "Selected metrics: "
        "nonpenetration="
        f"{selected['clearance_metrics']['minimum_nonpenetration_distance']:.6f} m, "
        "safety="
        f"{selected['clearance_metrics']['minimum_safety_clearance']:.6f} m, "
        "edge_safety="
        f"{selected['q_pick_home_to_pregrasp_edge']['minimum_safety_clearance']:.6f} m"
    )
    print(f"Diagnostics: {output_path}")


if __name__ == "__main__":
    main()
