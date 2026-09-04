#!/usr/bin/env python3
"""Run regression checks for Zerith PREGRASP collision scope."""

import argparse
import json
import sys

from collections import Counter
from pathlib import Path

import numpy as np

from pydrake.multibody.inverse_kinematics import (
    MinimumDistanceLowerBoundConstraint,
)

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_grasp_geometry import (
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)
from src.zerith_pregrasp_collision import (
    apply_safety_clearance_filters,
    build_pregrasp_planning_model,
    raw_scene_distance_records,
    robot_clearance_records,
)

EVAL_PACKAGE_XML = (
    REPOSITORY_ROOT / "models" / "zerith_pick_eval" / "package.xml"
)
ZERITH_MODEL_DIR = REPOSITORY_ROOT / "models" / "zerith_drake"
TARGET_BODY = "living_room_box_0::base_link"
COFFEE_TABLE_BODY = "living_room_coffee_table_0::base_link"
TORSO_BODY_NAME = "body_yaw_link"
LEFT_SHOULDER_BODY_NAME = "left_shoulder_roll_link"

# A deterministic configuration from the earlier unconstrained PREGRASP solve.
# At the default fixed base pose, it places the left wrist and other left-arm
# links inside the coffee-table collision geometry.
KNOWN_TABLE_COLLISION_Q_LEFT = np.array(
    [
        -0.9699175954762229,
        -0.32550079587958186,
        -0.06959550283731261,
        0.8124049255743832,
        0.16375110454248354,
        0.2360366435004506,
        0.19563942138273732,
    ]
)


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Verify that PREGRASP collision checks ignore environment-only "
            "support contacts while retaining robot-table collisions."
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
        "--minimum-distance",
        type=float,
        default=0.002,
    )
    parser.add_argument(
        "--influence-distance-offset",
        type=float,
        default=0.01,
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        help="Defaults to collision_scope_regression.json beside scene_dmd.",
    )
    return parser.parse_args()


def _contains_body_pair(record: dict, body_a: str, body_b: str) -> bool:
    """Return whether a distance record matches an unordered body pair."""
    return {record["body_a"], record["body_b"]} == {body_a, body_b}


def main() -> None:
    """Execute collision-scope regressions and write diagnostics."""
    args = _parse_args()
    if args.minimum_distance < 0.0:
        raise ValueError("minimum-distance must be nonnegative")
    if args.influence_distance_offset <= 0.0:
        raise ValueError("influence-distance-offset must be positive")

    model = build_pregrasp_planning_model(
        scene_dmd=args.scene_dmd.resolve(),
        scene_package_xml=args.scene_package_xml.resolve(),
        eval_package_xml=args.eval_package_xml.resolve(),
        robot_model_dir=args.robot_model_dir.resolve(),
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
    )
    q_zero = model.q_scene.copy()
    q_zero[model.arm_indices] = 0.0

    raw_near_pairs = raw_scene_distance_records(
        model,
        q_zero,
        args.minimum_distance,
    )
    robot_pairs_before_filters = robot_clearance_records(
        model,
        q_zero,
        args.minimum_distance,
        layer="safety",
    )
    target_support_pairs = [
        record
        for record in raw_near_pairs
        if _contains_body_pair(record, TARGET_BODY, COFFEE_TABLE_BODY)
    ]
    if not target_support_pairs:
        raise AssertionError(
            "Expected the red box to touch the coffee table at q_home"
        )

    filters = apply_safety_clearance_filters(
        model,
        influence_distance=(
            args.minimum_distance + args.influence_distance_offset
        ),
    )
    torso = model.plant.GetBodyByName(TORSO_BODY_NAME, model.zerith)
    left_shoulder = model.plant.GetBodyByName(
        LEFT_SHOULDER_BODY_NAME,
        model.zerith,
    )
    shoulder_torso_safety_filtered = (
        model.safety_checker.IsCollisionFilteredBetween(
            torso.index(),
            left_shoulder.index(),
        )
    )
    shoulder_torso_nonpenetration_filtered = (
        model.nonpenetration_checker.IsCollisionFilteredBetween(
            torso.index(),
            left_shoulder.index(),
        )
    )
    if not shoulder_torso_safety_filtered:
        raise AssertionError(
            "Left shoulder-to-torso pair must be filtered from the safety "
            "clearance layer"
        )
    if shoulder_torso_nonpenetration_filtered:
        raise AssertionError(
            "Left shoulder-to-torso pair must remain active in the "
            "nonpenetration layer"
        )
    q_zero_collision_free = (
        model.nonpenetration_checker.CheckContextConfigCollisionFree(
            model.nonpenetration_checker_context,
            q_zero,
        )
    )
    if not q_zero_collision_free:
        raise AssertionError(
            "q_zero should pass the nonpenetration collision checker"
        )

    collision_constraint = MinimumDistanceLowerBoundConstraint(
        collision_checker=model.safety_checker,
        collision_checker_context=model.safety_checker_context,
        bound=args.minimum_distance,
        influence_distance_offset=args.influence_distance_offset,
    )
    q_zero_constraint_satisfied = collision_constraint.CheckSatisfied(
        q_zero,
        1e-9,
    )
    if not q_zero_constraint_satisfied:
        raise AssertionError(
            "Environment support contacts incorrectly made the robot-scoped "
            "minimum-distance constraint infeasible at q_home"
        )

    q_table_collision = q_zero.copy()
    q_table_collision[model.arm_indices] = KNOWN_TABLE_COLLISION_Q_LEFT
    bad_pose_clearances = robot_clearance_records(
        model,
        q_table_collision,
        0.2,
        layer="nonpenetration",
    )
    wrist_table_penetrations = [
        record
        for record in bad_pose_clearances
        if record["distance_m"] < 0.0
        and "left_wrist" in record["body_a"]
        and record["body_b"] == COFFEE_TABLE_BODY
    ]
    if not wrist_table_penetrations:
        raise AssertionError(
            "Known bad pose should retain a left-wrist-to-table penetration"
        )
    bad_pose_collision_free = (
        model.nonpenetration_checker.CheckContextConfigCollisionFree(
            model.nonpenetration_checker_context,
            q_table_collision,
        )
    )
    if bad_pose_collision_free:
        raise AssertionError(
            "Known left-wrist table collision was incorrectly filtered"
        )

    output_path = (
        args.output_json.resolve()
        if args.output_json is not None
        else args.scene_dmd.resolve().parent
        / "collision_scope_regression.json"
    )
    output = {
        "passed": True,
        "minimum_distance_m": args.minimum_distance,
        "raw_q_zero_pair_category_counts": dict(
            Counter(record["category"] for record in raw_near_pairs)
        ),
        "raw_q_zero_pairs_below_minimum": raw_near_pairs,
        "robot_pairs_before_planning_filters": robot_pairs_before_filters,
        "planning_collision_filters": [
            {
                "body_a": item.body_a,
                "body_b": item.body_b,
                "reason": item.reason,
            }
            for item in filters
        ],
        "regressions": {
            "target_support_contact_present_and_q_zero_valid": True,
            "known_left_wrist_table_collision_detected": True,
            "environment_contacts_do_not_violate_ik_constraint": True,
            "left_shoulder_torso_filtered_only_from_safety_layer": True,
        },
        "target_support_pairs": target_support_pairs,
        "known_bad_pose_wrist_table_penetrations": (
            wrist_table_penetrations
        ),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print("PASS: target support contact does not invalidate q_zero")
    print("PASS: known left-wrist/table collision remains active")
    print("PASS: environment contacts do not enter the IK distance constraint")
    print("PASS: left shoulder/torso is filtered only from the safety layer")
    print(f"Diagnostics: {output_path}")


if __name__ == "__main__":
    main()
