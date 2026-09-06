#!/usr/bin/env python3
"""Summarize pair-level evidence for rejected Cartesian edge actions."""

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    """Parse trace and output paths."""
    parser = argparse.ArgumentParser(
        description=(
            "Extract pair, distance, and validity-layer evidence from an "
            "online-environment trace."
        )
    )
    parser.add_argument("trace_csv", type=Path)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    return parser.parse_args()


def _failure_layers(cartesian: dict) -> tuple[str, ...]:
    """Return the explicit invalid edge layers in stable order."""
    fields = (
        ("joint_limits", "joint_limits_valid"),
        ("nonpenetration", "nonpenetration_valid"),
        ("safety_clearance", "safety_clearance_valid"),
    )
    missing = [field for _, field in fields if field not in cartesian]
    if missing:
        raise ValueError(
            "Trace lacks detailed EdgeCheck fields: " + ", ".join(missing)
        )
    return tuple(name for name, field in fields if not cartesian[field])


def _pair_key(pair: dict | None) -> str:
    """Return a stable human-readable geometry-pair identity."""
    if pair is None:
        return "none"
    endpoints = sorted((
        f"{pair['body_a']}[{pair['geometry_a']}]",
        f"{pair['body_b']}[{pair['geometry_b']}]",
    ))
    return " <-> ".join(endpoints)


def audit_trace(trace_csv: Path) -> tuple[dict, list[dict]]:
    """Return aggregate and row-level rejected-edge evidence."""
    with trace_csv.open(encoding="utf-8") as stream:
        trace_rows = list(csv.DictReader(stream))

    rows = []
    for trace_row in trace_rows:
        decision = json.loads(trace_row["action_decision_json"])
        if (
            decision.get("status") != "rejected"
            or "cartesian_edge_collision_or_clearance"
            not in decision.get("reasons", ())
        ):
            continue
        cartesian = decision["cartesian"]
        pair = cartesian["limiting_nonpenetration_pair"]
        start_distance = cartesian["limiting_pair_start_distance_m"]
        end_distance = cartesian["limiting_pair_end_distance_m"]
        edge_translation = cartesian["validation_edge_translation_m"]
        policy = json.loads(trace_row["policy_diagnostics_json"])
        layers = _failure_layers(cartesian)
        rows.append({
            "step": int(trace_row["step"]),
            "simulation_time_s": float(trace_row["simulation_time_s"]),
            "policy_stage": policy["stage"],
            "failure_layers": "+".join(layers),
            "joint_limits_valid": cartesian["joint_limits_valid"],
            "nonpenetration_valid": cartesian["nonpenetration_valid"],
            "safety_clearance_valid": cartesian[
                "safety_clearance_valid"
            ],
            "limiting_geometry_pair": _pair_key(pair),
            "limiting_pair_json": json.dumps(
                pair,
                separators=(",", ":"),
            ),
            "minimum_signed_distance_m": cartesian[
                "minimum_nonpenetration_distance_m"
            ],
            "minimum_nonpenetration_margin_m": cartesian[
                "minimum_nonpenetration_margin_m"
            ],
            "minimum_nonpenetration_alpha": cartesian[
                "minimum_nonpenetration_margin_alpha"
            ],
            "pair_start_distance_m": start_distance,
            "pair_end_distance_m": end_distance,
            "pair_distance_change_m": (
                None
                if start_distance is None or end_distance is None
                else end_distance - start_distance
            ),
            "pair_monotonic_non_decreasing": cartesian[
                "limiting_pair_monotonic_non_decreasing"
            ],
            "pair_sample_distances_json": json.dumps(
                cartesian["limiting_pair_sample_distances_m"],
                separators=(",", ":"),
            ),
            "requested_twist_json": json.dumps(
                cartesian["requested_twist"],
                separators=(",", ":"),
            ),
            "achieved_twist_json": json.dumps(
                cartesian["achieved_twist"],
                separators=(",", ":"),
            ),
            "validation_edge_translation_m_json": json.dumps(
                edge_translation,
                separators=(",", ":"),
            ),
            "validation_edge_x_m": edge_translation[0],
            "validation_edge_y_m": edge_translation[1],
            "validation_edge_z_m": edge_translation[2],
        })

    layer_counts = Counter(row["failure_layers"] for row in rows)
    grouped = defaultdict(list)
    for row in rows:
        if not row["nonpenetration_valid"]:
            grouped[row["limiting_geometry_pair"]].append(row)
    pair_groups = []
    for pair_key, group in sorted(grouped.items()):
        changes = [row["pair_distance_change_m"] for row in group]
        pair_groups.append({
            "geometry_pair": pair_key,
            "count": len(group),
            "first_step": min(row["step"] for row in group),
            "last_step": max(row["step"] for row in group),
            "minimum_signed_distance_m": min(
                row["minimum_signed_distance_m"] for row in group
            ),
            "minimum_start_distance_m": min(
                row["pair_start_distance_m"] for row in group
            ),
            "minimum_end_distance_m": min(
                row["pair_end_distance_m"] for row in group
            ),
            "minimum_distance_change_m": min(changes),
            "maximum_distance_change_m": max(changes),
            "all_monotonic_non_decreasing": all(
                row["pair_monotonic_non_decreasing"] for row in group
            ),
            "minimum_validation_edge_z_m": min(
                row["validation_edge_z_m"] for row in group
            ),
            "maximum_validation_edge_z_m": max(
                row["validation_edge_z_m"] for row in group
            ),
        })
    aggregate = {
        "source_trace": str(trace_csv),
        "cartesian_edge_rejection_count": len(rows),
        "failure_layer_counts": dict(sorted(layer_counts.items())),
        "nonpenetration_rejection_count": sum(
            not row["nonpenetration_valid"] for row in rows
        ),
        "nonpenetration_pair_groups": pair_groups,
        "rejections": rows,
    }
    return aggregate, rows


def main() -> None:
    """Write JSON aggregate and CSV row-level audit artifacts."""
    args = _parse_args()
    aggregate, rows = audit_trace(args.trace_csv.resolve())
    if not rows:
        raise ValueError("Trace contains no rejected Cartesian edge actions")
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(aggregate, indent=2) + "\n",
        encoding="utf-8",
    )
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({
        "cartesian_edge_rejection_count": aggregate[
            "cartesian_edge_rejection_count"
        ],
        "failure_layer_counts": aggregate["failure_layer_counts"],
        "nonpenetration_rejection_count": aggregate[
            "nonpenetration_rejection_count"
        ],
        "nonpenetration_pair_groups": aggregate[
            "nonpenetration_pair_groups"
        ],
    }, indent=2))


if __name__ == "__main__":
    main()
