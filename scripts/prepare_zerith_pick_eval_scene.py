#!/usr/bin/env python3
"""Derive a reproducible Zerith small-box evaluation scene."""

import argparse
import json
import sys

from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT))

from src.zerith_grasp_geometry import (
    APPROACH_AXIS_GRASP,
    BOX_SIZE_METERS,
    CLOSING_AXIS_GRASP,
    GRASP_WIDTH_METERS,
    LEFT_GRASP_FRAME_NAME,
    LEFT_GRASP_PARENT_FRAME_NAME,
    LIFT_DISTANCE_METERS,
    PREGRASP_DISTANCE_METERS,
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
    UP_AXIS_GRASP,
    X_PARENT_GRASP_TRANSLATION_METERS,
)

DEFAULT_TARGET_MODEL = "living_room_box_0"
SMALL_BOX_URI = "package://zerith_pick_eval/small_red_box.sdf"


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


def _task_metadata(source_dmd: Path, target_model_name: str) -> dict:
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
        "source_target_pose_preserved": True,
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

    if not source_dmd.is_file():
        raise FileNotFoundError(f"Source DMD does not exist: {source_dmd}")
    if output_dmd == source_dmd:
        raise ValueError("Output DMD must not overwrite the source DMD")

    derived_text = _replace_model_file(
        source_dmd.read_text(encoding="utf-8"),
        args.target_model_name,
        SMALL_BOX_URI,
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    output_dmd.write_text(derived_text, encoding="utf-8")
    output_metadata.write_text(
        json.dumps(
            _task_metadata(source_dmd, args.target_model_name),
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"Derived DMD: {output_dmd}")
    print(f"Task metadata: {output_metadata}")
    print(f"Source DMD left unchanged: {source_dmd}")


if __name__ == "__main__":
    main()
