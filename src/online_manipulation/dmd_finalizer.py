"""Write selected free-body poses back to a Drake directives file."""

import dataclasses
import math
import re
from pathlib import Path
from typing import Any

import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw

from src.online_manipulation.specs import ObservedBodySpec


@dataclasses.dataclass(frozen=True)
class _PoseUpdate:
    """One body pose expressed in its original DMD base frame."""

    model_name: str
    body_name: str
    translation: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]


def _indent(line: str) -> int:
    """Return the number of leading spaces in one YAML line."""
    return len(line) - len(line.lstrip(" "))


def _scalar(value: str) -> str:
    """Decode the simple quoted or unquoted names used by DMD files."""
    stripped = value.strip()
    if len(stripped) >= 2 and stripped[0] == stripped[-1] and stripped[0] in "\"'":
        return stripped[1:-1]
    return stripped


def _format_vector(values: tuple[float, float, float]) -> str:
    """Format a deterministic three-vector for human-readable YAML."""
    return "[" + ", ".join(f"{value:.16g}" for value in values) + "]"


def _directive_blocks(lines: list[str]) -> list[tuple[int, int]]:
    """Return half-open ranges for top-level directive entries."""
    starts = [
        index
        for index, line in enumerate(lines)
        if re.match(r"^-\s+[A-Za-z_][A-Za-z0-9_]*:\s*$", line)
    ]
    return [
        (start, starts[index + 1] if index + 1 < len(starts) else len(lines))
        for index, start in enumerate(starts)
    ]


def _model_name(lines: list[str], start: int, end: int) -> str | None:
    """Return the name of an add_model directive block, if present."""
    if not re.match(r"^-\s+add_model:\s*$", lines[start]):
        return None
    for line in lines[start + 1 : end]:
        match = re.match(r"^\s+name:\s*([^#]+?)\s*$", line)
        if match:
            return _scalar(match.group(1))
    raise ValueError("add_model directive is missing name")


def _body_pose_range(
    lines: list[str],
    start: int,
    end: int,
    body_name: str,
) -> tuple[int, int, str]:
    """Find one body pose block and its base-frame name."""
    default_index = next(
        (
            index
            for index in range(start, end)
            if re.match(r"^\s+default_free_body_pose:\s*$", lines[index])
        ),
        None,
    )
    if default_index is None:
        raise ValueError("add_model directive has no default_free_body_pose")
    default_indent = _indent(lines[default_index])
    body_index = None
    for index in range(default_index + 1, end):
        line = lines[index]
        if line.strip() and _indent(line) <= default_indent:
            break
        match = re.match(r"^\s+([^:#]+):\s*$", line)
        if match and _indent(line) > default_indent:
            candidate = _scalar(match.group(1))
            if candidate == body_name:
                body_index = index
                break
    if body_index is None:
        raise ValueError(
            f"default_free_body_pose has no entry for body {body_name}"
        )
    body_indent = _indent(lines[body_index])
    body_end = end
    for index in range(body_index + 1, end):
        if lines[index].strip() and _indent(lines[index]) <= body_indent:
            body_end = index
            break
    base_frame = "world"
    for line in lines[body_index + 1 : body_end]:
        match = re.match(r"^\s+base_frame:\s*([^#]+?)\s*$", line)
        if match:
            base_frame = _scalar(match.group(1))
            break
    return body_index, body_end, base_frame


def _replace_pose_block(
    lines: list[str],
    body_start: int,
    body_end: int,
    update: _PoseUpdate,
) -> list[str]:
    """Replace translation and rotation while preserving other body keys."""
    result = list(lines[body_start:body_end])
    translation_index = next(
        (
            index
            for index, line in enumerate(result)
            if re.match(r"^\s+translation:\s*", line)
        ),
        None,
    )
    rotation_index = next(
        (
            index
            for index, line in enumerate(result)
            if re.match(r"^\s+rotation:\s*", line)
        ),
        None,
    )
    if translation_index is None or rotation_index is None:
        raise ValueError(
            f"Pose for {update.model_name}::{update.body_name} must contain "
            "translation and rotation"
        )
    translation_indent_count = _indent(result[translation_index])
    translation_end = translation_index + 1
    while translation_end < len(result):
        line = result[translation_end]
        # YAML allows a block sequence at the same indent as its key.
        if line.strip() and not (
            _indent(line) > translation_indent_count
            or (_indent(line) == translation_indent_count
                and line.lstrip().startswith("- "))
        ):
            break
        translation_end += 1
    translation_indent = " " * translation_indent_count
    translation_lines = [
        f"{translation_indent}translation: "
        f"{_format_vector(update.translation)}\n"
    ]
    rotation_indent_count = _indent(result[rotation_index])
    rotation_end = rotation_index + 1
    while rotation_end < len(result):
        if result[rotation_end].strip() and (
            _indent(result[rotation_end]) <= rotation_indent_count
        ):
            break
        rotation_end += 1
    rotation_indent = " " * rotation_indent_count
    child_indent = " " * (rotation_indent_count + 2)
    rotation_lines = [
        f"{rotation_indent}rotation: !Rpy\n",
        f"{child_indent}deg: {_format_vector(update.rpy_deg)}\n",
    ]
    for start, end, replacement in sorted(
        ((translation_index, translation_end, translation_lines),
         (rotation_index, rotation_end, rotation_lines)), reverse=True
    ):
        result[start:end] = replacement
    return result


def _pose_in_base_frame(
    plant: Any,
    plant_context: Any,
    body_spec: ObservedBodySpec,
    base_frame_name: str,
) -> RigidTransform:
    """Evaluate a selected body pose in the named DMD base frame."""
    model_instance = plant.GetModelInstanceByName(
        body_spec.model_instance_name
    )
    body = plant.GetBodyByName(body_spec.body_name, model_instance)
    world_from_body = plant.EvalBodyPoseInWorld(plant_context, body)
    if base_frame_name == "world":
        world_from_base = RigidTransform()
    else:
        base_frame = plant.GetFrameByName(base_frame_name)
        world_from_base = plant.CalcRelativeTransform(
            plant_context,
            plant.world_frame(),
            base_frame,
        )
    return world_from_base.inverse() @ world_from_body


def write_updated_dmd(
    *,
    input_path: Path,
    output_path: Path,
    plant: Any,
    plant_context: Any,
    body_specs: tuple[ObservedBodySpec, ...],
) -> tuple[str, ...]:
    """Write explicitly selected body poses and return updated public names.

    Only direct ``add_model.default_free_body_pose`` entries marked with
    ``write_back=True`` are changed. The source file is never overwritten.
    Every other line remains byte-for-byte identical.
    """
    source = Path(input_path).resolve()
    destination = Path(output_path).resolve()
    if source == destination:
        raise ValueError("DMD finalizer requires a distinct output path")
    selected = tuple(spec for spec in body_specs if spec.write_back)
    if not selected:
        raise ValueError("No observed body is marked write_back=True")

    lines = source.read_text(encoding="utf-8").splitlines(keepends=True)
    blocks = _directive_blocks(lines)
    replacements: list[tuple[int, int, list[str]]] = []
    for spec in selected:
        matches = [
            (start, end)
            for start, end in blocks
            if _model_name(lines, start, end) == spec.model_instance_name
        ]
        if len(matches) != 1:
            raise ValueError(
                f"Expected one direct add_model for "
                f"{spec.model_instance_name}, found {len(matches)}"
            )
        start, end = matches[0]
        body_start, body_end, base_frame_name = _body_pose_range(
            lines,
            start,
            end,
            spec.body_name,
        )
        base_from_body = _pose_in_base_frame(
            plant,
            plant_context,
            spec,
            base_frame_name,
        )
        rpy_deg = tuple(
            float(value)
            for value in np.degrees(
                RollPitchYaw(base_from_body.rotation()).vector()
            )
        )
        update = _PoseUpdate(
            model_name=spec.model_instance_name,
            body_name=spec.body_name,
            translation=tuple(
                float(value) for value in base_from_body.translation()
            ),
            rpy_deg=rpy_deg,
        )
        replacements.append(
            (
                body_start,
                body_end,
                _replace_pose_block(lines, body_start, body_end, update),
            )
        )

    for start, end, replacement in sorted(replacements, reverse=True):
        lines[start:end] = replacement
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("".join(lines), encoding="utf-8")
    return tuple(spec.observation_name for spec in selected)
