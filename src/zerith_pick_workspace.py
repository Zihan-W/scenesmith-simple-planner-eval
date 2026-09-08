"""Task-space constraints for Zerith pick-home posture selection."""

import dataclasses

from typing import Any

import numpy as np

from src.geometry_bounds import (
    BoundingBox,
    _CalcAabb,
    _MergeAabbs,
    _OabbToAabb,
)


LEFT_WRIST_GRIPPER_BODY_NAMES = (
    "left_wrist_roll_link",
    "left_wrist_yaw_link",
    "left_wrist_pitch_link",
    "left_end_effector_link",
    "left_jaw_camera_link",
    "left_jaw_left_finger_link",
    "left_jaw_right_finger_link",
)


def _bounds(box: BoundingBox) -> tuple[np.ndarray, np.ndarray]:
    """Return minimum and maximum corners for one axis-aligned box."""
    half_size = 0.5 * box.size
    return box.center - half_size, box.center + half_size


@dataclasses.dataclass
class PickWorkspaceEvaluator:
    """Evaluate whole-wrist clearance from the task's support table.

    World-axis-aligned bounds are conservative for rotated geometry. The
    outside-table test is the Minkowski expansion of the table projection by
    the current wrist/gripper half-size plus the requested safety margin.
    Therefore it does not rely on the grasp-frame point being outside while
    another part of the wrist remains under the table.
    """

    model: Any
    table_model_name: str
    table_body_name: str = "base_link"

    def __post_init__(self) -> None:
        """Resolve collision bodies and cache their body-frame AABBs."""
        plant = self.model.plant
        table_instance = plant.GetModelInstanceByName(self.table_model_name)
        self._table_body = plant.GetBodyByName(
            self.table_body_name,
            table_instance,
        )
        self._wrist_bodies = tuple(
            plant.GetBodyByName(name, self.model.zerith)
            for name in LEFT_WRIST_GRIPPER_BODY_NAMES
        )
        query = plant.get_geometry_query_input_port().Eval(
            self.model.plant_context
        )
        self._inspector = query.inspector()
        bodies = (self._table_body,) + self._wrist_bodies
        self._body_frame_aabbs = {
            body.index(): _CalcAabb(self._inspector, body) for body in bodies
        }

    def _world_aabb(self, body) -> BoundingBox:
        """Transform a cached collision AABB into the world frame."""
        X_WB = self.model.plant.EvalBodyPoseInWorld(
            self.model.plant_context,
            body,
        )
        return _OabbToAabb(
            self._body_frame_aabbs[body.index()],
            X_WB,
        )

    def evaluate(self, q: np.ndarray, safety_margin: float) -> dict:
        """Return table-workspace diagnostics for a full configuration."""
        if safety_margin < 0.0:
            raise ValueError("safety_margin must be nonnegative")
        self.model.plant.SetPositions(self.model.plant_context, q)
        table_aabb = self._world_aabb(self._table_body)
        wrist_aabb = _MergeAabbs(
            [self._world_aabb(body) for body in self._wrist_bodies]
        )
        table_min, table_max = _bounds(table_aabb)
        wrist_min, wrist_max = _bounds(wrist_aabb)

        vertical_clearance = float(wrist_min[2] - table_max[2])
        above_table = vertical_clearance >= safety_margin

        table_half_xy = 0.5 * table_aabb.size[:2]
        wrist_half_xy = 0.5 * wrist_aabb.size[:2]
        expansion_xy = wrist_half_xy + safety_margin
        center_offset_xy = np.abs(
            wrist_aabb.center[:2] - table_aabb.center[:2]
        )
        outside_axis_clearances = (
            center_offset_xy
            - table_half_xy
            - wrist_half_xy
        )
        outside_expanded_xy = bool(
            np.any(outside_axis_clearances >= safety_margin)
        )
        return {
            "satisfied": bool(above_table or outside_expanded_xy),
            "whole_wrist_gripper_above_table": bool(above_table),
            "whole_wrist_gripper_outside_expanded_xy_projection": (
                outside_expanded_xy
            ),
            "vertical_clearance_above_table_m": vertical_clearance,
            "outside_axis_clearances_m": outside_axis_clearances.tolist(),
            "required_safety_margin_m": safety_margin,
            "table_xy_expansion_for_wrist_gripper_m": (
                expansion_xy.tolist()
            ),
            "table_aabb_min_xyz_m": table_min.tolist(),
            "table_aabb_max_xyz_m": table_max.tolist(),
            "wrist_gripper_aabb_min_xyz_m": wrist_min.tolist(),
            "wrist_gripper_aabb_max_xyz_m": wrist_max.tolist(),
            "wrist_gripper_body_names": list(
                LEFT_WRIST_GRIPPER_BODY_NAMES
            ),
            "bounding_volume": (
                "conservative world-axis-aligned collision-geometry AABB"
            ),
        }


def edge_workspace_metrics(
    evaluator: PickWorkspaceEvaluator,
    q_start: np.ndarray,
    q_end: np.ndarray,
    safety_margin: float,
    max_joint_step: float,
) -> dict:
    """Numerically verify the workspace constraint along a joint edge."""
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
    first_unsatisfied_alpha = None
    minimum_vertical_clearance = float("inf")
    minimum_best_outside_clearance = float("inf")
    for alpha in np.linspace(0.0, 1.0, sample_count):
        q = q_start + alpha * (q_end - q_start)
        metrics = evaluator.evaluate(q, safety_margin)
        if not metrics["satisfied"] and first_unsatisfied_alpha is None:
            first_unsatisfied_alpha = float(alpha)
        minimum_vertical_clearance = min(
            minimum_vertical_clearance,
            metrics["vertical_clearance_above_table_m"],
        )
        minimum_best_outside_clearance = min(
            minimum_best_outside_clearance,
            max(metrics["outside_axis_clearances_m"]),
        )
    return {
        "workspace_constraint_satisfied": first_unsatisfied_alpha is None,
        "first_unsatisfied_alpha": first_unsatisfied_alpha,
        "minimum_vertical_clearance_above_table_m": (
            minimum_vertical_clearance
        ),
        "minimum_best_outside_axis_clearance_m": (
            minimum_best_outside_clearance
        ),
        "sample_count": sample_count,
        "maximum_joint_sample_step_rad": max_joint_step,
        "validation_kind": "high_density_numerical_joint_edge_sampling",
    }
