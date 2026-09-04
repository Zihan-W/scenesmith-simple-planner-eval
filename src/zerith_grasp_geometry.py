"""Geometric conventions for the Zerith left gripper and pick task."""

import numpy as np

LEFT_GRASP_FRAME_NAME = "left_grasp_frame"
LEFT_GRASP_PARENT_FRAME_NAME = "left_wrist_pitch_link"

# The frame origin is centered between the inner finger contact surfaces.
# +X points outward through the fingers (approach), +Y is the closing axis,
# and +Z is the gripper's local upward direction.
X_PARENT_GRASP_TRANSLATION_METERS = np.array([0.176, 0.0, -0.002])
APPROACH_AXIS_GRASP = np.array([1.0, 0.0, 0.0])
CLOSING_AXIS_GRASP = np.array([0.0, 1.0, 0.0])
UP_AXIS_GRASP = np.array([0.0, 0.0, 1.0])

BOX_SIZE_METERS = np.array([0.06, 0.04, 0.03])
GRASP_WIDTH_METERS = BOX_SIZE_METERS[1]
PREGRASP_DISTANCE_METERS = 0.09
LIFT_DISTANCE_METERS = 0.07
ROBOT_BASE_XYZ_METERS = np.array([2.65, 2.95, 0.1815])
ROBOT_BASE_YAW_DEG = 180.0


def add_left_grasp_frame(plant, zerith_model_instance):
    """Add and return the fixed left-gripper grasp frame before Finalize()."""
    from pydrake.all import FixedOffsetFrame, RigidTransform

    parent_frame = plant.GetFrameByName(
        LEFT_GRASP_PARENT_FRAME_NAME,
        zerith_model_instance,
    )
    return plant.AddFrame(
        FixedOffsetFrame(
            LEFT_GRASP_FRAME_NAME,
            parent_frame,
            RigidTransform(X_PARENT_GRASP_TRANSLATION_METERS),
        )
    )
