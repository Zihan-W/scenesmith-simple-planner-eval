"""Geometric conventions for the Zerith left gripper and pick task."""

import numpy as np

from src.zerith_robot_config import (
    PICK_RAIL_POSITION_METERS,
    ROBOT_BASE_XYZ_METERS as _ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)

from src.zerith_tcp import (
    LEFT_GRASP_FRAME_NAME, LEFT_GRASP_PARENT_FRAME_NAME,
    X_PARENT_GRASP_TRANSLATION_METERS, APPROACH_AXIS_GRASP,
    CLOSING_AXIS_GRASP, UP_AXIS_GRASP, add_left_grasp_frame,
)

BOX_SIZE_METERS = np.array([0.06, 0.04, 0.03])
GRASP_WIDTH_METERS = BOX_SIZE_METERS[1]
# The grasp-frame offset yields roughly 5-10 cm of collision-surface
# clearance from the calibrated box for the selected oblique approach.
PREGRASP_DISTANCE_METERS = 0.11
LIFT_DISTANCE_METERS = 0.07
ROBOT_BASE_XYZ_METERS = np.asarray(_ROBOT_BASE_XYZ_METERS, dtype=float)
