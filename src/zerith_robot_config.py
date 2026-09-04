"""Shared calibrated placement for Zerith in the pick-evaluation scene."""

# This placement was intentionally moved beside the coffee table. Keep it as
# the single default used by visualization, dynamics, IK, and task metadata.
ROBOT_BASE_XYZ_METERS = (2.65, 2.95, 0.1815)
ROBOT_BASE_YAW_DEG = 180.0

# Fixed task height selected by manual multi-view inspection. The upstream
# URDF still lacks meaningful rail effort and velocity limits, so the first
# pick task locks the rail at this position instead of actuating it.
PICK_RAIL_POSITION_METERS = 0.4
