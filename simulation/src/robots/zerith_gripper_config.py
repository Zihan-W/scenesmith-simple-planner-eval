"""Geometric calibration of Zerith's simulated parallel fingers.

The source CAD's minimum distal separation is approximately 76.2889 mm.
Rounding inward travel down to 38.14 mm leaves less than 10 micrometers of
clearance at empty closure, avoiding coincident convex surfaces. Public width
is measured relative to this geometric stop (not a hardware specification).
"""

FINGER_CLOSING_TRAVEL_M = 0.03814
GRIPPER_MAX_OPENING_M = 2.0 * FINGER_CLOSING_TRAVEL_M
