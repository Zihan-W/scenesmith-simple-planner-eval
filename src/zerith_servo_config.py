"""Zerith model-specific servo defaults; independent of environment imports."""

import dataclasses
from pathlib import Path

from src.zerith_gripper_config import GRIPPER_MAX_OPENING_M

ZERITH_PACKAGE_NAME = "zerith_drake"
ZERITH_URDF_RELATIVE_PATH = Path("urdf/zerith_drake.urdf")
# Minimum separation of the CAD-derived distal finger envelopes at q=0.
# This calibrates simulation width, not the advertised hardware stroke.
GRIPPER_MAX_OPENING = GRIPPER_MAX_OPENING_M
SUPPORTED_CONTACT_PARAMETERS = frozenset(
    ("penetration_allowance_m", "stiction_tolerance_m_s")
)


@dataclasses.dataclass(frozen=True)
class JointServoConfig:
    """Acceleration-domain PD gains and actuator torque limit."""

    name: str
    kp: float
    kd: float
    effort_limit: float


LEFT_ARM_SERVO_CONFIGS = (
    JointServoConfig("left_shoulder_pitch_joint", 320.0, 36.0, 36.0),
    JointServoConfig("left_shoulder_roll_joint", 320.0, 36.0, 36.0),
    JointServoConfig("left_shoulder_yaw_joint", 160.0, 26.0, 27.0),
    JointServoConfig("left_elbow_joint", 240.0, 32.0, 27.0),
    JointServoConfig("left_wrist_roll_joint", 1000.0, 64.0, 9.0),
    JointServoConfig("left_wrist_yaw_joint", 1000.0, 64.0, 9.0),
    JointServoConfig("left_wrist_pitch_joint", 1000.0, 64.0, 9.0),
)
LEFT_GRIPPER_SERVO_CONFIGS = (
    JointServoConfig("left_jaw_left_finger_joint", 2500.0, 100.0, 25.0),
    JointServoConfig("left_jaw_right_finger_joint", 2500.0, 100.0, 25.0),
)
ALL_SERVO_CONFIGS = LEFT_ARM_SERVO_CONFIGS + LEFT_GRIPPER_SERVO_CONFIGS
