"""Public observation dataclasses for online manipulation policies."""

import dataclasses
import math
from collections.abc import Mapping, Sequence
from typing import Any


def _finite_tuple(
    values: Sequence[float],
    *,
    name: str,
    expected_size: int | None = None,
) -> tuple[float, ...]:
    """Convert a numeric sequence to a validated tuple."""
    result = tuple(float(value) for value in values)
    if expected_size is not None and len(result) != expected_size:
        raise ValueError(f"{name} must contain {expected_size} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


@dataclasses.dataclass(frozen=True)
class Pose:
    """Rigid pose represented by meters and a wxyz quaternion."""

    translation_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        """Validate pose dimensions and finite values."""
        object.__setattr__(
            self,
            "translation_m",
            _finite_tuple(
                self.translation_m,
                name="translation_m",
                expected_size=3,
            ),
        )
        quaternion = _finite_tuple(
            self.quaternion_wxyz,
            name="quaternion_wxyz",
            expected_size=4,
        )
        norm_squared = sum(value * value for value in quaternion)
        if norm_squared <= 0.0:
            raise ValueError("quaternion_wxyz must be nonzero")
        object.__setattr__(self, "quaternion_wxyz", quaternion)

    def as_dict(self) -> dict[str, list[float]]:
        """Return a JSON-compatible pose mapping."""
        return {
            "translation_m": list(self.translation_m),
            "quaternion_wxyz": list(self.quaternion_wxyz),
        }


@dataclasses.dataclass(frozen=True)
class SpatialVelocity:
    """World-frame rotational and translational spatial velocity."""

    rotational_rad_s: tuple[float, float, float]
    translational_m_s: tuple[float, float, float]

    def __post_init__(self) -> None:
        """Validate both vector components."""
        object.__setattr__(
            self,
            "rotational_rad_s",
            _finite_tuple(
                self.rotational_rad_s,
                name="rotational_rad_s",
                expected_size=3,
            ),
        )
        object.__setattr__(
            self,
            "translational_m_s",
            _finite_tuple(
                self.translational_m_s,
                name="translational_m_s",
                expected_size=3,
            ),
        )

    def as_dict(self) -> dict[str, list[float]]:
        """Return a JSON-compatible velocity mapping."""
        return {
            "rotational_rad_s": list(self.rotational_rad_s),
            "translational_m_s": list(self.translational_m_s),
        }


@dataclasses.dataclass(frozen=True)
class RobotObservation:
    """Robot state and low-level command telemetry in adapter order."""

    joint_names: tuple[str, ...]
    q: tuple[float, ...]
    v: tuple[float, ...]
    q_commanded: tuple[float, ...]
    torque_commanded: tuple[float, ...]
    torque_applied: tuple[float, ...]
    torque_saturated: tuple[bool, ...]
    end_effector_pose: Pose
    end_effector_twist: SpatialVelocity
    gripper_width_m: float | None = None

    def __post_init__(self) -> None:
        """Validate ordered joint telemetry dimensions."""
        names = tuple(str(name) for name in self.joint_names)
        if not names or len(set(names)) != len(names):
            raise ValueError("joint_names must be nonempty and unique")
        size = len(names)
        object.__setattr__(self, "joint_names", names)
        for field_name in (
            "q",
            "v",
            "q_commanded",
            "torque_commanded",
            "torque_applied",
        ):
            object.__setattr__(
                self,
                field_name,
                _finite_tuple(
                    getattr(self, field_name),
                    name=field_name,
                    expected_size=size,
                ),
            )
        saturated = tuple(bool(value) for value in self.torque_saturated)
        if len(saturated) != size:
            raise ValueError("torque_saturated must match joint_names")
        object.__setattr__(self, "torque_saturated", saturated)
        if self.gripper_width_m is not None and (
            not math.isfinite(self.gripper_width_m)
            or self.gripper_width_m < 0.0
        ):
            raise ValueError("gripper_width_m must be finite and nonnegative")

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible robot observation."""
        return {
            "joint_names": list(self.joint_names),
            "q": list(self.q),
            "v": list(self.v),
            "q_commanded": list(self.q_commanded),
            "torque_commanded": list(self.torque_commanded),
            "torque_applied": list(self.torque_applied),
            "torque_saturated": list(self.torque_saturated),
            "end_effector_pose": self.end_effector_pose.as_dict(),
            "end_effector_twist": self.end_effector_twist.as_dict(),
            "gripper_width_m": self.gripper_width_m,
        }


@dataclasses.dataclass(frozen=True)
class ObjectObservation:
    """One scene object's world pose and spatial velocity."""

    pose: Pose
    spatial_velocity: SpatialVelocity

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible object observation."""
        return {
            "pose": self.pose.as_dict(),
            "spatial_velocity": self.spatial_velocity.as_dict(),
        }


@dataclasses.dataclass(frozen=True)
class ContactObservation:
    """One contact event expressed using qualified body names."""

    body_a: str
    body_b: str
    penetration_depth_m: float

    def __post_init__(self) -> None:
        """Validate contact identifiers and depth."""
        if not self.body_a or not self.body_b:
            raise ValueError("Contact body names must be nonempty")
        if not math.isfinite(self.penetration_depth_m):
            raise ValueError("penetration_depth_m must be finite")

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible contact observation."""
        return dataclasses.asdict(self)


@dataclasses.dataclass(frozen=True)
class Observation:
    """Complete public observation returned once per policy period."""

    time_s: float
    robot: RobotObservation
    objects: Mapping[str, ObjectObservation]
    contacts: tuple[ContactObservation, ...]
    task: Mapping[str, Any]

    def __post_init__(self) -> None:
        """Validate time and generic object namespace."""
        if not math.isfinite(self.time_s) or self.time_s < 0.0:
            raise ValueError("time_s must be finite and nonnegative")
        if any(not name for name in self.objects):
            raise ValueError("Object names must be nonempty")

    def as_dict(self) -> dict[str, Any]:
        """Return the required nested policy observation mapping."""
        return {
            "time": self.time_s,
            "robot": self.robot.as_dict(),
            "objects": {
                name: observation.as_dict()
                for name, observation in self.objects.items()
            },
            "contacts": [contact.as_dict() for contact in self.contacts],
            "task": dict(self.task),
        }
