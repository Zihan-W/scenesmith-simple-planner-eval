"""Typed policy actions for the online manipulation environment."""

import dataclasses
import math
from collections.abc import Sequence


def _finite_values(
    values: Sequence[float],
    *,
    name: str,
    expected_size: int | None = None,
) -> tuple[float, ...]:
    """Convert a numeric sequence to a finite immutable tuple."""
    result = tuple(float(value) for value in values)
    if expected_size is not None and len(result) != expected_size:
        raise ValueError(f"{name} must contain {expected_size} values")
    if not result:
        raise ValueError(f"{name} must not be empty")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def _joint_names(values: Sequence[str]) -> tuple[str, ...]:
    """Validate and freeze an ordered joint-name sequence."""
    names = tuple(str(value) for value in values)
    if not names or any(not name for name in names):
        raise ValueError("joint_names must contain nonempty names")
    if len(set(names)) != len(names):
        raise ValueError("joint_names must be unique")
    return names


@dataclasses.dataclass(frozen=True)
class HoldAction:
    """Keep the current arm and gripper command targets unchanged."""


@dataclasses.dataclass(frozen=True)
class JointPositionAction:
    """Set named joint positions in URDF units (radians or meters)."""

    joint_names: tuple[str, ...]
    positions: tuple[float, ...]

    def __post_init__(self) -> None:
        """Validate names and position values."""
        names = _joint_names(self.joint_names)
        positions = _finite_values(self.positions, name="positions")
        if len(names) != len(positions):
            raise ValueError("joint_names and positions must have equal length")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "positions", positions)


@dataclasses.dataclass(frozen=True)
class JointDeltaAction:
    """Increment named joint targets in URDF units (radians or meters)."""

    joint_names: tuple[str, ...]
    deltas: tuple[float, ...]

    def __post_init__(self) -> None:
        """Validate names and delta values."""
        names = _joint_names(self.joint_names)
        deltas = _finite_values(self.deltas, name="deltas")
        if len(names) != len(deltas):
            raise ValueError("joint_names and deltas must have equal length")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "deltas", deltas)


@dataclasses.dataclass(frozen=True)
class CartesianDeltaAction:
    """Increment an end-effector pose in an explicitly named frame.

    Translation is measured in meters. ``rotation_vector_rad`` is an
    axis-angle vector whose direction is the rotation axis and whose norm is
    the rotation angle in radians.
    """

    end_effector_frame: str
    reference_frame: str
    translation_m: tuple[float, float, float]
    rotation_vector_rad: tuple[float, float, float]

    def __post_init__(self) -> None:
        """Validate frame names and six-dimensional pose delta."""
        if not self.end_effector_frame or not self.reference_frame:
            raise ValueError("Cartesian action frame names must be nonempty")
        object.__setattr__(
            self,
            "translation_m",
            _finite_values(
                self.translation_m,
                name="translation_m",
                expected_size=3,
            ),
        )
        object.__setattr__(
            self,
            "rotation_vector_rad",
            _finite_values(
                self.rotation_vector_rad,
                name="rotation_vector_rad",
                expected_size=3,
            ),
        )


@dataclasses.dataclass(frozen=True)
class GripperAction:
    """Set the physical gripper opening width in meters."""

    width_m: float
    maximum_effort_n: float | None = None

    def __post_init__(self) -> None:
        """Validate gripper width and optional effort limit."""
        if not math.isfinite(self.width_m) or self.width_m < 0.0:
            raise ValueError("width_m must be finite and nonnegative")
        if self.maximum_effort_n is not None and (
            not math.isfinite(self.maximum_effort_n)
            or self.maximum_effort_n <= 0.0
        ):
            raise ValueError("maximum_effort_n must be finite and positive")


ArmAction = (
    HoldAction
    | JointPositionAction
    | JointDeltaAction
    | CartesianDeltaAction
)


@dataclasses.dataclass(frozen=True)
class CompositeAction:
    """Combine at most one arm action with at most one gripper action."""

    arm: ArmAction | None = None
    gripper: GripperAction | None = None

    def __post_init__(self) -> None:
        """Require at least one command component."""
        if self.arm is None and self.gripper is None:
            raise ValueError("CompositeAction must contain a command")


RobotAction = ArmAction | GripperAction | CompositeAction
