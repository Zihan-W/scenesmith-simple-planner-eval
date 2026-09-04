"""Configuration dataclasses for generic online manipulation."""

import dataclasses
import math
from collections.abc import Mapping
from pathlib import Path

from src.online_manipulation.observations import Pose


def _positive(value: float, name: str) -> float:
    """Validate and return one positive finite scalar."""
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


@dataclasses.dataclass(frozen=True)
class TimingConfig:
    """Physics, controller, and policy periods in seconds."""

    physics_dt: float = 0.001
    controller_dt: float = 0.005
    policy_dt: float = 0.1

    def __post_init__(self) -> None:
        """Require an exact integer multi-rate schedule."""
        for name in ("physics_dt", "controller_dt", "policy_dt"):
            object.__setattr__(self, name, _positive(getattr(self, name), name))
        controller_steps = round(self.controller_dt / self.physics_dt)
        policy_steps = round(self.policy_dt / self.controller_dt)
        if not math.isclose(
            controller_steps * self.physics_dt,
            self.controller_dt,
            abs_tol=1e-12,
        ):
            raise ValueError("controller_dt must be a multiple of physics_dt")
        if not math.isclose(
            policy_steps * self.controller_dt,
            self.policy_dt,
            abs_tol=1e-12,
        ):
            raise ValueError("policy_dt must be a multiple of controller_dt")

    @property
    def physics_steps_per_controller(self) -> int:
        """Return the number of physics steps per controller update."""
        return round(self.controller_dt / self.physics_dt)

    @property
    def controller_steps_per_policy(self) -> int:
        """Return the number of controller updates per policy step."""
        return round(self.policy_dt / self.controller_dt)


@dataclasses.dataclass(frozen=True)
class VisualizationConfig:
    """Optional Meshcat and recording configuration."""

    enabled: bool = False
    port: int | None = None
    record_html_path: Path | None = None
    realtime_rate: float = 0.0

    def __post_init__(self) -> None:
        """Validate optional port and realtime rate."""
        if self.port is not None and not 1 <= self.port <= 65535:
            raise ValueError("port must be within [1, 65535]")
        if not math.isfinite(self.realtime_rate) or self.realtime_rate < 0.0:
            raise ValueError("realtime_rate must be finite and nonnegative")


@dataclasses.dataclass(frozen=True)
class ObservedBodySpec:
    """Name one Drake body exposed in the generic object observation map."""

    observation_name: str
    model_instance_name: str
    body_name: str
    write_back: bool = False

    def __post_init__(self) -> None:
        """Require stable nonempty public and Drake identifiers."""
        if not all(
            (
                self.observation_name,
                self.model_instance_name,
                self.body_name,
            )
        ):
            raise ValueError("Observed body names must be nonempty")


@dataclasses.dataclass(frozen=True)
class ScenarioSpec:
    """Scene paths and runtime options independent of any robot or task.

    The Zerith runtime currently accepts ``penetration_allowance_m`` and
    ``stiction_tolerance_m_s`` as positive Drake contact parameters. Initial
    object poses are keyed by ``ObservedBodySpec.observation_name``.
    """

    dmd_path: Path
    package_xmls: tuple[Path, ...] = ()
    initial_object_poses: Mapping[str, Pose] = dataclasses.field(
        default_factory=dict
    )
    observed_bodies: tuple[ObservedBodySpec, ...] = ()
    contact_parameters: Mapping[str, float] = dataclasses.field(
        default_factory=dict
    )
    visualization: VisualizationConfig = dataclasses.field(
        default_factory=VisualizationConfig
    )
    output_directory: Path | None = None

    def __post_init__(self) -> None:
        """Freeze path sequences and validate contact parameters."""
        object.__setattr__(self, "dmd_path", Path(self.dmd_path))
        object.__setattr__(
            self,
            "package_xmls",
            tuple(Path(path) for path in self.package_xmls),
        )
        observed_bodies = tuple(self.observed_bodies)
        observation_names = tuple(
            body.observation_name for body in observed_bodies
        )
        if len(set(observation_names)) != len(observation_names):
            raise ValueError("Observed body names must be unique")
        object.__setattr__(self, "observed_bodies", observed_bodies)
        initial_object_poses = dict(self.initial_object_poses)
        unknown_initial_poses = (
            initial_object_poses.keys() - set(observation_names)
        )
        if unknown_initial_poses:
            raise ValueError(
                "Initial object poses are not declared as observed bodies: "
                f"{sorted(unknown_initial_poses)}"
            )
        if any(
            not isinstance(pose, Pose)
            for pose in initial_object_poses.values()
        ):
            raise TypeError("Initial object poses must be Pose instances")
        object.__setattr__(
            self,
            "initial_object_poses",
            initial_object_poses,
        )
        if self.output_directory is not None:
            object.__setattr__(
                self,
                "output_directory",
                Path(self.output_directory),
            )
        contact_parameters = dict(self.contact_parameters)
        for name, value in contact_parameters.items():
            if not name or not math.isfinite(float(value)):
                raise ValueError("Contact parameters require names and values")
        object.__setattr__(self, "contact_parameters", contact_parameters)


@dataclasses.dataclass(frozen=True)
class JointSpec:
    """Robot joint limits and acceleration-domain servo gains.

    Position and velocity use the joint's URDF coordinate unit: radians for
    revolute joints and meters for prismatic joints. Effort is N*m or N.
    """

    name: str
    joint_type: str
    position_lower: float
    position_upper: float
    velocity_limit: float
    effort_limit: float
    kp: float
    kd: float

    def __post_init__(self) -> None:
        """Validate one controlled-joint contract."""
        if not self.name:
            raise ValueError("JointSpec name must be nonempty")
        if self.joint_type not in ("revolute", "prismatic"):
            raise ValueError("joint_type must be revolute or prismatic")
        finite_values = (
            self.position_lower,
            self.position_upper,
            self.velocity_limit,
            self.effort_limit,
            self.kp,
            self.kd,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise ValueError("JointSpec values must be finite")
        if self.position_lower >= self.position_upper:
            raise ValueError("position_lower must be below position_upper")
        if self.velocity_limit <= 0.0 or self.effort_limit <= 0.0:
            raise ValueError("velocity and effort limits must be positive")
        if self.kp <= 0.0 or self.kd < 0.0:
            raise ValueError("kp must be positive and kd nonnegative")


@dataclasses.dataclass(frozen=True)
class GripperSpec:
    """Parallel-gripper joint mapping and physical width range."""

    joint_names: tuple[str, ...]
    minimum_width_m: float
    maximum_width_m: float

    def __post_init__(self) -> None:
        """Validate gripper names and width bounds."""
        names = tuple(str(name) for name in self.joint_names)
        if not names or any(not name for name in names):
            raise ValueError("Gripper joint_names must be nonempty")
        if len(set(names)) != len(names):
            raise ValueError("Gripper joint_names must be unique")
        if (
            not math.isfinite(self.minimum_width_m)
            or not math.isfinite(self.maximum_width_m)
            or self.minimum_width_m < 0.0
            or self.minimum_width_m >= self.maximum_width_m
        ):
            raise ValueError("Invalid gripper width limits")
        object.__setattr__(self, "joint_names", names)


@dataclasses.dataclass(frozen=True)
class RobotSpec:
    """Robot description consumed by a RobotAdapter implementation."""

    name: str
    model_instance_name: str
    package_name: str
    model_path: Path
    base_link_name: str
    base_pose: Pose
    controlled_joints: tuple[JointSpec, ...]
    locked_joint_positions: Mapping[str, float]
    end_effector_frame_name: str
    home_positions: tuple[float, ...]
    gripper: GripperSpec | None = None
    safety_exempt_body_pairs: tuple[tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        """Validate robot names, mappings, and ordered home positions."""
        required_names = (
            self.name,
            self.model_instance_name,
            self.package_name,
            self.base_link_name,
            self.end_effector_frame_name,
        )
        if any(not value for value in required_names):
            raise ValueError("RobotSpec names must be nonempty")
        object.__setattr__(self, "model_path", Path(self.model_path))
        joints = tuple(self.controlled_joints)
        joint_names = tuple(joint.name for joint in joints)
        if not joints or len(set(joint_names)) != len(joint_names):
            raise ValueError("controlled_joints must be nonempty and unique")
        home = tuple(float(value) for value in self.home_positions)
        if len(home) != len(joints) or not all(
            math.isfinite(value) for value in home
        ):
            raise ValueError("home_positions must match controlled_joints")
        for joint, value in zip(joints, home, strict=True):
            if not joint.position_lower <= value <= joint.position_upper:
                raise ValueError(f"Home position violates {joint.name} limits")
        controlled_and_gripper = set(joint_names)
        if self.gripper is not None:
            controlled_and_gripper.update(self.gripper.joint_names)
        overlap = controlled_and_gripper.intersection(
            self.locked_joint_positions
        )
        if overlap:
            raise ValueError(f"Controlled and locked joints overlap: {overlap}")
        if any(
            not name or not math.isfinite(float(value))
            for name, value in self.locked_joint_positions.items()
        ):
            raise ValueError("Locked joints require names and finite values")
        safety_pairs = tuple(
            tuple(sorted(pair)) for pair in self.safety_exempt_body_pairs
        )
        if any(
            len(pair) != 2 or not pair[0] or not pair[1]
            for pair in safety_pairs
        ):
            raise ValueError("Safety exemptions require qualified body pairs")
        object.__setattr__(self, "controlled_joints", joints)
        object.__setattr__(self, "home_positions", home)
        object.__setattr__(self, "safety_exempt_body_pairs", safety_pairs)

    @property
    def controlled_joint_names(self) -> tuple[str, ...]:
        """Return controlled joint names in public action order."""
        return tuple(joint.name for joint in self.controlled_joints)
