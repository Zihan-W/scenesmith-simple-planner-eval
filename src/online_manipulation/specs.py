"""Configuration dataclasses for generic online manipulation."""

import dataclasses
import math
from collections.abc import Mapping
from pathlib import Path

from src.online_manipulation.observations import CameraIntrinsics, Pose


def _positive(value: float, name: str) -> float:
    """Validate and return one positive finite scalar."""
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def _normalized_quaternion(
    quaternion_wxyz: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Return a unit quaternion for rigid-transform composition."""
    norm = math.sqrt(sum(value * value for value in quaternion_wxyz))
    return tuple(value / norm for value in quaternion_wxyz)


def _compose_poses(X_AB: Pose, X_BC: Pose) -> Pose:
    """Compose two public rigid poses without depending on Drake types."""
    aw, ax, ay, az = _normalized_quaternion(X_AB.quaternion_wxyz)
    bw, bx, by, bz = _normalized_quaternion(X_BC.quaternion_wxyz)
    quaternion = (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )
    x, y, z = X_BC.translation_m
    tx = 2.0 * (ay * z - az * y)
    ty = 2.0 * (az * x - ax * z)
    tz = 2.0 * (ax * y - ay * x)
    rotated = (
        x + aw * tx + ay * tz - az * ty,
        y + aw * ty + az * tx - ax * tz,
        z + aw * tz + ax * ty - ay * tx,
    )
    return Pose(
        tuple(
            parent + offset
            for parent, offset in zip(
                X_AB.translation_m,
                rotated,
                strict=True,
            )
        ),
        quaternion,
    )


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
class RendererSpec:
    """Scenario-owned render engine selection for robot-mounted sensors."""

    name: str = "online_environment_renderer"
    engine: str = "vtk"

    def __post_init__(self) -> None:
        """Validate the stable renderer name and supported engine."""
        if not self.name:
            raise ValueError("Renderer name must be nonempty")
        if self.engine != "vtk":
            raise ValueError("Only the vtk renderer is currently supported")


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
class PlanarPoseRandomizationSpec:
    """Reset-time world-X/Y and yaw offsets for one observed free body."""

    observation_name: str
    x_offset_range_m: tuple[float, float] = (0.0, 0.0)
    y_offset_range_m: tuple[float, float] = (0.0, 0.0)
    yaw_offset_range_rad: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self) -> None:
        """Validate finite, ordered offset ranges."""
        if not self.observation_name:
            raise ValueError("Randomized observation_name must be nonempty")
        for name in (
            "x_offset_range_m",
            "y_offset_range_m",
            "yaw_offset_range_rad",
        ):
            values = tuple(float(value) for value in getattr(self, name))
            if len(values) != 2 or not all(
                math.isfinite(value) for value in values
            ):
                raise ValueError(f"{name} must contain two finite values")
            if values[0] > values[1]:
                raise ValueError(f"{name} lower bound exceeds upper bound")
            object.__setattr__(self, name, values)


@dataclasses.dataclass(frozen=True)
class ScenarioSpec:
    """Scene paths and runtime options independent of any robot or task.

    The current Drake runtime accepts ``penetration_allowance_m`` and
    ``stiction_tolerance_m_s`` as positive Drake contact parameters. Initial
    object poses are keyed by ``ObservedBodySpec.observation_name``.
    """

    dmd_path: Path
    package_xmls: tuple[Path, ...] = ()
    initial_object_poses: Mapping[str, Pose] = dataclasses.field(
        default_factory=dict
    )
    observed_bodies: tuple[ObservedBodySpec, ...] = ()
    pose_randomizations: tuple[PlanarPoseRandomizationSpec, ...] = ()
    contact_parameters: Mapping[str, float] = dataclasses.field(
        default_factory=dict
    )
    visualization: VisualizationConfig = dataclasses.field(
        default_factory=VisualizationConfig
    )
    renderer: RendererSpec = dataclasses.field(default_factory=RendererSpec)
    output_directory: Path | None = None
    ground_body_names: tuple[str, ...] = ()

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
        randomizations = tuple(self.pose_randomizations)
        randomized_names = tuple(
            randomization.observation_name
            for randomization in randomizations
        )
        if len(set(randomized_names)) != len(randomized_names):
            raise ValueError("Pose randomization names must be unique")
        unknown_randomizations = set(randomized_names) - set(
            observation_names
        )
        if unknown_randomizations:
            raise ValueError(
                "Pose randomizations are not declared as observed bodies: "
                f"{sorted(unknown_randomizations)}"
            )
        object.__setattr__(self, "pose_randomizations", randomizations)
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
    contact_body_names: tuple[str, ...] = ()

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
class CameraSpec:
    """Robot-owned mount-to-optical geometry and simulation imaging data."""

    name: str
    parent_frame: str
    X_parent_camera_mount: Pose
    X_mount_camera_optical: Pose
    width: int = 320
    height: int = 240
    fov_y_rad: float = math.radians(60.0)
    near_m: float = 0.05
    far_m: float = 10.0
    update_period_s: float = 0.05
    modalities: tuple[str, ...] = ("rgb", "depth", "label")
    enabled: bool = True

    def __post_init__(self) -> None:
        """Validate one portable camera declaration."""
        if not self.name or not self.parent_frame:
            raise ValueError("Camera name and parent_frame must be nonempty")
        if self.width < 1 or self.height < 1:
            raise ValueError("Camera dimensions must be positive")
        scalar_values = (
            self.fov_y_rad,
            self.near_m,
            self.far_m,
            self.update_period_s,
        )
        if not all(math.isfinite(value) for value in scalar_values):
            raise ValueError("Camera scalar values must be finite")
        if not 0.0 < self.fov_y_rad < math.pi:
            raise ValueError("Camera fov_y_rad must lie within (0, pi)")
        if self.near_m <= 0.0 or self.near_m >= self.far_m:
            raise ValueError("Camera range must satisfy 0 < near < far")
        if self.update_period_s <= 0.0:
            raise ValueError("Camera update_period_s must be positive")
        modalities = tuple(self.modalities)
        allowed = frozenset(("rgb", "depth", "label"))
        if not modalities or len(set(modalities)) != len(modalities):
            raise ValueError("Camera modalities must be nonempty and unique")
        unknown = set(modalities) - allowed
        if unknown:
            raise ValueError(f"Unsupported camera modalities: {sorted(unknown)}")
        object.__setattr__(self, "modalities", modalities)

    @property
    def X_parent_camera_optical(self) -> Pose:
        """Return the composed parent-frame to Drake optical-frame pose."""
        return _compose_poses(
            self.X_parent_camera_mount,
            self.X_mount_camera_optical,
        )

    @property
    def X_parent_camera(self) -> Pose:
        """Return the final optical pose using the v0.2-compatible name."""
        return self.X_parent_camera_optical

    @property
    def intrinsics(self) -> CameraIntrinsics:
        """Return the configured simulation pinhole calibration."""
        focal = 0.5 * self.height / math.tan(0.5 * self.fov_y_rad)
        return CameraIntrinsics(
            width=self.width,
            height=self.height,
            focal_x_px=focal,
            focal_y_px=focal,
            center_x_px=0.5 * self.width - 0.5,
            center_y_px=0.5 * self.height - 0.5,
            fov_y_rad=self.fov_y_rad,
            near_m=self.near_m,
            far_m=self.far_m,
        )


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
    cameras: tuple[CameraSpec, ...] = ()
    arm_groups: Mapping[str, tuple[str, ...]] = dataclasses.field(default_factory=dict)
    end_effector_frames: Mapping[str, str] = dataclasses.field(default_factory=dict)
    grippers: Mapping[str, GripperSpec] = dataclasses.field(default_factory=dict)

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
        cameras = tuple(self.cameras)
        if any(not isinstance(camera, CameraSpec) for camera in cameras):
            raise TypeError("RobotSpec cameras must be CameraSpec instances")
        camera_names = tuple(camera.name for camera in cameras)
        if len(set(camera_names)) != len(camera_names):
            raise ValueError("RobotSpec camera names must be unique")
        object.__setattr__(self, "cameras", cameras)
        groups = {name: tuple(members) for name, members in self.arm_groups.items()}
        assigned = [joint for members in groups.values() for joint in members]
        if any(not name or not members for name, members in groups.items()):
            raise ValueError("Arm groups require a name and at least one joint")
        if len(assigned) != len(set(assigned)) or not set(assigned).issubset(joint_names):
            raise ValueError("Arm groups must contain unique controlled joints")
        if set(self.end_effector_frames) != set(groups):
            raise ValueError("Each named arm must declare exactly one end-effector frame")
        gripper_joints = [joint for gripper in self.grippers.values() for joint in gripper.joint_names]
        if (len(gripper_joints) != len(set(gripper_joints))
                or not set(gripper_joints).issubset(joint_names)
                or set(gripper_joints).intersection(assigned)):
            raise ValueError("Named gripper joints must be controlled and disjoint from arms")
        object.__setattr__(self, "arm_groups", groups)
        object.__setattr__(self, "end_effector_frames", dict(self.end_effector_frames))
        object.__setattr__(self, "grippers", dict(self.grippers))
        object.__setattr__(self, "locked_joint_positions", dict(self.locked_joint_positions))

    @property
    def controlled_joint_names(self) -> tuple[str, ...]:
        """Return controlled joint names in public action order."""
        return tuple(joint.name for joint in self.controlled_joints)
