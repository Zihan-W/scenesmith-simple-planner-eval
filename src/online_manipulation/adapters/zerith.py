"""Zerith-specific model, joint, gripper, and legacy-runtime adapter."""

from __future__ import annotations

import copy
import dataclasses
import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np
from pydrake.all import (
    Meshcat,
    RigidTransform,
    Role,
    RollPitchYaw,
)

from src.online_manipulation.actions import (
    CartesianDeltaAction,
    CartesianPoseAction,
    CompositeAction,
    GripperAction,
    HoldAction,
    JointDeltaAction,
    JointPositionAction,
    RobotAction,
)
from src.online_manipulation.environment import OnlineManipulationEnv
from src.online_manipulation.dmd_finalizer import write_updated_dmd
from src.online_manipulation.observations import (
    ContactObservation,
    ObjectObservation,
    Observation,
    Pose,
    RobotObservation,
    SpatialVelocity,
)
from src.online_manipulation.planning import (
    PlanningQuery,
    build_planning_query,
)
from src.online_manipulation.protocols import ContactPolicy, Task
from src.online_manipulation.tasks import NullTask
from src.online_manipulation.specs import (
    CameraSpec,
    GripperSpec,
    JointSpec,
    ObservedBodySpec,
    RobotSpec,
    ScenarioSpec,
    TimingConfig,
)
from src.zerith_servo_config import (
    ALL_SERVO_CONFIGS,
    GRIPPER_MAX_OPENING,
    LEFT_ARM_SERVO_CONFIGS,
    LEFT_GRIPPER_SERVO_CONFIGS,
    ZERITH_PACKAGE_NAME,
    ZERITH_URDF_RELATIVE_PATH,
)
from src.zerith_tcp import (
    LEFT_GRASP_FRAME_NAME,
    add_left_grasp_frame,
)

_BASE_LINK_NAME = "dipan_link"
_END_EFFECTOR_FRAME_NAME = LEFT_GRASP_FRAME_NAME
_RAIL_JOINT_NAME = "daogui_joint"


def _drake_pose(transform: RigidTransform) -> Pose:
    """Convert a Drake transform to the public pose representation."""
    return Pose(
        tuple(float(value) for value in transform.translation()),
        tuple(
            float(value)
            for value in transform.rotation().ToQuaternion().wxyz()
        ),
    )


def _yaw_pose(xyz: Sequence[float], yaw_deg: float) -> Pose:
    """Build a public base pose from world translation and yaw."""
    translation = tuple(float(value) for value in xyz)
    if len(translation) != 3:
        raise ValueError("robot_xyz must contain exactly three values")
    if not all(math.isfinite(value) for value in translation):
        raise ValueError("robot_xyz must contain finite values")
    if not math.isfinite(yaw_deg):
        raise ValueError("robot_yaw_deg must be finite")
    half_yaw = 0.5 * math.radians(yaw_deg)
    return Pose(
        translation,
        (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)),
    )


def _xyz_rpy_pose(
    xyz: Sequence[float],
    rpy: Sequence[float],
) -> Pose:
    """Build a public pose from one URDF xyz/rpy fixed-joint origin."""
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError("xyz and rpy must each contain three values")
    return _drake_pose(
        RigidTransform(
            RollPitchYaw(*tuple(float(value) for value in rpy)),
            tuple(float(value) for value in xyz),
        )
    )




def _joint_elements(urdf_path: Path) -> dict[str, ET.Element]:
    """Return all non-fixed URDF joints keyed by name."""
    root = ET.parse(urdf_path).getroot()
    return {
        joint.attrib["name"]: joint
        for joint in root.findall("joint")
        if joint.attrib["type"] != "fixed"
    }


def _joint_spec(
    joint: ET.Element,
    *,
    kp: float,
    kd: float,
    effort_limit: float,
) -> JointSpec:
    """Build one controlled-joint specification from the generated URDF."""
    limit = joint.find("limit")
    if limit is None:
        raise ValueError(f"Joint {joint.attrib['name']} has no limit element")
    joint_type = joint.attrib["type"]
    if joint_type not in ("revolute", "prismatic"):
        raise ValueError(
            f"Controlled joint {joint.attrib['name']} has type {joint_type}"
        )
    return JointSpec(
        name=joint.attrib["name"],
        joint_type=joint_type,
        position_lower=float(limit.attrib["lower"]),
        position_upper=float(limit.attrib["upper"]),
        velocity_limit=float(limit.attrib["velocity"]),
        effort_limit=float(effort_limit),
        kp=float(kp),
        kd=float(kd),
    )


def make_zerith_robot_spec(
    *,
    robot_model_dir: Path,
    robot_xyz: Sequence[float],
    robot_yaw_deg: float,
    rail_position: float,
    q_home_left: Sequence[float],
    cameras: Sequence[CameraSpec] = (),
    locked_joint_position_overrides: Mapping[str, float] | None = None,
) -> RobotSpec:
    """Create the calibrated fixed-rail, left-arm Zerith specification."""
    model_dir = Path(robot_model_dir).resolve()
    urdf_path = model_dir / ZERITH_URDF_RELATIVE_PATH
    if not urdf_path.is_file():
        raise FileNotFoundError(
            f"Zerith URDF does not exist: {urdf_path}\n"
            "Run python scripts/convert_zerith_for_drake.py."
        )
    q_home = tuple(float(value) for value in q_home_left)
    if len(q_home) != len(LEFT_ARM_SERVO_CONFIGS):
        raise ValueError("q_home_left must contain seven joint positions")

    joints_by_name = _joint_elements(urdf_path)
    controlled_joints = tuple(
        _joint_spec(
            joints_by_name[config.name],
            kp=config.kp,
            kd=config.kd,
            effort_limit=config.effort_limit,
        )
        for config in ALL_SERVO_CONFIGS
    )
    controlled_names = {joint.name for joint in controlled_joints}
    locked_positions = {
        name: rail_position if name == _RAIL_JOINT_NAME else 0.0
        for name in joints_by_name
        if name not in controlled_names
    }
    overrides = {
        str(name): float(value)
        for name, value in (locked_joint_position_overrides or {}).items()
    }
    unknown_overrides = overrides.keys() - locked_positions.keys()
    if unknown_overrides:
        raise ValueError(
            "Locked-joint overrides do not name locked Zerith joints: "
            f"{sorted(unknown_overrides)}"
        )
    for name, value in overrides.items():
        limit = joints_by_name[name].find("limit")
        if limit is None:
            raise ValueError(f"Locked joint {name} has no limits")
        if (
            not math.isfinite(value)
            or value < float(limit.attrib["lower"])
            or value > float(limit.attrib["upper"])
        ):
            raise ValueError(f"Locked-joint override violates {name} limits")
    locked_positions.update(overrides)
    root = ET.parse(urdf_path).getroot()
    model_instance_name = root.attrib["name"]
    gripper = GripperSpec(
        joint_names=tuple(config.name for config in LEFT_GRIPPER_SERVO_CONFIGS),
        minimum_width_m=0.0,
        maximum_width_m=GRIPPER_MAX_OPENING,
        contact_body_names=(
            "left_jaw_left_finger_link", "left_jaw_right_finger_link",
        ),
    )
    return RobotSpec(
        name="zerith_left_arm",
        model_instance_name=model_instance_name,
        package_name=ZERITH_PACKAGE_NAME,
        model_path=urdf_path,
        base_link_name=_BASE_LINK_NAME,
        base_pose=_yaw_pose(robot_xyz, robot_yaw_deg),
        controlled_joints=controlled_joints,
        locked_joint_positions=locked_positions,
        end_effector_frame_name=_END_EFFECTOR_FRAME_NAME,
        home_positions=q_home + (0.0, 0.0),
        gripper=gripper,
        safety_exempt_body_pairs=(
            (
                f"{model_instance_name}::body_yaw_link",
                f"{model_instance_name}::left_shoulder_roll_link",
            ),
        ),
        cameras=tuple(cameras),
        arm_groups={"left": tuple(config.name for config in LEFT_ARM_SERVO_CONFIGS)},
        end_effector_frames={"left": _END_EFFECTOR_FRAME_NAME},
        grippers={"left": gripper},
    )


def make_zerith_camera_specs(
    *,
    enabled_names: Sequence[str] = (),
    width: int = 320,
    height: int = 240,
    fov_y_rad: float = math.radians(60.0),
    near_m: float = 0.05,
    far_m: float = 10.0,
    update_period_s: float = 0.05,
    modalities: Sequence[str] = ("rgb", "depth", "label"),
) -> tuple[CameraSpec, ...]:
    """Return Zerith camera mounts with explicit simulation intrinsics.

    The upstream URDF provides three rigid camera links but no sensor
    intrinsics or update rates. These values therefore describe simulation
    cameras and are not asserted to match the physical robot.
    """
    # Each entry is copied from the corresponding fixed joint in the upstream
    # URDF: public name, direct parent link, xyz, and rpy. The child camera-link
    # axes already follow Drake's optical convention. That explicit identity
    # mount-to-optical transform is verified by projection tests; it is not a
    # default assumption for arbitrary CameraSpec instances.
    mounts = (
        (
            "left_wrist_camera",
            "left_wrist_pitch_link",
            (0.11933, 0.009, 0.060373),
            (-2.0071, 0.0, -1.5708),
        ),
        (
            "right_wrist_camera",
            "right_wrist_pitch_link",
            (0.11933, 0.0090006, 0.060373),
            (-2.0071, 0.0, -1.5708),
        ),
        (
            "head_camera",
            "neck_pitch_link",
            (0.0675568573382885, 0.0324999999999979, -0.0363332072227294),
            (-1.78023593389281, 0.0, -1.5707963267949),
        ),
    )
    requested = frozenset(enabled_names)
    available = frozenset(name for name, *_ in mounts)
    unknown = requested - available
    if unknown:
        raise ValueError(f"Unknown Zerith cameras: {sorted(unknown)}")
    return tuple(
        CameraSpec(
            name=name,
            parent_frame=parent_frame,
            X_parent_camera_mount=_xyz_rpy_pose(xyz, rpy),
            X_mount_camera_optical=_drake_pose(RigidTransform()),
            width=width,
            height=height,
            fov_y_rad=fov_y_rad,
            near_m=near_m,
            far_m=far_m,
            update_period_s=update_period_s,
            modalities=tuple(modalities),
            enabled=name in requested,
        )
        for name, parent_frame, xyz, rpy in mounts
    )


class ZerithRobotAdapter:
    """Implement RobotAdapter for the fixed-base Zerith left arm."""

    def __init__(self, spec: RobotSpec):
        """Store and validate a Zerith robot specification."""
        if spec.package_name != ZERITH_PACKAGE_NAME:
            raise ValueError(
                f"Expected package {ZERITH_PACKAGE_NAME}, "
                f"got {spec.package_name}"
            )
        expected_names = tuple(config.name for config in ALL_SERVO_CONFIGS)
        if spec.controlled_joint_names != expected_names:
            raise ValueError(
                "Zerith controlled joints must follow the validated servo "
                "order"
            )
        if spec.gripper is None:
            raise ValueError("ZerithRobotAdapter requires a gripper spec")
        self._spec = spec

    @property
    def spec(self) -> RobotSpec:
        """Return the immutable Zerith robot specification."""
        return self._spec

    def add_model(self, parser: Any) -> Any:
        """Register the Zerith package and add exactly one robot model."""
        package_root = self.spec.model_path.parent.parent
        parser.package_map().Add(self.spec.package_name, str(package_root))
        model_instances = parser.AddModels(str(self.spec.model_path))
        if len(model_instances) != 1:
            raise RuntimeError(
                f"Expected one Zerith model, got {len(model_instances)}"
            )
        return model_instances[0]

    def configure_model(self, plant: Any, model_instance: Any) -> None:
        """Weld the base and add actuators before Plant finalization."""
        base_frame = plant.GetFrameByName(
            self.spec.base_link_name,
            model_instance,
        )
        base_pose = self.spec.base_pose
        quaternion = np.asarray(base_pose.quaternion_wxyz, dtype=float)
        quaternion /= np.linalg.norm(quaternion)
        # The calibrated base pose is yaw-only. Recovering yaw here avoids
        # making the public Pose contract depend on Drake types.
        yaw = 2.0 * math.atan2(quaternion[3], quaternion[0])
        plant.WeldFrames(
            plant.world_frame(),
            base_frame,
            RigidTransform(
                RollPitchYaw(0.0, 0.0, yaw),
                np.asarray(base_pose.translation_m, dtype=float),
            ),
        )
        add_left_grasp_frame(plant, model_instance)
        for joint_spec in self.spec.controlled_joints:
            joint = plant.GetJointByName(joint_spec.name, model_instance)
            plant.AddJointActuator(
                f"{joint_spec.name}_actuator",
                joint,
                effort_limit=joint_spec.effort_limit,
            )

    def initialize_state(
        self,
        plant: Any,
        plant_context: Any,
        model_instance: Any,
    ) -> None:
        """Apply home positions and lock every configured passive joint."""
        positions = plant.GetPositions(plant_context).copy()
        for joint_spec, value in zip(
            self.spec.controlled_joints,
            self.spec.home_positions,
            strict=True,
        ):
            joint = plant.GetJointByName(joint_spec.name, model_instance)
            positions[joint.position_start()] = value
        for name, value in self.spec.locked_joint_positions.items():
            joint = plant.GetJointByName(name, model_instance)
            positions[joint.position_start()] = value
        plant.SetPositions(plant_context, positions)
        for name in self.spec.locked_joint_positions:
            plant.GetJointByName(name, model_instance).Lock(plant_context)

    def make_robot_observation(
        self,
        plant: Any,
        plant_context: Any,
        model_instance: Any,
        controller_state: Mapping[str, Any],
    ) -> RobotObservation:
        """Convert Drake state and controller telemetry to public fields."""
        positions = plant.GetPositions(plant_context)
        velocities = plant.GetVelocities(plant_context)
        joints = tuple(
            plant.GetJointByName(joint.name, model_instance)
            for joint in self.spec.controlled_joints
        )
        q = tuple(float(positions[joint.position_start()]) for joint in joints)
        v = tuple(float(velocities[joint.velocity_start()]) for joint in joints)
        end_effector = plant.GetFrameByName(
            self.spec.end_effector_frame_name,
            model_instance,
        )
        transform = end_effector.CalcPoseInWorld(plant_context)
        velocity = end_effector.CalcSpatialVelocityInWorld(plant_context)
        return RobotObservation(
            joint_names=self.spec.controlled_joint_names,
            q=q,
            v=v,
            q_commanded=controller_state["q_commanded"],
            torque_commanded=controller_state["torque_commanded"],
            torque_applied=controller_state["torque_applied"],
            torque_saturated=controller_state["torque_saturated"],
            end_effector_pose=_drake_pose(transform),
            end_effector_twist=SpatialVelocity(
                tuple(float(value) for value in velocity.rotational()),
                tuple(float(value) for value in velocity.translational()),
            ),
            gripper_width_m=self._gripper_width(q),
            end_effectors={
                name: _drake_pose(
                    plant.GetFrameByName(frame, model_instance).CalcPoseInWorld(
                        plant_context
                    )
                )
                for name, frame in self.spec.end_effector_frames.items()
            },
            gripper_widths_m={
                name: self._gripper_width(q) for name in self.spec.grippers
            },
        )

    def gripper_position_targets(
        self, width_m: float, name: str | None = None,
    ) -> Mapping[str, float]:
        """Map physical opening width to symmetric finger positions."""
        if name is not None and name not in self.spec.grippers:
            raise ValueError(f"Unknown gripper: {name}")
        gripper = self.spec.gripper if name is None else self.spec.grippers[name]
        if gripper is None:
            raise ValueError("Zerith gripper specification is missing")
        width = float(width_m)
        if not gripper.minimum_width_m <= width <= gripper.maximum_width_m:
            raise ValueError(
                "width_m violates the configured Zerith gripper limits"
            )
        inward_travel = 0.5 * (gripper.maximum_width_m - width)
        return {
            gripper.joint_names[0]: -inward_travel,
            gripper.joint_names[1]: inward_travel,
        }

    def _gripper_width(self, q: Sequence[float]) -> float:
        """Compute physical opening width from controlled joint positions."""
        gripper = self.spec.gripper
        if gripper is None:
            raise RuntimeError("Zerith gripper specification is missing")
        indices = tuple(
            self.spec.controlled_joint_names.index(name)
            for name in gripper.joint_names
        )
        return float(
            gripper.maximum_width_m - (q[indices[1]] - q[indices[0]])
        )


class ZerithLegacyActionTranslator:
    """Translate typed actions to the legacy seven-plus-one array."""

    def __init__(
        self,
        spec: RobotSpec,
        planning_query: PlanningQuery | None = None,
        maximum_joint_delta: float | None = None,
        maximum_cartesian_joint_delta: float | None = None,
    ):
        """Initialize held targets from the RobotSpec home configuration."""
        gripper = spec.gripper
        if gripper is None:
            raise ValueError("Legacy Zerith translation requires a gripper")
        self._spec = spec
        self._planning_query = planning_query
        if maximum_joint_delta is not None and maximum_joint_delta <= 0.0:
            raise ValueError("maximum_joint_delta must be positive")
        if (
            maximum_cartesian_joint_delta is not None
            and maximum_cartesian_joint_delta <= 0.0
        ):
            raise ValueError(
                "maximum_cartesian_joint_delta must be positive"
            )
        self._maximum_joint_delta = maximum_joint_delta
        self._maximum_cartesian_joint_delta = (
            maximum_cartesian_joint_delta
            if maximum_cartesian_joint_delta is not None
            else maximum_joint_delta
        )
        self._gripper = gripper
        self._arm_names = tuple(
            name
            for name in spec.controlled_joint_names
            if name not in gripper.joint_names
        )
        if len(self._arm_names) != len(LEFT_ARM_SERVO_CONFIGS):
            raise ValueError(
                "Legacy Zerith translation requires seven arm joints"
            )
        joint_specs = {joint.name: joint for joint in spec.controlled_joints}
        self._arm_position_lower = np.asarray(
            [joint_specs[name].position_lower for name in self._arm_names]
        )
        self._arm_position_upper = np.asarray(
            [joint_specs[name].position_upper for name in self._arm_names]
        )
        self._gripper_position_limits = {
            name: (
                joint_specs[name].position_lower,
                joint_specs[name].position_upper,
            )
            for name in self._gripper.joint_names
        }
        home_by_name = dict(
            zip(
                spec.controlled_joint_names,
                spec.home_positions,
                strict=True,
            )
        )
        self._home_arm = np.asarray(
            [home_by_name[name] for name in self._arm_names],
            dtype=float,
        )
        self.reset()

    def reset(self) -> None:
        """Restore held arm and gripper targets to RobotSpec home values."""
        self._desired_arm = self._home_arm.copy()
        self._desired_gripper_width = self._gripper.maximum_width_m
        self._measured_arm = self._home_arm.copy()
        self._measured_gripper_width = self._gripper.maximum_width_m
        self._measured_gripper_positions = {
            name: 0.0 for name in self._gripper.joint_names
        }
        self._last_decision = {
            "status": "accepted",
            "reasons": (),
            "requested_action_type": "reset",
        }

    @property
    def last_decision(self) -> dict[str, Any]:
        """Return diagnostics for the most recent action translation."""
        return dict(self._last_decision)

    def update_from_runtime_info(self, info: Mapping[str, Any]) -> None:
        """Synchronize held targets after legacy clipping is applied."""
        desired_arm = np.asarray(info["desired_q_left"], dtype=float)
        if desired_arm.shape != self._desired_arm.shape:
            raise ValueError("Legacy runtime returned invalid desired_q_left")
        self._desired_arm = desired_arm.copy()
        self._desired_gripper_width = float(info["desired_gripper_width"])

    def update_from_observation(self, observation: Observation) -> None:
        """Synchronize the physical arm and gripper state used by IK."""
        position_by_name = dict(
            zip(
                observation.robot.joint_names,
                observation.robot.q,
                strict=True,
            )
        )
        self._measured_arm = np.asarray(
            [position_by_name[name] for name in self._arm_names],
            dtype=float,
        )
        self._measured_gripper_positions = {
            name: float(np.clip(
                position_by_name[name],
                *self._gripper_position_limits[name],
            ))
            for name in self._gripper.joint_names
        }
        width = observation.robot.gripper_width_m
        if width is None:
            raise ValueError("Zerith Cartesian control requires gripper width")
        self._measured_gripper_width = float(width)

    def translate(
        self,
        action: RobotAction,
        contact_policy: ContactPolicy | None = None,
    ) -> np.ndarray:
        """Return one legacy action without mutating held target state."""
        reasons = []
        rejected = False
        cartesian_diagnostics = None
        arm_action: (
            HoldAction
            | JointPositionAction
            | JointDeltaAction
            | CartesianDeltaAction
            | CartesianPoseAction
            | None
        )
        gripper_action: GripperAction | None
        if isinstance(action, CompositeAction):
            arm_action = action.arm
            gripper_action = action.gripper
        elif isinstance(action, GripperAction):
            arm_action = None
            gripper_action = action
        else:
            arm_action = action
            gripper_action = None

        requested_arm_delta = np.zeros(len(self._arm_names))
        if arm_action is None or isinstance(arm_action, HoldAction):
            pass
        elif isinstance(arm_action, JointDeltaAction):
            requested_arm_delta = self._ordered_arm_values(
                arm_action.joint_names,
                arm_action.deltas,
            )
        elif isinstance(arm_action, JointPositionAction):
            requested = self._desired_arm.copy()
            for name, value in zip(
                arm_action.joint_names,
                arm_action.positions,
                strict=True,
            ):
                requested[self._arm_index(name)] = value
            requested_arm_delta = requested - self._desired_arm
        elif isinstance(arm_action, (CartesianDeltaAction, CartesianPoseAction)):
            requested_arm_delta, result = self._cartesian_arm_delta(
                arm_action,
                contact_policy,
            )
            cartesian_diagnostics = {
                "success": result.success,
                "reason": result.reason,
                "minimum_nonpenetration_distance_m": (
                    result.edge.minimum_nonpenetration_distance_m
                ),
                "minimum_safety_clearance_m": (
                    result.edge.minimum_safety_clearance_m
                ),
                "minimum_nonpenetration_alpha": (
                    result.edge.minimum_nonpenetration_alpha
                ),
                "minimum_safety_alpha": (
                    result.edge.minimum_safety_alpha
                ),
                "edge_sample_count": result.edge.sample_count,
                "limiting_nonpenetration_pair": (
                    dataclasses.asdict(
                        result.edge.limiting_nonpenetration_pair
                    )
                    if result.edge.limiting_nonpenetration_pair is not None
                    else None
                ),
                "limiting_pair_start_distance_m": (
                    result.edge.limiting_pair_start_distance_m
                ),
                "limiting_pair_end_distance_m": (
                    result.edge.limiting_pair_end_distance_m
                ),
                "limiting_pair_sample_distances_m": (
                    result.edge.limiting_pair_sample_distances_m
                ),
                "limiting_pair_monotonic_non_decreasing": (
                    result.edge.limiting_pair_monotonic_non_decreasing
                ),
                "minimum_nonpenetration_margin_m": (
                    result.edge.minimum_nonpenetration_margin_m
                ),
                "minimum_nonpenetration_margin_alpha": (
                    result.edge.minimum_nonpenetration_margin_alpha
                ),
                "joint_limits_valid": result.edge.joint_limits_valid,
                "nonpenetration_valid": (
                    result.edge.nonpenetration_valid
                ),
                "safety_clearance_valid": (
                    result.edge.safety_clearance_valid
                ),
                "requested_twist": result.requested_twist,
                "achieved_twist": result.achieved_twist,
                "candidate_configuration": result.configuration,
                "validation_start_configuration": (
                    result.validation_start_configuration
                ),
                "validation_edge_translation_m": (
                    result.validation_edge_translation_m
                ),
            }
            if not result.success:
                rejected = True
                reasons.append(f"cartesian_{result.reason}")
            elif getattr(result, "joint_delta_scaled", False):
                reasons.append("cartesian_joint_delta_scaled")
        else:
            raise TypeError(f"Unsupported arm action: {type(arm_action)}")

        applied_arm_delta = requested_arm_delta.copy()
        if self._maximum_joint_delta is not None:
            applied_arm_delta = np.clip(
                applied_arm_delta,
                -self._maximum_joint_delta,
                self._maximum_joint_delta,
            )
            if not np.array_equal(applied_arm_delta, requested_arm_delta):
                reasons.append("maximum_joint_delta")
        requested_target = self._desired_arm + applied_arm_delta
        applied_target = np.clip(
            requested_target,
            self._arm_position_lower,
            self._arm_position_upper,
        )
        if not np.array_equal(applied_target, requested_target):
            reasons.append("joint_position_limit")
        applied_arm_delta = applied_target - self._desired_arm

        requested_gripper_width = self._desired_gripper_width
        if gripper_action is not None:
            if gripper_action.maximum_effort_n is not None:
                raise NotImplementedError(
                    "Legacy Zerith gripper does not support per-action "
                    "maximum_effort_n"
                )
            requested_gripper_width = gripper_action.width_m
            if not (
                self._gripper.minimum_width_m
                <= requested_gripper_width
                <= self._gripper.maximum_width_m
            ):
                rejected = True
                reasons.append("gripper_width_limit")
        applied_gripper_width = requested_gripper_width
        if rejected:
            applied_arm_delta = np.zeros(len(self._arm_names))
            applied_gripper_width = self._desired_gripper_width
        if rejected:
            status = "rejected"
        elif reasons:
            status = "adjusted"
        else:
            status = "accepted"
        self._last_decision = {
            "status": status,
            "reasons": tuple(reasons),
            "requested_action_type": type(action).__name__,
            "requested_arm_delta": tuple(
                float(value) for value in requested_arm_delta
            ),
            "applied_arm_delta": tuple(
                float(value) for value in applied_arm_delta
            ),
            "requested_gripper_width_m": float(requested_gripper_width),
            "applied_gripper_width_m": float(applied_gripper_width),
        }
        if cartesian_diagnostics is not None:
            self._last_decision["cartesian"] = cartesian_diagnostics
        normalized_gripper = (
            2.0
            * applied_gripper_width
            / self._gripper.maximum_width_m
            - 1.0
        )
        return np.concatenate((applied_arm_delta, [normalized_gripper]))

    def _ordered_arm_values(
        self,
        names: Sequence[str],
        values: Sequence[float],
    ) -> np.ndarray:
        """Insert a sparse named command into legacy left-arm order."""
        ordered = np.zeros(len(self._arm_names))
        for name, value in zip(names, values, strict=True):
            ordered[self._arm_index(name)] = value
        return ordered

    def _arm_index(self, name: str) -> int:
        """Return one legacy arm index or fail with an actionable message."""
        if name not in self._arm_names:
            raise ValueError(
                f"Joint {name} is not a legacy Zerith arm joint; use "
                "GripperAction for finger motion"
            )
        return self._arm_names.index(name)

    def _cartesian_arm_delta(
        self,
        action: CartesianDeltaAction | CartesianPoseAction,
        contact_policy: ContactPolicy | None,
    ) -> tuple[np.ndarray, Any]:
        """Solve one safe online world-frame differential-IK increment."""
        if self._planning_query is None:
            raise NotImplementedError(
                "Cartesian actions require a configured PlanningQuery"
            )
        if action.reference_frame != "world":
            raise NotImplementedError(
                "The Cartesian action implementation supports "
                "only reference_frame='world'"
            )
        if action.end_effector_frame != self._spec.end_effector_frame_name:
            raise ValueError(
                "Cartesian action end_effector_frame does not match "
                "RobotSpec"
            )
        seed_by_name = dict(zip(self._arm_names, self._desired_arm))
        seed_by_name.update(self._measured_gripper_positions)
        q_seed = tuple(
            seed_by_name[name] for name in self._spec.controlled_joint_names
        )
        measured_by_name = dict(zip(self._arm_names, self._measured_arm))
        measured_by_name.update(self._measured_gripper_positions)
        validation_start = tuple(
            measured_by_name[name]
            for name in self._spec.controlled_joint_names
        )
        if self._maximum_cartesian_joint_delta is None:
            raise RuntimeError(
                "Cartesian actions require "
                "maximum_cartesian_joint_delta"
            )
        solve_kwargs = {}
        if contact_policy is not None:
            solve_kwargs["contact_policy"] = contact_policy
        if isinstance(action, CartesianPoseAction):
            solve = self._planning_query.differential_ik_to_pose
            solve_kwargs["target_pose"] = action.pose
        else:
            solve = self._planning_query.differential_ik_step
            solve_kwargs["translation_m"] = action.translation_m
            solve_kwargs["rotation_vector_rad"] = action.rotation_vector_rad
        result = solve(
            frame_name=action.end_effector_frame,
            seed=q_seed,
            validation_start=validation_start,
            maximum_joint_delta=self._maximum_cartesian_joint_delta,
            **solve_kwargs,
        )
        if not result.success:
            return np.zeros(len(self._arm_names)), result
        applied_configuration = np.asarray(result.configuration)
        solution_by_name = dict(
            zip(
                self._spec.controlled_joint_names,
                applied_configuration,
                strict=True,
            )
        )
        return (
            np.asarray([
                solution_by_name[name] - self._desired_arm[index]
                for index, name in enumerate(self._arm_names)
            ]),
            result,
        )

    def _gripper_joint_targets(self, width_m: float) -> dict[str, float]:
        """Return legacy Zerith finger positions for a physical width."""
        inward_travel = 0.5 * (
            self._gripper.maximum_width_m - width_m
        )
        return {
            self._gripper.joint_names[0]: -inward_travel,
            self._gripper.joint_names[1]: inward_travel,
        }




@dataclasses.dataclass(frozen=True)
class ZerithEnvironmentConfig:
    """Public composition config for the current Zerith runtime adapter."""

    scenario: ScenarioSpec
    robot_model_dir: Path
    robot_xyz: tuple[float, float, float]
    robot_yaw_deg: float
    rail_position: float
    q_home_left: tuple[float, ...]
    timing: TimingConfig = dataclasses.field(default_factory=TimingConfig)
    episode_duration: float = 30.0
    max_joint_delta: float = 0.1
    maximum_cartesian_joint_delta: float | None = None
    target_model_name: str | None = None
    target_body_name: str = "base_link"
    task: Task | None = None
    enable_planning_query: bool = False
    cameras: tuple[CameraSpec, ...] = ()
    locked_joint_position_overrides: Mapping[str, float] = dataclasses.field(
        default_factory=dict
    )
    joint_servo_settings: Mapping[str, Mapping[str, float]] = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        """Normalize paths and sequences and validate runtime limits."""
        object.__setattr__(self, "robot_model_dir", Path(self.robot_model_dir))
        object.__setattr__(
            self,
            "robot_xyz",
            tuple(float(value) for value in self.robot_xyz),
        )
        object.__setattr__(
            self,
            "q_home_left",
            tuple(float(value) for value in self.q_home_left),
        )
        if len(self.robot_xyz) != 3:
            raise ValueError("robot_xyz must contain three values")
        if len(self.q_home_left) != len(LEFT_ARM_SERVO_CONFIGS):
            raise ValueError("q_home_left must contain seven values")
        scalars = (
            self.robot_yaw_deg,
            self.rail_position,
            self.episode_duration,
            self.max_joint_delta,
        )
        if not all(math.isfinite(value) for value in scalars):
            raise ValueError("Zerith environment values must be finite")
        if self.episode_duration <= 0.0 or self.max_joint_delta <= 0.0:
            raise ValueError("Episode duration and joint delta must be positive")
        if (
            self.maximum_cartesian_joint_delta is not None
            and (
                not math.isfinite(self.maximum_cartesian_joint_delta)
                or self.maximum_cartesian_joint_delta <= 0.0
            )
        ):
            raise ValueError(
                "maximum_cartesian_joint_delta must be positive"
            )
        if not self.target_body_name:
            raise ValueError("target_body_name must be nonempty")
        object.__setattr__(self, "cameras", tuple(self.cameras))
        object.__setattr__(
            self,
            "locked_joint_position_overrides",
            dict(self.locked_joint_position_overrides),
        )

    def build_environment(self) -> OnlineManipulationEnv:
        """Build the public environment without exposing Drake internals."""
        adapter = ZerithRobotAdapter(
            make_zerith_robot_spec(
                robot_model_dir=self.robot_model_dir,
                robot_xyz=self.robot_xyz,
                robot_yaw_deg=self.robot_yaw_deg,
                rail_position=self.rail_position,
                q_home_left=self.q_home_left,
                cameras=self.cameras,
                locked_joint_position_overrides=(
                    self.locked_joint_position_overrides
                ),
            )
        )
        from src.online_manipulation.controller import configure_joint_servos
        adapter = ZerithRobotAdapter(dataclasses.replace(
            adapter.spec, controlled_joints=configure_joint_servos(
                adapter.spec.controlled_joints, self.joint_servo_settings)))
        from src.online_manipulation.runtime import RuntimeConfig

        # Public fixed-baseline composition now uses the same real runtime as
        # other mechanisms. Only the v0.2 action translation remains specific.
        return RuntimeConfig(
            scenario=self.scenario,
            robot_adapter=adapter,
            timing=self.timing,
            episode_duration=self.episode_duration,
            maximum_joint_delta=self.max_joint_delta,
            maximum_cartesian_joint_delta=(
                self.maximum_cartesian_joint_delta or self.max_joint_delta
            ),
            task_factory=lambda: copy.deepcopy(self.task) if self.task is not None else NullTask(),
            action_resolver_factory=lambda runtime: ZerithFixedCommandResolver(
                runtime, enable_planning=self.enable_planning_query,
                maximum_cartesian_joint_delta=self.maximum_cartesian_joint_delta),
        ).build_environment()


class ZerithFixedCommandResolver:
    """Keep v0.2 fixed action semantics on the shared integrator, not two plants."""

    def __init__(self, runtime, *, enable_planning, maximum_cartesian_joint_delta):
        self.runtime = runtime
        self.translator = ZerithLegacyActionTranslator(
            runtime.spec, planning_query=runtime.planning,
            maximum_joint_delta=runtime.config.maximum_joint_delta,
            maximum_cartesian_joint_delta=maximum_cartesian_joint_delta)

    def reset(self):
        self.translator.reset()

    def resolve(self, action, contact_policy):
        runtime = self.runtime
        observation = runtime._observation()
        self.translator.update_from_observation(observation)
        gripper = runtime.spec.gripper
        indices = [runtime.spec.controlled_joint_names.index(n) for n in gripper.joint_names]
        arm_indices = [i for i, n in enumerate(runtime.spec.controlled_joint_names) if n not in gripper.joint_names]
        width = gripper.maximum_width_m - runtime.command[indices[1]] + runtime.command[indices[0]]
        self.translator.update_from_runtime_info({"desired_q_left": runtime.command[arm_indices],
                                                  "desired_gripper_width": width})
        runtime.planning.synchronize_state(runtime.plant_context)
        legacy = self.translator.translate(action, contact_policy)
        candidate = runtime.command.copy()
        candidate[arm_indices] += legacy[:-1]
        desired_width = (legacy[-1] + 1) * gripper.maximum_width_m / 2
        for name, value in runtime.adapter.gripper_position_targets(desired_width).items():
            candidate[runtime.spec.controlled_joint_names.index(name)] = value
        decision = self.translator.last_decision
        decision["accepted"] = decision["status"] != "rejected"
        arm_action = action.arm if isinstance(action, CompositeAction) else action
        if (decision["accepted"]
                and isinstance(arm_action, (JointDeltaAction, JointPositionAction))):
            edge = runtime.check_command_edge(observation.robot.q, candidate, contact_policy)
            decision["edge"] = dataclasses.asdict(edge)
            if not edge.valid:
                decision.update(accepted=False, status="rejected",
                                reasons=("carried_joint_edge_rejected",))
        return candidate, decision
