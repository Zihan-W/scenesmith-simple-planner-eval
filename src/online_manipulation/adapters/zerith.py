"""Zerith-specific model, joint, gripper, and legacy-runtime adapter."""

import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np
from pydrake.all import Meshcat, RigidTransform, RollPitchYaw

from src.online_manipulation.observations import (
    Pose,
    RobotObservation,
    SpatialVelocity,
)
from src.online_manipulation.specs import (
    GripperSpec,
    JointSpec,
    RobotSpec,
    ScenarioSpec,
    TimingConfig,
)
from src.zerith_online_env import (
    ALL_SERVO_CONFIGS,
    GRIPPER_MAX_OPENING,
    LEFT_ARM_SERVO_CONFIGS,
    LEFT_GRIPPER_SERVO_CONFIGS,
    ZERITH_PACKAGE_NAME,
    ZERITH_URDF_RELATIVE_PATH,
    ZerithOnlineEnv,
)

_BASE_LINK_NAME = "dipan_link"
_END_EFFECTOR_FRAME_NAME = "left_end_effector_link"
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
    root = ET.parse(urdf_path).getroot()
    return RobotSpec(
        name="zerith_left_arm",
        model_instance_name=root.attrib["name"],
        package_name=ZERITH_PACKAGE_NAME,
        model_path=urdf_path,
        base_link_name=_BASE_LINK_NAME,
        base_pose=_yaw_pose(robot_xyz, robot_yaw_deg),
        controlled_joints=controlled_joints,
        locked_joint_positions=locked_positions,
        end_effector_frame_name=_END_EFFECTOR_FRAME_NAME,
        home_positions=q_home + (0.0, 0.0),
        gripper=GripperSpec(
            joint_names=tuple(
                config.name for config in LEFT_GRIPPER_SERVO_CONFIGS
            ),
            minimum_width_m=0.0,
            maximum_width_m=GRIPPER_MAX_OPENING,
        ),
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
        end_effector = plant.GetBodyByName(
            self.spec.end_effector_frame_name,
            model_instance,
        )
        transform = plant.EvalBodyPoseInWorld(plant_context, end_effector)
        velocity = plant.EvalBodySpatialVelocityInWorld(
            plant_context,
            end_effector,
        )
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
        )

    def gripper_position_targets(self, width_m: float) -> Mapping[str, float]:
        """Map physical opening width to symmetric finger positions."""
        gripper = self.spec.gripper
        if gripper is None:
            raise RuntimeError("Zerith gripper specification is missing")
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


def make_legacy_zerith_environment(
    *,
    scenario: ScenarioSpec,
    adapter: ZerithRobotAdapter,
    timing: TimingConfig,
    target_model_name: str,
    target_body_name: str = "base_link",
    episode_duration: float = 30.0,
    max_joint_delta: float = 0.1,
) -> ZerithOnlineEnv:
    """Construct the validated legacy runtime from generic specifications.

    This function is a temporary migration boundary. Target identity remains
    an explicit compatibility argument until Phase 4 moves it into Task.
    """
    if scenario.initial_object_poses:
        raise NotImplementedError(
            "Legacy Zerith runtime does not apply ScenarioSpec object poses"
        )
    if scenario.contact_parameters:
        raise NotImplementedError(
            "Legacy Zerith runtime does not apply contact parameters"
        )
    if not scenario.package_xmls:
        raise ValueError(
            "Legacy Zerith runtime requires the scene package.xml as the "
            "first ScenarioSpec.package_xmls entry"
        )
    scene_package_xml = scenario.package_xmls[0]
    additional_package_xmls = scenario.package_xmls[1:]
    meshcat = None
    if scenario.visualization.enabled:
        meshcat = Meshcat(scenario.visualization.port)

    controlled_home = dict(
        zip(
            adapter.spec.controlled_joint_names,
            adapter.spec.home_positions,
            strict=True,
        )
    )
    q_home_left = tuple(
        controlled_home[config.name] for config in LEFT_ARM_SERVO_CONFIGS
    )
    yaw_quaternion = adapter.spec.base_pose.quaternion_wxyz
    robot_yaw_deg = math.degrees(
        2.0 * math.atan2(yaw_quaternion[3], yaw_quaternion[0])
    )
    return ZerithOnlineEnv(
        scene_dmd=scenario.dmd_path,
        robot_model_dir=adapter.spec.model_path.parent.parent,
        target_model_name=target_model_name,
        scene_package_xml=scene_package_xml,
        additional_package_xmls=additional_package_xmls,
        target_body_name=target_body_name,
        robot_xyz=adapter.spec.base_pose.translation_m,
        robot_yaw_deg=robot_yaw_deg,
        rail_position=adapter.spec.locked_joint_positions[_RAIL_JOINT_NAME],
        q_home=q_home_left,
        physics_dt=timing.physics_dt,
        controller_dt=timing.controller_dt,
        policy_dt=timing.policy_dt,
        episode_duration=episode_duration,
        max_joint_delta=max_joint_delta,
        realtime_rate=scenario.visualization.realtime_rate,
        meshcat=meshcat,
    )
