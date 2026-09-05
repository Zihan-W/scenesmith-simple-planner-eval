"""Zerith-specific model, joint, gripper, and legacy-runtime adapter."""

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
from src.online_manipulation.planning import PlanningQuery
from src.online_manipulation.protocols import ContactPolicy, Task
from src.online_manipulation.specs import (
    GripperSpec,
    JointSpec,
    ObservedBodySpec,
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
from src.zerith_grasp_geometry import (
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
    model_instance_name = root.attrib["name"]
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
        gripper=GripperSpec(
            joint_names=tuple(
                config.name for config in LEFT_GRIPPER_SERVO_CONFIGS
            ),
            minimum_width_m=0.0,
            maximum_width_m=GRIPPER_MAX_OPENING,
        ),
        safety_exempt_body_pairs=(
            (
                f"{model_instance_name}::body_yaw_link",
                f"{model_instance_name}::left_shoulder_roll_link",
            ),
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
        elif isinstance(arm_action, CartesianDeltaAction):
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
                "requested_twist": result.requested_twist,
                "achieved_twist": result.achieved_twist,
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
        action: CartesianDeltaAction,
        contact_policy: ContactPolicy | None,
    ) -> tuple[np.ndarray, Any]:
        """Solve one safe online world-frame differential-IK increment."""
        if self._planning_query is None:
            raise NotImplementedError(
                "CartesianDeltaAction requires a configured PlanningQuery"
            )
        if action.reference_frame != "world":
            raise NotImplementedError(
                "The initial CartesianDeltaAction implementation supports "
                "only reference_frame='world'"
            )
        if action.end_effector_frame != self._spec.end_effector_frame_name:
            raise ValueError(
                "CartesianDeltaAction end_effector_frame does not match "
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
                "CartesianDeltaAction requires "
                "maximum_cartesian_joint_delta"
            )
        solve_kwargs = {}
        if contact_policy is not None:
            solve_kwargs["contact_policy"] = contact_policy
        result = self._planning_query.differential_ik_step(
            translation_m=action.translation_m,
            rotation_vector_rad=action.rotation_vector_rad,
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


class LegacyZerithRuntimeBackend:
    """Normalize the validated ZerithOnlineEnv behind the generic facade."""

    def __init__(
        self,
        runtime: ZerithOnlineEnv,
        adapter: ZerithRobotAdapter,
        scenario: ScenarioSpec,
        planning_query: PlanningQuery | None = None,
        maximum_cartesian_joint_delta: float | None = None,
    ):
        """Store the runtime and explicit generic object observation list."""
        self.runtime = runtime
        self.adapter = adapter
        self.scenario = scenario
        self.observed_bodies = tuple(scenario.observed_bodies)
        self.planning_query = planning_query
        self.action_translator = ZerithLegacyActionTranslator(
            adapter.spec,
            planning_query=planning_query,
            maximum_joint_delta=runtime.max_joint_delta,
            maximum_cartesian_joint_delta=maximum_cartesian_joint_delta,
        )

    @property
    def control_log(self):
        """Expose legacy controller diagnostics during migration."""
        return self.runtime.control_log

    def reset(self) -> tuple[Observation, dict]:
        """Reset runtime and translator state, then normalize observation."""
        self.runtime.reset()
        self.action_translator.reset()
        observation = self._observation()
        self.action_translator.update_from_observation(observation)
        distance, is_lower_bound = self._minimum_robot_signed_distance()
        return observation, {
            "control_updates": 0,
            "physics_steps_per_control": (
                self.runtime.physics_steps_per_control
            ),
            "minimum_collision_distance_m": distance,
            "minimum_collision_distance_is_lower_bound": is_lower_bound,
        }

    def step(
        self,
        action: RobotAction,
        contact_policy: ContactPolicy,
    ) -> tuple[Observation, bool, dict]:
        """Translate and execute one typed action for one policy period."""
        if self.planning_query is not None:
            observation = self._observation()
            self.action_translator.update_from_observation(observation)
            self.planning_query.set_observed_body_poses(
                {
                    name: object_observation.pose
                    for name, object_observation in observation.objects.items()
                }
            )
        legacy_action = self.action_translator.translate(
            action,
            contact_policy,
        )
        _, _, done, info = self.runtime.step(legacy_action)
        self.action_translator.update_from_runtime_info(info)
        normalized_info = dict(info)
        normalized_info["legacy_action"] = legacy_action.copy()
        decision = self.action_translator.last_decision
        if info["action_clipped"]:
            reasons = tuple(decision["reasons"]) + (
                "legacy_runtime_limit",
            )
            decision.update(status="adjusted", reasons=reasons)
        normalized_info["action_decision"] = decision
        distance, is_lower_bound = self._minimum_robot_signed_distance()
        normalized_info["minimum_collision_distance_m"] = distance
        normalized_info[
            "minimum_collision_distance_is_lower_bound"
        ] = is_lower_bound
        return self._observation(), done, normalized_info

    def robot_penetrations(self):
        """Expose the legacy robot penetration diagnostic during migration."""
        return self.runtime.robot_penetrations()

    def write_updated_scenario(self, output_path: Path) -> tuple[str, ...]:
        """Write selected free bodies from the active simulation context."""
        return write_updated_dmd(
            input_path=self.scenario.dmd_path,
            output_path=output_path,
            plant=self.runtime.plant,
            plant_context=self.runtime.plant_context,
            body_specs=self.scenario.observed_bodies,
        )

    def start_recording(self) -> None:
        """Start Meshcat recording for a visualization-enabled scenario."""
        if self.runtime.meshcat is None:
            raise RuntimeError(
                "Scenario visualization must be enabled to record HTML"
            )
        self.runtime.meshcat.StartRecording()

    def save_recording(self, output_path: Path) -> None:
        """Stop and write the active Meshcat recording as standalone HTML."""
        if self.runtime.meshcat is None:
            raise RuntimeError(
                "Scenario visualization must be enabled to record HTML"
            )
        self.runtime.meshcat.StopRecording()
        self.runtime.meshcat.PublishRecording()
        destination = Path(output_path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            self.runtime.meshcat.StaticHtml(),
            encoding="utf-8",
        )

    def _minimum_robot_signed_distance(
        self,
        query_radius_m: float = 0.05,
    ) -> tuple[float, bool]:
        """Return filtered robot-related distance or a query-radius bound."""
        plant = self.runtime.plant
        plant_context = self.runtime.plant_context
        query = plant.get_geometry_query_input_port().Eval(plant_context)
        inspector = query.inspector()
        robot_geometry_ids = set()
        for body_index in plant.GetBodyIndices(
            self.runtime.robot_model_instance
        ):
            body = plant.get_body(body_index)
            frame_id = plant.GetBodyFrameIdOrThrow(body.index())
            robot_geometry_ids.update(
                inspector.GetGeometries(frame_id, Role.kProximity)
            )
        distances = [
            float(pair.distance)
            for pair in query.ComputeSignedDistancePairwiseClosestPoints(
                query_radius_m
            )
            if pair.id_A in robot_geometry_ids
            or pair.id_B in robot_geometry_ids
        ]
        if distances:
            return min(distances), False
        return query_radius_m, True

    def _observation(self) -> Observation:
        """Read a generic observation from the active Drake context."""
        plant = self.runtime.plant
        plant_context = self.runtime.plant_context
        robot = self.adapter.make_robot_observation(
            plant,
            plant_context,
            self.runtime.robot_model_instance,
            self.runtime.controller_state,
        )
        objects = {}
        for body_spec in self.observed_bodies:
            model_instance = plant.GetModelInstanceByName(
                body_spec.model_instance_name
            )
            body = plant.GetBodyByName(body_spec.body_name, model_instance)
            transform = plant.EvalBodyPoseInWorld(plant_context, body)
            velocity = plant.EvalBodySpatialVelocityInWorld(
                plant_context,
                body,
            )
            objects[body_spec.observation_name] = ObjectObservation(
                pose=_drake_pose(transform),
                spatial_velocity=SpatialVelocity(
                    tuple(float(value) for value in velocity.rotational()),
                    tuple(float(value) for value in velocity.translational()),
                ),
            )
        query = plant.get_geometry_query_input_port().Eval(plant_context)
        inspector = query.inspector()

        def qualified_body_name(frame_id):
            """Return a stable model-instance-qualified body name."""
            body = plant.GetBodyFromFrameId(frame_id)
            model_name = plant.GetModelInstanceName(body.model_instance())
            return f"{model_name}::{body.name()}"

        contacts = tuple(
            ContactObservation(
                body_a=qualified_body_name(inspector.GetFrameId(pair.id_A)),
                body_b=qualified_body_name(inspector.GetFrameId(pair.id_B)),
                penetration_depth_m=float(pair.depth),
            )
            for pair in query.ComputePointPairPenetration()
        )
        return Observation(
            time_s=float(plant_context.get_time()),
            robot=robot,
            objects=objects,
            contacts=contacts,
            task={},
        )


def make_legacy_zerith_environment(
    *,
    scenario: ScenarioSpec,
    adapter: ZerithRobotAdapter,
    timing: TimingConfig,
    target_model_name: str | None = None,
    target_body_name: str = "base_link",
    episode_duration: float = 30.0,
    max_joint_delta: float = 0.1,
) -> ZerithOnlineEnv:
    """Construct the validated legacy runtime from generic specifications.

    This function is a temporary migration boundary. The optional target is
    retained only for callers of the legacy dictionary observation; generic
    observations use ``ScenarioSpec.observed_bodies`` instead.
    """
    if not scenario.package_xmls:
        raise ValueError(
            "Legacy Zerith runtime requires the scene package.xml as the "
            "first ScenarioSpec.package_xmls entry"
        )
    scene_package_xml = scenario.package_xmls[0]
    additional_package_xmls = scenario.package_xmls[1:]
    body_specs_by_name = {
        body.observation_name: body for body in scenario.observed_bodies
    }
    initial_body_poses = {
        (
            body_specs_by_name[name].model_instance_name,
            body_specs_by_name[name].body_name,
        ): pose
        for name, pose in scenario.initial_object_poses.items()
    }
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
        initial_body_poses=initial_body_poses,
        contact_parameters=scenario.contact_parameters,
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
        servo_joint_specs=adapter.spec.controlled_joints,
    )


def make_legacy_zerith_online_environment(
    *,
    scenario: ScenarioSpec,
    adapter: ZerithRobotAdapter,
    timing: TimingConfig,
    target_model_name: str | None = None,
    target_body_name: str = "base_link",
    episode_duration: float = 30.0,
    max_joint_delta: float = 0.1,
    maximum_cartesian_joint_delta: float | None = None,
    planning_query: PlanningQuery | None = None,
    task: Task | None = None,
) -> OnlineManipulationEnv:
    """Build the typed Phase 3 facade over the validated legacy runtime."""
    runtime = make_legacy_zerith_environment(
        scenario=scenario,
        adapter=adapter,
        timing=timing,
        target_model_name=target_model_name,
        target_body_name=target_body_name,
        episode_duration=episode_duration,
        max_joint_delta=max_joint_delta,
    )
    backend = LegacyZerithRuntimeBackend(
        runtime=runtime,
        adapter=adapter,
        scenario=scenario,
        planning_query=planning_query,
        maximum_cartesian_joint_delta=maximum_cartesian_joint_delta,
    )
    return OnlineManipulationEnv(backend, task=task)
