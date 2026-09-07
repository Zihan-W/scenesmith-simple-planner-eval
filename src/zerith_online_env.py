"""Online fixed-base Zerith control environment built on Drake.

The environment exposes a small Gym-like reset / step API. A policy updates
an eight-dimensional action at policy_dt while a gravity-compensated
joint-space PD servo runs at controller_dt and the Drake plant advances at
physics_dt.
"""

import csv
import dataclasses
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    BodyIndex,
    CameraInfo,
    ClippingRange,
    DepthRange,
    DepthRenderCamera,
    DiagramBuilder,
    LoadModelDirectives,
    MakeRenderEngineVtk,
    Meshcat,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Parser,
    ProcessModelDirectives,
    Quaternion,
    RenderCameraCore,
    RenderEngineVtkParams,
    RigidTransform,
    Role,
    RollPitchYaw,
    RgbdSensor,
    RgbdSensorDiscrete,
    Simulator,
)

from src.online_manipulation._drake_camera_time import sampled_image_time
from src.online_manipulation.controller import CoupledInverseDynamicsServo
from src.online_manipulation.drake_utils import (
    register_package_xml,
    set_free_body_world_pose,
)
from src.online_manipulation.observations import CameraObservation, Pose
from src.online_manipulation.specs import CameraSpec, JointSpec, RendererSpec
from src.zerith_grasp_geometry import add_left_grasp_frame
from src.zerith_gripper_config import GRIPPER_MAX_OPENING_M
from src.zerith_robot_config import (
    ROBOT_BASE_XYZ_METERS,
    ROBOT_BASE_YAW_DEG,
)

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

@dataclasses.dataclass(frozen=True)
class ControlSample:
    """One low-level servo sample recorded at controller frequency."""

    time: float
    q: np.ndarray
    q_desired: np.ndarray
    gravity_torque: np.ndarray
    pd_torque: np.ndarray
    raw_torque: np.ndarray
    applied_torque: np.ndarray
    saturated: np.ndarray


@dataclasses.dataclass(frozen=True)
class Penetration:
    """One active penetration pair involving the Zerith model."""

    depth: float
    frame_a: str
    frame_b: str


def find_package_xml(scene_dmd: Path) -> Path:
    """Find the nearest package.xml containing a scene DMD."""
    for directory in scene_dmd.parents:
        package_xml = directory / "package.xml"
        if package_xml.is_file():
            return package_xml
    raise FileNotFoundError(
        f"Could not find package.xml above {scene_dmd}. "
        "Pass scene_package_xml explicitly."
    )


def _register_package_xml(parser: Parser, package_xml: Path) -> None:
    """Register a ROS-style package.xml in a Drake parser."""
    register_package_xml(parser, package_xml)


def _pose_vector(transform: RigidTransform) -> np.ndarray:
    """Return pose as [x, y, z, qw, qx, qy, qz]."""
    return np.concatenate(
        (
            transform.translation(),
            transform.rotation().ToQuaternion().wxyz(),
        )
    )


def _validate_period_ratio(
    long_period: float,
    short_period: float,
    names: str,
) -> int:
    """Validate an integer timing ratio and return its rounded value."""
    if long_period <= 0.0 or short_period <= 0.0:
        raise ValueError(f"{names} periods must be positive")
    ratio = long_period / short_period
    rounded_ratio = round(ratio)
    if rounded_ratio < 1 or not np.isclose(ratio, rounded_ratio, atol=1e-12):
        raise ValueError(f"{names} must have an integer ratio, got {ratio}")
    return rounded_ratio


class ZerithOnlineEnv:
    """Fixed-base online control environment for Zerith's left arm.

    The action is [delta_q_left[7], gripper_command]. Joint deltas are in
    radians and accumulate on the previously held target. The gripper command
    is absolute and normalized: -1 is closed and +1 is fully open.

    Observation poses use [x, y, z, qw, qx, qy, qz] and spatial velocity uses
    [wx, wy, wz, vx, vy, vz].
    """

    def __init__(
        self,
        *,
        scene_dmd: Path,
        robot_model_dir: Path,
        target_model_name: str | None = None,
        scene_package_xml: Path | None = None,
        additional_package_xmls: Sequence[Path] = (),
        target_body_name: str = "base_link",
        initial_body_poses: Mapping[tuple[str, str], Pose] | None = None,
        contact_parameters: Mapping[str, float] | None = None,
        robot_xyz: Sequence[float] = ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg: float = ROBOT_BASE_YAW_DEG,
        rail_position: float = 0.0,
        q_home: Sequence[float] | None = None,
        physics_dt: float = 0.001,
        controller_dt: float = 0.005,
        policy_dt: float = 0.1,
        episode_duration: float = 30.0,
        max_joint_delta: float = 0.1,
        realtime_rate: float = 0.0,
        meshcat: Meshcat | None = None,
        servo_joint_specs: Sequence[JointSpec] | None = None,
        locked_joint_positions: Mapping[str, float] | None = None,
        camera_specs: Sequence[CameraSpec] = (),
        renderer_spec: RendererSpec = RendererSpec(),
    ):
        """Build the Drake diagram and initialize immutable model metadata."""
        self._scene_dmd = Path(scene_dmd).resolve()
        self._robot_model_dir = Path(robot_model_dir).resolve()
        self._scene_package_xml = (
            Path(scene_package_xml).resolve()
            if scene_package_xml is not None
            else find_package_xml(self._scene_dmd)
        )
        self._additional_package_xmls = tuple(
            Path(path).resolve() for path in additional_package_xmls
        )
        self._initial_body_poses = dict(initial_body_poses or {})
        if any(
            len(name_pair) != 2 or not all(name_pair)
            for name_pair in self._initial_body_poses
        ):
            raise ValueError(
                "initial_body_poses keys must be (model_name, body_name)"
            )
        self._contact_parameters = {
            name: float(value)
            for name, value in (contact_parameters or {}).items()
        }
        unknown_contact_parameters = (
            self._contact_parameters.keys() - SUPPORTED_CONTACT_PARAMETERS
        )
        if unknown_contact_parameters:
            raise ValueError(
                "Unsupported contact parameters: "
                f"{sorted(unknown_contact_parameters)}"
            )
        if any(
            not np.isfinite(value) or value <= 0.0
            for value in self._contact_parameters.values()
        ):
            raise ValueError("Contact parameters must be finite and positive")
        self._robot_urdf = (
            self._robot_model_dir / ZERITH_URDF_RELATIVE_PATH
        )
        if not self._scene_dmd.is_file():
            raise FileNotFoundError(
                f"Scene DMD does not exist: {self._scene_dmd}"
            )
        if not self._scene_package_xml.is_file():
            raise FileNotFoundError(
                f"Scene package.xml does not exist: {self._scene_package_xml}"
            )
        if not self._robot_urdf.is_file():
            raise FileNotFoundError(
                f"Zerith URDF does not exist: {self._robot_urdf}\n"
                "Run python scripts/convert_zerith_for_drake.py."
            )
        for package_xml in self._additional_package_xmls:
            if not package_xml.is_file():
                raise FileNotFoundError(
                    f"Additional package.xml does not exist: {package_xml}"
                )

        self.physics_dt = float(physics_dt)
        self.controller_dt = float(controller_dt)
        self.policy_dt = float(policy_dt)
        self.episode_duration = float(episode_duration)
        self.max_joint_delta = float(max_joint_delta)
        self.realtime_rate = float(realtime_rate)
        if self.episode_duration <= 0.0:
            raise ValueError("episode_duration must be positive")
        if self.max_joint_delta <= 0.0:
            raise ValueError("max_joint_delta must be positive")
        self._physics_steps_per_control = _validate_period_ratio(
            self.controller_dt,
            self.physics_dt,
            "controller_dt / physics_dt",
        )
        self._control_steps_per_policy = _validate_period_ratio(
            self.policy_dt,
            self.controller_dt,
            "policy_dt / controller_dt",
        )

        if q_home is None:
            q_home = np.zeros(len(LEFT_ARM_SERVO_CONFIGS))
        self._q_home = np.asarray(q_home, dtype=float)
        if self._q_home.shape != (len(LEFT_ARM_SERVO_CONFIGS),):
            raise ValueError(
                "q_home must contain exactly seven left-arm joint positions"
            )
        self._rail_position = float(rail_position)
        self._servo_joint_specs = (
            tuple(servo_joint_specs)
            if servo_joint_specs is not None
            else None
        )
        self._locked_joint_positions = {
            str(name): float(value)
            for name, value in (locked_joint_positions or {}).items()
        }
        self._camera_specs = tuple(
            camera for camera in camera_specs if camera.enabled
        )
        self._renderer_spec = renderer_spec
        if self._servo_joint_specs is not None and tuple(
            spec.name for spec in self._servo_joint_specs
        ) != tuple(config.name for config in ALL_SERVO_CONFIGS):
            raise ValueError(
                "servo_joint_specs must match validated Zerith servo order"
            )

        self.meshcat = meshcat
        if self.meshcat is not None:
            self.meshcat.Delete()

        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder,
            time_step=self.physics_dt,
        )
        if "penetration_allowance_m" in self._contact_parameters:
            self.plant.set_penetration_allowance(
                self._contact_parameters["penetration_allowance_m"]
            )
        if "stiction_tolerance_m_s" in self._contact_parameters:
            self.plant.set_stiction_tolerance(
                self._contact_parameters["stiction_tolerance_m_s"]
            )
        parser = Parser(self.plant)
        parser.SetAutoRenaming(True)
        _register_package_xml(parser, self._scene_package_xml)
        for package_xml in self._additional_package_xmls:
            _register_package_xml(parser, package_xml)
        parser.package_map().Add(
            ZERITH_PACKAGE_NAME,
            str(self._robot_model_dir),
        )
        directives = LoadModelDirectives(str(self._scene_dmd))
        ProcessModelDirectives(directives, parser)
        model_instances = parser.AddModels(str(self._robot_urdf))
        if len(model_instances) != 1:
            raise RuntimeError(
                f"Expected one Zerith model, got {len(model_instances)}"
            )
        self._zerith = model_instances[0]
        add_left_grasp_frame(self.plant, self._zerith)
        self._target_body = None
        if target_model_name is not None:
            target_instance = self.plant.GetModelInstanceByName(
                target_model_name
            )
            self._target_body = self.plant.GetBodyByName(
                target_body_name,
                target_instance,
            )
        self._end_effector_body = self.plant.GetBodyByName(
            "left_end_effector_link",
            self._zerith,
        )

        base_frame = self.plant.GetFrameByName("dipan_link", self._zerith)
        self.plant.WeldFrames(
            self.plant.world_frame(),
            base_frame,
            RigidTransform(
                RollPitchYaw(0.0, 0.0, np.deg2rad(robot_yaw_deg)),
                np.asarray(robot_xyz, dtype=float),
            ),
        )

        self._actuators = []
        for index, config in enumerate(ALL_SERVO_CONFIGS):
            joint = self.plant.GetJointByName(config.name, self._zerith)
            effort_limit = (
                self._servo_joint_specs[index].effort_limit
                if self._servo_joint_specs is not None
                else config.effort_limit
            )
            self._actuators.append(
                self.plant.AddJointActuator(
                    f"{config.name}_actuator",
                    joint,
                    effort_limit=effort_limit,
                )
            )

        self.plant.Finalize()
        self._render_label_names = self._collect_render_label_names()
        self._arm_joints = tuple(
            self.plant.GetJointByName(config.name, self._zerith)
            for config in LEFT_ARM_SERVO_CONFIGS
        )
        self._gripper_joints = tuple(
            self.plant.GetJointByName(config.name, self._zerith)
            for config in LEFT_GRIPPER_SERVO_CONFIGS
        )
        self._all_joints = self._arm_joints + self._gripper_joints
        if self._servo_joint_specs is None:
            gripper_names = {
                config.name for config in LEFT_GRIPPER_SERVO_CONFIGS
            }
            self._servo_joint_specs = tuple(
                JointSpec(
                    name=config.name,
                    joint_type=(
                        "prismatic"
                        if config.name in gripper_names
                        else "revolute"
                    ),
                    position_lower=float(
                        joint.position_lower_limits()[0]
                    ),
                    position_upper=float(
                        joint.position_upper_limits()[0]
                    ),
                    velocity_limit=float(joint.velocity_upper_limits()[0]),
                    effort_limit=config.effort_limit,
                    kp=config.kp,
                    kd=config.kd,
                )
                for config, joint in zip(
                    ALL_SERVO_CONFIGS,
                    self._all_joints,
                    strict=True,
                )
            )
        self._servo = CoupledInverseDynamicsServo(
            plant=self.plant,
            joints=self._all_joints,
            actuators=self._actuators,
            joint_specs=self._servo_joint_specs,
        )
        self._rail_joint = self.plant.GetJointByName(
            "daogui_joint",
            self._zerith,
        )
        if not (
            self._rail_joint.position_lower_limits()[0]
            <= self._rail_position
            <= self._rail_joint.position_upper_limits()[0]
        ):
            raise ValueError("rail_position violates daogui_joint limits")
        self._arm_position_lower_limits = np.array(
            [joint.position_lower_limits()[0] for joint in self._arm_joints]
        )
        self._arm_position_upper_limits = np.array(
            [joint.position_upper_limits()[0] for joint in self._arm_joints]
        )
        if np.any(self._q_home < self._arm_position_lower_limits) or np.any(
            self._q_home > self._arm_position_upper_limits
        ):
            raise ValueError("q_home violates a left-arm joint position limit")

        self._camera_systems = {}
        if self._camera_specs:
            if self._renderer_spec.engine != "vtk":
                raise ValueError("Only the vtk renderer is currently supported")
            self.scene_graph.AddRenderer(
                self._renderer_spec.name,
                MakeRenderEngineVtk(RenderEngineVtkParams()),
            )
            for camera_spec in self._camera_specs:
                parent_frame = self.plant.GetFrameByName(
                    camera_spec.parent_frame,
                    self._zerith,
                )
                parent_body = parent_frame.body()
                parent_id = self.plant.GetBodyFrameIdOrThrow(
                    parent_body.index()
                )
                X_BP = parent_frame.GetFixedPoseInBodyFrame()
                X_PO = RigidTransform(
                    Quaternion(
                        camera_spec.X_parent_camera_optical.quaternion_wxyz
                    ),
                    camera_spec.X_parent_camera_optical.translation_m,
                )
                camera_info = CameraInfo(
                    camera_spec.width,
                    camera_spec.height,
                    camera_spec.fov_y_rad,
                )
                core = RenderCameraCore(
                    self._renderer_spec.name,
                    camera_info,
                    ClippingRange(camera_spec.near_m, camera_spec.far_m),
                    RigidTransform(),
                )
                depth_camera = DepthRenderCamera(
                    core,
                    DepthRange(camera_spec.near_m, camera_spec.far_m),
                )
                continuous_sensor = RgbdSensor(
                    parent_id,
                    X_BP @ X_PO,
                    depth_camera,
                    False,
                )
                sensor = builder.AddSystem(
                    RgbdSensorDiscrete(
                        continuous_sensor,
                        period=camera_spec.update_period_s,
                        render_label_image="label" in camera_spec.modalities,
                    )
                )
                sensor.set_name(camera_spec.name)
                builder.Connect(
                    self.scene_graph.get_query_output_port(),
                    sensor.query_object_input_port(),
                )
                self._camera_systems[camera_spec.name] = (
                    camera_spec,
                    sensor,
                )

        if self.meshcat is not None:
            MeshcatVisualizer.AddToBuilder(
                builder,
                self.scene_graph,
                self.meshcat,
            )
            collision_params = MeshcatVisualizerParams()
            collision_params.prefix = "collision"
            collision_params.role = Role.kProximity
            collision_params.visible_by_default = False
            MeshcatVisualizer.AddToBuilder(
                builder,
                self.scene_graph,
                self.meshcat,
                collision_params,
            )

        self.diagram = builder.Build()
        self._simulator: Simulator | None = None
        self._desired_q_left = self._q_home.copy()
        self._desired_gripper_width = GRIPPER_MAX_OPENING
        self._done = False
        self._control_log: list[ControlSample] = []
        self._last_action_clipped = False

    @property
    def control_log(self) -> tuple[ControlSample, ...]:
        """Return low-level controller samples recorded this episode."""
        return tuple(self._control_log)

    @property
    def q_home(self) -> np.ndarray:
        """Return a copy of the configured seven-joint home posture."""
        return self._q_home.copy()

    @property
    def physics_steps_per_control(self) -> int:
        """Return the exact physics-step count per controller update."""
        return self._physics_steps_per_control

    @property
    def robot_model_instance(self):
        """Return the Zerith model instance for compatibility adapters."""
        return self._zerith

    @property
    def plant_context(self):
        """Return the active mutable Plant context."""
        return self._plant_context()

    @property
    def controller_state(self) -> dict[str, np.ndarray]:
        """Return current targets and latest torque-limit telemetry."""
        if not self._control_log:
            raise RuntimeError("Call reset() before reading controller state")
        sample = self._control_log[-1]
        return {
            "q_commanded": sample.q_desired.copy(),
            "torque_commanded": sample.raw_torque.copy(),
            "torque_applied": sample.applied_torque.copy(),
            "torque_saturated": sample.saturated.copy(),
        }

    @property
    def sensor_observations(self) -> dict[str, CameraObservation]:
        """Return the latest sampled-and-held public camera observations."""
        if self._simulator is None:
            raise RuntimeError("Call reset() before reading camera sensors")
        root_context = self._simulator.get_context()
        observations = {}
        for name, (spec, sensor) in self._camera_systems.items():
            context = sensor.GetMyContextFromRoot(root_context)
            modalities = spec.modalities
            rgb = None
            if "rgb" in modalities:
                rgba = sensor.color_image_output_port().Eval(context).data
                rgb = np.asarray(rgba[:, :, :3], dtype=np.uint8)
            depth = None
            if "depth" in modalities:
                depth_data = sensor.depth_image_32F_output_port().Eval(
                    context
                ).data
                depth = np.asarray(depth_data[:, :, 0], dtype=np.float32)
            label = None
            if "label" in modalities:
                label_data = sensor.label_image_output_port().Eval(context).data
                label = np.asarray(label_data[:, :, 0], dtype=np.int16)
            timestamp = sampled_image_time(
                sensor,
                context,
                root_context,
            )
            X_WC = sensor.body_pose_in_world_output_port().Eval(context)
            observations[name] = CameraObservation(
                frame=name,
                timestamp_s=timestamp,
                pose=Pose(
                    tuple(float(value) for value in X_WC.translation()),
                    tuple(
                        float(value)
                        for value in X_WC.rotation().ToQuaternion().wxyz()
                    ),
                ),
                intrinsics=spec.intrinsics,
                rgb=rgb,
                depth=depth,
                label=label,
                label_names=(
                    self._render_label_names if label is not None else {}
                ),
            )
        return observations

    def _collect_render_label_names(self) -> dict[int, str]:
        """Map Drake render labels to model-qualified body names."""
        inspector = self.scene_graph.model_inspector()
        names = {}
        for index in range(self.plant.num_bodies()):
            body = self.plant.get_body(BodyIndex(index))
            frame_id = self.plant.GetBodyFrameIdOrThrow(body.index())
            model_name = self.plant.GetModelInstanceName(
                body.model_instance()
            )
            qualified_name = f"{model_name}::{body.name()}"
            for geometry_id in inspector.GetGeometries(
                frame_id,
                Role.kPerception,
            ):
                properties = inspector.GetPerceptionProperties(geometry_id)
                label = int(properties.GetProperty("label", "id"))
                previous = names.setdefault(label, qualified_name)
                if previous != qualified_name:
                    raise RuntimeError(
                        f"Render label {label} names both {previous} and "
                        f"{qualified_name}"
                    )
        return names

    def _plant_context(self):
        """Return mutable plant context owned by the active simulator."""
        if self._simulator is None:
            raise RuntimeError("Call reset() before accessing the environment")
        return self.plant.GetMyMutableContextFromRoot(
            self._simulator.get_mutable_context()
        )

    def _set_initial_configuration(
        self,
        plant_context,
        initial_body_poses: Mapping[tuple[str, str], Pose] | None = None,
    ) -> None:
        """Set episode initial state before Simulator.Initialize()."""
        positions = self.plant.GetPositions(plant_context).copy()
        for joint, value in zip(
            self._arm_joints,
            self._q_home,
            strict=True,
        ):
            positions[joint.position_start()] = value
        positions[self._rail_joint.position_start()] = self._rail_position
        for name, value in self._locked_joint_positions.items():
            joint = self.plant.GetJointByName(name, self._zerith)
            if joint.num_positions() != 1:
                raise ValueError(
                    f"Locked joint {name} must have one position coordinate"
                )
            positions[joint.position_start()] = value
        # The Zerith finger joints are open at zero. Moving the left finger
        # negative and the right finger positive closes the gripper.
        positions[self._gripper_joints[0].position_start()] = 0.0
        positions[self._gripper_joints[1].position_start()] = 0.0
        self.plant.SetPositions(plant_context, positions)

        body_poses = dict(self._initial_body_poses)
        body_poses.update(initial_body_poses or {})
        for (model_name, body_name), pose in body_poses.items():
            model_instance = self.plant.GetModelInstanceByName(model_name)
            body = self.plant.GetBodyByName(body_name, model_instance)
            quaternion = np.asarray(pose.quaternion_wxyz, dtype=float)
            quaternion /= np.linalg.norm(quaternion)
            set_free_body_world_pose(
                self.plant,
                plant_context,
                body,
                RigidTransform(
                    Quaternion(quaternion),
                    np.asarray(pose.translation_m, dtype=float),
                ),
            )

        controlled_names = {config.name for config in ALL_SERVO_CONFIGS}
        for joint_index in self.plant.GetJointIndices(self._zerith):
            joint = self.plant.get_joint(joint_index)
            if joint.num_velocities() == 0 or joint.name() in controlled_names:
                continue
            joint.Lock(plant_context)

    def reset(
        self,
        initial_body_poses: Mapping[tuple[str, str], Pose] | None = None,
    ) -> dict[str, Any]:
        """Reset the episode and return the initial observation.

        State is assigned only before the new simulator is initialized; no
        state teleportation occurs inside step.
        """
        root_context = self.diagram.CreateDefaultContext()
        plant_context = self.plant.GetMyMutableContextFromRoot(root_context)
        self._set_initial_configuration(plant_context, initial_body_poses)
        self._desired_q_left = self._q_home.copy()
        self._desired_gripper_width = GRIPPER_MAX_OPENING
        self._done = False
        self._control_log.clear()
        self._last_action_clipped = False

        self._simulator = Simulator(self.diagram, root_context)
        self._simulator.set_target_realtime_rate(self.realtime_rate)
        self._update_servo()
        self._simulator.Initialize()
        # Process sample-and-hold camera events scheduled at t=0 so reset()
        # always returns a real first frame rather than zero-filled defaults.
        if self._camera_systems:
            self._simulator.AdvanceTo(0.0)
        self.diagram.ForcedPublish(self._simulator.get_context())
        return self._observation()

    def _measured_controlled_state(
        self,
        plant_context,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return position and velocity vectors for all nine servo joints."""
        positions = self.plant.GetPositions(plant_context)
        velocities = self.plant.GetVelocities(plant_context)
        q = np.array(
            [positions[joint.position_start()] for joint in self._all_joints]
        )
        v = np.array(
            [velocities[joint.velocity_start()] for joint in self._all_joints]
        )
        return q, v

    def _desired_controlled_positions(self) -> np.ndarray:
        """Return held arm and gripper targets in actuator order."""
        inward_travel = 0.5 * (
            GRIPPER_MAX_OPENING - self._desired_gripper_width
        )
        return np.concatenate(
            (
                self._desired_q_left,
                np.array([-inward_travel, inward_travel]),
            )
        )

    def _update_servo(self) -> None:
        """Update coupled inverse-dynamics PD torque and record one sample."""
        if self._simulator is None:
            raise RuntimeError("Call reset() before updating the controller")
        plant_context = self._plant_context()
        q_desired = self._desired_controlled_positions()
        output = self._servo.compute(
            plant_context,
            q_desired,
        )
        self.plant.get_actuation_input_port().FixValue(
            plant_context,
            output.actuation,
        )
        self._control_log.append(
            ControlSample(
                time=float(plant_context.get_time()),
                q=output.q.copy(),
                q_desired=output.q_desired.copy(),
                gravity_torque=output.gravity_torque.copy(),
                pd_torque=output.pd_torque.copy(),
                raw_torque=output.raw_torque.copy(),
                applied_torque=output.applied_torque.copy(),
                saturated=output.saturated.copy(),
            )
        )

    def _contact_count(self, plant_context) -> int:
        """Return number of active point and hydroelastic contacts."""
        results = self.plant.get_contact_results_output_port().Eval(
            plant_context
        )
        return (
            results.num_point_pair_contacts()
            + results.num_hydroelastic_contacts()
        )

    def _observation(self) -> dict[str, Any]:
        """Build the policy observation from the current Drake context."""
        plant_context = self._plant_context()
        q, v = self._measured_controlled_state(plant_context)
        end_effector_pose = self.plant.EvalBodyPoseInWorld(
            plant_context,
            self._end_effector_body,
        )
        observation = {
            "q_left": q[: len(LEFT_ARM_SERVO_CONFIGS)].copy(),
            "v_left": v[: len(LEFT_ARM_SERVO_CONFIGS)].copy(),
            "gripper_width": float(
                GRIPPER_MAX_OPENING - (q[-1] - q[-2])
            ),
            "end_effector_pose": _pose_vector(end_effector_pose),
            "contact_count": self._contact_count(plant_context),
            "simulation_time": float(plant_context.get_time()),
        }
        if self._target_body is not None:
            target_pose = self.plant.EvalBodyPoseInWorld(
                plant_context,
                self._target_body,
            )
            target_velocity = self.plant.EvalBodySpatialVelocityInWorld(
                plant_context,
                self._target_body,
            )
            observation.update(
                red_box_pose=_pose_vector(target_pose),
                red_box_velocity=np.concatenate(
                    (
                        target_velocity.rotational(),
                        target_velocity.translational(),
                    )
                ),
            )
        return observation

    def step(
        self,
        action: Sequence[float],
    ) -> tuple[dict[str, Any], float, bool, dict[str, Any]]:
        """Hold one policy action while advancing lower-rate simulation.

        Args:
            action: Seven joint-target increments in radians followed by one
                absolute normalized gripper command. -1 closes and +1 opens
                the gripper.

        Returns:
            (observation, reward, done, info) after policy_dt seconds. Reward
            is currently zero because task reward belongs to the evaluation
            layer.
        """
        if self._simulator is None:
            raise RuntimeError("Call reset() before step(action)")
        if self._done:
            raise RuntimeError(
                "Episode is done; call reset() before step(action)"
            )
        action_array = np.asarray(action, dtype=float)
        if action_array.shape != (8,):
            raise ValueError(
                f"Expected action shape (8,), got {action_array.shape}"
            )
        if not np.all(np.isfinite(action_array)):
            raise ValueError("Action contains a non-finite value")

        requested_delta = action_array[:7]
        applied_delta = np.clip(
            requested_delta,
            -self.max_joint_delta,
            self.max_joint_delta,
        )
        unclipped_target = self._desired_q_left + applied_delta
        clipped_target = np.clip(
            unclipped_target,
            self._arm_position_lower_limits,
            self._arm_position_upper_limits,
        )
        gripper_command = float(np.clip(action_array[7], -1.0, 1.0))
        self._last_action_clipped = bool(
            not np.array_equal(requested_delta, applied_delta)
            or not np.array_equal(unclipped_target, clipped_target)
            or gripper_command != action_array[7]
        )
        self._desired_q_left = clipped_target
        self._desired_gripper_width = (
            0.5 * GRIPPER_MAX_OPENING * (gripper_command + 1.0)
        )

        control_updates = 0
        for _ in range(self._control_steps_per_policy):
            current_time = self._simulator.get_context().get_time()
            if current_time >= self.episode_duration:
                break
            self._update_servo()
            next_time = min(
                current_time + self.controller_dt,
                self.episode_duration,
            )
            self._simulator.AdvanceTo(next_time)
            control_updates += 1

        observation = self._observation()
        self._done = bool(
            observation["simulation_time"]
            >= self.episode_duration - 1e-12
        )
        last_sample = self._control_log[-1]
        saturated_joint_names = [
            config.name
            for config, saturated in zip(
                ALL_SERVO_CONFIGS,
                last_sample.saturated,
                strict=True,
            )
            if saturated
        ]
        info = {
            "desired_q_left": self._desired_q_left.copy(),
            "desired_gripper_width": self._desired_gripper_width,
            "action_clipped": self._last_action_clipped,
            "saturated_joint_names": saturated_joint_names,
            "control_updates": control_updates,
            "physics_steps_per_control": self._physics_steps_per_control,
        }
        return observation, 0.0, self._done, info

    def robot_penetrations(self) -> list[Penetration]:
        """Return active filtered penetration pairs involving Zerith."""
        plant_context = self._plant_context()
        robot_geometry_ids = set()
        for body_index in self.plant.GetBodyIndices(self._zerith):
            body = self.plant.get_body(body_index)
            robot_geometry_ids.update(
                self.plant.GetCollisionGeometriesForBody(body)
            )

        query_object = self.plant.get_geometry_query_input_port().Eval(
            plant_context
        )
        inspector = query_object.inspector()
        penetrations = []
        for pair in query_object.ComputePointPairPenetration():
            if (
                pair.id_A not in robot_geometry_ids
                and pair.id_B not in robot_geometry_ids
            ):
                continue
            penetrations.append(
                Penetration(
                    depth=pair.depth,
                    frame_a=inspector.GetName(
                        inspector.GetFrameId(pair.id_A)
                    ),
                    frame_b=inspector.GetName(
                        inspector.GetFrameId(pair.id_B)
                    ),
                )
            )
        return sorted(
            penetrations,
            key=lambda penetration: penetration.depth,
            reverse=True,
        )

    def write_control_log(self, output_path: Path) -> None:
        """Write controller-frequency state and torque diagnostics to CSV."""
        output_path = Path(output_path)
        fieldnames = ["time"]
        quantities = (
            "q",
            "q_desired",
            "gravity_torque",
            "pd_torque",
            "raw_torque",
            "applied_torque",
            "saturated",
        )
        for config in ALL_SERVO_CONFIGS:
            for quantity in quantities:
                fieldnames.append(f"{config.name}.{quantity}")

        with output_path.open("w", newline="") as output_file:
            writer = csv.DictWriter(output_file, fieldnames=fieldnames)
            writer.writeheader()
            for sample in self._control_log:
                row: dict[str, float | bool] = {"time": sample.time}
                values = {
                    "q": sample.q,
                    "q_desired": sample.q_desired,
                    "gravity_torque": sample.gravity_torque,
                    "pd_torque": sample.pd_torque,
                    "raw_torque": sample.raw_torque,
                    "applied_torque": sample.applied_torque,
                    "saturated": sample.saturated,
                }
                for joint_index, config in enumerate(ALL_SERVO_CONFIGS):
                    for quantity, vector in values.items():
                        row[f"{config.name}.{quantity}"] = vector[joint_index]
                writer.writerow(row)
