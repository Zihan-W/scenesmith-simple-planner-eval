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

from src.zerith_servo_config import (
    ALL_SERVO_CONFIGS, GRIPPER_MAX_OPENING, JointServoConfig,
    LEFT_ARM_SERVO_CONFIGS, LEFT_GRIPPER_SERVO_CONFIGS,
    SUPPORTED_CONTACT_PARAMETERS, ZERITH_PACKAGE_NAME, ZERITH_URDF_RELATIVE_PATH,
)


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
    """Legacy fixed-left dictionary API, forwarding to the shared DrakeRuntime.

    This preserves old diagnostic script commands. No model construction,
    physics loop or contact solver is implemented here. New clients should use
    make_env and typed actions.
    """

    def __init__(
        self,
        *,
        scene_dmd,
        robot_model_dir,
        target_model_name=None,
        scene_package_xml=None,
        additional_package_xmls=(),
        target_body_name="base_link",
        initial_body_poses=None,
        contact_parameters=None,
        robot_xyz=ROBOT_BASE_XYZ_METERS,
        robot_yaw_deg=ROBOT_BASE_YAW_DEG,
        rail_position=0.0,
        q_home=None,
        physics_dt=0.001,
        controller_dt=0.005,
        policy_dt=0.1,
        episode_duration=30.0,
        max_joint_delta=0.1,
        realtime_rate=0.0,
        meshcat=None,
        servo_joint_specs=None,
        locked_joint_positions=None,
        camera_specs=(),
        renderer_spec=RendererSpec(),
    ):
        """Translate the historical constructor into generic composition."""
        from src.online_manipulation.adapters.zerith import (
            ZerithRobotAdapter,
            ZerithFixedCommandResolver,
            make_zerith_robot_spec,
        )
        from src.online_manipulation.runtime import DrakeRuntime, RuntimeConfig
        from src.online_manipulation.specs import (
            ObservedBodySpec,
            ScenarioSpec,
            TimingConfig,
            VisualizationConfig,
        )

        self.physics_dt, self.controller_dt, self.policy_dt = (
            physics_dt,
            controller_dt,
            policy_dt,
        )
        self.episode_duration, self.max_joint_delta = episode_duration, max_joint_delta
        self._q_home = (
            np.zeros(7) if q_home is None else np.asarray(q_home, dtype=float)
        )
        self._target_name = target_model_name
        self._target_body_name = target_body_name
        self._initial_body_poses = dict(initial_body_poses or {})
        pairs = list(self._initial_body_poses)
        if (
            target_model_name is not None
            and (target_model_name, target_body_name) not in pairs
        ):
            pairs.append((target_model_name, target_body_name))
        self._body_aliases = {
            pair: f"legacy_object_{i}" for i, pair in enumerate(pairs)
        }
        scene = Path(scene_dmd).resolve()
        package = (
            Path(scene_package_xml) if scene_package_xml else find_package_xml(scene)
        )
        spec = make_zerith_robot_spec(
            robot_model_dir=Path(robot_model_dir),
            robot_xyz=robot_xyz,
            robot_yaw_deg=robot_yaw_deg,
            rail_position=rail_position,
            q_home_left=self._q_home,
            cameras=camera_specs,
        )
        if servo_joint_specs is not None:
            spec = dataclasses.replace(spec, controlled_joints=tuple(servo_joint_specs))
        if locked_joint_positions is not None:
            spec = dataclasses.replace(
                spec, locked_joint_positions=dict(locked_joint_positions)
            )
        scenario = ScenarioSpec(
            scene,
            (package,) + tuple(Path(p) for p in additional_package_xmls),
            observed_bodies=tuple(
                ObservedBodySpec(alias, *pair)
                for pair, alias in self._body_aliases.items()
            ),
            initial_object_poses={
                self._body_aliases[pair]: pose
                for pair, pose in self._initial_body_poses.items()
            },
            contact_parameters=contact_parameters or {},
            renderer=renderer_spec,
            visualization=VisualizationConfig(
                enabled=meshcat is not None, realtime_rate=realtime_rate
            ),
        )
        config = RuntimeConfig(
            scenario,
            ZerithRobotAdapter(spec),
            TimingConfig(physics_dt, controller_dt, policy_dt),
            episode_duration=episode_duration,
            maximum_joint_delta=max_joint_delta,
            action_resolver_factory=lambda runtime: ZerithFixedCommandResolver(
                runtime, enable_planning=False, maximum_cartesian_joint_delta=None
            ),
        )
        self.runtime = DrakeRuntime(config, meshcat=meshcat)
        self.plant, self.diagram, self.meshcat = (
            self.runtime.plant,
            self.runtime.diagram,
            self.runtime.meshcat,
        )
        self._last_observation = None

    @property
    def q_home(self):
        """Return the configured legacy arm initial posture."""
        return self._q_home.copy()

    @property
    def robot_model_instance(self):
        """Expose the same model to historical diagnostic scripts."""
        return self.runtime.instance

    @property
    def plant_context(self):
        """Expose the same live context for historical read-only diagnostics."""
        return self.runtime.plant_context

    @property
    def physics_steps_per_control(self):
        """Return the exact multi-rate ratio."""
        return self.runtime.timing.physics_steps_per_controller

    @property
    def control_log(self):
        """Adapt generic servo records into the historical diagnostic schema."""
        return tuple(
            ControlSample(
                time=t,
                q=o.q,
                q_desired=o.q_desired,
                gravity_torque=o.gravity_torque,
                pd_torque=o.pd_torque,
                raw_torque=o.raw_torque,
                applied_torque=o.applied_torque,
                saturated=o.saturated,
            )
            for t, o in self.runtime.control_log
        )

    @property
    def controller_state(self):
        """Return the generic controller's current targets and torque outputs."""
        output = self.runtime.output
        return {
            "q_commanded": self.runtime.command.copy(),
            "torque_commanded": output.raw_torque.copy(),
            "torque_applied": output.applied_torque.copy(),
            "torque_saturated": output.saturated.copy(),
        }

    @property
    def sensor_observations(self):
        """Return the same sampled public camera observations."""
        return self.runtime.cameras.observe(self.runtime.simulator.get_context())

    def reset(self, initial_body_poses=None):
        """Apply explicitly registered object initial states before reset only."""
        poses = dict(self._initial_body_poses)
        poses.update(initial_body_poses or {})
        unknown = poses.keys() - self._body_aliases.keys()
        if unknown:
            raise ValueError(
                f"Register initial object bodies at construction: {unknown}"
            )
        self.runtime.scenario = dataclasses.replace(
            self.runtime.scenario,
            initial_object_poses={
                self._body_aliases[pair]: pose for pair, pose in poses.items()
            },
        )
        self._last_observation, _ = self.runtime.reset(np.random.default_rng(0))
        return self._observation()

    def _observation(self):
        """Preserve the old dictionary schema and end-effector-body pose."""
        obs = self._last_observation
        wrist = self.plant.GetBodyByName(
            "left_end_effector_link", self.runtime.instance
        )
        result = {
            "q_left": np.array(obs.robot.q[:7]),
            "v_left": np.array(obs.robot.v[:7]),
            "gripper_width": obs.robot.gripper_width_m,
            "end_effector_pose": _pose_vector(
                wrist.EvalPoseInWorld(self.plant_context)
            ),
            "contact_count": len(obs.contacts),
            "simulation_time": obs.time_s,
        }
        if self._target_name is not None:
            body = self.plant.GetBodyByName(
                self._target_body_name,
                self.plant.GetModelInstanceByName(self._target_name),
            )
            velocity = body.EvalSpatialVelocityInWorld(self.plant_context)
            result.update(
                red_box_pose=_pose_vector(body.EvalPoseInWorld(self.plant_context)),
                red_box_velocity=np.r_[velocity.rotational(), velocity.translational()],
            )
        return result

    def step(self, action):
        """Translate an 8-vector once; the shared runtime advances all time."""
        from src.online_manipulation.actions import (
            CompositeAction,
            JointDeltaAction,
            GripperAction,
        )
        from src.online_manipulation.contact import FREE_MOTION_CONTACT_POLICY

        action = np.asarray(action, dtype=float)
        if action.shape != (8,) or not np.all(np.isfinite(action)):
            raise ValueError("Expected eight finite action values")
        width = (float(np.clip(action[-1], -1, 1)) + 1) * GRIPPER_MAX_OPENING / 2
        command = CompositeAction(
            JointDeltaAction(
                tuple(j.name for j in LEFT_ARM_SERVO_CONFIGS), tuple(action[:7])
            ),
            GripperAction(width),
        )
        self._last_observation, done, info = self.runtime.step(
            command, FREE_MOTION_CONTACT_POLICY
        )
        q = self.runtime.command
        info.update(
            desired_q_left=q[:7].copy(),
            desired_gripper_width=GRIPPER_MAX_OPENING - q[-1] + q[-2],
            action_clipped=info["action_decision"]["status"] == "adjusted"
            or abs(action[-1]) > 1,
            saturated_joint_names=[
                j.name
                for j, sat in zip(ALL_SERVO_CONFIGS, self.runtime.output.saturated)
                if sat
            ],
            physics_steps_per_control=self.physics_steps_per_control,
        )
        return self._observation(), 0.0, done, info

    def robot_penetrations(self):
        """Return real filtered proximity pairs involving this robot."""
        query = self.plant.get_geometry_query_input_port().Eval(self.plant_context)
        inspector = query.inspector()
        result = []
        for pair in query.ComputePointPairPenetration():
            frames = [inspector.GetFrameId(g) for g in (pair.id_A, pair.id_B)]
            if any(
                self.plant.GetBodyFromFrameId(frame).model_instance()
                == self.runtime.instance
                for frame in frames
            ):
                result.append(
                    Penetration(
                        float(pair.depth),
                        *(inspector.GetName(frame) for frame in frames),
                    )
                )
        return sorted(result, key=lambda pair: pair.depth, reverse=True)

    def write_control_log(self, output_path):
        """Preserve the controller-frequency CSV consumed by diagnostics."""
        quantities = (
            "q",
            "q_desired",
            "gravity_torque",
            "pd_torque",
            "raw_torque",
            "applied_torque",
            "saturated",
        )
        fields = ["time"] + [
            f"{joint.name}.{quantity}"
            for joint in ALL_SERVO_CONFIGS
            for quantity in quantities
        ]
        with Path(output_path).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for sample in self.control_log:
                row = {"time": sample.time}
                row.update(
                    {
                        f"{joint.name}.{quantity}": getattr(sample, quantity)[index]
                        for index, joint in enumerate(ALL_SERVO_CONFIGS)
                        for quantity in quantities
                    }
                )
                writer.writerow(row)
