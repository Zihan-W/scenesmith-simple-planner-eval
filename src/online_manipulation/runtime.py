"""Real description-driven Drake execution behind OnlineManipulationEnv.

The adapter configures the model. This runtime never welds a base, chooses a
robot, or moves a body to satisfy a task. One Simulator owns all physics time.
The fixed baseline is supported for A02/A11, not counted as a mobile mode.
"""

import copy
import dataclasses
from pathlib import Path
from typing import Callable

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Meshcat,
    MeshcatVisualizer,
    Parser,
    Simulator,
)

from src.online_manipulation.actions import (
    HoldAction,
    JointDeltaAction,
    JointPositionAction,
    CartesianDeltaAction,
    CartesianPoseAction,
    CompositeAction,
    GripperAction,
    RobotCommand,
    BaseVelocityAction,
)
from src.online_manipulation.adapters.description import drake_pose, public_pose
from src.online_manipulation.controller import CoupledInverseDynamicsServo
from src.online_manipulation.dmd_finalizer import write_updated_dmd
from src.online_manipulation.drake_utils import set_free_body_world_pose
from src.online_manipulation.environment import OnlineManipulationEnv
from src.online_manipulation.model import populate_model, support_contact_limits
from src.online_manipulation.observations import (
    ContactObservation,
    ObjectObservation,
    Observation,
    SpatialVelocity,
)
from src.online_manipulation.planning import PlanningQuery
from src.online_manipulation.sensors import CameraSystems
from src.online_manipulation.specs import ScenarioSpec, TimingConfig
from src.online_manipulation.tasks import NullTask
from src.online_manipulation.base import PlanarKinematicBase, WheelDrivenDynamicBase
from src.online_manipulation.contact import SupportContactPolicy


@dataclasses.dataclass(frozen=True)
class RuntimeConfig:
    """Compose a robot adapter, scene, control and fresh task factory."""

    scenario: ScenarioSpec
    robot_adapter: object
    timing: TimingConfig = dataclasses.field(default_factory=TimingConfig)
    task_factory: Callable = NullTask
    episode_duration: float = 30.0
    maximum_joint_delta: float = 0.1
    maximum_cartesian_joint_delta: float = 0.02
    action_resolver_factory: Callable | None = None

    def build_environment(self):
        """Construct independently owned runtime and task state."""
        return OnlineManipulationEnv(
            DrakeRuntime(self), copy.deepcopy(self.task_factory())
        )


class DrakeRuntime:
    """Integrate physical joint control without fixed action dimensions."""

    def __init__(self, config, *, meshcat=None):
        self.config, self.scenario, self.timing = config, config.scenario, config.timing
        self.adapter = copy.deepcopy(config.robot_adapter)
        self.spec = self.adapter.spec
        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder, time_step=self.timing.physics_dt
        )
        for key, value in self.scenario.contact_parameters.items():
            if key == "penetration_allowance_m":
                self.plant.set_penetration_allowance(value)
            elif key == "stiction_tolerance_m_s":
                self.plant.set_stiction_tolerance(value)
            else:
                raise ValueError(f"Unknown contact parameter: {key}")
        self.instance = populate_model(Parser(self.plant), self.scenario, self.adapter)
        self.plant.Finalize()
        self.joints = tuple(
            self.plant.GetJointByName(name, self.instance)
            for name in self.spec.controlled_joint_names
        )
        actuators = tuple(
            self.plant.GetJointActuatorByName(f"{j.name()}_actuator", self.instance)
            for j in self.joints
        )
        self.servo = CoupledInverseDynamicsServo(
            plant=self.plant,
            joints=self.joints,
            actuators=actuators,
            joint_specs=self.spec.controlled_joints,
        )
        self.cameras = CameraSystems(
            builder,
            self.plant,
            self.scene_graph,
            self.instance,
            self.spec.cameras,
            self.scenario.renderer,
        )
        self.meshcat, self.visualizer = None, None
        if self.scenario.visualization.enabled:
            self.meshcat = (
                meshcat
                if meshcat is not None
                else Meshcat(self.scenario.visualization.port)
            )
            self.visualizer = MeshcatVisualizer.AddToBuilder(
                builder, self.scene_graph, self.meshcat
            )
        self.diagram = builder.Build()
        self.planning = PlanningQuery(
            diagram=self.diagram,
            plant=self.plant,
            robot_model_instance=self.instance,
            robot_adapter=self.adapter,
            observed_bodies=self.scenario.observed_bodies,
            support_limits_m=support_contact_limits(self.adapter, self.scenario),
        )
        self.simulator = None
        self.control_log = []
        self._done = False
        self.action_resolver = (
            config.action_resolver_factory(self)
            if config.action_resolver_factory
            else None
        )
        self.base_backend = None
        if hasattr(self.adapter, "base_config"):
            cls = (
                PlanarKinematicBase
                if self.adapter.base_mode == "planar_kinematic"
                else WheelDrivenDynamicBase
            )
            self.base_backend = cls(self, self.adapter.base_config)

    @property
    def plant_context(self):
        """Return internal context; policies consume Observation instead."""
        if self.simulator is None:
            raise RuntimeError("Call reset before accessing the runtime")
        return self.plant.GetMyMutableContextFromRoot(
            self.simulator.get_mutable_context()
        )

    def reset(self, rng):
        """Initialize robot and free objects before starting a fresh simulator."""
        root = self.diagram.CreateDefaultContext()
        context = self.plant.GetMyMutableContextFromRoot(root)
        self.adapter.initialize_state(self.plant, context, self.instance)
        randomization_record = {}
        for body_spec in self.scenario.observed_bodies:
            body = self._body(body_spec)
            pose = self.scenario.initial_object_poses.get(body_spec.observation_name)
            transform = drake_pose(pose) if pose else body.EvalPoseInWorld(context)
            randomized = False
            for randomization in self.scenario.pose_randomizations:
                if randomization.observation_name == body_spec.observation_name:
                    from pydrake.all import RigidTransform, RotationMatrix

                    xy = [
                        rng.uniform(*randomization.x_offset_range_m),
                        rng.uniform(*randomization.y_offset_range_m),
                        0,
                    ]
                    yaw = rng.uniform(*randomization.yaw_offset_range_rad)
                    transform = RigidTransform(
                        RotationMatrix.MakeZRotation(yaw) @ transform.rotation(),
                        transform.translation() + np.array(xy),
                    )
                    randomized = True
                    randomization_record[body_spec.observation_name] = {
                        "x_offset_m": float(xy[0]),
                        "y_offset_m": float(xy[1]),
                        "yaw_offset_rad": float(yaw),
                        "pose": public_pose(transform).as_dict(),
                    }
            if pose is not None or randomized:
                set_free_body_world_pose(self.plant, context, body, transform)
        self.command = np.array(self.spec.home_positions)
        if self.action_resolver is not None:
            self.action_resolver.reset()
        self.control_log.clear()
        self._done = False
        self.simulator = Simulator(self.diagram, root)
        if self.base_backend is not None:
            self.base_backend.reset()
        self.simulator.set_target_realtime_rate(
            self.scenario.visualization.realtime_rate
        )
        self._servo_update()
        self.simulator.Initialize()
        if self.cameras.systems:
            self.simulator.AdvanceTo(0.0)
        self.diagram.ForcedPublish(self.simulator.get_context())
        return self._observation(), {
            "policy_dt": self.timing.policy_dt,
            "episode_randomization": randomization_record,
            "base_mode": (
                self.adapter.base_mode
                if hasattr(self.adapter, "base_mode")
                else "fixed"
            ),
        }

    def _body(self, spec):
        return self.plant.GetBodyByName(
            spec.body_name, self.plant.GetModelInstanceByName(spec.model_instance_name)
        )

    def get_planning_query(self):
        """Refresh a planning-only context from the full current physical state."""
        self.planning.synchronize_state(self.plant_context)
        return self.planning

    def _servo_update(self):
        self.output = self.servo.compute(self.plant_context, self.command)
        actuation = self.output.actuation.copy()
        if isinstance(self.base_backend, WheelDrivenDynamicBase):
            actuation += self.base_backend.wheel_actuation(self.timing.controller_dt)
        self.plant.get_actuation_input_port().FixValue(self.plant_context, actuation)
        self.control_log.append((self.plant_context.get_time(), self.output))

    def _candidate(self, action, contact_policy):
        """Resolve one command without mutating held targets or simulation."""
        candidate = self.command.copy()
        if isinstance(action, HoldAction):
            return candidate, {"accepted": True, "reason": "hold"}
        if isinstance(action, RobotCommand):
            return self._combined_candidate(action, contact_policy)
        if isinstance(action, BaseVelocityAction):
            return candidate, {
                "accepted": self.base_backend is not None,
                "reason": "accepted" if self.base_backend is not None else "fixed_base",
            }
        if not isinstance(action, (JointDeltaAction, JointPositionAction)):
            raise TypeError(f"Unsupported action: {type(action).__name__}")
        values = (
            action.deltas if isinstance(action, JointDeltaAction) else action.positions
        )
        for name, value in zip(action.joint_names, values, strict=True):
            index = self.spec.controlled_joint_names.index(name)
            joint = self.spec.controlled_joints[index]
            delta = (
                value
                if isinstance(action, JointDeltaAction)
                else value - self.command[index]
            )
            limit = min(
                self.config.maximum_joint_delta,
                joint.velocity_limit * self.timing.policy_dt,
            )
            candidate[index] = np.clip(
                self.command[index] + np.clip(delta, -limit, limit),
                joint.position_lower,
                joint.position_upper,
            )
        self.planning.synchronize_state(self.plant_context)
        actual = self.plant.GetPositions(self.plant_context)
        start = [actual[j.position_start()] for j in self.joints]
        edge = self.check_command_edge(start, candidate, contact_policy)
        return candidate, {
            "accepted": edge.valid,
            "reason": "accepted" if edge.valid else "joint_edge_rejected",
            "edge": dataclasses.asdict(edge),
        }

    def _combined_candidate(self, action, contact_policy):
        """Resolve all components at one state, then check the joint edge jointly."""
        self.planning.synchronize_state(self.plant_context)
        candidate = self.command.copy()
        components = {}
        actual = self.plant.GetPositions(self.plant_context)
        start = np.array([actual[j.position_start()] for j in self.joints])
        cartesian_seed = self.command.copy()
        for gripper in self.spec.grippers.values():
            for name in gripper.joint_names:
                index = self.spec.controlled_joint_names.index(name)
                cartesian_seed[index] = start[index]
        if action.base is not None and self.base_backend is None:
            components["base"] = {"accepted": False, "reason": "fixed_base"}
        for group, command in action.arms.items():
            key = f"arm:{group}"
            if group not in self.spec.arm_groups:
                components[key] = {"accepted": False, "reason": "unknown_arm"}
                continue
            names = self.spec.arm_groups[group]
            if isinstance(command, HoldAction):
                components[key] = {"accepted": True, "reason": "hold"}
                continue
            if isinstance(command, (JointDeltaAction, JointPositionAction)):
                if not set(command.joint_names).issubset(names):
                    components[key] = {
                        "accepted": False,
                        "reason": "joint_not_in_group",
                    }
                    continue
                values = (
                    command.deltas
                    if isinstance(command, JointDeltaAction)
                    else command.positions
                )
                for name, value in zip(command.joint_names, values, strict=True):
                    index = self.spec.controlled_joint_names.index(name)
                    joint = self.spec.controlled_joints[index]
                    delta = (
                        value
                        if isinstance(command, JointDeltaAction)
                        else value - self.command[index]
                    )
                    limit = min(
                        self.config.maximum_joint_delta,
                        joint.velocity_limit * self.timing.policy_dt,
                    )
                    candidate[index] = np.clip(
                        self.command[index] + np.clip(delta, -limit, limit),
                        joint.position_lower,
                        joint.position_upper,
                    )
            elif isinstance(command, (CartesianDeltaAction, CartesianPoseAction)):
                frame_name = self.spec.end_effector_frames[group]
                if (
                    command.reference_frame != "world"
                    or command.end_effector_frame != frame_name
                ):
                    components[key] = {"accepted": False, "reason": "unsupported_frame"}
                    continue
                if isinstance(command, CartesianDeltaAction):
                    translation, rotation = (
                        command.translation_m,
                        command.rotation_vector_rad,
                    )
                else:
                    pose = self.planning.frame_pose_at(
                        cartesian_seed, self.spec.model_instance_name, frame_name
                    )
                    translation = (
                        np.asarray(command.pose.translation_m) - pose.translation_m
                    )
                    relative = (
                        drake_pose(command.pose).rotation()
                        @ drake_pose(pose).rotation().inverse()
                    )
                    aa = relative.ToAngleAxis()
                    rotation = aa.axis() * aa.angle()
                # Each arm uses the same commanded FK seed; the final joint
                # candidate is validated as a whole, not as independent edges.
                result = self.planning.differential_ik_step(
                    translation_m=translation,
                    rotation_vector_rad=rotation,
                    frame_name=frame_name,
                    seed=cartesian_seed,
                    maximum_joint_delta=self.config.maximum_cartesian_joint_delta,
                    validation_start=start,
                    contact_policy=contact_policy,
                    active_joint_names=names,
                )
                for name in names:
                    index = self.spec.controlled_joint_names.index(name)
                    candidate[index] = result.configuration[index]
            else:
                raise TypeError(f"Unsupported arm action: {type(command).__name__}")
            components[key] = {"accepted": True, "reason": "resolved"}
        for name, command in action.grippers.items():
            key = f"gripper:{name}"
            if name not in self.spec.grippers:
                components[key] = {"accepted": False, "reason": "unknown_gripper"}
                continue
            gripper = self.spec.grippers[name]
            if command.maximum_effort_n is not None:
                components[key] = {
                    "accepted": False,
                    "reason": "per_action_effort_unsupported",
                }
                continue
            if (
                not gripper.minimum_width_m
                <= command.width_m
                <= gripper.maximum_width_m
            ):
                components[key] = {"accepted": False, "reason": "width_out_of_range"}
                continue
            for joint_name, value in self.adapter.gripper_position_targets(
                command.width_m, name
            ).items():
                candidate[self.spec.controlled_joint_names.index(joint_name)] = value
            components[key] = {"accepted": True, "reason": "resolved"}
        if any(not c["accepted"] for c in components.values()):
            return candidate, {
                "accepted": False,
                "reason": "component_rejected",
                "components": components,
            }
        edge = self.check_command_edge(start, candidate, contact_policy)
        return candidate, {
            "accepted": edge.valid,
            "reason": "accepted" if edge.valid else "joint_edge_rejected",
            "components": components,
            "edge": dataclasses.asdict(edge),
        }

    def step(self, action, contact_policy):
        """Commit all accepted targets once and advance one policy period."""
        if self.simulator is None or self._done:
            raise RuntimeError("Call reset before step (or after episode end)")
        if isinstance(self.base_backend, WheelDrivenDynamicBase):
            support_names = (
                self.adapter.support_body_names + self.adapter.wheel_body_names
            )
            limits = {
                tuple(
                    sorted((f"{self.spec.model_instance_name}::{body}", ground))
                ): self.adapter.base_config.support_contact_allowance_m
                for body in support_names
                for ground in self.scenario.ground_body_names
            }
            contact_policy = SupportContactPolicy(contact_policy, limits)
        if isinstance(action, RobotCommand):
            candidate, decision = self._combined_candidate(action, contact_policy)
        elif self.action_resolver is not None:
            candidate, decision = self.action_resolver.resolve(action, contact_policy)
        else:
            candidate, decision = self._candidate(action, contact_policy)
        base_action = (
            action
            if isinstance(action, BaseVelocityAction)
            else action.base if isinstance(action, RobotCommand) else None
        )
        if self.base_backend is not None:
            base_rejection = self.base_backend.validate_command(base_action)
            if base_rejection:
                decision.update(accepted=False, reason=base_rejection)
        if decision["accepted"]:
            self.command = candidate
        if self.base_backend is not None:
            self.base_backend.command(base_action if decision["accepted"] else None)
        start = float(self.plant_context.get_time())
        updates = 0
        for _ in range(self.timing.controller_steps_per_policy):
            now = self.plant_context.get_time()
            if now >= self.config.episode_duration - 1e-12:
                break
            self._servo_update()
            if isinstance(self.base_backend, PlanarKinematicBase):
                for _ in range(self.timing.physics_steps_per_controller):
                    t = self.plant_context.get_time()
                    if t >= self.config.episode_duration - 1e-12:
                        break
                    dt = min(self.timing.physics_dt, self.config.episode_duration - t)
                    self.base_backend.before_physics(dt)
                    self.simulator.AdvanceTo(t + dt)
            else:
                self.simulator.AdvanceTo(
                    min(now + self.timing.controller_dt, self.config.episode_duration)
                )
            updates += 1
        observation = self._observation()
        self._done = observation.time_s >= self.config.episode_duration - 1e-12
        return (
            observation,
            self._done,
            {
                "action_decision": decision,
                "step_start_time_s": start,
                "control_updates": updates,
                "physics_steps_per_control": self.timing.physics_steps_per_controller,
            },
        )

    def check_command_edge(self, start, candidate, contact_policy):
        """Check carried geometry, retaining measured fingers during squeeze.

        A Task-attested grasp can request further compliant closing against an
        object. Its servo target is not a feasible finger pose. Use measured
        finger positions for that closing direction only; opening and all arm
        motion still use candidate geometry. No physical pose/filter is changed.
        """
        geometry = np.array(candidate, dtype=float)
        grippers = self.spec.grippers or (
            {None: self.spec.gripper} if self.spec.gripper is not None else {})
        for name, gripper in grippers.items():
            fingers = [f"{self.spec.model_instance_name}::{body}"
                       for body in gripper.contact_body_names]
            held = any(
                fingers and all(contact_policy.permits(finger, carried.body_name)
                                for finger in fingers)
                for carried in contact_policy.carried_bodies)
            if not held:
                continue
            opened = self.adapter.gripper_position_targets(gripper.maximum_width_m, name)
            indices = [self.spec.controlled_joint_names.index(n) for n in gripper.joint_names]
            if all(abs(candidate[i] - opened[n]) >= abs(start[i] - opened[n])
                   for i, n in zip(indices, gripper.joint_names, strict=True)):
                geometry[indices] = np.asarray(start)[indices]
        return self.planning.check_edge(start, geometry, contact_policy=contact_policy)

    def _observation(self):
        context = self.plant_context
        robot = self.adapter.make_robot_observation(
            self.plant,
            context,
            self.instance,
            {
                "q_commanded": tuple(self.command),
                "torque_commanded": tuple(self.output.raw_torque),
                "torque_applied": tuple(self.output.applied_torque),
                "torque_saturated": tuple(self.output.saturated),
            },
        )
        objects = {}
        for spec in self.scenario.observed_bodies:
            body = self._body(spec)
            velocity = body.EvalSpatialVelocityInWorld(context)
            objects[spec.observation_name] = ObjectObservation(
                public_pose(body.EvalPoseInWorld(context)),
                SpatialVelocity(
                    tuple(velocity.rotational()), tuple(velocity.translational())
                ),
            )
        query = self.plant.get_geometry_query_input_port().Eval(context)
        inspector = query.inspector()

        def body_name(geometry):
            body = self.plant.GetBodyFromFrameId(inspector.GetFrameId(geometry))
            return f"{self.plant.GetModelInstanceName(body.model_instance())}::{body.name()}"

        contacts = tuple(
            ContactObservation(body_name(p.id_A), body_name(p.id_B), float(p.depth))
            for p in query.ComputePointPairPenetration()
        )
        base = self.base_backend.observe() if self.base_backend else {}
        if isinstance(self.base_backend, PlanarKinematicBase):
            # The ideal PlanarJoint is locked in the dynamics solve. Report
            # the prescribed base contribution to TCP world twist explicitly.
            omega = np.array([0.0, 0.0, base["yaw_rate_rad_s"]])
            offset = (
                np.array(robot.end_effector_pose.translation_m)
                - base["pose"]["translation_m"]
            )
            twist = robot.end_effector_twist
            robot = dataclasses.replace(
                robot,
                end_effector_twist=SpatialVelocity(
                    tuple(np.array(twist.rotational_rad_s) + omega),
                    tuple(
                        np.array(twist.translational_m_s)
                        + base["linear_velocity_world_m_s"]
                        + np.cross(omega, offset)
                    ),
                ),
            )
        return Observation(
            time_s=float(context.get_time()),
            robot=robot,
            objects=objects,
            contacts=contacts,
            task={},
            sensors=self.cameras.observe(self.simulator.get_context()),
            base=base,
        )

    def check_base_edge(self, start, end):
        """Sample full robot geometry for a prescribed planar increment.

        Ground support is excluded only here, from locomotion obstacle checks.
        Robot self-collision filters and dynamic contact properties are unchanged.
        """
        self.planning.synchronize_state(self.plant_context)
        context = self.planning.context
        joint = self.plant.GetJointByName(self.adapter.planar_joint_name, self.instance)
        robot_bodies = set(self.plant.GetBodyIndices(self.instance))
        failures = []
        # Physics increments are small, but always include the entire edge.
        for alpha in np.linspace(0, 1, 3):
            pose = start + alpha * (end - start)
            joint.set_translation(context, pose[:2])
            joint.set_rotation(context, pose[2])
            query = self.plant.get_geometry_query_input_port().Eval(context)
            inspector = query.inspector()
            for p in query.ComputeSignedDistancePairwiseClosestPoints(0.005):
                a = self.plant.GetBodyFromFrameId(inspector.GetFrameId(p.id_A))
                b = self.plant.GetBodyFromFrameId(inspector.GetFrameId(p.id_B))
                if (a.index() in robot_bodies) == (b.index() in robot_bodies):
                    continue
                environment = b if a.index() in robot_bodies else a
                environment_name = f"{self.plant.GetModelInstanceName(environment.model_instance())}::{environment.name()}"
                if environment_name in self.scenario.ground_body_names:
                    continue
                failures.append(
                    {
                        "body_a": a.name(),
                        "body_b": b.name(),
                        "distance_m": float(p.distance),
                        "alpha": float(alpha),
                    }
                )
            if failures:
                return False, failures
        return True, []

    def write_updated_scenario(self, output_path):
        """Write selected free-object state into a new DMD."""
        return write_updated_dmd(
            input_path=self.scenario.dmd_path,
            output_path=Path(output_path),
            plant=self.plant,
            plant_context=self.plant_context,
            body_specs=self.scenario.observed_bodies,
        )

    def start_recording(self):
        """Begin optional Meshcat recording."""
        if self.visualizer is None:
            raise RuntimeError("Enable visualization before recording")
        self.visualizer.StartRecording()

    def save_recording(self, output_path):
        """Publish and save the actual simulator animation."""
        if self.visualizer is None:
            raise RuntimeError("Enable visualization before recording")
        self.visualizer.StopRecording()
        self.visualizer.PublishRecording()
        Path(output_path).write_text(self.meshcat.StaticHtml())
