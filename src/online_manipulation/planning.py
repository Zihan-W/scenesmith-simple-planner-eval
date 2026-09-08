"""Read-only planning queries backed by an independent Drake context."""

import dataclasses
import math
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np
from pydrake.all import (
    InverseKinematics,
    JacobianWrtVariable,
    LoadModelDirectives,
    ProcessModelDirectives,
    Quaternion,
    RigidTransform,
    RotationMatrix,
    Solve,
)
from pydrake.planning import RobotDiagramBuilder

from src.online_manipulation.contact import (
    CarriedBody,
    FREE_MOTION_CONTACT_POLICY,
    penetration_limit,
    SupportContactPolicy,
)
from src.online_manipulation.drake_utils import (
    register_package_xml,
    set_free_body_world_pose,
)
from src.online_manipulation.observations import Pose
from src.online_manipulation.protocols import ContactPolicy, RobotAdapter
from src.online_manipulation.specs import (
    ObservedBodySpec,
    ScenarioSpec,
    TimingConfig,
)


@dataclasses.dataclass(frozen=True)
class CollisionPair:
    """One robot-related signed-distance pair."""

    distance_m: float
    body_a: str
    body_b: str
    geometry_a: str
    geometry_b: str


@dataclasses.dataclass(frozen=True)
class ClearanceMetrics:
    """Separate nonpenetration and non-allowed safety distances."""

    minimum_nonpenetration_distance_m: float
    minimum_safety_clearance_m: float
    nearest_nonpenetration_pair: CollisionPair | None
    nearest_safety_pair: CollisionPair | None
    influence_distance_m: float


@dataclasses.dataclass(frozen=True)
class ConfigurationCheck:
    """Validity and clearance diagnostics for one configuration."""

    valid: bool
    configuration: tuple[float, ...]
    clearance: ClearanceMetrics
    within_joint_limits: bool = True
    nonpenetration_valid: bool = True
    safety_clearance_valid: bool = True


@dataclasses.dataclass(frozen=True)
class EdgeCheck:
    """High-density numerical validation of a joint-space edge."""

    valid: bool
    sample_count: int
    minimum_nonpenetration_distance_m: float
    minimum_safety_clearance_m: float
    minimum_nonpenetration_alpha: float
    minimum_safety_alpha: float
    limiting_nonpenetration_pair: CollisionPair | None = None
    limiting_pair_start_distance_m: float | None = None
    limiting_pair_end_distance_m: float | None = None
    limiting_pair_sample_distances_m: tuple[float | None, ...] = ()
    limiting_pair_monotonic_non_decreasing: bool | None = None
    minimum_nonpenetration_margin_m: float = float("inf")
    minimum_nonpenetration_margin_alpha: float = 0.0
    joint_limits_valid: bool = True
    nonpenetration_valid: bool = True
    safety_clearance_valid: bool = True


@dataclasses.dataclass(frozen=True)
class IkResult:
    """One kinematic IK result with independent collision validation."""

    success: bool
    reason: str
    configuration: tuple[float, ...]
    solver_result: str
    position_error_m: float
    orientation_error_rad: float
    clearance: ClearanceMetrics


@dataclasses.dataclass(frozen=True)
class DifferentialIkResult:
    """One bounded online differential-IK step and edge validation."""

    success: bool
    reason: str
    configuration: tuple[float, ...]
    requested_twist: tuple[float, ...]
    achieved_twist: tuple[float, ...]
    joint_delta_scaled: bool
    edge: EdgeCheck
    validation_start_configuration: tuple[float, ...] = ()
    validation_edge_translation_m: tuple[float, float, float] = (
        0.0,
        0.0,
        0.0,
    )


def _qualified_body_name(plant: Any, body: Any) -> str:
    """Return a stable ``model_instance::body`` name."""
    return (
        f"{plant.GetModelInstanceName(body.model_instance())}::{body.name()}"
    )


def _pose(transform: Any) -> Pose:
    """Convert a Drake transform to the public pose representation."""
    return Pose(
        tuple(float(value) for value in transform.translation()),
        tuple(
            float(value)
            for value in transform.rotation().ToQuaternion().wxyz()
        ),
    )


def _rigid_transform(pose: Pose) -> RigidTransform:
    """Convert a public pose into a Drake rigid transform."""
    quaternion = np.asarray(pose.quaternion_wxyz, dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return RigidTransform(
        Quaternion(quaternion),
        np.asarray(pose.translation_m, dtype=float),
    )


class PlanningQuery:
    """Query one robot without advancing the environment's real context."""

    def __init__(
        self,
        *,
        diagram: Any,
        plant: Any,
        robot_model_instance: Any,
        robot_adapter: RobotAdapter,
        observed_bodies: Sequence[ObservedBodySpec] = (),
        support_limits_m: Mapping[tuple[str, str], float] | None = None,
    ):
        """Create and initialize a context owned only by planning queries."""
        self.diagram = diagram
        self.plant = plant
        self.robot_model_instance = robot_model_instance
        self.robot_adapter = robot_adapter
        self.observed_bodies = tuple(observed_bodies)
        self.support_limits_m = dict(support_limits_m or {})
        self._root_context = diagram.CreateDefaultContext()
        self._plant_context = plant.GetMyMutableContextFromRoot(
            self._root_context
        )
        robot_adapter.initialize_state(
            plant,
            self._plant_context,
            robot_model_instance,
        )
        self._controlled_joints = tuple(
            plant.GetJointByName(spec.name, robot_model_instance)
            for spec in robot_adapter.spec.controlled_joints
        )
        self._controlled_indices = np.asarray(
            [joint.position_start() for joint in self._controlled_joints],
            dtype=int,
        )
        self._controlled_velocity_indices = np.asarray(
            [joint.velocity_start() for joint in self._controlled_joints],
            dtype=int,
        )
        self._base_positions = plant.GetPositions(self._plant_context).copy()
        self._robot_body_indices = set(
            plant.GetBodyIndices(robot_model_instance)
        )
        gripper_names = (
            set(robot_adapter.spec.gripper.joint_names)
            if robot_adapter.spec.gripper is not None
            else set()
        )
        for gripper_spec in robot_adapter.spec.grippers.values():
            gripper_names.update(gripper_spec.joint_names)
        safety_active_joint_names = {
            spec.name
            for spec in robot_adapter.spec.controlled_joints
            if spec.name not in gripper_names
        }
        self._safety_joint_influences = self._joint_influences(
            safety_active_joint_names
        )
        self._safety_exempt_pairs = set(
            robot_adapter.spec.safety_exempt_body_pairs
        )

    @property
    def context(self) -> Any:
        """Return the independent mutable Plant context for advanced users."""
        return self._plant_context

    def synchronize_state(self, actual_context: Any) -> None:
        """Copy full actual state into planning only, never into simulation."""
        self._base_positions = self.plant.GetPositions(actual_context).copy()
        self.plant.SetPositions(self._plant_context, self._base_positions)
        self.plant.SetVelocities(self._plant_context, self.plant.GetVelocities(actual_context))
        self._root_context.SetTime(actual_context.get_time())

    @property
    def state_time_s(self) -> float:
        """Simulation time of the last synchronized planning snapshot."""
        return float(self._root_context.get_time())

    def _contact_policy(self, policy):
        """Keep robot support limits separate from task-specific contacts."""
        if self.support_limits_m and not isinstance(policy, SupportContactPolicy):
            return SupportContactPolicy(policy, self.support_limits_m)
        return policy

    def configuration(self) -> tuple[float, ...]:
        """Return controlled joint positions in RobotSpec order."""
        positions = self.plant.GetPositions(self._plant_context)
        return tuple(float(positions[index]) for index in self._controlled_indices)

    def joint_limits(self) -> dict[str, tuple[float, float]]:
        """Return configured position limits keyed by joint name."""
        return {
            spec.name: (spec.position_lower, spec.position_upper)
            for spec in self.robot_adapter.spec.controlled_joints
        }

    def set_observed_body_poses(
        self,
        poses: Mapping[str, Pose],
    ) -> None:
        """Synchronize mutable observed bodies into the planning context.

        Fixed observed bodies already have the same pose in the independently
        loaded planning model and are therefore left unchanged. Articulated
        bodies cannot be synchronized from a body pose alone and fail loudly.
        """
        expected = {spec.observation_name for spec in self.observed_bodies}
        unknown = poses.keys() - expected
        if unknown:
            raise KeyError(f"Unknown observed body poses: {sorted(unknown)}")
        for spec in self.observed_bodies:
            if spec.observation_name not in poses:
                continue
            model_instance = self.plant.GetModelInstanceByName(
                spec.model_instance_name
            )
            body = self.plant.GetBodyByName(spec.body_name, model_instance)
            child_joints = [
                self.plant.get_joint(index)
                for index in self.plant.GetJointIndices(model_instance)
                if self.plant.get_joint(index).child_body().index()
                == body.index()
            ]
            floating_joints = [
                joint
                for joint in child_joints
                if joint.num_positions() == 7
                and joint.num_velocities() == 6
            ]
            if len(floating_joints) > 1:
                raise ValueError(
                    f"Observed body has multiple floating joints: "
                    f"{spec.model_instance_name}::{spec.body_name}"
                )
            if not floating_joints:
                if all(joint.num_velocities() == 0 for joint in child_joints):
                    continue
                raise ValueError(
                    f"Observed body is articulated rather than free: "
                    f"{spec.model_instance_name}::{spec.body_name}"
                )
            pose = poses[spec.observation_name]
            quaternion = np.asarray(pose.quaternion_wxyz, dtype=float)
            quaternion /= np.linalg.norm(quaternion)
            world_from_body = RigidTransform(
                Quaternion(quaternion),
                np.asarray(pose.translation_m, dtype=float),
            )
            set_free_body_world_pose(
                self.plant,
                self._plant_context,
                body,
                world_from_body,
            )
        current_positions = self.plant.GetPositions(
            self._plant_context
        )
        fixed_indices = np.setdiff1d(
            np.arange(self.plant.num_positions()),
            self._controlled_indices,
        )
        self._base_positions[fixed_indices] = current_positions[fixed_indices]

    def body_pose(self, model_instance_name: str, body_name: str) -> Pose:
        """Return a body's world pose from the planning context."""
        model_instance = self.plant.GetModelInstanceByName(
            model_instance_name
        )
        body = self.plant.GetBodyByName(body_name, model_instance)
        return _pose(
            self.plant.EvalBodyPoseInWorld(self._plant_context, body)
        )

    def frame_pose(self, model_instance_name: str, frame_name: str) -> Pose:
        """Return a frame's world pose from the planning context."""
        model_instance = self.plant.GetModelInstanceByName(
            model_instance_name
        )
        frame = self.plant.GetFrameByName(frame_name, model_instance)
        transform = self.plant.CalcRelativeTransform(
            self._plant_context,
            self.plant.world_frame(),
            frame,
        )
        return _pose(transform)

    def frame_pose_at(
        self,
        configuration: Sequence[float],
        model_instance_name: str,
        frame_name: str,
    ) -> Pose:
        """Return a frame pose at a controlled configuration."""
        self._set_configuration(configuration)
        return self.frame_pose(model_instance_name, frame_name)

    def collision_pairs(
        self,
        configuration: Sequence[float],
        *,
        influence_distance_m: float = 0.05,
        additional_body_names: Sequence[str] = (),
        carried_bodies: Sequence[CarriedBody] = (),
    ) -> tuple[CollisionPair, ...]:
        """Return robot/carried-body distances within an influence range."""
        if influence_distance_m <= 0.0:
            raise ValueError("influence_distance_m must be positive")
        self._set_configuration(configuration)
        self._set_carried_body_poses(carried_bodies)
        query = self.plant.get_geometry_query_input_port().Eval(
            self._plant_context
        )
        inspector = query.inspector()
        additional_body_indices = {
            self._body_from_qualified_name(name).index()
            for name in additional_body_names
        }
        pairs = []
        for pair in query.ComputeSignedDistancePairwiseClosestPoints(
            max_distance=influence_distance_m
        ):
            body_a = self.plant.GetBodyFromFrameId(
                inspector.GetFrameId(pair.id_A)
            )
            body_b = self.plant.GetBodyFromFrameId(
                inspector.GetFrameId(pair.id_B)
            )
            if (
                body_a.index() not in self._robot_body_indices
                and body_b.index() not in self._robot_body_indices
                and body_a.index() not in additional_body_indices
                and body_b.index() not in additional_body_indices
            ):
                continue
            pairs.append(
                CollisionPair(
                    distance_m=float(pair.distance),
                    body_a=_qualified_body_name(self.plant, body_a),
                    body_b=_qualified_body_name(self.plant, body_b),
                    geometry_a=inspector.GetName(pair.id_A),
                    geometry_b=inspector.GetName(pair.id_B),
                )
            )
        return tuple(sorted(pairs, key=lambda item: item.distance_m))

    def clearance(
        self,
        configuration: Sequence[float],
        *,
        contact_policy: ContactPolicy = FREE_MOTION_CONTACT_POLICY,
        influence_distance_m: float = 0.05,
    ) -> ClearanceMetrics:
        """Return strict nonpenetration and policy-filtered safety layers."""
        contact_policy = self._contact_policy(contact_policy)
        pairs = self.collision_pairs(
            configuration,
            influence_distance_m=influence_distance_m,
            additional_body_names=tuple(contact_policy.monitored_bodies),
            carried_bodies=contact_policy.carried_bodies,
        )
        return self._clearance_from_pairs(
            pairs,
            contact_policy=contact_policy,
            influence_distance_m=influence_distance_m,
        )

    def check_configuration(
        self,
        configuration: Sequence[float],
        *,
        contact_policy: ContactPolicy = FREE_MOTION_CONTACT_POLICY,
        minimum_safety_clearance_m: float = 0.005,
        influence_distance_m: float = 0.05,
    ) -> ConfigurationCheck:
        """Check joint limits, nonpenetration, and safety clearance."""
        return self._check_configuration(
            configuration,
            contact_policy=contact_policy,
            minimum_safety_clearance_m=minimum_safety_clearance_m,
            influence_distance_m=influence_distance_m,
        )[0]

    def _check_configuration(
        self,
        configuration: Sequence[float],
        *,
        contact_policy: ContactPolicy,
        minimum_safety_clearance_m: float,
        influence_distance_m: float,
    ) -> tuple[ConfigurationCheck, tuple[CollisionPair, ...]]:
        """Return one configuration check and its evaluated distance pairs."""
        contact_policy = self._contact_policy(contact_policy)
        values = self._configuration_array(configuration)
        within_limits = all(
            # The discrete contact solver can put an open finger ~1e-11 m
            # outside its stop. This is a numerical state-check tolerance,
            # not a relaxed command limit or collision clearance.
            spec.position_lower - 1e-9 <= value <= spec.position_upper + 1e-9
            for spec, value in zip(
                self.robot_adapter.spec.controlled_joints,
                values,
                strict=True,
            )
        )
        pairs = self.collision_pairs(
            values,
            influence_distance_m=influence_distance_m,
            additional_body_names=tuple(contact_policy.monitored_bodies),
            carried_bodies=contact_policy.carried_bodies,
        )
        clearance = self._clearance_from_pairs(
            pairs,
            contact_policy=contact_policy,
            influence_distance_m=influence_distance_m,
        )
        nonpenetrating = all(
            pair.distance_m
            >= (
                -penetration_limit(contact_policy, pair.body_a, pair.body_b)
                if contact_policy.permits(pair.body_a, pair.body_b)
                else 0.0
            )
            for pair in pairs
        )
        safety_clearance_valid = (
            clearance.minimum_safety_clearance_m
            >= minimum_safety_clearance_m
        )
        return ConfigurationCheck(
            valid=bool(
                within_limits
                and nonpenetrating
                and safety_clearance_valid
            ),
            configuration=tuple(float(value) for value in values),
            clearance=clearance,
            within_joint_limits=bool(within_limits),
            nonpenetration_valid=bool(nonpenetrating),
            safety_clearance_valid=bool(safety_clearance_valid),
        ), pairs

    def check_edge(
        self,
        start: Sequence[float],
        end: Sequence[float],
        *,
        contact_policy: ContactPolicy = FREE_MOTION_CONTACT_POLICY,
        minimum_safety_clearance_m: float = 0.005,
        influence_distance_m: float = 0.05,
        maximum_joint_step: float = 0.002,
    ) -> EdgeCheck:
        """Densely sample one joint-space edge without advancing simulation."""
        contact_policy = self._contact_policy(contact_policy)
        if maximum_joint_step <= 0.0:
            raise ValueError("maximum_joint_step must be positive")
        q_start = self._configuration_array(start)
        q_end = self._configuration_array(end)
        sample_count = max(
            2,
            int(
                math.ceil(
                    float(np.max(np.abs(q_end - q_start)))
                    / maximum_joint_step
                )
            )
            + 1,
        )
        minimum_nonpenetration = float("inf")
        minimum_safety = float("inf")
        minimum_nonpenetration_alpha = 0.0
        minimum_safety_alpha = 0.0
        limiting_nonpenetration_pair = None
        minimum_nonpenetration_margin = float("inf")
        minimum_nonpenetration_margin_alpha = 0.0
        sampled_pairs = []
        joint_limits_valid = True
        nonpenetration_valid = True
        safety_clearance_valid = True
        valid = True
        for alpha in np.linspace(0.0, 1.0, sample_count):
            check, pairs = self._check_configuration(
                q_start + alpha * (q_end - q_start),
                contact_policy=contact_policy,
                minimum_safety_clearance_m=minimum_safety_clearance_m,
                influence_distance_m=influence_distance_m,
            )
            sampled_pairs.append(pairs)
            valid = valid and check.valid
            joint_limits_valid = (
                joint_limits_valid and check.within_joint_limits
            )
            nonpenetration_valid = (
                nonpenetration_valid and check.nonpenetration_valid
            )
            safety_clearance_valid = (
                safety_clearance_valid
                and check.safety_clearance_valid
            )
            if (
                check.clearance.minimum_nonpenetration_distance_m
                < minimum_nonpenetration
            ):
                minimum_nonpenetration = (
                    check.clearance.minimum_nonpenetration_distance_m
                )
                minimum_nonpenetration_alpha = float(alpha)
            if (
                check.clearance.minimum_safety_clearance_m
                < minimum_safety
            ):
                minimum_safety = (
                    check.clearance.minimum_safety_clearance_m
                )
                minimum_safety_alpha = float(alpha)
            for pair in pairs:
                margin = self._nonpenetration_margin(pair, contact_policy)
                if margin < minimum_nonpenetration_margin:
                    minimum_nonpenetration_margin = margin
                    minimum_nonpenetration_margin_alpha = float(alpha)
                    limiting_nonpenetration_pair = pair
        limiting_pair_samples = tuple(
            self._distance_for_pair(pairs, limiting_nonpenetration_pair)
            for pairs in sampled_pairs
        )
        limiting_pair_start_distance = limiting_pair_samples[0]
        limiting_pair_end_distance = limiting_pair_samples[-1]
        finite_pair_samples = tuple(
            distance
            for distance in limiting_pair_samples
            if distance is not None
        )
        monotonic_non_decreasing = (
            all(
                right >= left - 1e-12
                for left, right in zip(
                    finite_pair_samples,
                    finite_pair_samples[1:],
                )
            )
            if len(finite_pair_samples) == len(limiting_pair_samples)
            else None
        )
        if not math.isfinite(minimum_nonpenetration_margin):
            minimum_nonpenetration_margin = influence_distance_m
        return EdgeCheck(
            valid=valid,
            sample_count=sample_count,
            minimum_nonpenetration_distance_m=minimum_nonpenetration,
            minimum_safety_clearance_m=minimum_safety,
            minimum_nonpenetration_alpha=minimum_nonpenetration_alpha,
            minimum_safety_alpha=minimum_safety_alpha,
            limiting_nonpenetration_pair=limiting_nonpenetration_pair,
            limiting_pair_start_distance_m=limiting_pair_start_distance,
            limiting_pair_end_distance_m=limiting_pair_end_distance,
            limiting_pair_sample_distances_m=limiting_pair_samples,
            limiting_pair_monotonic_non_decreasing=(
                monotonic_non_decreasing
            ),
            minimum_nonpenetration_margin_m=(
                minimum_nonpenetration_margin
            ),
            minimum_nonpenetration_margin_alpha=(
                minimum_nonpenetration_margin_alpha
            ),
            joint_limits_valid=joint_limits_valid,
            nonpenetration_valid=nonpenetration_valid,
            safety_clearance_valid=safety_clearance_valid,
        )

    def solve_ik(
        self,
        target_pose: Pose,
        *,
        frame_name: str | None = None,
        seed: Sequence[float] | None = None,
        position_tolerance_m: float = 0.002,
        orientation_tolerance_rad: float = math.radians(5.0),
        contact_policy: ContactPolicy = FREE_MOTION_CONTACT_POLICY,
        minimum_safety_clearance_m: float = 0.005,
        influence_distance_m: float = 0.05,
    ) -> IkResult:
        """Solve pose IK, then independently validate its endpoint."""
        if position_tolerance_m <= 0.0:
            raise ValueError("position_tolerance_m must be positive")
        if orientation_tolerance_rad <= 0.0:
            raise ValueError("orientation_tolerance_rad must be positive")
        q_seed = (
            self._configuration_array(seed)
            if seed is not None
            else np.asarray(self.configuration())
        )
        self._set_configuration(q_seed)
        frame = self.plant.GetFrameByName(
            frame_name or self.robot_adapter.spec.end_effector_frame_name,
            self.robot_model_instance,
        )
        desired_position = np.asarray(target_pose.translation_m)
        desired_rotation = RotationMatrix(
            Quaternion(np.asarray(target_pose.quaternion_wxyz))
        )
        ik = InverseKinematics(
            self.plant,
            self._plant_context,
            with_joint_limits=True,
        )
        ik.AddPositionConstraint(
            frameB=frame,
            p_BQ=np.zeros((3, 1)),
            frameA=self.plant.world_frame(),
            p_AQ_lower=(
                desired_position - position_tolerance_m
            ).reshape(3, 1),
            p_AQ_upper=(
                desired_position + position_tolerance_m
            ).reshape(3, 1),
        )
        ik.AddOrientationConstraint(
            frameAbar=self.plant.world_frame(),
            R_AbarA=desired_rotation,
            frameBbar=frame,
            R_BbarB=RotationMatrix(),
            theta_bound=orientation_tolerance_rad,
        )
        program = ik.prog()
        q = ik.q()
        fixed_indices = np.setdiff1d(
            np.arange(self.plant.num_positions()),
            self._controlled_indices,
        )
        program.AddBoundingBoxConstraint(
            self._base_positions[fixed_indices],
            self._base_positions[fixed_indices],
            q[fixed_indices],
        )
        gripper = self.robot_adapter.spec.gripper
        if gripper is not None:
            gripper_controlled_indices = np.asarray(
                [
                    index
                    for index, spec in enumerate(
                        self.robot_adapter.spec.controlled_joints
                    )
                    if spec.name in gripper.joint_names
                ],
                dtype=int,
            )
            program.AddBoundingBoxConstraint(
                q_seed[gripper_controlled_indices],
                q_seed[gripper_controlled_indices],
                q[self._controlled_indices[gripper_controlled_indices]],
            )
        program.AddQuadraticErrorCost(
            np.eye(len(self._controlled_indices)),
            q_seed,
            q[self._controlled_indices],
        )
        initial = self._base_positions.copy()
        initial[self._controlled_indices] = q_seed
        program.SetInitialGuess(q, initial)
        result = Solve(program)
        solution = result.GetSolution(q)[self._controlled_indices]
        check = self.check_configuration(
            solution,
            contact_policy=contact_policy,
            minimum_safety_clearance_m=minimum_safety_clearance_m,
            influence_distance_m=influence_distance_m,
        )
        actual = self.frame_pose(
            self.robot_adapter.spec.model_instance_name,
            frame.name(),
        )
        actual_rotation = RotationMatrix(
            Quaternion(np.asarray(actual.quaternion_wxyz))
        )
        orientation_error = (
            desired_rotation.inverse() @ actual_rotation
        ).ToAngleAxis().angle()
        position_error = float(
            np.linalg.norm(
                np.asarray(actual.translation_m) - desired_position
            )
        )
        success = bool(result.is_success() and check.valid)
        reason = "success"
        if not result.is_success():
            reason = "solver_failed"
        elif not check.valid:
            reason = "endpoint_collision_or_clearance"
        return IkResult(
            success=success,
            reason=reason,
            configuration=tuple(float(value) for value in solution),
            solver_result=str(result.get_solution_result()),
            position_error_m=position_error,
            orientation_error_rad=float(orientation_error),
            clearance=check.clearance,
        )

    def differential_ik_to_pose(
        self,
        *,
        target_pose: Pose,
        frame_name: str,
        seed: Sequence[float],
        maximum_joint_delta: float,
        validation_start: Sequence[float] | None = None,
        contact_policy: ContactPolicy = FREE_MOTION_CONTACT_POLICY,
    ) -> DifferentialIkResult:
        """Take one bounded online step toward an absolute world-frame pose.

        Error is recomputed at the commanded configuration on each call.
        The shortest world-expressed rotation is log(R_goal R_current^T).
        The existing delta solver supplies joint limits and full edge checks,
        including the measured starting state and any carried-body policy.
        This local method may reject or stall; callers must check observations
        and action diagnostics rather than interpreting acceptance as arrival.
        """
        current = _rigid_transform(self.frame_pose_at(
            seed, self.robot_adapter.spec.model_instance_name, frame_name
        ))
        target = _rigid_transform(target_pose)
        rotation_error = (
            target.rotation() @ current.rotation().inverse()
        ).ToAngleAxis()
        return self.differential_ik_step(
            translation_m=target.translation() - current.translation(),
            rotation_vector_rad=rotation_error.axis() * rotation_error.angle(),
            frame_name=frame_name,
            seed=seed,
            maximum_joint_delta=maximum_joint_delta,
            validation_start=validation_start,
            contact_policy=contact_policy,
        )

    def differential_ik_step(
        self,
        *,
        translation_m: Sequence[float],
        rotation_vector_rad: Sequence[float],
        frame_name: str,
        seed: Sequence[float],
        maximum_joint_delta: float,
        validation_start: Sequence[float] | None = None,
        contact_policy: ContactPolicy = FREE_MOTION_CONTACT_POLICY,
        damping: float = 1e-4,
        active_joint_names: Sequence[str] | None = None,
    ) -> DifferentialIkResult:
        """Map one small world-frame pose delta to a safe joint-space edge."""
        translation = np.asarray(translation_m, dtype=float)
        rotation = np.asarray(rotation_vector_rad, dtype=float)
        if translation.shape != (3,) or rotation.shape != (3,):
            raise ValueError("Differential IK deltas must be 3-vectors")
        if not np.all(np.isfinite(translation)) or not np.all(
            np.isfinite(rotation)
        ):
            raise ValueError("Differential IK deltas must be finite")
        if maximum_joint_delta <= 0.0 or damping <= 0.0:
            raise ValueError("Differential IK limits must be positive")
        q_seed = self._configuration_array(seed)
        self._set_configuration(q_seed)
        frame = self.plant.GetFrameByName(
            frame_name,
            self.robot_model_instance,
        )
        jacobian = self.plant.CalcJacobianSpatialVelocity(
            self._plant_context,
            JacobianWrtVariable.kV,
            frame,
            np.zeros(3),
            self.plant.world_frame(),
            self.plant.world_frame(),
        )[:, self._controlled_velocity_indices]
        gripper = self.robot_adapter.spec.gripper
        active_indices = np.asarray(
            [
                index
                for index, spec in enumerate(
                    self.robot_adapter.spec.controlled_joints
                )
                if (spec.name in active_joint_names if active_joint_names is not None
                    else gripper is None or spec.name not in gripper.joint_names)
            ],
            dtype=int,
        )
        active_jacobian = jacobian[:, active_indices]
        requested_twist = np.concatenate((rotation, translation))

        # Translation is the primary manipulation objective. Solving all six
        # twist components with one least-squares system can reverse a small
        # requested translation near an arm singularity in order to reduce a
        # zero-orientation residual. First satisfy translation, then use its
        # Jacobian nullspace for the requested angular increment.
        linear_jacobian = active_jacobian[3:, :]
        angular_jacobian = active_jacobian[:3, :]
        linear_pseudoinverse = np.linalg.pinv(
            linear_jacobian,
            rcond=damping,
        )
        primary_delta = linear_pseudoinverse @ translation
        translation_nullspace = (
            np.eye(len(active_indices))
            - linear_pseudoinverse @ linear_jacobian
        )
        angular_residual = rotation - angular_jacobian @ primary_delta
        nullspace_angular_jacobian = (
            angular_jacobian @ translation_nullspace
        )
        secondary_delta = translation_nullspace @ (
            nullspace_angular_jacobian.T
            @ np.linalg.solve(
                nullspace_angular_jacobian
                @ nullspace_angular_jacobian.T
                + damping * np.eye(3),
                angular_residual,
            )
        )
        active_delta = primary_delta + secondary_delta
        joint_delta_scaled = False
        largest_delta = float(np.max(np.abs(active_delta)))
        if largest_delta > maximum_joint_delta:
            # Scale the complete motion uniformly. Scaling the translation
            # term before adding a separately clipped orientation correction
            # can leave a large unintended wrist rotation. During a grasp,
            # that rotation jams the fingers against the supported object
            # instead of producing the requested Cartesian lift.
            active_delta *= maximum_joint_delta / largest_delta
            joint_delta_scaled = True
        delta = np.zeros_like(q_seed)
        delta[active_indices] = active_delta
        candidate = q_seed + delta
        q_edge_start = (
            self._configuration_array(validation_start)
            if validation_start is not None
            else q_seed
        )
        edge = self.check_edge(
            q_edge_start,
            candidate,
            contact_policy=contact_policy,
            maximum_joint_step=min(0.002, maximum_joint_delta),
        )
        validation_start_pose = self.frame_pose_at(
            q_edge_start,
            self.plant.GetModelInstanceName(self.robot_model_instance),
            frame_name,
        )
        candidate_pose = self.frame_pose_at(
            candidate,
            self.plant.GetModelInstanceName(self.robot_model_instance),
            frame_name,
        )
        validation_edge_translation = (
            np.asarray(candidate_pose.translation_m)
            - np.asarray(validation_start_pose.translation_m)
        )
        achieved_twist = jacobian @ delta
        return DifferentialIkResult(
            success=edge.valid,
            reason="success" if edge.valid else "edge_collision_or_clearance",
            configuration=tuple(float(value) for value in candidate),
            requested_twist=tuple(float(value) for value in requested_twist),
            achieved_twist=tuple(float(value) for value in achieved_twist),
            joint_delta_scaled=joint_delta_scaled,
            edge=edge,
            validation_start_configuration=tuple(
                float(value) for value in q_edge_start
            ),
            validation_edge_translation_m=tuple(
                float(value) for value in validation_edge_translation
            ),
        )

    def _configuration_array(
        self,
        configuration: Sequence[float],
    ) -> np.ndarray:
        """Validate a controlled configuration in RobotSpec order."""
        values = np.asarray(configuration, dtype=float)
        if values.shape != (len(self._controlled_indices),):
            raise ValueError(
                "Configuration must match the controlled joint count"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("Configuration must contain finite values")
        return values

    def _set_configuration(self, configuration: Sequence[float]) -> None:
        """Assign only the independent planning context's robot positions."""
        values = self._configuration_array(configuration)
        positions = self._base_positions.copy()
        positions[self._controlled_indices] = values
        self.plant.SetPositions(self._plant_context, positions)

    def _joint_influences(
        self,
        active_joint_names: set[str],
    ) -> dict[Any, frozenset[str]]:
        """Map bodies to active controlled joints affecting world pose."""
        influences = {int(self.plant.world_body().index()): frozenset()}
        pending = [
            self.plant.get_joint(index)
            for index in self.plant.GetJointIndices(
                self.robot_model_instance
            )
        ]
        while pending:
            remaining = []
            for joint in pending:
                parent_index = int(joint.parent_body().index())
                if parent_index not in influences:
                    remaining.append(joint)
                    continue
                child_influences = set(influences[parent_index])
                if joint.name() in active_joint_names:
                    child_influences.add(joint.name())
                influences[int(joint.child_body().index())] = frozenset(
                    child_influences
                )
            if len(remaining) == len(pending):
                names = [joint.name() for joint in remaining]
                raise RuntimeError(
                    f"Could not resolve robot joint influences: {names}"
                )
            pending = remaining
        for body_index in range(self.plant.num_bodies()):
            influences.setdefault(body_index, frozenset())
        return influences

    def _safety_pair_exempt(
        self,
        pair: CollisionPair,
        contact_policy: ContactPolicy,
    ) -> bool:
        """Return whether a pair is excluded only from safety clearance."""
        if contact_policy.permits(pair.body_a, pair.body_b):
            return True
        canonical = tuple(sorted((pair.body_a, pair.body_b)))
        if canonical in self._safety_exempt_pairs:
            return True
        body_a = self._body_from_qualified_name(pair.body_a)
        body_b = self._body_from_qualified_name(pair.body_b)
        if (
            body_a.index() not in self._robot_body_indices
            and body_b.index() not in self._robot_body_indices
        ):
            return False
        return (
            self._safety_joint_influences[int(body_a.index())]
            == self._safety_joint_influences[int(body_b.index())]
        )

    def _clearance_from_pairs(
        self,
        pairs: Sequence[CollisionPair],
        *,
        contact_policy: ContactPolicy,
        influence_distance_m: float,
    ) -> ClearanceMetrics:
        """Build dual-layer metrics from precomputed collision pairs."""
        nearest_nonpenetration = pairs[0] if pairs else None
        safety_pairs = tuple(
            pair
            for pair in pairs
            if not self._safety_pair_exempt(pair, contact_policy)
        )
        nearest_safety = safety_pairs[0] if safety_pairs else None
        return ClearanceMetrics(
            minimum_nonpenetration_distance_m=(
                nearest_nonpenetration.distance_m
                if nearest_nonpenetration is not None
                else influence_distance_m
            ),
            minimum_safety_clearance_m=(
                nearest_safety.distance_m
                if nearest_safety is not None
                else influence_distance_m
            ),
            nearest_nonpenetration_pair=nearest_nonpenetration,
            nearest_safety_pair=nearest_safety,
            influence_distance_m=influence_distance_m,
        )

    def _distance_for_pair(
        self,
        pairs: Sequence[CollisionPair],
        target: CollisionPair | None,
    ) -> float | None:
        """Return one identified pair's distance from evaluated pairs."""
        if target is None:
            return None
        target_identity = self._collision_pair_identity(target)
        for pair in pairs:
            if self._collision_pair_identity(pair) == target_identity:
                return pair.distance_m
        return None

    @staticmethod
    def _nonpenetration_margin(
        pair: CollisionPair,
        contact_policy: ContactPolicy,
    ) -> float:
        """Return signed distance above the pair-specific lower bound."""
        lower_bound = (
            -penetration_limit(contact_policy, pair.body_a, pair.body_b)
            if contact_policy.permits(pair.body_a, pair.body_b)
            else 0.0
        )
        return pair.distance_m - lower_bound

    @staticmethod
    def _collision_pair_identity(
        pair: CollisionPair,
    ) -> tuple[tuple[str, str], tuple[str, str]]:
        """Return a stable unordered identity for one geometry pair."""
        return tuple(sorted((
            (pair.body_a, pair.geometry_a),
            (pair.body_b, pair.geometry_b),
        )))

    def _body_from_qualified_name(self, name: str) -> Any:
        """Resolve a ``model_instance::body`` name in the query plant."""
        model_name, body_name = name.split("::", maxsplit=1)
        model_instance = self.plant.GetModelInstanceByName(model_name)
        return self.plant.GetBodyByName(body_name, model_instance)

    def _set_carried_body_poses(
        self,
        carried_bodies: Sequence[CarriedBody],
    ) -> None:
        """Apply planning-only carried poses at the current configuration."""
        for carried in carried_bodies:
            body = self._body_from_qualified_name(carried.body_name)
            carrier_frame = self.plant.GetFrameByName(
                carried.carrier_frame_name,
                self.robot_model_instance,
            )
            world_from_carrier = carrier_frame.CalcPoseInWorld(
                self._plant_context
            )
            initial_world_from_carrier = _rigid_transform(
                carried.carrier_pose_world
            )
            initial_world_from_body = _rigid_transform(
                carried.body_pose_world
            )
            carrier_from_body = (
                initial_world_from_carrier.inverse()
                @ initial_world_from_body
            )
            set_free_body_world_pose(
                self.plant,
                self._plant_context,
                body,
                world_from_carrier @ carrier_from_body,
            )


def build_planning_query(
    *,
    scenario: ScenarioSpec,
    robot_adapter: RobotAdapter,
    timing: TimingConfig,
) -> PlanningQuery:
    """Build a planning-only RobotDiagram from public specifications."""
    builder = RobotDiagramBuilder(time_step=timing.physics_dt)
    from src.online_manipulation.model import populate_model, support_contact_limits

    parser = builder.parser()
    plant = builder.plant()
    robot_model_instance = populate_model(parser, scenario, robot_adapter)
    plant.Finalize()
    diagram = builder.Build()
    query = PlanningQuery(
        diagram=diagram,
        plant=plant,
        robot_model_instance=robot_model_instance,
        robot_adapter=robot_adapter,
        observed_bodies=scenario.observed_bodies,
        support_limits_m=support_contact_limits(robot_adapter, scenario),
    )
    query.set_observed_body_poses(scenario.initial_object_poses)
    return query
