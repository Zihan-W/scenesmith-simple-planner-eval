"""Read-only planning queries backed by an independent Drake context."""

import dataclasses
import math
from collections.abc import Sequence
from typing import Any

import numpy as np
from pydrake.all import (
    InverseKinematics,
    LoadModelDirectives,
    ProcessModelDirectives,
    Quaternion,
    RotationMatrix,
    Solve,
)
from pydrake.planning import RobotDiagramBuilder

from src.online_manipulation.contact import (
    FREE_MOTION_CONTACT_POLICY,
)
from src.online_manipulation.drake_utils import register_package_xml
from src.online_manipulation.observations import Pose
from src.online_manipulation.protocols import ContactPolicy, RobotAdapter
from src.online_manipulation.specs import ScenarioSpec, TimingConfig


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


@dataclasses.dataclass(frozen=True)
class EdgeCheck:
    """High-density numerical validation of a joint-space edge."""

    valid: bool
    sample_count: int
    minimum_nonpenetration_distance_m: float
    minimum_safety_clearance_m: float
    minimum_nonpenetration_alpha: float
    minimum_safety_alpha: float


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


class PlanningQuery:
    """Query one robot without advancing the environment's real context."""

    def __init__(
        self,
        *,
        diagram: Any,
        plant: Any,
        robot_model_instance: Any,
        robot_adapter: RobotAdapter,
    ):
        """Create and initialize a context owned only by planning queries."""
        self.diagram = diagram
        self.plant = plant
        self.robot_model_instance = robot_model_instance
        self.robot_adapter = robot_adapter
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
        self._base_positions = plant.GetPositions(self._plant_context).copy()
        self._robot_body_indices = set(
            plant.GetBodyIndices(robot_model_instance)
        )
        gripper_names = (
            set(robot_adapter.spec.gripper.joint_names)
            if robot_adapter.spec.gripper is not None
            else set()
        )
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
    ) -> tuple[CollisionPair, ...]:
        """Return robot-related signed distances within an influence range."""
        if influence_distance_m <= 0.0:
            raise ValueError("influence_distance_m must be positive")
        self._set_configuration(configuration)
        query = self.plant.get_geometry_query_input_port().Eval(
            self._plant_context
        )
        inspector = query.inspector()
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
        pairs = self.collision_pairs(
            configuration,
            influence_distance_m=influence_distance_m,
        )
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

    def check_configuration(
        self,
        configuration: Sequence[float],
        *,
        contact_policy: ContactPolicy = FREE_MOTION_CONTACT_POLICY,
        minimum_safety_clearance_m: float = 0.005,
        influence_distance_m: float = 0.05,
    ) -> ConfigurationCheck:
        """Check joint limits, nonpenetration, and safety clearance."""
        values = self._configuration_array(configuration)
        within_limits = all(
            spec.position_lower <= value <= spec.position_upper
            for spec, value in zip(
                self.robot_adapter.spec.controlled_joints,
                values,
                strict=True,
            )
        )
        clearance = self.clearance(
            values,
            contact_policy=contact_policy,
            influence_distance_m=influence_distance_m,
        )
        return ConfigurationCheck(
            valid=bool(
                within_limits
                and clearance.minimum_nonpenetration_distance_m >= 0.0
                and clearance.minimum_safety_clearance_m
                >= minimum_safety_clearance_m
            ),
            configuration=tuple(float(value) for value in values),
            clearance=clearance,
        )

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
        valid = True
        for alpha in np.linspace(0.0, 1.0, sample_count):
            check = self.check_configuration(
                q_start + alpha * (q_end - q_start),
                contact_policy=contact_policy,
                minimum_safety_clearance_m=minimum_safety_clearance_m,
                influence_distance_m=influence_distance_m,
            )
            valid = valid and check.valid
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
        return EdgeCheck(
            valid=valid,
            sample_count=sample_count,
            minimum_nonpenetration_distance_m=minimum_nonpenetration,
            minimum_safety_clearance_m=minimum_safety,
            minimum_nonpenetration_alpha=minimum_nonpenetration_alpha,
            minimum_safety_alpha=minimum_safety_alpha,
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
        return (
            self._safety_joint_influences[int(body_a.index())]
            == self._safety_joint_influences[int(body_b.index())]
        )

    def _body_from_qualified_name(self, name: str) -> Any:
        """Resolve a ``model_instance::body`` name in the query plant."""
        model_name, body_name = name.split("::", maxsplit=1)
        model_instance = self.plant.GetModelInstanceByName(model_name)
        return self.plant.GetBodyByName(body_name, model_instance)


def build_planning_query(
    *,
    scenario: ScenarioSpec,
    robot_adapter: RobotAdapter,
    timing: TimingConfig,
) -> PlanningQuery:
    """Build a planning-only RobotDiagram from public specifications."""
    builder = RobotDiagramBuilder(time_step=timing.physics_dt)
    parser = builder.parser()
    parser.SetAutoRenaming(True)
    for package_xml in scenario.package_xmls:
        register_package_xml(parser, package_xml)
    ProcessModelDirectives(
        LoadModelDirectives(str(scenario.dmd_path)),
        parser,
    )
    plant = builder.plant()
    robot_model_instance = robot_adapter.add_model(parser)
    robot_adapter.configure_model(plant, robot_model_instance)
    plant.Finalize()
    diagram = builder.Build()
    return PlanningQuery(
        diagram=diagram,
        plant=plant,
        robot_model_instance=robot_model_instance,
        robot_adapter=robot_adapter,
    )
