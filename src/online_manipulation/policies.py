"""Small external policies demonstrating the online environment contract."""

import dataclasses
import math

import numpy as np

from src.online_manipulation.actions import (
    CartesianDeltaAction,
    CompositeAction,
    GripperAction,
    HoldAction,
    JointDeltaAction,
    JointPositionAction,
    RobotAction,
)
from src.online_manipulation.observations import Observation, Pose


def _normalized_quaternion(values: tuple[float, ...]) -> np.ndarray:
    """Return one normalized wxyz quaternion as an array."""
    quaternion = np.asarray(values, dtype=float)
    return quaternion / np.linalg.norm(quaternion)


def _quaternion_product(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Return the Hamilton product of two wxyz quaternions."""
    left_w, left_xyz = left[0], left[1:]
    right_w, right_xyz = right[0], right[1:]
    return np.concatenate((
        [left_w * right_w - left_xyz @ right_xyz],
        left_w * right_xyz
        + right_w * left_xyz
        + np.cross(left_xyz, right_xyz),
    ))


def _rotate_vector(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate a three-vector by one normalized wxyz quaternion."""
    vector_quaternion = np.concatenate(([0.0], vector))
    conjugate = quaternion * np.array([1.0, -1.0, -1.0, -1.0])
    return _quaternion_product(
        _quaternion_product(quaternion, vector_quaternion),
        conjugate,
    )[1:]


def _world_pose(target_pose: Pose, pose_in_target: Pose) -> Pose:
    """Compose a target world pose with an end-effector relative pose."""
    target_quaternion = _normalized_quaternion(
        target_pose.quaternion_wxyz
    )
    relative_quaternion = _normalized_quaternion(
        pose_in_target.quaternion_wxyz
    )
    translation = np.asarray(target_pose.translation_m) + _rotate_vector(
        target_quaternion,
        np.asarray(pose_in_target.translation_m),
    )
    quaternion = _quaternion_product(
        target_quaternion,
        relative_quaternion,
    )
    quaternion /= np.linalg.norm(quaternion)
    return Pose(tuple(translation), tuple(quaternion))


def _world_rotation_error(current: Pose, desired: Pose) -> np.ndarray:
    """Return the shortest world-frame rotation vector to ``desired``."""
    current_quaternion = _normalized_quaternion(current.quaternion_wxyz)
    desired_quaternion = _normalized_quaternion(desired.quaternion_wxyz)
    current_inverse = current_quaternion * np.array(
        [1.0, -1.0, -1.0, -1.0]
    )
    error = _quaternion_product(desired_quaternion, current_inverse)
    if error[0] < 0.0:
        error = -error
    vector_norm = float(np.linalg.norm(error[1:]))
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, max(0.0, error[0]))
    return error[1:] * (angle / vector_norm)


class HoldPolicy:
    """Keep the most recently held robot command unchanged."""

    def reset(self, observation: Observation, info: dict) -> None:
        """Accept a new episode without storing scene-specific state."""
        del observation, info

    def act(self, observation: Observation) -> RobotAction:
        """Return one explicit hold action."""
        del observation
        return HoldAction()


@dataclasses.dataclass(frozen=True)
class JointStepPolicyConfig:
    """One named joint-target increment followed by hold actions."""

    joint_name: str
    delta: float

    def __post_init__(self) -> None:
        """Validate the named finite joint increment."""
        if not self.joint_name:
            raise ValueError("joint_name must be nonempty")
        if not math.isfinite(self.delta) or self.delta == 0.0:
            raise ValueError("delta must be finite and nonzero")


class JointStepPolicy:
    """Send one named joint increment, then hold the resulting target."""

    def __init__(self, config: JointStepPolicyConfig):
        """Store the immutable step configuration."""
        self.config = config
        self._sent = False

    def reset(self, observation: Observation, info: dict) -> None:
        """Arm the one-shot step for every new episode."""
        del info
        if self.config.joint_name not in observation.robot.joint_names:
            raise ValueError(
                f"Joint {self.config.joint_name} is absent from observation"
            )
        self._sent = False

    def act(self, observation: Observation) -> RobotAction:
        """Return the configured step once and explicit holds afterwards."""
        del observation
        if self._sent:
            return HoldAction()
        self._sent = True
        return JointDeltaAction(
            (self.config.joint_name,),
            (self.config.delta,),
        )


@dataclasses.dataclass(frozen=True)
class PickLiftPolicyConfig:
    """Geometry and thresholds for an observation-driven pick-and-lift."""

    target_observation_name: str
    arm_joint_names: tuple[str, ...]
    pregrasp_joint_positions: tuple[float, ...]
    end_effector_frame: str
    approach_axis_world: tuple[float, float, float]
    finger_contact_bodies: tuple[str, str]
    target_contact_body: str
    open_width_m: float
    closed_width_m: float
    approach_distance_m: float
    support_contact_bodies: tuple[str, ...] = ()
    lift_distance_m: float = 0.1
    verify_lift_distance_m: float = 0.01
    cartesian_step_m: float = 0.003
    maximum_lateral_correction_m: float = 0.001
    joint_position_tolerance: float = 0.02
    joint_velocity_tolerance: float = 0.05
    cartesian_tracking_tolerance: float = 0.01
    contact_cartesian_tracking_tolerance: float = 0.04
    cartesian_velocity_tolerance: float = 0.2
    maximum_alignment_error_m: float = 0.01
    minimum_grasp_width_margin_m: float = 0.002
    stable_pregrasp_steps: int = 5
    stable_contact_steps: int = 3
    stable_verify_steps: int = 3
    maximum_close_steps: int = 100
    maximum_verify_steps: int = 100
    maximum_lost_contact_steps: int = 5
    staging_pose_in_target: Pose | None = None
    grasp_pose_in_target: Pose | None = None
    cartesian_rotation_step_rad: float = math.radians(2.0)
    target_pose_position_tolerance_m: float = 0.002
    target_pose_orientation_tolerance_rad: float = math.radians(2.0)
    maximum_target_drift_m: float = 0.01
    maximum_alignment_steps: int = 300
    maximum_approach_steps: int = 300
    stable_target_pose_steps: int = 3

    def __post_init__(self) -> None:
        """Normalize immutable sequences and validate physical thresholds."""
        joint_names = tuple(self.arm_joint_names)
        positions = tuple(float(value) for value in self.pregrasp_joint_positions)
        axis = np.asarray(self.approach_axis_world, dtype=float)
        contacts = tuple(self.finger_contact_bodies)
        supports = tuple(self.support_contact_bodies)
        relative_poses = (
            self.staging_pose_in_target,
            self.grasp_pose_in_target,
        )
        if not self.target_observation_name or not self.end_effector_frame:
            raise ValueError("PickLiftPolicy names must be nonempty")
        if (
            not joint_names
            or len(joint_names) != len(positions)
            or len(set(joint_names)) != len(joint_names)
        ):
            raise ValueError("Arm names and PREGRASP positions must align")
        if axis.shape != (3,) or not np.all(np.isfinite(axis)):
            raise ValueError("approach_axis_world must be a finite 3-vector")
        norm = float(np.linalg.norm(axis))
        if norm <= 0.0:
            raise ValueError("approach_axis_world must be nonzero")
        if (
            len(contacts) != 2
            or len(set(contacts)) != 2
            or any(not name for name in contacts)
        ):
            raise ValueError(
                "Exactly two distinct finger contact bodies are required"
            )
        if not self.target_contact_body:
            raise ValueError("target_contact_body must be nonempty")
        if any(not name for name in supports):
            raise ValueError("support_contact_bodies must be nonempty names")
        if (relative_poses[0] is None) != (relative_poses[1] is None):
            raise ValueError(
                "staging_pose_in_target and grasp_pose_in_target must be "
                "provided together"
            )
        if self.target_contact_body in contacts or (
            self.target_contact_body in supports
        ):
            raise ValueError("The target body cannot also be a contact body")
        positive = (
            self.open_width_m,
            self.approach_distance_m,
            self.lift_distance_m,
            self.verify_lift_distance_m,
            self.cartesian_step_m,
            self.maximum_lateral_correction_m,
            self.joint_position_tolerance,
            self.joint_velocity_tolerance,
            self.cartesian_tracking_tolerance,
            self.contact_cartesian_tracking_tolerance,
            self.cartesian_velocity_tolerance,
            self.maximum_alignment_error_m,
            self.minimum_grasp_width_margin_m,
            self.cartesian_rotation_step_rad,
            self.target_pose_position_tolerance_m,
            self.target_pose_orientation_tolerance_rad,
            self.maximum_target_drift_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("PickLiftPolicy thresholds must be positive")
        if (
            not math.isfinite(self.closed_width_m)
            or self.closed_width_m < 0.0
            or self.closed_width_m >= self.open_width_m
        ):
            raise ValueError("closed_width_m must be below open_width_m")
        if min(
            self.stable_pregrasp_steps,
            self.stable_contact_steps,
            self.stable_verify_steps,
            self.maximum_close_steps,
            self.maximum_verify_steps,
            self.maximum_lost_contact_steps,
            self.maximum_alignment_steps,
            self.maximum_approach_steps,
            self.stable_target_pose_steps,
        ) < 1:
            raise ValueError("PickLiftPolicy step counts must be positive")
        object.__setattr__(self, "arm_joint_names", joint_names)
        object.__setattr__(self, "pregrasp_joint_positions", positions)
        object.__setattr__(
            self,
            "approach_axis_world",
            tuple(float(value) for value in axis / norm),
        )
        object.__setattr__(self, "finger_contact_bodies", contacts)
        object.__setattr__(self, "support_contact_bodies", supports)


class PickLiftPolicy:
    """Online PICK_HOME-to-PREGRASP-to-contact-to-lift state machine.

    The policy produces only public actions and observes physical state. It
    never mutates simulator state, attaches the target, or owns task success.
    """

    _PREGRASP = "pregrasp"
    _ALIGN = "align"
    _APPROACH = "approach"
    _CLOSE = "close"
    _VERIFY = "verify"
    _LIFT = "lift"
    _HOLD = "hold"
    _FAILED = "failed"

    def __init__(self, config: PickLiftPolicyConfig):
        """Store configuration; reset initializes episode state."""
        self.config = config
        self._stage = self._PREGRASP
        self._stable_steps = 0
        self._close_steps = 0
        self._verify_steps = 0
        self._alignment_steps = 0
        self._approach_steps = 0
        self._target_pose_steps = 0
        self._lost_contact_steps = 0
        self._lift_start_target_z = 0.0
        self._initial_target_translation = np.zeros(3)
        self._failure_reason: str | None = None
        self._last_cartesian_tracking_error = 0.0
        self._last_cartesian_joint_speed = 0.0
        self._last_cartesian_ready = True
        self._last_cartesian_contact_mode = False

    @property
    def stage(self) -> str:
        """Return the current externally owned policy stage."""
        return self._stage

    @property
    def stop_reason(self) -> str | None:
        """Expose policy failure through the explicit runner lifecycle contract."""
        return self._failure_reason

    def diagnostics(self) -> dict[str, object]:
        """Return state-machine diagnostics for episode artifacts."""
        return {
            "stage": self._stage,
            "failure_reason": self._failure_reason,
            "stable_steps": self._stable_steps,
            "close_steps": self._close_steps,
            "verify_steps": self._verify_steps,
            "alignment_steps": self._alignment_steps,
            "approach_steps": self._approach_steps,
            "target_pose_steps": self._target_pose_steps,
            "lost_contact_steps": self._lost_contact_steps,
            "cartesian_tracking_error": (
                self._last_cartesian_tracking_error
            ),
            "cartesian_joint_speed": self._last_cartesian_joint_speed,
            "cartesian_ready": self._last_cartesian_ready,
            "cartesian_contact_mode": self._last_cartesian_contact_mode,
        }

    def reset(self, observation: Observation, info: dict) -> None:
        """Reset stage counters and validate required observation names."""
        del info
        missing = set(self.config.arm_joint_names) - set(
            observation.robot.joint_names
        )
        if missing:
            raise ValueError(f"Arm joints are absent from observation: {missing}")
        if self.config.target_observation_name not in observation.objects:
            raise ValueError(
                "Pick target is absent from observation: "
                f"{self.config.target_observation_name}"
            )
        self._stage = self._PREGRASP
        self._stable_steps = 0
        self._close_steps = 0
        self._verify_steps = 0
        self._alignment_steps = 0
        self._approach_steps = 0
        self._target_pose_steps = 0
        self._lost_contact_steps = 0
        self._lift_start_target_z = self._target_z(observation)
        self._initial_target_translation = np.asarray(
            observation.objects[
                self.config.target_observation_name
            ].pose.translation_m,
            dtype=float,
        )
        self._failure_reason = None
        self._last_cartesian_tracking_error = 0.0
        self._last_cartesian_joint_speed = 0.0
        self._last_cartesian_ready = True
        self._last_cartesian_contact_mode = False

    def act(self, observation: Observation) -> RobotAction:
        """Advance the staged policy from the latest physical observation."""
        if self._stage == self._PREGRASP:
            return self._act_pregrasp(observation)
        if self._stage == self._ALIGN:
            return self._act_align(observation)
        if self._stage == self._APPROACH:
            return self._act_approach(observation)
        if self._stage == self._CLOSE:
            return self._act_close(observation)
        if self._stage == self._VERIFY:
            return self._act_verify(observation)
        if self._stage == self._LIFT:
            return self._act_lift(observation)
        if self._stage == self._HOLD:
            return self._act_hold(observation)
        return HoldAction()

    def _act_pregrasp(self, observation: Observation) -> RobotAction:
        """Track the joint-space PREGRASP and require a stable arrival."""
        position_by_name = dict(
            zip(
                observation.robot.joint_names,
                observation.robot.q,
                strict=True,
            )
        )
        velocity_by_name = dict(
            zip(
                observation.robot.joint_names,
                observation.robot.v,
                strict=True,
            )
        )
        maximum_error = max(
            abs(position_by_name[name] - target)
            for name, target in zip(
                self.config.arm_joint_names,
                self.config.pregrasp_joint_positions,
                strict=True,
            )
        )
        maximum_speed = max(
            abs(velocity_by_name[name])
            for name in self.config.arm_joint_names
        )
        if (
            maximum_error <= self.config.joint_position_tolerance
            and maximum_speed <= self.config.joint_velocity_tolerance
        ):
            self._stable_steps += 1
        else:
            self._stable_steps = 0
        if self._stable_steps >= self.config.stable_pregrasp_steps:
            self._stage = (
                self._ALIGN
                if self.config.staging_pose_in_target is not None
                else self._APPROACH
            )
            self._target_pose_steps = 0
            return HoldAction()
        return CompositeAction(
            arm=JointPositionAction(
                self.config.arm_joint_names,
                self.config.pregrasp_joint_positions,
            ),
            gripper=GripperAction(self.config.open_width_m),
        )

    def _act_align(self, observation: Observation) -> RobotAction:
        """Align to a target-relative staging pose through online steps."""
        self._alignment_steps += 1
        if self._alignment_steps > self.config.maximum_alignment_steps:
            self._fail("alignment_timeout")
            return HoldAction()
        if self.config.staging_pose_in_target is None:
            raise RuntimeError("Target-relative staging pose is missing")
        return self._act_target_relative_pose(
            observation,
            self.config.staging_pose_in_target,
            next_stage=self._APPROACH,
        )

    def _act_approach(self, observation: Observation) -> RobotAction:
        """Move toward the target in bounded Cartesian increments."""
        self._approach_steps += 1
        if self._approach_steps > self.config.maximum_approach_steps:
            self._fail("approach_timeout")
            return HoldAction()
        if self.config.grasp_pose_in_target is not None:
            return self._act_target_relative_pose(
                observation,
                self.config.grasp_pose_in_target,
                next_stage=self._CLOSE,
            )
        if not self._ready_for_cartesian_step(observation):
            return HoldAction()
        axis = np.asarray(self.config.approach_axis_world)
        current = np.asarray(
            observation.robot.end_effector_pose.translation_m,
            dtype=float,
        )
        target = np.asarray(
            observation.objects[
                self.config.target_observation_name
            ].pose.translation_m,
            dtype=float,
        )
        offset = target - current
        remaining = float(axis @ offset)
        perpendicular_error = float(
            np.linalg.norm(offset - axis * remaining)
        )
        maximum_remaining = (
            self.config.approach_distance_m
            + 2.0 * self.config.cartesian_step_m
        )
        if (
            perpendicular_error > self.config.maximum_alignment_error_m
            or remaining < -self.config.cartesian_step_m
            or remaining > maximum_remaining
        ):
            self._fail("approach_pose_guard")
            return HoldAction()
        if remaining <= 0.5 * self.config.cartesian_step_m:
            self._stage = self._CLOSE
            self._close_steps = 0
            self._stable_steps = 0
            return GripperAction(self.config.closed_width_m)
        distance = min(self.config.cartesian_step_m, remaining)
        perpendicular_offset = offset - axis * remaining
        correction_norm = float(np.linalg.norm(perpendicular_offset))
        if correction_norm > 0.0:
            perpendicular_offset *= min(
                1.0,
                self.config.maximum_lateral_correction_m / correction_norm,
            )
        translation = axis * distance + perpendicular_offset
        translation_norm = float(np.linalg.norm(translation))
        if translation_norm > self.config.cartesian_step_m:
            translation *= self.config.cartesian_step_m / translation_norm
        return CompositeAction(
            arm=CartesianDeltaAction(
                end_effector_frame=self.config.end_effector_frame,
                reference_frame="world",
                translation_m=tuple(float(value) for value in translation),
                rotation_vector_rad=(0.0, 0.0, 0.0),
            ),
            gripper=GripperAction(self.config.open_width_m),
        )

    def _act_target_relative_pose(
        self,
        observation: Observation,
        pose_in_target: Pose,
        *,
        next_stage: str,
    ) -> RobotAction:
        """Track one live target-relative pose with bounded Cartesian steps."""
        target_pose = observation.objects[
            self.config.target_observation_name
        ].pose
        target_displacement = float(np.linalg.norm(
            np.asarray(target_pose.translation_m)
            - self._initial_target_translation
        ))
        if target_displacement > self.config.maximum_target_drift_m:
            self._fail("target_moved_before_grasp")
            return HoldAction()

        desired_pose = _world_pose(target_pose, pose_in_target)
        current_pose = observation.robot.end_effector_pose
        translation = (
            np.asarray(desired_pose.translation_m)
            - np.asarray(current_pose.translation_m)
        )
        rotation = _world_rotation_error(current_pose, desired_pose)
        position_error = float(np.linalg.norm(translation))
        orientation_error = float(np.linalg.norm(rotation))
        if (
            position_error <= self.config.target_pose_position_tolerance_m
            and orientation_error
            <= self.config.target_pose_orientation_tolerance_rad
            and self._ready_for_cartesian_step(observation)
        ):
            self._target_pose_steps += 1
        else:
            self._target_pose_steps = 0
        if self._target_pose_steps >= self.config.stable_target_pose_steps:
            self._stage = next_stage
            self._target_pose_steps = 0
            if next_stage == self._CLOSE:
                self._close_steps = 0
                self._stable_steps = 0
                return GripperAction(self.config.closed_width_m)
            return HoldAction()
        if not self._ready_for_cartesian_step(observation):
            return HoldAction()

        if position_error > self.config.cartesian_step_m:
            translation *= self.config.cartesian_step_m / position_error
        if orientation_error > self.config.cartesian_rotation_step_rad:
            rotation *= (
                self.config.cartesian_rotation_step_rad / orientation_error
            )
        return CompositeAction(
            arm=CartesianDeltaAction(
                end_effector_frame=self.config.end_effector_frame,
                reference_frame="world",
                translation_m=tuple(float(value) for value in translation),
                rotation_vector_rad=tuple(float(value) for value in rotation),
            ),
            gripper=GripperAction(self.config.open_width_m),
        )

    def _act_close(self, observation: Observation) -> RobotAction:
        """Close until stable bilateral physical target contact is observed."""
        self._close_steps += 1
        if self._has_bilateral_target_contact(observation):
            self._stable_steps += 1
        else:
            self._stable_steps = 0
        if self._stable_steps >= self.config.stable_contact_steps:
            self._stage = self._VERIFY
            self._lift_start_target_z = self._target_z(observation)
            self._verify_steps = 0
            self._lost_contact_steps = 0
            self._stable_steps = 0
            # Contact loads can leave the measured arm offset from its last
            # free-space target. Rebase the held servo target on the measured
            # contact posture before issuing Cartesian lift increments. This
            # is an ordinary online command; it does not alter simulator
            # state or the target body pose.
            position_by_name = dict(zip(
                observation.robot.joint_names,
                observation.robot.q,
                strict=True,
            ))
            return CompositeAction(
                arm=JointPositionAction(
                    self.config.arm_joint_names,
                    tuple(
                        position_by_name[name]
                        for name in self.config.arm_joint_names
                    ),
                ),
                gripper=GripperAction(self.config.closed_width_m),
            )
        if self._close_steps >= self.config.maximum_close_steps:
            self._fail("bilateral_contact_timeout")
            return HoldAction()
        return GripperAction(self.config.closed_width_m)

    def _act_verify(self, observation: Observation) -> RobotAction:
        """Break target support while proving a nonempty bilateral grasp."""
        self._verify_steps += 1
        bilateral = self._has_bilateral_target_contact(observation)
        if bilateral:
            self._lost_contact_steps = 0
        else:
            self._lost_contact_steps += 1
        width = observation.robot.gripper_width_m
        if width is None:
            raise ValueError("PickLiftPolicy requires observed gripper width")
        if bilateral and (
            width
            <= self.config.closed_width_m
            + self.config.minimum_grasp_width_margin_m
        ):
            self._fail("empty_gripper_closure")
            return HoldAction()
        if self._unexpected_target_contacts(observation):
            self._fail("unexpected_target_contact")
            return HoldAction()
        if self._lost_contact_steps >= self.config.maximum_lost_contact_steps:
            self._fail("lost_bilateral_contact_during_verify")
            return HoldAction()

        lifted = self._target_z(observation) - self._lift_start_target_z
        support_contact = self._has_target_contact_with(
            observation,
            self.config.support_contact_bodies,
        )
        if (
            bilateral
            and not support_contact
            and lifted >= self.config.verify_lift_distance_m
        ):
            self._stable_steps += 1
        else:
            self._stable_steps = 0
        if self._stable_steps >= self.config.stable_verify_steps:
            self._stage = self._LIFT
            self._lost_contact_steps = 0
            return HoldAction()
        if self._verify_steps >= self.config.maximum_verify_steps:
            self._fail("support_breakaway_timeout")
            return HoldAction()
        if not bilateral or not self._ready_for_cartesian_step(
            observation,
            contact_mode=True,
        ):
            return GripperAction(self.config.closed_width_m)
        remaining = max(
            0.0,
            self.config.verify_lift_distance_m - lifted,
        )
        if support_contact:
            # A tilted object can raise its center by the nominal verify
            # distance while one edge remains physically supported. Continue
            # bounded online lift steps until the observed support contact is
            # actually gone; the VERIFY timeout remains the hard limit.
            remaining = max(remaining, self.config.cartesian_step_m)
        distance = min(self.config.cartesian_step_m, remaining)
        if distance <= 0.0:
            return GripperAction(self.config.closed_width_m)
        return CompositeAction(
            arm=CartesianDeltaAction(
                end_effector_frame=self.config.end_effector_frame,
                reference_frame="world",
                translation_m=(0.0, 0.0, distance),
                rotation_vector_rad=(0.0, 0.0, 0.0),
            ),
            gripper=GripperAction(self.config.closed_width_m),
        )

    def _act_lift(self, observation: Observation) -> RobotAction:
        """Lift in bounded world-Z increments while retaining gripper force."""
        if self._has_bilateral_target_contact(observation):
            self._lost_contact_steps = 0
        else:
            self._lost_contact_steps += 1
        if (
            self._lost_contact_steps >= self.config.maximum_lost_contact_steps
        ):
            self._fail("lost_bilateral_contact_during_lift")
            return HoldAction()
        if self._has_target_contact_with(
                observation,
                self.config.support_contact_bodies,
        ):
            self._fail("support_recontact_during_lift")
            return HoldAction()
        if self._unexpected_target_contacts(observation):
            self._fail("unexpected_target_contact")
            return HoldAction()
        if not self._ready_for_cartesian_step(
            observation,
            contact_mode=True,
        ):
            return GripperAction(self.config.closed_width_m)
        lifted = self._target_z(observation) - self._lift_start_target_z
        remaining = self.config.lift_distance_m - lifted
        if remaining <= 0.5 * self.config.cartesian_step_m:
            self._stage = self._HOLD
            return HoldAction()
        distance = min(self.config.cartesian_step_m, remaining)
        return CompositeAction(
            arm=CartesianDeltaAction(
                end_effector_frame=self.config.end_effector_frame,
                reference_frame="world",
                translation_m=(0.0, 0.0, distance),
                rotation_vector_rad=(0.0, 0.0, 0.0),
            ),
            gripper=GripperAction(self.config.closed_width_m),
        )

    def _act_hold(self, observation: Observation) -> RobotAction:
        """Retain the grasp while task-owned stable-hold timing completes."""
        if (
            not self._has_bilateral_target_contact(observation)
        ):
            self._fail("lost_bilateral_contact_during_hold")
            return HoldAction()
        if self._has_target_contact_with(
                observation,
                self.config.support_contact_bodies,
        ):
            self._fail("support_recontact_during_hold")
            return HoldAction()
        if self._unexpected_target_contacts(observation):
            self._fail("unexpected_target_contact")
            return HoldAction()
        return GripperAction(self.config.closed_width_m)

    def _fail(self, reason: str) -> None:
        """Enter the terminal policy hold state with an explicit reason."""
        self._stage = self._FAILED
        self._failure_reason = reason

    def _ready_for_cartesian_step(
        self,
        observation: Observation,
        *,
        contact_mode: bool = False,
    ) -> bool:
        """Require the previous servo target to settle before another step.

        A bilateral grasp creates a steady contact load, so the arm can have a
        small, bounded position offset even after velocity settles. Contact
        phases use a separately configured tracking tolerance while retaining
        the same velocity gate.
        """
        joint_indices = tuple(
            observation.robot.joint_names.index(name)
            for name in self.config.arm_joint_names
        )
        tracking_error = max(
            abs(
                observation.robot.q[index]
                - observation.robot.q_commanded[index]
            )
            for index in joint_indices
        )
        speed = max(
            abs(observation.robot.v[index]) for index in joint_indices
        )
        tolerance = (
            self.config.contact_cartesian_tracking_tolerance
            if contact_mode
            else self.config.cartesian_tracking_tolerance
        )
        ready = (
            tracking_error <= tolerance
            and speed <= self.config.cartesian_velocity_tolerance
        )
        self._last_cartesian_tracking_error = tracking_error
        self._last_cartesian_joint_speed = speed
        self._last_cartesian_ready = ready
        self._last_cartesian_contact_mode = contact_mode
        return ready

    def _has_bilateral_target_contact(
        self,
        observation: Observation,
    ) -> bool:
        """Return whether both configured fingers contact the target body."""
        contacting = set()
        target = self.config.target_contact_body
        for contact in observation.contacts:
            pair = {contact.body_a, contact.body_b}
            if target not in pair:
                continue
            contacting.update(pair & set(self.config.finger_contact_bodies))
        return contacting == set(self.config.finger_contact_bodies)

    def _has_target_contact_with(
        self,
        observation: Observation,
        bodies: tuple[str, ...],
    ) -> bool:
        """Return whether the target contacts any body in ``bodies``."""
        target = self.config.target_contact_body
        candidates = set(bodies)
        return any(
            target in {contact.body_a, contact.body_b}
            and bool({contact.body_a, contact.body_b} & candidates)
            for contact in observation.contacts
        )

    def _unexpected_target_contacts(
        self,
        observation: Observation,
    ) -> tuple[str, ...]:
        """Return target contacts outside gripper and support allowlists."""
        target = self.config.target_contact_body
        expected = set(self.config.finger_contact_bodies) | set(
            self.config.support_contact_bodies
        )
        unexpected = set()
        for contact in observation.contacts:
            pair = {contact.body_a, contact.body_b}
            if target not in pair:
                continue
            unexpected.update(pair - {target} - expected)
        return tuple(sorted(unexpected))

    def _target_z(self, observation: Observation) -> float:
        """Return current target-body world height."""
        return observation.objects[
            self.config.target_observation_name
        ].pose.translation_m[2]
