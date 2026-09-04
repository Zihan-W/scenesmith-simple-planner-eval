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
from src.online_manipulation.observations import Observation


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
    lift_distance_m: float = 0.1
    cartesian_step_m: float = 0.003
    joint_position_tolerance: float = 0.02
    joint_velocity_tolerance: float = 0.05
    stable_pregrasp_steps: int = 5
    stable_contact_steps: int = 3
    maximum_close_steps: int = 100

    def __post_init__(self) -> None:
        """Normalize immutable sequences and validate physical thresholds."""
        joint_names = tuple(self.arm_joint_names)
        positions = tuple(float(value) for value in self.pregrasp_joint_positions)
        axis = np.asarray(self.approach_axis_world, dtype=float)
        contacts = tuple(self.finger_contact_bodies)
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
        if len(contacts) != 2 or any(not name for name in contacts):
            raise ValueError("Exactly two finger contact bodies are required")
        positive = (
            self.open_width_m,
            self.approach_distance_m,
            self.lift_distance_m,
            self.cartesian_step_m,
            self.joint_position_tolerance,
            self.joint_velocity_tolerance,
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
            self.maximum_close_steps,
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


class PickLiftPolicy:
    """Online PICK_HOME-to-PREGRASP-to-contact-to-lift state machine.

    The policy produces only public actions and observes physical state. It
    never mutates simulator state, attaches the target, or owns task success.
    """

    _PREGRASP = "pregrasp"
    _APPROACH = "approach"
    _CLOSE = "close"
    _LIFT = "lift"
    _HOLD = "hold"
    _FAILED = "failed"

    def __init__(self, config: PickLiftPolicyConfig):
        """Store configuration; reset initializes episode state."""
        self.config = config
        self._stage = self._PREGRASP
        self._stable_steps = 0
        self._close_steps = 0
        self._approach_start = np.zeros(3)
        self._lift_start_target_z = 0.0

    @property
    def stage(self) -> str:
        """Return the current externally owned policy stage."""
        return self._stage

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
        self._approach_start = np.asarray(
            observation.robot.end_effector_pose.translation_m,
            dtype=float,
        )
        self._lift_start_target_z = self._target_z(observation)

    def act(self, observation: Observation) -> RobotAction:
        """Advance the staged policy from the latest physical observation."""
        if self._stage == self._PREGRASP:
            return self._act_pregrasp(observation)
        if self._stage == self._APPROACH:
            return self._act_approach(observation)
        if self._stage == self._CLOSE:
            return self._act_close(observation)
        if self._stage == self._LIFT:
            return self._act_lift(observation)
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
            self._stage = self._APPROACH
            self._approach_start = np.asarray(
                observation.robot.end_effector_pose.translation_m,
                dtype=float,
            )
            return HoldAction()
        return CompositeAction(
            arm=JointPositionAction(
                self.config.arm_joint_names,
                self.config.pregrasp_joint_positions,
            ),
            gripper=GripperAction(self.config.open_width_m),
        )

    def _act_approach(self, observation: Observation) -> RobotAction:
        """Move toward the target in bounded Cartesian increments."""
        axis = np.asarray(self.config.approach_axis_world)
        current = np.asarray(
            observation.robot.end_effector_pose.translation_m,
            dtype=float,
        )
        progress = float(axis @ (current - self._approach_start))
        remaining = self.config.approach_distance_m - progress
        if remaining <= 0.5 * self.config.cartesian_step_m:
            self._stage = self._CLOSE
            self._close_steps = 0
            self._stable_steps = 0
            return GripperAction(self.config.closed_width_m)
        distance = min(self.config.cartesian_step_m, remaining)
        return CompositeAction(
            arm=CartesianDeltaAction(
                end_effector_frame=self.config.end_effector_frame,
                reference_frame="world",
                translation_m=tuple(float(value) for value in axis * distance),
                rotation_vector_rad=(0.0, 0.0, 0.0),
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
            self._stage = self._LIFT
            self._lift_start_target_z = self._target_z(observation)
            return HoldAction()
        if self._close_steps >= self.config.maximum_close_steps:
            self._stage = self._FAILED
            return HoldAction()
        return GripperAction(self.config.closed_width_m)

    def _act_lift(self, observation: Observation) -> RobotAction:
        """Lift in bounded world-Z increments while retaining gripper force."""
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

    def _target_z(self, observation: Observation) -> float:
        """Return current target-body world height."""
        return observation.objects[
            self.config.target_observation_name
        ].pose.translation_m[2]
