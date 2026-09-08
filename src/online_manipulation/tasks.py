"""Replaceable task implementations for online manipulation."""

import dataclasses
import math
from collections.abc import Mapping
from typing import Any

import numpy as np

from src.online_manipulation.actions import (
    BaseVelocityAction,
    CartesianDeltaAction,
    CartesianPoseAction,
    CompositeAction,
    JointDeltaAction,
    JointPositionAction,
    RobotCommand,
    RobotAction,
)
from src.online_manipulation.contact import (
    CarriedBody,
    FREE_MOTION_CONTACT_POLICY,
    PairContactPolicy,
)
from src.online_manipulation.protocols import ContactPolicy, TaskEvaluation


class NullTask:
    """Neutral task for controller and environment regression tests."""

    def reset(self, env: Any, rng: np.random.Generator) -> Mapping[str, Any]:
        """Return deterministic reset metadata without changing the scene."""
        del env, rng
        return {"task_name": "null"}

    def observe(self, env: Any) -> Mapping[str, Any]:
        """Return the neutral task observation."""
        del env
        return {"task_name": "null", "status": "running"}

    def evaluate(self, env: Any) -> TaskEvaluation:
        """Return zero reward and no task-owned termination."""
        del env
        return TaskEvaluation()

    def allowed_contacts(
        self,
        env: Any,
        action: RobotAction,
    ) -> ContactPolicy:
        """Disallow task-specific contacts during free motion."""
        del env, action
        return FREE_MOTION_CONTACT_POLICY

    def finalize(self, env: Any) -> Mapping[str, Any]:
        """Return neutral final task metadata."""
        del env
        return {"task_name": "null", "success": False}


@dataclasses.dataclass(frozen=True)
class PickLiftTaskConfig:
    """Object and contact configuration for a generic pick-and-lift task."""

    target_observation_name: str
    gripper_contact_bodies: tuple[str, ...]
    target_contact_body: str
    support_contact_bodies: tuple[str, ...] = ()
    required_lift_m: float = 0.08
    required_hold_s: float = 3.0
    maximum_target_translational_speed_m_s: float = 0.02
    maximum_target_rotational_speed_rad_s: float = 0.5
    maximum_allowed_contact_penetration_m: float = 0.0001
    carrier_arm_name: str | None = None
    carrier_gripper_name: str | None = None

    def __post_init__(self) -> None:
        """Validate target identifiers and success thresholds."""
        contacts = tuple(self.gripper_contact_bodies)
        supports = tuple(self.support_contact_bodies)
        if (
            not self.target_observation_name
            or not self.target_contact_body
            or len(contacts) != 2
            or len(set(contacts)) != 2
            or any(not name for name in contacts)
            or any(not name for name in supports)
        ):
            raise ValueError(
                "PickLiftTask requires two distinct gripper bodies and "
                "nonempty target/support body names"
            )
        if self.target_contact_body in contacts or (
            self.target_contact_body in supports
        ):
            raise ValueError("The target body cannot also be a contact body")
        if (
            not math.isfinite(self.required_lift_m)
            or not math.isfinite(self.required_hold_s)
            or self.required_lift_m <= 0.0
            or self.required_hold_s <= 0.0
        ):
            raise ValueError("PickLiftTask thresholds must be positive")
        speeds = (
            self.maximum_target_translational_speed_m_s,
            self.maximum_target_rotational_speed_rad_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in speeds):
            raise ValueError("PickLiftTask velocity limits must be positive")
        if (
            not math.isfinite(self.maximum_allowed_contact_penetration_m)
            or self.maximum_allowed_contact_penetration_m <= 0.0
        ):
            raise ValueError("PickLiftTask contact penetration limit must be positive")
        object.__setattr__(self, "gripper_contact_bodies", contacts)
        object.__setattr__(self, "support_contact_bodies", supports)


class PickLiftTask:
    """Evaluate physical target lift without owning a motion state machine."""

    def __init__(self, config: PickLiftTaskConfig):
        """Store task configuration; reset initializes episode state."""
        self.config = config
        self._initial_height_m: float | None = None
        self._held_above_threshold_s = 0.0
        self._last_time_s = 0.0
        self._success = False

    def reset(self, env: Any, rng: np.random.Generator) -> Mapping[str, Any]:
        """Capture the target's initial physical height."""
        del rng
        target = self._target(env)
        self._initial_height_m = target.pose.translation_m[2]
        self._held_above_threshold_s = 0.0
        self._last_time_s = env.observation.time_s
        self._success = False
        return {
            "task_name": "pick_lift",
            "target": self.config.target_observation_name,
            "initial_height_m": self._initial_height_m,
        }

    def observe(self, env: Any) -> Mapping[str, Any]:
        """Return current lift progress and success thresholds."""
        lift_m = self._lift_m(env)
        contact_state = self._contact_state(env)
        target = self._target(env)
        translational_speed = float(
            np.linalg.norm(target.spatial_velocity.translational_m_s)
        )
        rotational_speed = float(
            np.linalg.norm(target.spatial_velocity.rotational_rad_s)
        )
        return {
            "task_name": "pick_lift",
            "target": self.config.target_observation_name,
            "lift_m": lift_m,
            "required_lift_m": self.config.required_lift_m,
            "held_above_threshold_s": self._held_above_threshold_s,
            "required_hold_s": self.config.required_hold_s,
            **contact_state,
            "target_translational_speed_m_s": translational_speed,
            "target_rotational_speed_rad_s": rotational_speed,
            "success": self._success,
        }

    def evaluate(self, env: Any) -> TaskEvaluation:
        """Accumulate stable lifted time and report terminal success."""
        now = env.observation.time_s
        elapsed = now - self._last_time_s
        if elapsed < 0.0:
            raise ValueError("Simulation time moved backwards")
        lift_m = self._lift_m(env)
        contact_state = self._contact_state(env)
        target = self._target(env)
        translational_speed = float(
            np.linalg.norm(target.spatial_velocity.translational_m_s)
        )
        rotational_speed = float(
            np.linalg.norm(target.spatial_velocity.rotational_rad_s)
        )
        stable_lift = bool(
            lift_m >= self.config.required_lift_m
            and contact_state["bilateral_gripper_contact"]
            and not contact_state["support_contact"]
            and not contact_state["unexpected_target_contacts"]
            and translational_speed
            <= self.config.maximum_target_translational_speed_m_s
            and rotational_speed <= self.config.maximum_target_rotational_speed_rad_s
        )
        if stable_lift:
            self._held_above_threshold_s += elapsed
        else:
            self._held_above_threshold_s = 0.0
        self._last_time_s = now
        self._success = self._held_above_threshold_s >= self.config.required_hold_s
        return TaskEvaluation(
            reward=1.0 if self._success else 0.0,
            terminated=self._success,
            success=self._success,
            reason="lift_held" if self._success else "running",
            metrics={
                "lift_m": lift_m,
                "held_above_threshold_s": self._held_above_threshold_s,
                "stable_lift": stable_lift,
                **contact_state,
                "target_translational_speed_m_s": translational_speed,
                "target_rotational_speed_rad_s": rotational_speed,
            },
        )

    def allowed_contacts(
        self,
        env: Any,
        action: RobotAction,
    ) -> ContactPolicy:
        """Allow configured gripper-target pairs in planning safety checks."""
        carried_bodies = ()
        if self._contact_state(env)["bilateral_gripper_contact"]:
            base = action.base if isinstance(action, RobotCommand) else action
            if isinstance(base, BaseVelocityAction) and (
                base.velocity_m_s != 0.0 or base.yaw_rate_rad_s != 0.0
            ):
                raise ValueError(
                    "PickLift carrying with a moving base is not supported; "
                    "stop the base before manipulating the target"
                )
            query = env.get_planning_query()
            spec = query.robot_adapter.spec
            grippers = spec.grippers or {"": spec.gripper}
            matches = [
                name
                for name, gripper in grippers.items()
                if gripper is not None
                and set(self.config.gripper_contact_bodies)
                == {
                    f"{spec.model_instance_name}::{body}"
                    for body in gripper.contact_body_names
                }
            ]
            if len(matches) != 1:
                raise ValueError(
                    "PickLift carrier gripper is missing or ambiguous in RobotSpec"
                )
            gripper_name = matches[0]
            if (
                self.config.carrier_gripper_name is not None
                and self.config.carrier_gripper_name != gripper_name
            ):
                raise ValueError(
                    "PickLift carrier gripper does not match contact bodies"
                )
            groups = spec.arm_groups or {
                "": tuple(
                    n
                    for n in spec.controlled_joint_names
                    if n not in grippers[gripper_name].joint_names
                )
            }
            frames = spec.end_effector_frames or {"": spec.end_effector_frame_name}
            arm_name = self.config.carrier_arm_name
            if arm_name is None:
                if len(groups) != 1:
                    raise ValueError(
                        "PickLift with multiple arms requires carrier_arm_name"
                    )
                arm_name = next(iter(groups))
            if arm_name not in groups:
                raise ValueError(f"Unknown PickLift carrier arm: {arm_name}")
            frame = frames[arm_name]
            command = (
                action.arms.get(arm_name)
                if isinstance(action, RobotCommand)
                else action.arm if isinstance(action, CompositeAction) else action
            )
            if isinstance(command, (CartesianDeltaAction, CartesianPoseAction)):
                if command.end_effector_frame != frame:
                    raise ValueError(
                        "PickLift action frame differs from its bound carrier"
                    )
            elif isinstance(command, (JointDeltaAction, JointPositionAction)):
                if not set(command.joint_names).issubset(groups[arm_name]):
                    raise ValueError(
                        "PickLift joint action must address its bound carrier arm"
                    )
            carried_bodies = (
                CarriedBody(
                    body_name=self.config.target_contact_body,
                    carrier_frame_name=frame,
                    body_pose_world=self._target(env).pose,
                    carrier_pose_world=query.frame_pose(
                        spec.model_instance_name, frame
                    ),
                ),
            )
        return PairContactPolicy.from_pairs(
            "pick_lift_target_contact",
            tuple(
                (body, self.config.target_contact_body)
                for body in self.config.gripper_contact_bodies
            )
            + tuple(
                (body, self.config.target_contact_body)
                for body in self.config.support_contact_bodies
            ),
            carried_bodies=carried_bodies,
            maximum_allowed_penetration_m=(
                self.config.maximum_allowed_contact_penetration_m
            ),
        )

    def finalize(self, env: Any) -> Mapping[str, Any]:
        """Return final physical task metrics."""
        return {
            "task_name": "pick_lift",
            "target": self.config.target_observation_name,
            "success": self._success,
            "lift_m": self._lift_m(env),
            "held_above_threshold_s": self._held_above_threshold_s,
            **self._contact_state(env),
        }

    def _contact_state(self, env: Any) -> dict[str, Any]:
        """Classify physical contacts involving the configured target."""
        finger_contacts = {body: False for body in self.config.gripper_contact_bodies}
        support_contact = False
        unexpected_contacts = set()
        known_contacts = set(self.config.gripper_contact_bodies) | set(
            self.config.support_contact_bodies
        )
        for contact in env.observation.contacts:
            if self.config.target_contact_body == contact.body_a:
                other_body = contact.body_b
            elif self.config.target_contact_body == contact.body_b:
                other_body = contact.body_a
            else:
                continue
            if other_body in finger_contacts:
                finger_contacts[other_body] = True
            elif other_body in self.config.support_contact_bodies:
                support_contact = True
            elif other_body not in known_contacts:
                unexpected_contacts.add(other_body)
        finger_flags = tuple(
            finger_contacts[body] for body in self.config.gripper_contact_bodies
        )
        return {
            "finger_contacts": finger_flags,
            "bilateral_gripper_contact": all(finger_flags),
            "support_contact": support_contact,
            "unexpected_target_contacts": tuple(sorted(unexpected_contacts)),
        }

    def _target(self, env: Any):
        """Return the configured generic target observation."""
        try:
            return env.observation.objects[self.config.target_observation_name]
        except KeyError as error:
            raise KeyError(
                "PickLiftTask target is absent from Observation.objects: "
                f"{self.config.target_observation_name}"
            ) from error

    def _lift_m(self, env: Any) -> float:
        """Return target vertical displacement from reset."""
        if self._initial_height_m is None:
            raise RuntimeError("Call task.reset() before evaluation")
        return float(self._target(env).pose.translation_m[2] - self._initial_height_m)
