"""Replaceable task implementations for online manipulation."""

import dataclasses
import math
from collections.abc import Mapping
from typing import Any

import numpy as np

from src.online_manipulation.actions import RobotAction
from src.online_manipulation.contact import (
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
    required_lift_m: float = 0.08
    required_hold_s: float = 3.0

    def __post_init__(self) -> None:
        """Validate target identifiers and success thresholds."""
        contacts = tuple(self.gripper_contact_bodies)
        if (
            not self.target_observation_name
            or not self.target_contact_body
            or not contacts
            or any(not name for name in contacts)
        ):
            raise ValueError("PickLiftTask body names must be nonempty")
        if (
            not math.isfinite(self.required_lift_m)
            or not math.isfinite(self.required_hold_s)
            or self.required_lift_m <= 0.0
            or self.required_hold_s <= 0.0
        ):
            raise ValueError("PickLiftTask thresholds must be positive")
        object.__setattr__(self, "gripper_contact_bodies", contacts)


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
        return {
            "task_name": "pick_lift",
            "target": self.config.target_observation_name,
            "lift_m": lift_m,
            "required_lift_m": self.config.required_lift_m,
            "held_above_threshold_s": self._held_above_threshold_s,
            "required_hold_s": self.config.required_hold_s,
            "success": self._success,
        }

    def evaluate(self, env: Any) -> TaskEvaluation:
        """Accumulate stable lifted time and report terminal success."""
        now = env.observation.time_s
        elapsed = now - self._last_time_s
        if elapsed < 0.0:
            raise ValueError("Simulation time moved backwards")
        lift_m = self._lift_m(env)
        if lift_m >= self.config.required_lift_m:
            self._held_above_threshold_s += elapsed
        else:
            self._held_above_threshold_s = 0.0
        self._last_time_s = now
        self._success = (
            self._held_above_threshold_s >= self.config.required_hold_s
        )
        return TaskEvaluation(
            reward=1.0 if self._success else 0.0,
            terminated=self._success,
            success=self._success,
            reason="lift_held" if self._success else "running",
            metrics={
                "lift_m": lift_m,
                "held_above_threshold_s": self._held_above_threshold_s,
            },
        )

    def allowed_contacts(
        self,
        env: Any,
        action: RobotAction,
    ) -> ContactPolicy:
        """Allow configured gripper-target pairs in planning safety checks."""
        del env, action
        return PairContactPolicy.from_pairs(
            "pick_lift_target_contact",
            (
                (body, self.config.target_contact_body)
                for body in self.config.gripper_contact_bodies
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
        }

    def _target(self, env: Any):
        """Return the configured generic target observation."""
        try:
            return env.observation.objects[
                self.config.target_observation_name
            ]
        except KeyError as error:
            raise KeyError(
                "PickLiftTask target is absent from Observation.objects: "
                f"{self.config.target_observation_name}"
            ) from error

    def _lift_m(self, env: Any) -> float:
        """Return target vertical displacement from reset."""
        if self._initial_height_m is None:
            raise RuntimeError("Call task.reset() before evaluation")
        return float(
            self._target(env).pose.translation_m[2] - self._initial_height_m
        )
