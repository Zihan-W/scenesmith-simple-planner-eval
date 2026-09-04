"""Minimal Behavior Tree leaf that advances one online policy period."""

import dataclasses
import enum
import math
from collections.abc import Mapping
from typing import Any

from src.online_manipulation.actions import (
    HoldAction,
    JointPositionAction,
)
from src.online_manipulation.observations import Observation
from src.online_manipulation.protocols import OnlineEnvironment


class BehaviorTreeStatus(enum.Enum):
    """Small status vocabulary shared by common Behavior Tree libraries."""

    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"


@dataclasses.dataclass(frozen=True)
class TickResult:
    """Environment transition produced by one Behavior Tree leaf tick."""

    status: BehaviorTreeStatus
    observation: Observation
    reward: float
    terminated: bool
    truncated: bool
    info: Mapping[str, Any]


class JointTargetLeaf:
    """Drive one named joint to an absolute target using repeated ticks.

    This is intentionally one leaf rather than a Behavior Tree framework. A
    caller can wrap its ``tick`` method in py_trees, BehaviorTree.CPP, or a
    project-specific tree while preserving the one-tick/one-policy-step rule.
    """

    def __init__(
        self,
        joint_name: str,
        target_position: float,
        *,
        position_tolerance: float = 0.01,
        velocity_tolerance: float = 0.05,
    ):
        """Store one joint-space leaf goal and completion tolerances."""
        if not joint_name:
            raise ValueError("joint_name must be nonempty")
        values = (
            target_position,
            position_tolerance,
            velocity_tolerance,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("JointTargetLeaf values must be finite")
        if position_tolerance <= 0.0 or velocity_tolerance <= 0.0:
            raise ValueError("JointTargetLeaf tolerances must be positive")
        self.joint_name = joint_name
        self.target_position = float(target_position)
        self.position_tolerance = float(position_tolerance)
        self.velocity_tolerance = float(velocity_tolerance)

    def tick(
        self,
        env: OnlineEnvironment,
        observation: Observation,
    ) -> TickResult:
        """Send one action, advance one policy period, and return BT status."""
        joint_index = self._joint_index(observation)
        reached_before = self._reached(observation, joint_index)
        action = (
            HoldAction()
            if reached_before
            else JointPositionAction(
                joint_names=(self.joint_name,),
                positions=(self.target_position,),
            )
        )
        next_observation, reward, terminated, truncated, info = env.step(action)
        next_joint_index = self._joint_index(next_observation)
        reached_after = self._reached(next_observation, next_joint_index)
        if reached_after:
            status = BehaviorTreeStatus.SUCCESS
        elif terminated or truncated:
            status = BehaviorTreeStatus.FAILURE
        else:
            status = BehaviorTreeStatus.RUNNING
        return TickResult(
            status=status,
            observation=next_observation,
            reward=reward,
            terminated=terminated,
            truncated=truncated,
            info=info,
        )

    def _joint_index(self, observation: Observation) -> int:
        """Resolve the configured joint in the current observation."""
        try:
            return observation.robot.joint_names.index(self.joint_name)
        except ValueError as error:
            raise ValueError(
                f"Joint is absent from observation: {self.joint_name}"
            ) from error

    def _reached(self, observation: Observation, joint_index: int) -> bool:
        """Return whether the measured position and speed meet the goal."""
        return (
            abs(
                observation.robot.q[joint_index]
                - self.target_position
            )
            <= self.position_tolerance
            and abs(observation.robot.v[joint_index])
            <= self.velocity_tolerance
        )
