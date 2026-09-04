"""Runtime-checkable public protocols for environment collaborators."""

import dataclasses
from collections.abc import Mapping
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

import numpy as np

from src.online_manipulation.actions import RobotAction
from src.online_manipulation.observations import Observation, RobotObservation
from src.online_manipulation.specs import RobotSpec


@runtime_checkable
class ContactPolicy(Protocol):
    """Read-only decision interface for task-specific allowed contacts."""

    @property
    def name(self) -> str:
        """Return a stable policy name."""

    def permits(self, body_a: str, body_b: str) -> bool:
        """Return whether the qualified body pair may contact."""


@runtime_checkable
class RobotAdapter(Protocol):
    """Robot-specific bridge between public contracts and Drake objects."""

    @property
    def spec(self) -> RobotSpec:
        """Return the immutable robot description."""

    def add_model(self, parser: Any) -> Any:
        """Register packages and add exactly one robot model."""

    def configure_model(self, plant: Any, model_instance: Any) -> None:
        """Weld the base and perform pre-Finalize model configuration."""

    def initialize_state(
        self,
        plant: Any,
        plant_context: Any,
        model_instance: Any,
    ) -> None:
        """Apply home and locked joint positions during reset."""

    def make_robot_observation(
        self,
        plant: Any,
        plant_context: Any,
        model_instance: Any,
        controller_state: Mapping[str, Any],
    ) -> RobotObservation:
        """Map Drake state and controller telemetry to the public schema."""

    def gripper_position_targets(self, width_m: float) -> Mapping[str, float]:
        """Map physical gripper width to named joint targets."""


@runtime_checkable
class Task(Protocol):
    """Replaceable task lifecycle used by OnlineManipulationEnv."""

    def reset(self, env: Any, rng: np.random.Generator) -> Mapping[str, Any]:
        """Reset task state and return initial task metadata."""

    def observe(self, env: Any) -> Mapping[str, Any]:
        """Return task-specific observation fields."""

    def evaluate(self, env: Any) -> "TaskEvaluation":
        """Return reward and termination state for the current context."""

    def allowed_contacts(
        self,
        env: Any,
        action: RobotAction,
    ) -> ContactPolicy:
        """Return the planning safety contact policy for this action."""

    def finalize(self, env: Any) -> Mapping[str, Any]:
        """Return task metadata recorded at episode completion."""


@runtime_checkable
class Policy(Protocol):
    """External online policy contract."""

    def reset(self, observation: Observation, info: Mapping[str, Any]) -> None:
        """Reset policy state for a new episode."""

    def act(self, observation: Observation) -> RobotAction:
        """Return the next action from the latest observation."""


@dataclasses.dataclass(frozen=True)
class TaskEvaluation:
    """Task-owned reward, termination, and diagnostic result."""

    reward: float = 0.0
    terminated: bool = False
    truncated: bool = False
    success: bool = False
    reason: str = "running"
    metrics: Mapping[str, Any] = dataclasses.field(default_factory=dict)


@runtime_checkable
class OnlineEnvironment(Protocol):
    """Gym-like public environment contract."""

    def reset(self, seed: int | None = None) -> tuple[Observation, dict]:
        """Reset simulation, controller, and task state."""

    def step(
        self,
        action: RobotAction,
    ) -> tuple[Observation, float, bool, bool, dict]:
        """Advance exactly one policy period."""

    def write_updated_scenario(self, output_path: Path) -> None:
        """Write current free-object poses without modifying the input DMD."""
