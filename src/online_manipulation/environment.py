"""Generic typed facade for an online manipulation runtime backend."""

from pathlib import Path
from typing import Protocol

from src.online_manipulation.actions import RobotAction
from src.online_manipulation.observations import Observation


class RuntimeBackend(Protocol):
    """Internal bridge implemented by a concrete Drake runtime."""

    def reset(self) -> tuple[Observation, dict]:
        """Reset backend state and return a normalized observation."""

    def step(
        self,
        action: RobotAction,
    ) -> tuple[Observation, bool, dict]:
        """Apply one typed action for exactly one policy period."""


class OnlineManipulationEnv:
    """Expose deterministic Gym-style reset and step over a runtime backend.

    This Phase 3 facade owns no robot, scene-object, task, or controller names.
    The concrete backend performs the temporary translation to the validated
    runtime. Task reward and termination are added in Phase 4.
    """

    def __init__(self, backend: RuntimeBackend):
        """Store a concrete backend behind the public typed interface."""
        self._backend = backend

    @property
    def backend(self) -> RuntimeBackend:
        """Return the runtime bridge for diagnostics during migration."""
        return self._backend

    def reset(self, seed: int | None = None) -> tuple[Observation, dict]:
        """Reset all runtime state and record the caller-provided seed."""
        observation, info = self._backend.reset()
        result_info = dict(info)
        result_info["seed"] = seed
        return observation, result_info

    def step(
        self,
        action: RobotAction,
    ) -> tuple[Observation, float, bool, bool, dict]:
        """Advance one policy period using a structured action.

        The legacy runtime's episode-duration boundary is reported as
        truncation. Task-owned reward and termination remain neutral until a
        Task is connected in Phase 4.
        """
        observation, time_limit_reached, info = self._backend.step(action)
        return observation, 0.0, False, time_limit_reached, info

    def write_updated_scenario(self, output_path: Path) -> None:
        """Write final object poses after the Phase 5 finalizer is installed."""
        raise NotImplementedError(
            "DMD scenario write-back is implemented in Phase 5"
        )
