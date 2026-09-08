"""Generic typed facade for an online manipulation runtime backend."""

import dataclasses
from pathlib import Path
from typing import Protocol

import numpy as np

from src.online_manipulation.actions import RobotAction
from src.online_manipulation.observations import Observation
from src.online_manipulation.protocols import ContactPolicy, Task
from src.online_manipulation.tasks import NullTask


class RuntimeBackend(Protocol):
    """Internal bridge implemented by a concrete Drake runtime."""

    def reset(
        self,
        rng: np.random.Generator,
    ) -> tuple[Observation, dict]:
        """Reset backend state and return a normalized observation."""

    def step(
        self,
        action: RobotAction,
        contact_policy: ContactPolicy,
    ) -> tuple[Observation, bool, dict]:
        """Apply one typed action for exactly one policy period."""

    def write_updated_scenario(self, output_path: Path) -> tuple[str, ...]:
        """Write explicitly selected free-body poses to a new DMD file."""

    def start_recording(self) -> None:
        """Start an optional visualization recording."""

    def save_recording(self, output_path: Path) -> None:
        """Stop and save an active visualization recording."""


class OnlineManipulationEnv:
    """Expose deterministic Gym-style reset and step over a runtime backend.

    This Phase 3 facade owns no robot, scene-object, task, or controller names.
    The concrete backend performs the temporary translation to the validated
    runtime. Task reward, termination, observation, and contact policy remain
    owned by a replaceable Task implementation.
    """

    def __init__(
        self,
        backend: RuntimeBackend,
        task: Task | None = None,
    ):
        """Store a concrete backend and independently replaceable task."""
        self._backend = backend
        self._task = task if task is not None else NullTask()
        self._observation: Observation | None = None
        self._rng = np.random.default_rng()

    @property
    def backend(self) -> RuntimeBackend:
        """Return the runtime bridge for diagnostics during migration."""
        return self._backend

    @property
    def observation(self) -> Observation:
        """Return the latest complete public observation."""
        if self._observation is None:
            raise RuntimeError("Call reset() before reading observation")
        return self._observation

    def reset(self, seed: int | None = None) -> tuple[Observation, dict]:
        """Reset all runtime state and record the caller-provided seed."""
        self._rng = np.random.default_rng(seed)
        observation, info = self._backend.reset(self._rng)
        self._observation = observation
        task_reset = self._task.reset(self, self._rng)
        self._observation = dataclasses.replace(
            observation,
            task=dict(self._task.observe(self)),
        )
        result_info = dict(info)
        result_info["seed"] = seed
        result_info["task_reset"] = dict(task_reset)
        return self._observation, result_info

    def step(
        self,
        action: RobotAction,
    ) -> tuple[Observation, float, bool, bool, dict]:
        """Advance one policy period using a structured action.

        The runtime's episode-duration boundary is reported as truncation.
        Reward, task termination, and task observation come only from Task.
        """
        contact_policy = self._task.allowed_contacts(self, action)
        observation, time_limit_reached, info = self._backend.step(
            action,
            contact_policy,
        )
        self._observation = observation
        evaluation = self._task.evaluate(self)
        self._observation = dataclasses.replace(
            observation,
            task=dict(self._task.observe(self)),
        )
        result_info = dict(info)
        result_info["contact_policy"] = contact_policy.name
        result_info["task"] = {
            "success": evaluation.success,
            "reason": evaluation.reason,
            "metrics": dict(evaluation.metrics),
        }
        return (
            self._observation,
            evaluation.reward,
            evaluation.terminated,
            time_limit_reached or evaluation.truncated,
            result_info,
        )

    def write_updated_scenario(self, output_path: Path) -> tuple[str, ...]:
        """Write selected final object poses without changing the input DMD."""
        return self._backend.write_updated_scenario(Path(output_path))

    def get_planning_query(self):
        """Return a planning snapshot synchronized to current base and objects.

        Reacquire after step/reset. Querying or changing candidate configurations
        affects only its planning Context, never the simulated robot.
        """
        return self._backend.get_planning_query()

    def finalize_episode(self) -> dict:
        """Return task-owned final metadata for the current episode."""
        if self._observation is None:
            raise RuntimeError("Call reset() before finalizing an episode")
        return dict(self._task.finalize(self))

    def start_recording(self) -> None:
        """Start backend visualization recording when configured."""
        self._backend.start_recording()

    def save_recording(self, output_path: Path) -> None:
        """Save an active backend visualization recording."""
        self._backend.save_recording(Path(output_path))
