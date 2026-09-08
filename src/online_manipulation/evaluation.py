"""Replaceable online evaluation, separate from task bindings/contact rules."""

import dataclasses
from collections.abc import Mapping
from typing import Protocol, runtime_checkable

from src.online_manipulation.observations import Observation
from src.online_manipulation.protocols import TaskEvaluation


@runtime_checkable
class Evaluator(Protocol):
    """Per-episode evaluator; factories must return fresh, unshared instances.

    reset receives the initial observation and task reset metadata. evaluate
    receives current observations plus the task's baseline result, once per
    policy step. All temporal classifier state belongs to this instance, not
    Runtime. Return TaskEvaluation (reward/terminated/truncated/success/reason/
    metrics). No Drake context or physical mutation is available here.
    Offline VLM evaluation is a separate consumer of episode artifacts.
    """

    def reset(self, observation: Observation, info: Mapping) -> None:
        """Clear classifier state at every independent reset."""

    def evaluate(self, observation: Observation, baseline: TaskEvaluation) -> TaskEvaluation:
        """Return a public, machine-readable result for the current state."""


class TaskResultEvaluator:
    """Preserve the existing task result exactly, including PickLift timing."""

    def reset(self, observation, info):
        """This pass-through evaluator has no episode state."""

    def evaluate(self, observation, baseline):
        """Return the same object without reinterpreting success conditions."""
        return baseline


class EvaluatedTask:
    """Delegate bindings/contacts to Task and final decisions to Evaluator."""

    def __init__(self, task, evaluator):
        if not isinstance(evaluator, Evaluator):
            raise TypeError("Evaluator must implement reset and evaluate")
        self.task, self.evaluator = task, evaluator
        self.result = TaskEvaluation()

    def reset(self, env, rng):
        info = self.task.reset(env, rng)
        self.result = TaskEvaluation()
        self.evaluator.reset(dataclasses.replace(env.observation, task=self.task.observe(env)), info)
        return info

    def observe(self, env):
        return {**self.task.observe(env), "evaluation": {
            "success": self.result.success, "reason": self.result.reason,
            "metrics": dict(self.result.metrics)}}

    def evaluate(self, env):
        baseline = self.task.evaluate(env)
        self.result = self.evaluator.evaluate(
            dataclasses.replace(env.observation, task=self.task.observe(env)), baseline)
        if not isinstance(self.result, TaskEvaluation):
            raise TypeError("Evaluator.evaluate must return TaskEvaluation")
        return self.result

    def allowed_contacts(self, env, action):
        return self.task.allowed_contacts(env, action)

    def finalize(self, env):
        return {**self.task.finalize(env), "success": self.result.success,
                "evaluation_reason": self.result.reason,
                "evaluation_metrics": dict(self.result.metrics)}
