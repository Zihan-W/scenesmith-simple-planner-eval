"""VLM-TAMP semantic-only request with strict v3 validation and bounded retry."""

from __future__ import annotations

import dataclasses
import json
import math
import time

from .model_budget import model_stage_budget
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from planner.src.tamp.hierarchy import PredicateGoal, parse_semantic_goals
from planner.src.bt.generation import GenerationError, ProviderError
from planner.src.tamp.model import goal_response_format


@dataclasses.dataclass(frozen=True)
class ModelSettings:
    model: str = "gpt-4.1-mini"
    temperature: float = 0.2
    max_attempts: int = 3
    response_format: str = "text"
    request_timeout_s: float = 30.0
    stage_timeout_s: float = 90.0

    def __post_init__(self):
        for value in (self.request_timeout_s, self.stage_timeout_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Model time budgets must be finite and positive")
        if not self.model or not 0 <= self.temperature <= 2 or not 1 <= self.max_attempts <= 5:
            raise ValueError("Invalid TAMP model settings")
        if self.response_format not in ("text", "json_object", "json_schema"):
            raise ValueError("Unknown model response format")


class SemanticModelError(RuntimeError):
    def __init__(self, message, *, provider_failure=None):
        super().__init__(message)
        self.provider_failure = provider_failure


class SemanticSubgoalPlanner:
    def __init__(self, client, settings: ModelSettings, *, prompt_path: Path | None = None,
                 trace=None):
        self.client = client
        self.settings = settings
        path = prompt_path or (Path(__file__).resolve().parents[2] / "resources" / "prompts") / "tamp_subgoal_v3.txt"
        self.prompt = path.read_text(encoding="utf-8")
        self.calls = 0
        self.trace = trace

    def propose(self, **kwargs):
        """Generate within a single bounded stage, including all retries."""
        with model_stage_budget(self.client, self.settings) as deadline:
            return self._propose(deadline=deadline, **kwargs)

    def _propose(self, *, deadline, task: str, world: Mapping[str, Any],
                predicate_arity: Mapping[str, int],
                feedback: tuple[Mapping[str, Any], ...] = (),
                images: tuple[Mapping[str, str], ...] = ()) -> tuple[PredicateGoal, ...]:
        known_objects = frozenset(world.get("objects", {}))
        if not task.strip() or not known_objects:
            raise ValueError("Semantic planning requires a task and observed objects")
        semantic_world = {
            "objects": {name: {key: value for key, value in metadata.items()
                               if key in {"category", "movable", "articulated", "surface"}}
                        for name, metadata in world["objects"].items()},
            "facts": world.get("facts", []),
        }
        satisfied = {PredicateGoal(item["predicate"], tuple(item["arguments"]))
                     for item in semantic_world["facts"]}
        payload = {"task": task, "world": semantic_world,
                   "known_objects": sorted(known_objects),
                   "predicates": dict(predicate_arity),
                   "abstract_physical_feedback": list(feedback),
                   "observations": [{key: item[key] for key in (
                       "camera", "timestamp_s", "annotations") if key in item}
                       for item in images]}
        content = [{"type": "text", "text": json.dumps(payload, ensure_ascii=False, default=str)}]
        for item in images:
            content.append({"type": "image_url", "image_url": {
                "path": item["path"], "sha256": item["sha256"],
                "mime_type": "image/png"}})
        messages = [{"role": "system", "content": self.prompt},
                    {"role": "user", "content": content}]
        response_format = goal_response_format(
            self.settings.response_format, schema_name="scenesmith.tamp.goals.v3",
            predicate_arity=predicate_arity, known_objects=known_objects)
        format_options = {} if response_format is None else {"response_format": response_format}
        errors = []
        provider_failure = None
        for attempt in range(self.settings.max_attempts):
            if time.perf_counter() >= deadline:
                raise SemanticModelError("model_stage_wall_time_exhausted", provider_failure=provider_failure)
            self.calls += 1
            response = None
            if self.trace is not None:
                self.trace({"event": "semantic_model_request", "call": self.calls,
                            "model": self.settings.model, "temperature": self.settings.temperature,
                            "response_format": response_format,
                            "messages": messages})
            try:
                response = self.client.complete(
                    model=self.settings.model, messages=messages,
                    temperature=self.settings.temperature,
                    **format_options,
                )
                if time.perf_counter() >= deadline:
                    raise GenerationError("model_stage_wall_time_exhausted")
                goals = parse_semantic_goals(
                    response.content, predicate_arity=predicate_arity,
                    known_objects=known_objects,
                )
                raw_goals = goals
                goals = tuple(dict.fromkeys(goal for goal in goals if goal not in satisfied))
                if self.trace is not None and goals != raw_goals:
                    self.trace({"event": "semantic_goals_filtered", "call": self.calls,
                                "raw_goals": [dataclasses.asdict(goal) for goal in raw_goals],
                                "remaining_goals": [dataclasses.asdict(goal) for goal in goals]})
                if not goals:
                    raise ValueError("All proposed goals are already observed; return unmet task goals")
            except (GenerationError, ValueError, TypeError) as error:
                errors.append(str(error))
                provider_failure = error.as_dict() if isinstance(error, ProviderError) else None
                if self.trace is not None:
                    self.trace({"event": "semantic_model_error", "call": self.calls,
                                "reason": str(error), "provider_failure": provider_failure})
                if provider_failure is not None and not provider_failure["retryable"]:
                    raise SemanticModelError(str(error), provider_failure=provider_failure) from error
                if response is not None and attempt + 1 < self.settings.max_attempts:
                    messages.extend((
                        {"role": "assistant", "content": response.content},
                        {"role": "user", "content": (
                            "Invalid semantic goal JSON: " + str(error)
                            + ". Return only a valid goals.v3 JSON object."
                        )},
                    ))
            else:
                if self.trace is not None:
                    self.trace({"event": "semantic_model_response", "call": self.calls,
                                "model": getattr(response, "model", self.settings.model),
                                "response_id": getattr(response, "response_id", None),
                                "usage": getattr(response, "usage", None),
                                "goals": [dataclasses.asdict(goal) for goal in goals]})
                return goals
        raise SemanticModelError(
            f"No valid semantic response after {self.settings.max_attempts} attempts: {errors[-1]}",
            provider_failure=provider_failure
        )
