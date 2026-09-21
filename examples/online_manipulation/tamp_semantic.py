"""VLM-TAMP semantic-only request with strict v3 validation and bounded retry."""

from __future__ import annotations

import dataclasses
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from examples.online_manipulation.tamp_hierarchy import PredicateGoal, parse_semantic_goals
from examples.online_manipulation.bt_generation import GenerationError
from examples.online_manipulation.tamp_model import goal_response_format


@dataclasses.dataclass(frozen=True)
class ModelSettings:
    model: str = "gpt-4.1-mini"
    temperature: float = 0.2
    max_attempts: int = 3
    response_format: str = "text"

    def __post_init__(self):
        if not self.model or not 0 <= self.temperature <= 2 or not 1 <= self.max_attempts <= 5:
            raise ValueError("Invalid TAMP model settings")
        if self.response_format not in ("text", "json_object", "json_schema"):
            raise ValueError("Unknown model response format")


class SemanticModelError(RuntimeError):
    pass


class SemanticSubgoalPlanner:
    def __init__(self, client, settings: ModelSettings, *, prompt_path: Path | None = None,
                 trace=None):
        self.client = client
        self.settings = settings
        path = prompt_path or Path(__file__).with_name("prompts") / "tamp_subgoal_v3.txt"
        self.prompt = path.read_text(encoding="utf-8")
        self.calls = 0
        self.trace = trace

    def propose(self, *, task: str, world: Mapping[str, Any],
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
        for attempt in range(self.settings.max_attempts):
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
                goals = parse_semantic_goals(
                    response.content, predicate_arity=predicate_arity,
                    known_objects=known_objects,
                )
            except (GenerationError, ValueError, TypeError) as error:
                errors.append(str(error))
                if self.trace is not None:
                    self.trace({"event": "semantic_model_error", "call": self.calls,
                                "reason": str(error)})
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
            f"No valid semantic response after {self.settings.max_attempts} attempts: {errors[-1]}"
        )
