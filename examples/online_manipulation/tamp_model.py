"""TAMP provider response formats and bounded legacy schema-validated requests."""

from __future__ import annotations

import copy
from pathlib import Path

from examples.online_manipulation.bt_generation import GenerationError


class PlannerModelError(RuntimeError):
    """No provider response passed validation within the configured budget."""


def load_prompt(name: str) -> str:
    """Read a repository-owned, versioned TAMP prompt."""
    return (Path(__file__).with_name("prompts") / name).read_text(encoding="utf-8")


def json_response_format(mode):
    """Select an explicit provider format; never silently downgrade a schema."""
    if mode == "text":
        return None
    if mode == "json_object":
        return {"type": "json_object"}
    raise ValueError("This legacy free-key program requires text or json_object format")


def goal_response_format(mode, *, schema_name, predicate_arity, known_objects,
                         legacy_target=False, max_goals=12):
    """Build a strict response schema from the current symbolic vocabulary."""
    if mode != "json_schema":
        return json_response_format(mode)
    if not predicate_arity or not known_objects:
        raise ValueError("Structured semantic output requires predicates and objects")
    variants = []
    for predicate, arity in sorted(predicate_arity.items()):
        argument = {"type": "string", "enum": sorted(known_objects)}
        properties = {"predicate": {"type": "string", "enum": [predicate]}}
        if legacy_target:
            properties["target"] = argument
        else:
            properties["arguments"] = {"type": "array", "items": argument,
                                       "minItems": arity, "maxItems": arity}
        variants.append({"type": "object", "properties": properties,
                         "required": list(properties), "additionalProperties": False})
    schema = {
        "type": "object", "additionalProperties": False,
        "required": ["schema", "subgoals"],
        "properties": {
            "schema": {"type": "string", "enum": [schema_name]},
            "subgoals": {"type": "array", "minItems": 1, "maxItems": max_goals,
                         "items": {"anyOf": variants}},
        },
    }
    return {"type": "json_schema", "json_schema": {
        "name": schema_name.replace(".", "_"), "strict": True, "schema": schema}}


def validated_completion(*, client, model, messages, validate, max_attempts=3,
                         temperature=None, attempts=None, response_format=None):
    """Retry transport/schema failures, never physical execution or grounding.

    Return the validated value and original text. Omitted temperature preserves
    the existing client protocol; live CLI callers supply their model config.
    """
    if not model or not isinstance(max_attempts, int) or not 1 <= max_attempts <= 5:
        raise ValueError("A model and max_attempts in [1, 5] are required")
    if temperature is not None and not 0 <= temperature <= 2:
        raise ValueError("temperature must be in [0, 2]")
    messages = copy.deepcopy(messages)
    error_message = ""
    for number in range(1, max_attempts + 1):
        response = None
        try:
            options = {} if temperature is None else {"temperature": temperature}
            if response_format is not None:
                options["response_format"] = response_format
            response = client.complete(model=model, messages=messages, **options)
            value = validate(response.content)
        except (GenerationError, ValueError, KeyError, TypeError) as error:
            error_message = str(error)
            if attempts is not None:
                attempts.append({"attempt": number, "model": model,
                                 "valid": False, "error": error_message})
            if response is not None:
                messages.extend((
                    {"role": "assistant", "content": response.content},
                    {"role": "user", "content": (
                        "Invalid JSON/schema: " + error_message
                        + ". Return only a corrected JSON object matching the required schema.")},
                ))
        else:
            if attempts is not None:
                attempts.append({"attempt": number, "model": model, "valid": True})
            return value, response.content
    raise PlannerModelError(
        f"No valid model response after {max_attempts} attempts: {error_message}")
