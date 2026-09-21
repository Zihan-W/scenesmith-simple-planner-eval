"""One versioned scene-and-task-to-BT generation pipeline for SceneSmith.

The public boundary is one request JSON document and one result JSON document.
Provider credentials and endpoint configuration are transport settings and never
appear in either document.
"""

from __future__ import annotations

import argparse
import base64
import dataclasses
import hashlib
import json
import mimetypes
import os
from pathlib import Path
from typing import Protocol
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from planner.src.bt.core import skill_registry, to_dict, to_mdsl, to_mermaid
from planner.src.bt.picklift import (
    compile_response as compile_picklift,
    validate_inputs as validate_picklift,
)
from planner.src.bt.navigation import (
    compile_model_response as compile_navigation,
    generate_tree as validate_navigation_plan,
    planning_prompt as navigation_prompt,
)
from planner.src.bt.visualization import render_viewer


REQUEST_SCHEMA = "scenesmith.bt_generation.request.v1"
RESULT_SCHEMA = "scenesmith.bt_generation.result.v1"
LEGACY_REQUEST_SCHEMA = "scenesmith.picklift_bt_generation.request.v1"
LEGACY_RESULT_SCHEMA = "scenesmith.picklift_bt_generation.result.v1"
PROMPT_VERSION = "microsoft-scene-aware-bt-v1"
CAMERA_ORDER = ("head_camera", "left_wrist_camera")
_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"

# Task-specific compilers are execution contracts, not separate generation pipelines.
# A new skill extends the relevant compiler/registry; the model call, provenance,
# retry loop, output format, and CLI below remain shared.
PROFILES = {
    "scenesmith.bt.picklift_task_plan.v1": {
        "name": "picklift", "compile": compile_picklift,
        "environment_schemas": {"scenesmith.verigraph_pick.environment.v1"},
    },
    "scenesmith.bt.task_plan.v1": {
        "name": "navigation", "compile": compile_navigation,
        "environment_schemas": {"scenesmith.bt.environment.v1", "scenesmith.bt.environment.v2"},
    },
}


class GenerationError(RuntimeError):
    """The provider or strict BT compiler could not produce a result."""


@dataclasses.dataclass(frozen=True)
class ChatCompletion:
    """Provider-neutral subset of a Chat Completions response."""

    content: str
    model: str
    response_id: str
    finish_reason: str | None = None
    usage: dict | None = None


class ChatClient(Protocol):
    """Transport dependency injected into :func:`generate`."""

    def complete(self, *, model: str, messages: list[dict],
                 temperature: float | None = None,
                 response_format: dict | None = None) -> ChatCompletion:
        ...


@dataclasses.dataclass(frozen=True)
class LoadedRequest:
    document: dict
    request_sha256: str
    base_dir: Path
    environment: dict
    environment_sha256: str
    task_plan: dict
    task_plan_sha256: str
    images: tuple[dict, ...]
    profile: str


def _canonical_bytes(value) -> bytes:
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _expect_object(value, keys, label):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError(f"{label} must contain exactly {sorted(keys)}")
    return value


def _load_bound_file(base_dir: Path, reference: dict, label: str):
    _expect_object(reference, ("path", "sha256"), label)
    if not isinstance(reference["path"], str) or not reference["path"].strip():
        raise ValueError(f"{label}.path must be a non-empty string")
    expected = reference["sha256"]
    if not isinstance(expected, str) or len(expected) != 64:
        raise ValueError(f"{label}.sha256 must be a SHA-256 hex digest")
    path = (base_dir / reference["path"]).resolve()
    try:
        path.relative_to(base_dir)
    except ValueError as error:
        raise ValueError(f"{label}.path must remain inside the request directory") from error
    data = path.read_bytes()
    actual = _sha256(data)
    if actual != expected:
        raise ValueError(f"{label} SHA-256 mismatch")
    return path, data, actual


def load_request(request_path) -> LoadedRequest:
    """Load a generic request or a legacy PickLift request with bound inputs."""

    path = Path(request_path).resolve()
    document = json.loads(path.read_text(encoding="utf-8"))
    _expect_object(
        document,
        ("schema", "request_id", "environment", "task", "observations", "model"),
        "request",
    )
    if document["schema"] not in (REQUEST_SCHEMA, LEGACY_REQUEST_SCHEMA):
        raise ValueError(f"request.schema must be {REQUEST_SCHEMA}")
    if not isinstance(document["request_id"], str) or not document["request_id"].strip():
        raise ValueError("request_id must be a non-empty string")

    environment_path, environment_bytes, environment_sha = _load_bound_file(
        path.parent, document["environment"], "environment"
    )
    task = _expect_object(
        document["task"], ("description", "plan_path", "plan_sha256"), "task"
    )
    if not isinstance(task["description"], str) or not task["description"].strip():
        raise ValueError("task.description must be a non-empty string")
    task_plan_path, task_plan_bytes, task_plan_sha = _load_bound_file(
        path.parent,
        {"path": task["plan_path"], "sha256": task["plan_sha256"]},
        "task plan",
    )
    environment = json.loads(environment_bytes)
    task_plan = json.loads(task_plan_bytes)
    profile = PROFILES.get(task_plan.get("schema"))
    if profile is None or environment.get("schema") not in profile["environment_schemas"]:
        raise ValueError("Unsupported environment/task-plan schema combination")
    if document["schema"] == LEGACY_REQUEST_SCHEMA and profile["name"] != "picklift":
        raise ValueError("Legacy request only supports PickLift")
    if profile["name"] == "picklift":
        validate_picklift(environment, task_plan)
    else:
        if environment["schema"] == "scenesmith.bt.environment.v2":
            extraction = environment.get("extraction", {})
            if extraction.get("method") != "scenesmith_runtime_reset_and_drake_proximity_geometry":
                raise ValueError("Navigation environment v2 must come from the runtime extractor")
            if not environment.get("navigation_geometry", {}).get("obstacle_aabbs_xy_m"):
                raise ValueError("Navigation environment has no extracted geometry")
        # The canonical compiler also checks every step and available skill.
        validate_navigation_plan(environment, task_plan)

    observations = document["observations"]
    expected_cameras = CAMERA_ORDER if profile["name"] == "picklift" else ()
    if not isinstance(observations, list) or len(observations) != len(expected_cameras):
        raise ValueError(
            "observations must contain head and left-wrist images"
            if expected_cameras else "navigation observations must be empty"
        )
    images = []
    for index, (observation, expected_camera) in enumerate(
        zip(observations, expected_cameras, strict=True), 1
    ):
        _expect_object(
            observation, ("camera", "path", "sha256", "media_type"),
            f"observations[{index - 1}]",
        )
        if observation["camera"] != expected_camera:
            raise ValueError(f"observation {index} must be {expected_camera}")
        if observation["media_type"] != "image/png":
            raise ValueError("first-person observations must be image/png")
        image_path, image_bytes, image_sha = _load_bound_file(
            path.parent,
            {"path": observation["path"], "sha256": observation["sha256"]},
            f"observation {expected_camera}",
        )
        if not image_bytes.startswith(_PNG_SIGNATURE):
            raise ValueError(f"observation {expected_camera} is not a PNG image")
        images.append({
            "camera": expected_camera,
            "path": image_path,
            "sha256": image_sha,
            "media_type": observation["media_type"],
        })

    model = _expect_object(document["model"], ("id", "max_attempts"), "model")
    if not isinstance(model["id"], str) or not model["id"].strip():
        raise ValueError("model.id must be a non-empty string")
    if not isinstance(model["max_attempts"], int) or not 1 <= model["max_attempts"] <= 5:
        raise ValueError("model.max_attempts must be an integer from 1 to 5")

    runtime = environment.get("runtime_snapshot", {})
    cameras = runtime.get("robot_camera_observations", {})
    visibility = runtime.get("target_camera_visibility", {})
    for image in images:
        camera = cameras.get(image["camera"], {})
        recorded = camera.get("files", {}).get("rgb", {}).get("sha256")
        if recorded != image["sha256"]:
            raise ValueError(
                f"{image['camera']} is not bound to the environment reset observation"
            )
        target = visibility.get(image["camera"], {})
        if target.get("visible") is not True or int(target.get("pixel_count", 0)) <= 0:
            raise ValueError(f"target is not visible in {image['camera']}")

    return LoadedRequest(
        document=document,
        request_sha256=_sha256(_canonical_bytes(document)),
        base_dir=path.parent,
        environment=environment,
        environment_sha256=environment_sha,
        task_plan=task_plan,
        task_plan_sha256=task_plan_sha,
        images=tuple(images),
        profile=profile["name"],
    )


def _planning_environment(loaded: LoadedRequest) -> dict:
    environment = loaded.environment
    runtime = environment["runtime_snapshot"]
    semantic = environment["verigraph"]["metadata"]["environment"]
    camera_observations = {}
    for image in loaded.images:
        name = image["camera"]
        camera = runtime["robot_camera_observations"][name]
        camera_observations[name] = {
            "frame": camera["frame"],
            "timestamp_s": camera["timestamp_s"],
            "pose": camera["pose"],
            "intrinsics": camera["intrinsics"],
            "rgb_sha256": image["sha256"],
            "target_visibility": runtime["target_camera_visibility"][name],
        }
    return {
        "semantic_environment": semantic,
        "runtime_task": runtime["task"],
        "runtime_robot": runtime["robot"],
        "robot_camera_observations": camera_observations,
        "available_bt_skills": runtime["available_bt_skills"],
    }


def _skill_prompt(profile: str) -> str:
    return json.dumps(skill_registry(profile), ensure_ascii=False, indent=2)


def build_messages(loaded: LoadedRequest) -> list[dict]:
    """Build auditable path-based multimodal messages without embedding secrets."""

    if loaded.profile == "navigation":
        return [
            {"role": "system", "content": "Generate a SceneSmith behavior tree from validated scene and task inputs."},
            {"role": "user", "content": (
                navigation_prompt(loaded.environment, loaded.task_plan)
                + "\nTask instruction: " + loaded.document["task"]["description"]
                + "\nBT semantics: sequence runs children in order; selector returns the first SUCCESS or RUNNING. "
                "The trusted compiler adds the root success guard."
            )},
        ]
    semantics = (
        "A sequence ticks its children in order and fails when a child fails. "
        "A selector ticks children in order and returns RUNNING or SUCCESS from the first "
        "child with that status; it fails only when every child fails. The trusted runtime "
        "adds a root selector with PickLiftSucceeded as the success guard."
    )
    contract = (
        "Return only one strict JSON object with exactly MAIN_SEQUENCE and ULTIMATE_GOAL. "
        "MAIN_SEQUENCE must be one MDSL sequence string. Use only the finite ROBOT ACTION "
        "LIST and preserve every validated task-plan step exactly once and in order. Do not "
        "add conditions, branches, retries, goal checks, unsupported skills, Markdown, or "
        "prose. All arguments are double-quoted JSON strings and must match the ROBOT ACTION "
        "LIST signatures. ULTIMATE_GOAL must be a non-empty natural-language "
        "description containing the required lift distance and hold duration. Generation does "
        "not prove execution success."
    )
    payload = {
        "environment": _planning_environment(loaded),
        "instruction": loaded.document["task"]["description"],
        "validated_task_plan": loaded.task_plan,
    }
    content = [{
        "type": "text",
        "text": (
            "Start working. The following scene metadata and task are authoritative.\n"
            + json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            + "\n\nFIRST-PERSON OBSERVATION CONTRACT: Image 1 is head_camera and Image 2 "
            "is left_wrist_camera. Both are synchronized robot-mounted reset observations; "
            "no external or third-person image is provided. Use both views together.\n\n"
            + contract
        ),
    }]
    for image in loaded.images:
        content.append({
            "type": "image_url",
            "image_url": {
                "path": str(image["path"]),
                "sha256": image["sha256"],
                "mime_type": image["media_type"],
            },
        })
    return [
        {
            "role": "system",
            "content": (
                "You are an excellent interpreter of human instructions. Given an instruction "
                "and information about the working environment, break it down into a sequence "
                "of robot actions."
            ),
        },
        {
            "role": "user",
            "content": (
                "Use behavior-tree semantics from the Microsoft scene-aware robot BT planner "
                "adaptation. " + semantics + "\n\nROBOT ACTION LIST:\n" + _skill_prompt("picklift")
            ),
        },
        {"role": "assistant", "content": "Waiting for task input."},
        {"role": "user", "content": content},
    ]


def _repair_prompt(error: Exception) -> str:
    return (
        f"Strict compiler validation failed: {error}. Return a corrected response using the "
        "same scene, instruction, images, and validated task plan. Return exactly one JSON "
        "object with exactly MAIN_SEQUENCE and ULTIMATE_GOAL. MAIN_SEQUENCE must be one MDSL "
        "sequence string. Preserve task steps once and in order. Use only skills from the "
        "validated task plan. Do not emit Markdown or prose."
    )


def generate(loaded: LoadedRequest, client: ChatClient) -> dict:
    """Generate and strictly compile one task-independent result document."""

    messages = build_messages(loaded)
    model = loaded.document["model"]
    attempts = []
    accepted = None
    response = root = None
    for number in range(1, model["max_attempts"] + 1):
        completion = client.complete(model=model["id"], messages=messages)
        if not isinstance(completion.content, str) or not completion.content.strip():
            raise GenerationError("Provider returned an empty text response")
        validation_error = None
        try:
            response, root = PROFILES[loaded.task_plan["schema"]]["compile"](
                loaded.environment, loaded.task_plan, completion.content
            )
            accepted = completion
        except ValueError as error:
            validation_error = str(error)
        attempts.append({
            "attempt": number,
            "provider_response_id": completion.response_id,
            "model": completion.model,
            "finish_reason": completion.finish_reason,
            "usage": completion.usage,
            "validation_error": validation_error,
        })
        if accepted is not None:
            break
        if number < model["max_attempts"]:
            messages.extend([
                {"role": "assistant", "content": completion.content},
                {"role": "user", "content": _repair_prompt(ValueError(validation_error))},
            ])
    if accepted is None or response is None or root is None:
        last = attempts[-1]["validation_error"] if attempts else "no provider response"
        raise GenerationError(
            f"No response passed strict compilation after {model['max_attempts']} attempts: {last}"
        )

    mdsl = to_mdsl(root)
    generation = {
        "architecture": "Microsoft scene-aware robot BT planner prompt adaptation",
        "prompt_version": PROMPT_VERSION if loaded.document["schema"] == REQUEST_SCHEMA else "microsoft-scene-aware-picklift-first-person-v1",
        "model_requested": model["id"],
        "model_returned": accepted.model,
        "provider_response_id": accepted.response_id,
        "accepted_attempt": len(attempts),
        "api_calls_completed": len(attempts),
        "attempts": attempts,
        "environment_sha256": loaded.environment_sha256,
        "task_plan_sha256": loaded.task_plan_sha256,
        "request_sha256": loaded.request_sha256,
        "multimodal": bool(loaded.images),
        "robot_first_person_only": bool(loaded.images),
        "profile": loaded.profile,
        "reference_answer_used": False,
        "views": [
            {"camera": image["camera"], "sha256": image["sha256"]}
            for image in loaded.images
        ],
    }
    return {
        "schema": LEGACY_RESULT_SCHEMA if loaded.document["schema"] == LEGACY_REQUEST_SCHEMA else RESULT_SCHEMA,
        "request_id": loaded.document["request_id"],
        "request_sha256": loaded.request_sha256,
        "raw_response": accepted.content,
        "parsed_response": response,
        "generation": generation,
        "mdsl": mdsl,
        "mdsl_sha256": _sha256(mdsl.encode("utf-8")),
        "tree": to_dict(root),
    }


def write_result(result: dict, output_dir) -> Path:
    """Write the executable result and automatically create an offline BT viewer."""

    expected = {
        "schema", "request_id", "request_sha256", "raw_response", "parsed_response",
        "generation", "mdsl", "mdsl_sha256", "tree",
    }
    _expect_object(result, expected, "result")
    if result["schema"] not in (RESULT_SCHEMA, LEGACY_RESULT_SCHEMA):
        raise ValueError(f"result.schema must be {RESULT_SCHEMA}")
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    targets = (
        destination / "generated_plan.json",
        destination / "generated_bt.json",
        destination / "generated_bt.mdsl",
        destination / "generated_bt.mmd",
        destination / "generated_bt.html",
    )
    if any(target.exists() for target in targets):
        raise FileExistsError("BT generation output already exists")
    viewer = render_viewer(result["tree"], plan=result)
    targets[0].write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    targets[1].write_text(
        json.dumps(result["tree"], ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    targets[2].write_text(result["mdsl"], encoding="utf-8")
    targets[3].write_text(to_mermaid(result["tree"]), encoding="utf-8")
    targets[4].write_text(viewer, encoding="utf-8")
    return targets[0]


class OpenAICompatibleChatClient:
    """Small OpenAI-compatible Chat Completions transport adapter."""

    def __init__(self, *, base_url: str, api_key: str, timeout_s=180.0,
                 max_tokens=2048, token_parameter="max_completion_tokens"):
        if not base_url or not api_key:
            raise ValueError("base_url and api_key are required")
        if token_parameter not in ("max_tokens", "max_completion_tokens"):
            raise ValueError("Unsupported token parameter")
        self.base_url = base_url.rstrip("/")
        self.api_key = api_key
        self.timeout_s = float(timeout_s)
        self.max_tokens = int(max_tokens)
        self.token_parameter = token_parameter

    @staticmethod
    def _wire_messages(messages):
        result = []
        for message in messages:
            output = dict(message)
            if isinstance(output.get("content"), list):
                content = []
                for item in output["content"]:
                    item = dict(item)
                    image = item.get("image_url")
                    if item.get("type") == "image_url" and isinstance(image, dict):
                        path = Path(image["path"])
                        actual = _sha256(path.read_bytes())
                        if actual != image["sha256"]:
                            raise GenerationError("Observation changed after request validation")
                        media = image.get("mime_type") or mimetypes.guess_type(path.name)[0]
                        encoded = base64.b64encode(path.read_bytes()).decode("ascii")
                        item["image_url"] = {"url": f"data:{media};base64,{encoded}"}
                    content.append(item)
                output["content"] = content
            result.append(output)
        return result

    def complete(self, *, model: str, messages: list[dict],
                 temperature: float | None = None,
                 response_format: dict | None = None) -> ChatCompletion:
        body = {
            "model": model,
            "messages": self._wire_messages(messages),
            self.token_parameter: self.max_tokens,
            "stream": False,
        }
        if temperature is not None:
            body["temperature"] = temperature
        if response_format is not None:
            body["response_format"] = response_format
        request = Request(
            self.base_url + "/chat/completions",
            data=json.dumps(body).encode("utf-8"),
            headers={
                "Authorization": "Bearer " + self.api_key,
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                payload = json.load(response)
        except HTTPError as error:
            raise GenerationError(f"Provider HTTP {error.code}") from None
        except (URLError, TimeoutError, OSError, json.JSONDecodeError):
            raise GenerationError("Provider connection failed or returned invalid JSON") from None
        try:
            choice = payload["choices"][0]
            content = choice["message"]["content"]
        except (KeyError, IndexError, TypeError):
            raise GenerationError("Provider returned an invalid Chat Completions response") from None
        if choice.get("finish_reason") not in ("stop", None):
            raise GenerationError("Provider did not finish one complete response")
        if not isinstance(content, str) or not content.strip():
            raise GenerationError("Provider returned an empty text response")
        return ChatCompletion(
            content=content,
            model=str(payload.get("model", model)),
            response_id=str(payload.get("id", "")),
            finish_reason=choice.get("finish_reason"),
            usage=payload.get("usage"),
        )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate a strictly compiled SceneSmith BT from one versioned request."
    )
    parser.add_argument("--request", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--model-response", type=Path,
                        help="Replay a recorded model response without contacting a provider")
    parser.add_argument("--base-url", default=os.environ.get("OPENAI_BASE_URL"))
    parser.add_argument("--api-key-env", default="OPENAI_API_KEY")
    parser.add_argument(
        "--token-parameter", choices=("max_tokens", "max_completion_tokens"),
        default="max_completion_tokens",
    )
    parser.add_argument("--max-tokens", type=int, default=2048)
    parser.add_argument("--timeout-s", type=float, default=180.0)
    args = parser.parse_args(argv)
    loaded = load_request(args.request)
    if args.model_response:
        class RecordedClient:
            def complete(self, *, model, messages):
                return ChatCompletion(
                    content=args.model_response.read_text(encoding="utf-8"),
                    model=model, response_id="recorded-response", finish_reason="stop",
                )
        client = RecordedClient()
    else:
        api_key = os.environ.get(args.api_key_env)
        if not args.base_url or not api_key:
            parser.error("--base-url/OPENAI_BASE_URL and the API-key environment variable are required")
        client = OpenAICompatibleChatClient(
            base_url=args.base_url,
            api_key=api_key,
            timeout_s=args.timeout_s,
            max_tokens=args.max_tokens,
            token_parameter=args.token_parameter,
        )
    output = write_result(generate(loaded, client), args.output_dir)
    artifact_sha = _sha256(output.read_bytes())
    print(json.dumps({
        "output": str(output.resolve()),
        "sha256": artifact_sha,
        "visualization": str((output.parent / "generated_bt.html").resolve()),
    }))


if __name__ == "__main__":
    main()
