# PickLift BT generation module

`scenesmith-generate-picklift-bt` is the fixed boundary between perception/task
preparation and BT execution. It accepts one
`scenesmith.picklift_bt_generation.request.v1` JSON file and writes one executable
`scenesmith.picklift_bt_generation.result.v1` file plus JSON, MDSL, and Mermaid
projections of the same tree.

The v1 input binds an authoritative VeriGraph environment, validated task plan,
task description, robot head image, robot left-wrist image, and model selection by
SHA-256. Paths are relative to the request file and must remain inside its directory.
The image order is fixed: `head_camera`, then `left_wrist_camera`. Both images must
match the reset-camera hashes in the environment and must contain visible target
label pixels. Provider URLs and API keys are transport settings and are deliberately
excluded from the request.

The output is accepted only after the existing strict PickLift compiler confirms
that the model returned exactly `MAIN_SEQUENCE` and `ULTIMATE_GOAL`, used only the
finite skill registry, and preserved the validated task steps. Compiler failures can
be fed back for at most `model.max_attempts`; the module never supplies a reference
answer. Every provider attempt and validation error is recorded in `generation`.

Run it with:

```bash
export OPENAI_BASE_URL='https://provider.example/v1'
export OPENAI_API_KEY='...'

scenesmith-generate-picklift-bt \
  --request /path/to/request.json \
  --output-dir /path/to/fresh-output
```

The normative JSON Schemas are
`docs/contracts/picklift_bt_generation_request.schema.json` and
`docs/contracts/picklift_bt_generation_result.schema.json`. The Python module also
performs semantic checks that JSON Schema cannot express, including file hashes,
camera-to-reset binding, target visibility, VeriGraph provenance, and strict BT
compilation.

A request has exactly this shape; every path is relative to `request.json`:

```json
{
  "schema": "scenesmith.picklift_bt_generation.request.v1",
  "request_id": "picklift-run-001",
  "environment": {"path": "input/environment.json", "sha256": "<64 lowercase hex>"},
  "task": {
    "description": "Wait 1.0 seconds, then lift pick_target by 0.08 m and hold it for 3.0 seconds.",
    "plan_path": "input/task_plan.json",
    "plan_sha256": "<64 lowercase hex>"
  },
  "observations": [
    {"camera": "head_camera", "path": "input/cameras/head_camera_rgb.png", "sha256": "<64 lowercase hex>", "media_type": "image/png"},
    {"camera": "left_wrist_camera", "path": "input/cameras/left_wrist_camera_rgb.png", "sha256": "<64 lowercase hex>", "media_type": "image/png"}
  ],
  "model": {"id": "gpt-4.1-mini-2025-04-14", "max_attempts": 3}
}
```

`generated_plan.json` has exactly nine top-level fields: `schema`, `request_id`,
`request_sha256`, `raw_response`, `parsed_response`, `generation`, `mdsl`,
`mdsl_sha256`, and `tree`. The existing PickLift execution policy recompiles the raw
response and verifies the environment, MDSL, and tree hashes before control starts.

This boundary is intentionally PickLift-specific. `ExecutePickLift` remains a
composite calibrated leaf, so a future task should add a new versioned skill registry
and executor rather than silently assigning new semantics to this v1 contract.
