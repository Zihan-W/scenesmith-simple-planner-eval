# Online Environment Quickstart

These commands are validated for the repository at
`/root/workspace/scenesmith-simple-planner-eval`. Activate the repository
virtual environment first:

```bash
cd /root/workspace/scenesmith-simple-planner-eval
source .venv/bin/activate
```

The generated Drake OBJ meshes must exist before the first run. Generate them
from the checked-out Zerith submodule with this command:

```bash
python scripts/convert_zerith_for_drake.py --source-model-dir external/Zerith_Model/ZR_H1PRO-1.2.00.H.V4.3_URDF_2025.12.02 --output-model-dir models/zerith_drake
```

## Minimal public API smoke test

This example uses the self-contained minimal DMD and imports only the public
API. It can be launched from any working directory:

```bash
PYTHONPATH=/root/workspace/scenesmith-simple-planner-eval MPLCONFIGDIR=/tmp/matplotlib-cache /root/workspace/scenesmith-simple-planner-eval/.venv/bin/python /root/workspace/scenesmith-simple-planner-eval/examples/online_manipulation/public_api_client.py --repository-root /root/workspace/scenesmith-simple-planner-eval --seed 0
```

Expected output ends with a JSON object containing
`"action_type": "HoldAction"`, `"simulation_time_s": 0.1`, and
`"action_status": "accepted"`.

## Validated PickLift

The complete PickLift demonstration depends on a SceneSmith-generated scene
package at
`/root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000` and on the
generated calibration files under this repository's ignored
`output/zerith_pick_eval` directory. Those assets are **not** contained in the
`online-env-v0.1` Git tag. Prepare or copy them before running this command.

Run one fixed-seed physical PickLift with live Meshcat and an HTML recording:

```bash
cd /root/workspace/scenesmith-simple-planner-eval
python -B scripts/run_zerith_online_example.py pick-lift /root/workspace/scenesmith-simple-planner-eval/output/zerith_pick_eval/zerith_pick_eval.dmd.yaml --scene-package-xml /root/workspace/scenesmith/outputs/2026-09-02/10-01-49/scene_000/package.xml --pick-home-json /root/workspace/scenesmith-simple-planner-eval/output/zerith_pick_eval/pick_home.json --output-root /root/workspace/scenesmith-simple-planner-eval/output/quickstart_pick_lift_seed_500 --episodes 1 --seed 500 --max-steps 1200 --maximum-joint-step 0.1 --maximum-cartesian-joint-step 0.02 --closed-width 0 --meshcat --meshcat-port 7025 --record-html --write-final-dmd
```

The terminal prints a `Meshcat URL` while the simulation is running. A
successful validated run ends with:

```text
Episode 0: success=True, reason=lift_held, steps=276
```

Artifacts are written to
`output/quickstart_pick_lift_seed_500/episode_000_seed_500/`:

* `simulation.html`: standalone Meshcat recording;
* `summary.json`: episode result and metrics;
* `trace.csv`: one row per policy step;
* `final.dmd.yaml`: a new DMD containing final free-body poses.

The input DMD is never modified. Use a new `--output-root` if the documented
output directory already exists; benchmark outputs are intentionally not
overwritten.
