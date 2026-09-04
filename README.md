# SceneSmith: Simple Planner Evaluation

This repository is a companion to [SceneSmith](https://scenesmith.github.io/), providing a simple model-based planner and simulation wrapper to demonstrate robot evaluation using agentically-generated indoor scenes.

For the main SceneSmith codebase and research, please visit the [SceneSmith GitHub repository](https://github.com/nepfaff/scenesmith).

The general online environment is being migrated under the contracts in
[`docs/ONLINE_ENV_REQUIREMENTS.md`](docs/ONLINE_ENV_REQUIREMENTS.md). See
[`docs/ONLINE_ENV_ARCHITECTURE.md`](docs/ONLINE_ENV_ARCHITECTURE.md) for module
boundaries and [`docs/ONLINE_ENV_PROGRESS.md`](docs/ONLINE_ENV_PROGRESS.md) for
verified commands and exact regression results.

## Robot Evaluation Pipeline

This repository focuses on the **Policy Interface** and **Validation** stages of the SceneSmith evaluation pipeline. For a comprehensive overview of how to generate scenes and perform end-to-end evaluation, refer to the [Robot Evaluation section of the main SceneSmith repository](https://github.com/nepfaff/scenesmith?tab=readme-ov-file#-robot-evaluation).

## Getting Started

### 1. Environment Setup

We recommend using a virtual environment to manage dependencies:

```bash
# Create a virtual environment
python3 -m venv venv

# Activate the environment
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Scene Dataset

Download the example scenes dataset from Hugging Face:

[nepfaff/scenesmith-example-scenes](https://huggingface.co/datasets/nepfaff/scenesmith-example-scenes)

Place the downloaded models in the `models/` directory. We have included an example scene and robot task in the `models/21-20-10_cleaned/scene_000/` directory to help get you started.

### 3. Experimental Zerith Setup

Initialize the upstream Zerith model and generate its Drake-compatible OBJ
meshes before running the Zerith examples:

```bash
git submodule update --init --recursive
python scripts/convert_zerith_for_drake.py
python scripts/convert_zerith_for_drake.py --check
```

The generated OBJ files are deterministic derivatives of the upstream STL
files and are intentionally excluded from Git. The generated URDF and
conversion manifest are committed for review. Visual geometry keeps the full
OBJ meshes. Collision geometry uses three boxes for `dipan_link` and primitive
box/cylinder assemblies for both wrists, palms, and fingers; other links still
use whole-mesh convex proxies.

Validate both wrists near their joint limits and each gripper in open,
half-open, and closed configurations without custom collision filters:

```bash
python scripts/validate_zerith_collision_proxies.py
```

Use the `collision` tree in Meshcat after launching a Zerith visualizer to
compare proximity geometry with the visual OBJ meshes.

Inspect left-arm kinematics in a SceneSmith scene:

```bash
python scripts/visualize_zerith_left_arm.py \
    <scene-root>/combined_house/house_furniture_welded.dmd.yaml \
    --robot-xyz <x> <y> <z> \
    --robot-yaw-deg <yaw>
```

Run the online finite-torque left-arm control regression:

```bash
python scripts/simulate_zerith_left_arm.py \
    <scene-root>/combined_house/house_furniture_welded.dmd.yaml
```

This legacy-compatible test uses a 1 kHz Drake plant, a 200 Hz coupled
inverse-dynamics PD servo, and a 10 Hz policy interface. It performs a
five-second home-pose hold
followed by a positive and negative step test for each left-arm joint. It does
not load a plan or use RRT or TOPPRA. Controller-frequency state, gravity, PD,
raw, applied, and saturation values are written to
`zerith_online_control.csv`; the run is recorded in
`zerith_online_control.html`.

Policies can use the environment directly:

```python
from pathlib import Path

import numpy as np

from src.zerith_online_env import ZerithOnlineEnv

env = ZerithOnlineEnv(
    scene_dmd=Path("<scene-root>/combined_house/house_furniture_welded.dmd.yaml"),
    robot_model_dir=Path("models/zerith_drake"),
    target_model_name="living_room_box_0",
)
observation = env.reset()

done = False
while not done:
    action = np.r_[np.zeros(7), 1.0]
    observation, reward, done, info = env.step(action)
```

The action is seven accumulated joint-target increments in radians followed by
one normalized gripper command (`-1` closed, `+1` open). Each action is held
for one policy period. The first version returns zero reward; task rewards and
termination belong to the evaluation layer.

Prepare the tracked 6 x 4 x 3 cm red-box task without modifying the original
SceneSmith output:

```bash
python scripts/prepare_zerith_pick_eval_scene.py \
    <scene-root>/combined_house/house_furniture_welded.dmd.yaml
```

This writes `output/zerith_pick_eval/zerith_pick_eval.dmd.yaml` and synchronized
`task_metadata.yaml`. The box link origin is its geometric center. By default,
the red box is aligned with the open left gripper at rail position 0.4 m and
the zero seven-joint left-arm posture. In the top view, its nearest face is
1 cm beyond the fingertip front plane; this is not a 3D gap because the
fingertips remain above the table. `living_room_vase_0` is moved to the exact
mirrored location on the robot's right side. Both objects have their Z
positions solved against the local coffee-table collision surface. The red
box source yaw is retained, and its roll and pitch follow the coffee-table
body. Four tiled collision boxes preserve its exact dimensions while providing
stable multipoint support. The original scene is not edited. Register both the
original scene package and `models/zerith_pick_eval/package.xml` when loading
the derived DMD.

The authoritative Zerith placement for this pick task is
`xyz=(2.65, 2.95, 0.1815) m`, `yaw=180 deg`. It is shared by visualization,
dynamics, IK, and generated task metadata; CLI arguments may explicitly
override it for a different experiment. DMD `!Rpy deg` values are degrees, and
the generated metadata records the target yaw in both degrees and radians.

Inspect the vertical rail and the calibrated box from multiple views:

```bash
python scripts/visualize_zerith_left_arm.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml \
    --additional-package-xml models/zerith_pick_eval/package.xml \
    --robot-xyz 2.65 2.95 0.1815 \
    --robot-yaw-deg 180 \
    --rail-position 0.4 \
    --q-left 0 0 0 0 0 0 0
```

The `daogui_joint` slider covers the upstream URDF range from 0 to 0.8 m and
moves the complete torso and arm assembly along world Z. The upstream model
sets both effort and velocity to zero and provides no vendor operating height,
speed, or force data. Those values remain uncalibrated and must not be guessed.

Verify that the calibrated target remains stable as a free body for five
seconds:

```bash
python scripts/validate_zerith_target_settle.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml
```

Search collision-safe static left-arm postures jointly at each rail height:

```bash
python scripts/search_zerith_rail_postures.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml
```

The search does not assign a weighted "natural posture" score or approve a
rail height. It records collision distance, normalized joint-limit margin,
elbow bend, wrist midpoint deviation, and grasp-frame Jacobian metrics, then
writes a per-height visual shortlist to
`output/zerith_pick_eval/rail_postures.json`. The right arm remains fixed, so a
height can be rejected because the right wrist intersects the table even when
the left-arm solve itself is well conditioned.

> **Calibration status:** This task fixes `daogui_joint=0.4 m`. PREGRASP and
> the online path are validated only up to the open-gripper PREGRASP pose;
> APPROACH, gripper closure, and grasp execution are intentionally out of
> scope.

First search the generic collision-regression posture `q_safe_home`:

```bash
python scripts/search_zerith_safe_home.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml \
    --rail-position 0.4
```

Then search collision-constrained PREGRASP targets for the left arm:

```bash
python scripts/validate_zerith_pregrasp_ik.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml \
    --rail-position 0.4 \
    --pregrasp-distance 0.11 \
    --approach-tilt-deg 15 30 60 \
    --yaw-offset-deg 0 -20 20 \
    --num-random-seeds 0 \
    --exhaustive
```

The check fixes the base, every non-left-arm joint, and every free scene body;
the box stays at its derived calibrated pose. By default it searches
horizontal, oblique, and top-down approaches at five yaw offsets using
`q_safe_home` and four deterministic random initial guesses. The collision
model has two planning-only layers: all real candidate pairs must remain
nonpenetrating, while environment and non-assembly self-collision pairs target
7 mm and must retain at least 5 mm clearance. The dynamics collision filters
are unchanged. The left-shoulder-to-torso assembly is the sole explicit
safety-layer whitelist: its maximum observed separation was 5.1649 mm in an
81 x 81 joint-range grid search. That is high-density numerical evidence, not
a mathematical proof.

Finally search the task-specific `q_pick_home`, then execute the online motion:

```bash
python scripts/search_zerith_pick_home.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml \
    --pregrasp-json output/zerith_pick_eval/pregrasp_ik.json \
    --rail-position 0.4

python scripts/validate_zerith_safe_home_dynamics.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml \
    --home-json output/zerith_pick_eval/pick_home.json \
    --home-key q_pick_home \
    --rail-position 0.4

python scripts/simulate_zerith_pick_home_to_pregrasp.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml \
    --pick-home-json output/zerith_pick_eval/pick_home.json \
    --rail-position 0.4
```

`q_pick_home` is separate from `q_safe_home`. Its whole wrist-and-gripper
collision AABB must be above the coffee table or outside the table projection
expanded by the wrist-and-gripper footprint and 5 mm. The complete straight
joint edge to PREGRASP is checked with high-density numerical sampling. The
execution is an observation-driven 10 Hz policy over a 200 Hz inverse-dynamics
servo and a 1 kHz plant; it does not use RRT, TOPPRA, or a precomputed
trajectory.

Inspect the saved PREGRASP posture from front, side, and top by orbiting the
Meshcat camera:

```bash
python scripts/visualize_zerith_left_arm.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml \
    --additional-package-xml models/zerith_pick_eval/package.xml \
    --rail-position 0.4 \
    --configuration-json output/zerith_pick_eval/pick_home.json \
    --configuration-key q_pregrasp
```

To watch one online execution in real time while keeping the gripper open:

```bash
python scripts/simulate_zerith_pick_home_to_pregrasp.py \
    output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
    --scene-package-xml <scene-root>/package.xml \
    --pick-home-json output/zerith_pick_eval/pick_home.json \
    --rail-position 0.4 \
    --episodes 1 \
    --meshcat \
    --realtime-rate 1 \
    --record-html output/zerith_pick_eval/online_pregrasp.html
```

## Online Manipulation API

The public API uses typed observations and actions. A policy is an external
object with `reset()` and `act()` methods; it does not receive a Drake Context
and does not modify the environment:

```python
from src.online_manipulation import JointDeltaAction, run_episode


class MyPolicy:
    def reset(self, observation, info):
        del observation, info

    def act(self, observation):
        joint = observation.robot.joint_names[0]
        return JointDeltaAction(joint_names=(joint,), deltas=(0.001,))


result = run_episode(
    env=env,
    policy=MyPolicy(),
    seed=0,
    max_steps=100,
    output_directory="output/my_policy/episode_000",
    record_html=True,
    write_final_dmd=True,
)
```

`env.step(action)` advances exactly one policy period. With the default
`TimingConfig`, that is 0.1 s containing 20 controller updates at 200 Hz and 5
physics steps per controller update at 1 kHz. The command is held between
policy updates; no future waypoint or offline trajectory is consumed.

Every successful step reports how the safety layer handled its command:

```python
decision = info["action_decision"]
print(decision["status"])   # accepted, adjusted, or rejected
print(decision["reasons"])  # e.g. ("maximum_joint_delta",)
```

Joint-step and joint-limit adjustments include requested and applied arm
deltas. A collision-invalid Cartesian edge or out-of-range gripper width is
atomically rejected as a hold command; it is never partially executed.
Programming/configuration errors such as an unknown joint or unsupported frame
still raise an exception. Episode CSV traces include the complete decision as
`action_decision_json`.

The complete Zerith construction, task selection, and batch runner wiring are
in [`scripts/run_zerith_online_example.py`](scripts/run_zerith_online_example.py).
Run replaceable Hold and JointStep policies without editing the environment:

```bash
python -B scripts/run_zerith_online_example.py hold \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml <scene-root>/package.xml \
  --pick-home-json output/zerith_pick_eval/pick_home.json \
  --output-root output/online_examples/hold

python -B scripts/run_zerith_online_example.py joint-step \
  output/zerith_pick_eval/zerith_pick_eval.dmd.yaml \
  --scene-package-xml <scene-root>/package.xml \
  --pick-home-json output/zerith_pick_eval/pick_home.json \
  --output-root output/online_examples/joint_step \
  --episodes 3
```

Each episode gets a distinct directory containing `summary.json` and
`trace.csv`. Add `--record-html` for `simulation.html` and
`--write-final-dmd` for a reloadable `final.dmd.yaml`. Output directories are
never silently overwritten.

### Behavior Tree tick

[`examples/online_manipulation/behavior_tree_tick.py`](examples/online_manipulation/behavior_tree_tick.py)
shows one minimal leaf rather than introducing a Behavior Tree framework. One
tick reads the latest observation, sends one typed action, and advances exactly
one environment policy period:

```python
from examples.online_manipulation import BehaviorTreeStatus, JointTargetLeaf

observation, info = env.reset(seed=0)
leaf = JointTargetLeaf("left_shoulder_pitch_joint", 0.1)

status = BehaviorTreeStatus.RUNNING
while status is BehaviorTreeStatus.RUNNING:
    tick = leaf.tick(env, observation)
    observation = tick.observation
    status = tick.status
```

The same leaf can be wrapped by py_trees, BehaviorTree.CPP bindings, or a
project-specific tree. Tree state remains outside the environment.

### TAMP query and execution handoff

[`examples/online_manipulation/tamp_execution.py`](examples/online_manipulation/tamp_execution.py)
shows the intended TAMP boundary. The planner synchronizes observed free-body
poses, validates the current configuration and complete direct edge in an
independent context, then sends absolute typed joint targets online:

```python
from examples.online_manipulation import execute_validated_joint_goal

observation, info = env.reset(seed=0)
execution = execute_validated_joint_goal(
    env=env,
    query=planning_query,
    observation=observation,
    goal_positions={"left_shoulder_pitch_joint": 0.1},
)
```

An invalid start or edge fails before `env.step()` is called. This helper does
not search a path or perform TOPPRA; a full TAMP system can supply multiple
validated edges through the same query/action interface.

### Replace task or scene

Task identity, reward, allowed contacts, termination, and final metrics belong
to the Task object. Replace `NullTask()` with another Task implementation when
constructing `OnlineManipulationEnv`; no environment-core edit is required.
`NullTask` and `PickLiftTask` are the reference implementations in
[`src/online_manipulation/tasks.py`](src/online_manipulation/tasks.py).

Scene paths and observed objects belong to `ScenarioSpec`:

```python
from pathlib import Path
from src.online_manipulation import ObservedBodySpec, Pose, ScenarioSpec

scenario = ScenarioSpec(
    dmd_path=Path("my_scene/house.dmd.yaml"),
    package_xmls=(Path("my_scene/package.xml"),),
    initial_object_poses={
        "movable_object": Pose(
            translation_m=(0.5, 0.2, 0.8),
            quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        ),
    },
    observed_bodies=(
        ObservedBodySpec(
            observation_name="movable_object",
            model_instance_name="scene_model_name",
            body_name="base_link",
            write_back=True,
        ),
    ),
    contact_parameters={
        "penetration_allowance_m": 0.001,
        "stiction_tolerance_m_s": 0.01,
    },
)
```

Changing a DMD scene or its package map changes this configuration, not the
controller. Initial poses are keyed by the public observation name and are
applied consistently to both simulation and planning contexts. Both listed
contact parameters must be positive; unsupported names fail explicitly.
`write_back=True` is deliberately opt-in: only selected free-body poses are
copied into the final DMD, and the input DMD is never modified. `NullTask`
does not require a target model; task-specific target identity stays outside
the environment core.

If `policy.act()` or `env.step()` raises during an episode, the runner re-raises
the original exception after writing `failure.json`, the partial `trace.csv`,
and the current `simulation.html` recording when enabled.

## Usage

### Running Experiments

To run a standard interactive evaluation on a single scene:

```bash
bash run_experiment.sh models/21-20-10_cleaned/scene_000
```

To run a **non-interactive** evaluation (best for unattended runs or clusters):

```bash
bash run_experiment_noninteractive.sh models/21-20-10_cleaned/scene_000
```

To run evaluations on an entire folder of scenes:

```bash
bash run_experiment_folder.sh models/21-20-10_cleaned/
```

To run folder evaluations in **parallel** using multiple workers:

```bash
# Usage: ./run_experiment_folder_parallel.sh <folder> <num_workers> [--skip-existing]
bash run_experiment_folder_parallel.sh models/21-20-10_cleaned/ 4
```

## Manual Pipeline Workflow

If you need to run the stages of the evaluation pipeline manually, follow these steps from the root of the repository:

### 1. Prepare Scene Directives
Add the robot to the scene directives:
```bash
python3 scripts/add_robot_to_directives.py \
    models/21-20-10_cleaned/scene_000/combined_house/house.dmd.yaml \
    models/21-20-10_cleaned/scene_000/combined_house/robot_commands.json \
    scene_000.dmd.yaml
```

### 2. Visualization
Visualize the prepared scene:
```bash
python3 scripts/visualize_dmd_scene.py \
    scene_000.dmd.yaml \
    --package-xml models/21-20-10_cleaned/scene_000/package.xml \
    --package-xml models/iiwa/package.xml
```

Alternatively, use Drake's model visualizer:
```bash
export ROS_PACKAGE_PATH=$(pwd)/models/iiwa:$(pwd)/models/21-20-10_cleaned/scene_000
python3 -m pydrake.visualization.model_visualizer scene_000.dmd.yaml
```

### 3. Compute Grasp Configuration
Compute a valid grasp and place configuration:
```bash
python3 scripts/compute_grasp_config.py \
    models/21-20-10_cleaned/scene_000/combined_house/robot_commands.json \
    scene_000.dmd.yaml \
    --package-xml models/21-20-10_cleaned/scene_000/package.xml \
    --package-xml models/iiwa/package.xml
```

### 4. Planning
Compute a plan given the generated robot waypoints:
```bash
python3 scripts/plan_robot_waypoints_rrt.py \
    models/21-20-10_cleaned/scene_000/combined_house/robot_commands.json \
    scene_000.dmd.yaml \
    robot_waypoints.json \
    --package-xml models/21-20-10_cleaned/scene_000/package.xml \
    --package-xml models/iiwa/package.xml \
    --out-traj robot_plan.json
```

### 5. Simulation
Simulate the generated plan:
```bash
python3 scripts/simulate.py \
    scene_000.dmd.yaml \
    robot_plan.json \
    --package-xml models/21-20-10_cleaned/scene_000/package.xml \
    --package-xml models/iiwa/package.xml \
    --ee-vel 1 \
    --ee-accel 1 \
    --write-updated-scenario out.dmd.yaml
```
