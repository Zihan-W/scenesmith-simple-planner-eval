# SceneSmith: Simple Planner Evaluation

This repository is a companion to [SceneSmith](https://scenesmith.github.io/), providing a simple model-based planner and simulation wrapper to demonstrate robot evaluation using agentically-generated indoor scenes.

For the main SceneSmith codebase and research, please visit the [SceneSmith GitHub repository](https://github.com/nepfaff/scenesmith).

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

This test uses a 1 kHz Drake plant, a 200 Hz gravity-compensated PD
servo, and a 10 Hz policy interface. It performs a five-second home-pose hold
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
