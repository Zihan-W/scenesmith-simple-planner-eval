# SceneSmith: Simple Planner Evaluation

This repository is a companion to [SceneSmith](https://scenesmith.github.io/), providing a simple model-based planner and simulation wrapper to demonstrate robot evaluation using agentically-generated indoor scenes.

For the main SceneSmith codebase and research, please visit the [SceneSmith GitHub repository](https://github.com/nepfaff/scenesmith).

## 🤖 Robot Evaluation Pipeline

This repository focuses on the **Policy Interface** and **Validation** stages of the SceneSmith evaluation pipeline. For a comprehensive overview of how to generate scenes and perform end-to-end evaluation, refer to the [Robot Evaluation section of the main SceneSmith repository](https://github.com/nepfaff/scenesmith?tab=readme-ov-file#-robot-evaluation).

The full evaluation process consists of four stages:

1.  **Generate Prompts**: An LLM converts a high-level task into diverse scene prompts.
2.  **Generate Scenes**: Scenes are generated using the SceneSmith pipeline from these prompts.
3.  **Policy Interface (this repo)**: Scenes are converted into robot-executable poses for model-based policies.
4.  **Validate (this repo)**: Task completion is verified using geometric and visual observations.

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

Place the downloaded models in the `models/` directory.

## Usage

### Running Experiments

To run evaluations on an entire folder of scenes:

```bash
bash run_experiment_folder.sh models/21-20-10_cleaned/
```

To run an experiment on a single scene:

```bash
bash run_experiment.sh models/21-20-10_cleaned/scene_000
```

## Manual Pipeline Workflow

If you need to run the stages of the evaluation pipeline manually, follow these steps from the root of the repository:

### 1. Prepare Scene Directives
Add the robot to the scene directives based on the task:
```bash
python3 scripts/add_robot_to_directives.py \
    models/scene_008/combined_house/house.dmd.yaml \
    models/scene_008/pick_candle_task.json \
    pick_candle_task.dmd.yaml
```

### 2. Visualization
Visualize the prepared scene:
```bash
python3 scripts/visualize_dmd_scene.py \
    pick_candle_task.dmd.yaml \
    --package-xml models/iiwa/package.xml \
    --package-xml models/scene_008/package.xml
```

Alternatively, use Drake's model visualizer:
```bash
export ROS_PACKAGE_PATH=$(pwd)/models/iiwa:$(pwd)/models/scene_008
python3 -m pydrake.visualization.model_visualizer pick_candle_task.dmd.yaml
```

### 3. Compute Grasp Configuration
Compute a valid grasp and place configuration:
```bash
python3 scripts/compute_grasp_config.py \
    models/scene_008/pick_candle_task.json \
    pick_candle_task.dmd.yaml \
    --package-xml models/iiwa/package.xml \
    --package-xml models/scene_008/package.xml
```

### 4. Planning
Compute a plan given the generated robot waypoints:
```bash
python3 scripts/plan_robot_waypoints_rrt.py \
    models/scene_008/pick_candle_task.json \
    pick_candle_task.dmd.yaml \
    robot_waypoints.json \
    --package-xml models/iiwa/package.xml \
    --package-xml models/scene_008/package.xml \
    --out-traj robot_plan.json
```

### 5. Simulation
Simulate the generated plan:
```bash
python3 scripts/simulate.py \
    pick_candle_task.dmd.yaml \
    robot_plan.json \
    --package-xml models/iiwa/package.xml \
    --package-xml models/scene_008/package.xml \
    --ee-vel 1 \
    --ee-accel 1 \
    --write-updated-scenario out.dmd.yaml
```