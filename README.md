### Useful commands:

To do all-in-one, just run `bash run_experiment.sh scene_000`.

### Old:

*All commands should be run from the root of the repository.*

Add the robot to the directives (based on the task):
```
python3 scripts/add_robot_to_directives.py \
    models/scene_008/combined_house/house.dmd.yaml \
    models/scene_008/pick_candle_task.json \
    pick_candle_task.dmd.yaml
```

Visualize the directives file (with the robot):
```
python3 scripts/visualize_dmd_scene.py \
    pick_candle_task.dmd.yaml \
    --package-xml models/iiwa/package.xml \
    --package-xml models/scene_008/package.xml
```

Visualize the directives file with Drake's `model_visualizer`:
```
export ROS_PACKAGE_PATH=/home/tommy/Documents/programming/work/rlg/snippets/policy-eval-for-nicholas/models/iiwa:/home/tommy/Documents/programming/work/rlg/snippets/policy-eval-for-nicholas/models/scene_008;
python3 -m pydrake.visualization.model_visualizer pick_candle_task.dmd.yaml
```

Compute a grasp and place configuration for the manipuland:
```
python3 scripts/compute_grasp_config.py \
    models/scene_008/pick_candle_task.json \
    pick_candle_task.dmd.yaml \
    --package-xml models/iiwa/package.xml \
    --package-xml models/scene_008/package.xml
```

Compute a plan given robot waypoints from the grasp computation script:
```
python3 scripts/plan_robot_waypoints_rrt.py \
    models/scene_008/pick_candle_task.json \
    pick_candle_task.dmd.yaml \
    robot_waypoints.json \
    --package-xml models/iiwa/package.xml \
    --package-xml models/scene_008/package.xml \
    --out-traj robot_plan.json
```

Simulate a plan from the planning script:
```
python3 scripts/simulate.py \
    pick_candle_task.dmd.yaml \
    robot_plan.json \
    --package-xml models/iiwa/package.xml \
    --package-xml models/scene_008/package.xml \
    --ee-vel 1 \
    --ee-accel 1 \
    --write-updated-scenario out.dmd.yaml
```
(Don't include the last two lines to run TOPPRA with only joint-space limits)