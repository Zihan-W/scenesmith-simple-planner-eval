export SCENE=$1

python3 scripts/add_robot_to_directives.py $SCENE/combined_house/house.dmd.yaml $SCENE/combined_house/robot_commands.json robot_task.dmd.yaml

python3 scripts/compute_grasp_config_noninteractive.py $SCENE/combined_house/robot_commands.json robot_task.dmd.yaml --package-xml $SCENE/package.xml --package-xml models/iiwa/package.xml

python3 scripts/plan_robot_waypoints_rrt_noninteractive.py $SCENE/combined_house/robot_commands.json robot_task.dmd.yaml robot_waypoints.json --package-xml $SCENE/package.xml --package-xml models/iiwa/package.xml --out-traj robot_plan.json

python3 scripts/simulate_noninteractive.py robot_task.dmd.yaml robot_plan.json --package-xml $SCENE/package.xml --package-xml models/iiwa/package.xml --ee-vel 1 --ee-accel 1 --write-updated-scenario out.dmd.yaml