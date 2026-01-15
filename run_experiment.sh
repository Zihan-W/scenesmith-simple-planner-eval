export SCENE=$1

python3 scripts/add_robot_to_directives.py models/21-20-10_cleaned/$SCENE/combined_house/house.dmd.yaml models/21-20-10_cleaned/$SCENE/combined_house/robot_commands.json robot_task.dmd.yaml
python3 scripts/compute_grasp_config.py models/21-20-10_cleaned/$SCENE/combined_house/robot_commands.json robot_task.dmd.yaml --package-xml models/21-20-10_cleaned/$SCENE/package.xml --package-xml models/iiwa/package.xml
python3 scripts/plan_robot_waypoints_rrt.py models/21-20-10_cleaned/$SCENE/combined_house/robot_commands.json robot_task.dmd.yaml robot_waypoints.json --package-xml models/21-20-10_cleaned/$SCENE/package.xml --package-xml models/iiwa/package.xml --out-traj robot_plan.json
python3 scripts/simulate.py robot_task.dmd.yaml robot_plan.json --package-xml models/21-20-10_cleaned/$SCENE/package.xml --package-xml models/iiwa/package.xml --ee-vel 1 --ee-accel 1 --write-updated-scenario out.dmd.yaml