export SCENE=$1

export TIMELIMIT=60m
export GRACEPERIOD=5s

rm robot_task.dmd.yaml
rm robot_waypoints.json
rm robot_plan.json
rm simulation_good.html
rm simulation_bad.html
rm out_good.dmd.yaml
rm out_bad.dmd.yaml

# timeout -k $GRACEPERIOD $TIMELIMIT \
#     python3 scripts/add_robot_to_directives.py \
#         $SCENE/combined_house/house.dmd.yaml \
#         $SCENE/combined_house/robot_commands.json \
#         robot_task.dmd.yaml

timeout -k $GRACEPERIOD $TIMELIMIT \
    python3 scripts/add_robot_to_directives.py \
        $SCENE/combined_house/house_furniture_welded.dmd.yaml \
        $SCENE/combined_house/robot_commands.json \
        robot_task.dmd.yaml

timeout -k $GRACEPERIOD $TIMELIMIT \
    python3 scripts/compute_grasp_config_noninteractive.py \
        $SCENE/combined_house/robot_commands.json \
        robot_task.dmd.yaml \
        --package-xml $SCENE/package.xml \
        --package-xml models/iiwa/package.xml

if [ ! -f robot_waypoints.json ]; then
    echo "Failed to compute grasp or place configurations in the allotted time."
    exit 1
fi

timeout -k $GRACEPERIOD $TIMELIMIT \
    python3 scripts/plan_robot_waypoints_rrt_noninteractive.py \
        $SCENE/combined_house/robot_commands.json \
        robot_task.dmd.yaml \
        robot_waypoints.json \
        --package-xml $SCENE/package.xml \
        --package-xml models/iiwa/package.xml \
        --out-traj robot_plan.json

if [ ! -f robot_plan.json ]; then
    echo "Failed to compute robot plan in the allotted time."
    exit 2
fi

python3 scripts/simulate_noninteractive.py \
    robot_task.dmd.yaml \
    robot_plan.json \
    --package-xml $SCENE/package.xml \
    --package-xml models/iiwa/package.xml \
    --ee-vel 1 \
    --ee-accel 1 \
    --write-updated-scenario out_good.dmd.yaml

mv simulation.html simulation_good.html

python3 scripts/simulate_noninteractive.py \
    robot_task.dmd.yaml \
    robot_plan.json \
    --package-xml $SCENE/package.xml \
    --package-xml models/iiwa/package.xml \
    --ee-vel 1 \
    --ee-accel 1 \
    --write-updated-scenario out_bad.dmd.yaml

mv simulation.html simulation_bad.html