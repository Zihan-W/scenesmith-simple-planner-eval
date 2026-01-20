export SCENE=$1

# WORKDIR allows parallel workers to write outputs to isolated directories
WORKDIR="${WORKDIR:-.}"

export TIMELIMIT=120m
export GRACEPERIOD=5s

echo "Running scene $SCENE"

rm -f "$WORKDIR/robot_task.dmd.yaml"
rm -f "$WORKDIR/robot_waypoints.json"
rm -f "$WORKDIR/robot_plan.json"
rm -f "$WORKDIR/simulation_good.html"
rm -f "$WORKDIR/simulation_bad.html"
rm -f "$WORKDIR/out_good.dmd.yaml"
rm -f "$WORKDIR/out_bad.dmd.yaml"

# timeout -k $GRACEPERIOD $TIMELIMIT \
#     python3 scripts/add_robot_to_directives.py \
#         $SCENE/combined_house/house.dmd.yaml \
#         $SCENE/combined_house/robot_commands.json \
#         "$WORKDIR/robot_task.dmd.yaml"

timeout -k $GRACEPERIOD $TIMELIMIT \
    python3 scripts/add_robot_to_directives.py \
        $SCENE/combined_house/house_furniture_welded.dmd.yaml \
        $SCENE/combined_house/robot_commands.json \
        "$WORKDIR/robot_task.dmd.yaml"

timeout -k $GRACEPERIOD $TIMELIMIT \
    python3 scripts/compute_grasp_config_noninteractive.py \
        $SCENE/combined_house/robot_commands.json \
        "$WORKDIR/robot_task.dmd.yaml" \
        --package-xml $SCENE/package.xml \
        --package-xml models/iiwa/package.xml \
        --out-waypoints "$WORKDIR/robot_waypoints.json"

if [ ! -f "$WORKDIR/robot_waypoints.json" ]; then
    echo "Failed to compute grasp or place configurations in the allotted time for scene $SCENE."
    exit 1
fi

timeout -k $GRACEPERIOD $TIMELIMIT \
    python3 scripts/plan_robot_waypoints_rrt_noninteractive.py \
        $SCENE/combined_house/robot_commands.json \
        "$WORKDIR/robot_task.dmd.yaml" \
        "$WORKDIR/robot_waypoints.json" \
        --package-xml $SCENE/package.xml \
        --package-xml models/iiwa/package.xml \
        --out-traj "$WORKDIR/robot_plan.json"

if [ ! -f "$WORKDIR/robot_plan.json" ]; then
    echo "Failed to compute robot plan in the allotted time for scene $SCENE."
    exit 2
fi

python3 scripts/simulate_noninteractive.py \
    "$WORKDIR/robot_task.dmd.yaml" \
    "$WORKDIR/robot_plan.json" \
    --package-xml $SCENE/package.xml \
    --package-xml models/iiwa/package.xml \
    --ee-vel 1 \
    --ee-accel 1 \
    --write-updated-scenario "$WORKDIR/out_good.dmd.yaml" \
    --record-html "$WORKDIR/simulation_good.html"

python3 scripts/simulate_noninteractive.py \
    "$WORKDIR/robot_task.dmd.yaml" \
    "$WORKDIR/robot_plan.json" \
    --package-xml $SCENE/package.xml \
    --package-xml models/iiwa/package.xml \
    --write-updated-scenario "$WORKDIR/out_bad.dmd.yaml" \
    --record-html "$WORKDIR/simulation_bad.html"

echo "Successfully finished running scene $SCENE"
