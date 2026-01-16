export FOLDER=$1

for DIR in $FOLDER*/; do
    mkdir -p output/$DIR
    bash run_experiment_noninteractive.sh $DIR
    mv \
        robot_task.dmd.yaml \
        robot_waypoints.json \
        robot_plan.json \
        simulation_good.html \
        simulation_bad.html \
        out_good.dmd.yaml \
        out_bad.dmd.yaml \
        output/$DIR
done