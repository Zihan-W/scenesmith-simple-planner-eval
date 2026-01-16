export FOLDER=$1

export SUCCESS=0
export GRASPFAILURE=0
export PLANFAILURE=0

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

    status=$?
    case $status in
        0)
            ((SUCCESS++))
            ;;
        1)
            ((GRASPFAILURE++))
            ;;
        2)
            ((PLANFAILURE++))
            ;;
        *)
            echo "Warning: unexpected exit code $status for $DIR"
            ;;
    esac
done

echo "================ Summary ================"
echo "SUCCESS:       $SUCCESS"
echo "GRASPFAILURE:  $GRASPFAILURE"
echo "PLANFAILURE:   $PLANFAILURE"