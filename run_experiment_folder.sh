SKIP_EXISTING=false
FOLDER=""

for arg in "$@"; do
    case $arg in
        --skip-existing)
            SKIP_EXISTING=true
            ;;
        *)
            FOLDER=$arg
            ;;
    esac
done

export SUCCESS=0
export GRASPFAILURE=0
export PLANFAILURE=0
export SKIPPED=0

# Handle Ctrl+C - kill all child processes
cleanup() {
    echo ""
    echo "Interrupted. Killing child processes..."
    pkill -P $$ 2>/dev/null
    wait 2>/dev/null
    exit 130
}
trap cleanup SIGINT SIGTERM

for DIR in $FOLDER*/; do
    # Skip if both output files already exist
    if $SKIP_EXISTING && \
       [ -f "output/$DIR/out_good.dmd.yaml" ] && \
       [ -f "output/$DIR/out_bad.dmd.yaml" ]; then
        echo "Skipping $DIR (already completed)"
        ((SKIPPED++))
        continue
    fi

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
echo "SKIPPED:       $SKIPPED"