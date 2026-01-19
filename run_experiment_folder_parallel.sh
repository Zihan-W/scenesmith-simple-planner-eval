#!/bin/bash
#
# Parallel experiment runner
# Usage: ./run_experiment_folder_parallel.sh <folder> <num_workers> [--skip-existing]
#

set -e

# Parse arguments
SKIP_EXISTING=false
FOLDER=""
NUM_WORKERS=""

for arg in "$@"; do
    case $arg in
        --skip-existing)
            SKIP_EXISTING=true
            ;;
        *)
            if [ -z "$FOLDER" ]; then
                FOLDER=$arg
            elif [ -z "$NUM_WORKERS" ]; then
                NUM_WORKERS=$arg
            fi
            ;;
    esac
done

# Validate arguments
if [ -z "$FOLDER" ] || [ -z "$NUM_WORKERS" ]; then
    echo "Usage: $0 <folder> <num_workers> [--skip-existing]"
    echo ""
    echo "Arguments:"
    echo "  folder         Directory containing experiment subdirectories (with trailing slash)"
    echo "  num_workers    Number of parallel workers"
    echo "  --skip-existing  Skip experiments that already have output files"
    echo ""
    echo "Example:"
    echo "  $0 models/15-15-01_cleaned/ 4 --skip-existing"
    exit 1
fi

if ! [[ "$NUM_WORKERS" =~ ^[0-9]+$ ]] || [ "$NUM_WORKERS" -lt 1 ]; then
    echo "Error: num_workers must be a positive integer"
    exit 1
fi

# Create unique run ID
RUN_ID=$(date +%Y%m%d_%H%M%S)_$$
LOG_DIR="logs/run_$RUN_ID"
mkdir -p "$LOG_DIR"

echo "============================================"
echo "Parallel Experiment Runner"
echo "============================================"
echo "Folder:       $FOLDER"
echo "Workers:      $NUM_WORKERS"
echo "Skip existing: $SKIP_EXISTING"
echo "Run ID:       $RUN_ID"
echo "Log dir:      $LOG_DIR"
echo "============================================"

# Collect all experiment directories
DIRS=()
for DIR in $FOLDER*/; do
    if [ -d "$DIR" ]; then
        # Skip if both output files already exist and --skip-existing is set
        if $SKIP_EXISTING && \
           [ -f "output/$DIR/out_good.dmd.yaml" ] && \
           [ -f "output/$DIR/out_bad.dmd.yaml" ]; then
            echo "Skipping $DIR (already completed)"
            continue
        fi
        DIRS+=("$DIR")
    fi
done

TOTAL_DIRS=${#DIRS[@]}

if [ "$TOTAL_DIRS" -eq 0 ]; then
    echo "No experiments to run."
    exit 0
fi

echo "Found $TOTAL_DIRS experiment(s) to process"
echo ""

# Array to track worker PIDs
declare -a WORKER_PIDS

# Cleanup function for graceful shutdown
cleanup() {
    echo ""
    echo "Interrupted. Stopping workers..."

    # Kill all worker processes
    for pid in "${WORKER_PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null
        fi
    done

    # Wait for workers to finish
    for pid in "${WORKER_PIDS[@]}"; do
        wait "$pid" 2>/dev/null
    done

    # Print partial results
    aggregate_results

    exit 130
}
trap cleanup SIGINT SIGTERM

# Worker function
run_worker() {
    local WORKER_ID=$1
    shift
    local WORKER_DIRS=("$@")

    local WORKDIR="$LOG_DIR/worker_${WORKER_ID}_workdir"
    local STATE_FILE="$LOG_DIR/worker_${WORKER_ID}_state.json"
    local LOG_FILE="$LOG_DIR/worker_${WORKER_ID}.log"

    mkdir -p "$WORKDIR"

    # Initialize state file
    echo '{"success":0,"graspfailure":0,"planfailure":0,"processed":0}' > "$STATE_FILE"

    local SUCCESS=0
    local GRASPFAILURE=0
    local PLANFAILURE=0
    local PROCESSED=0

    for DIR in "${WORKER_DIRS[@]}"; do
        echo "[Worker $WORKER_ID] Processing: $DIR" >> "$LOG_FILE"

        mkdir -p "output/$DIR"

        # Run the experiment with isolated WORKDIR
        WORKDIR="$WORKDIR" bash run_experiment_noninteractive.sh "$DIR" >> "$LOG_FILE" 2>&1
        local status=$?

        # Move output files to final destination
        for f in robot_task.dmd.yaml robot_waypoints.json robot_plan.json \
                 simulation_good.html simulation_bad.html out_good.dmd.yaml out_bad.dmd.yaml; do
            if [ -f "$WORKDIR/$f" ]; then
                mv "$WORKDIR/$f" "output/$DIR/" 2>/dev/null || true
            fi
        done

        # Update statistics
        case $status in
            0)
                ((SUCCESS++))
                echo "[Worker $WORKER_ID] SUCCESS: $DIR" >> "$LOG_FILE"
                ;;
            1)
                ((GRASPFAILURE++))
                echo "[Worker $WORKER_ID] GRASPFAILURE: $DIR" >> "$LOG_FILE"
                ;;
            2)
                ((PLANFAILURE++))
                echo "[Worker $WORKER_ID] PLANFAILURE: $DIR" >> "$LOG_FILE"
                ;;
            *)
                echo "[Worker $WORKER_ID] UNKNOWN($status): $DIR" >> "$LOG_FILE"
                ;;
        esac

        ((PROCESSED++))

        # Update state file
        echo "{\"success\":$SUCCESS,\"graspfailure\":$GRASPFAILURE,\"planfailure\":$PLANFAILURE,\"processed\":$PROCESSED}" > "$STATE_FILE"
    done

    echo "[Worker $WORKER_ID] Finished. SUCCESS=$SUCCESS GRASPFAILURE=$GRASPFAILURE PLANFAILURE=$PLANFAILURE" >> "$LOG_FILE"
}

# Function to aggregate results from all workers
aggregate_results() {
    local TOTAL_SUCCESS=0
    local TOTAL_GRASPFAILURE=0
    local TOTAL_PLANFAILURE=0
    local TOTAL_PROCESSED=0

    for i in $(seq 0 $((NUM_WORKERS - 1))); do
        local STATE_FILE="$LOG_DIR/worker_${i}_state.json"
        if [ -f "$STATE_FILE" ]; then
            local SUCCESS=$(grep -o '"success":[0-9]*' "$STATE_FILE" | grep -o '[0-9]*')
            local GRASPFAILURE=$(grep -o '"graspfailure":[0-9]*' "$STATE_FILE" | grep -o '[0-9]*')
            local PLANFAILURE=$(grep -o '"planfailure":[0-9]*' "$STATE_FILE" | grep -o '[0-9]*')
            local PROCESSED=$(grep -o '"processed":[0-9]*' "$STATE_FILE" | grep -o '[0-9]*')

            TOTAL_SUCCESS=$((TOTAL_SUCCESS + ${SUCCESS:-0}))
            TOTAL_GRASPFAILURE=$((TOTAL_GRASPFAILURE + ${GRASPFAILURE:-0}))
            TOTAL_PLANFAILURE=$((TOTAL_PLANFAILURE + ${PLANFAILURE:-0}))
            TOTAL_PROCESSED=$((TOTAL_PROCESSED + ${PROCESSED:-0}))
        fi
    done

    # Count skipped (if --skip-existing was used)
    local TOTAL_SKIPPED=0
    if $SKIP_EXISTING; then
        for DIR in $FOLDER*/; do
            if [ -d "$DIR" ] && \
               [ -f "output/$DIR/out_good.dmd.yaml" ] && \
               [ -f "output/$DIR/out_bad.dmd.yaml" ]; then
                # Check if it was already skipped before we started
                local WAS_PROCESSED=false
                for d in "${DIRS[@]}"; do
                    if [ "$d" = "$DIR" ]; then
                        WAS_PROCESSED=true
                        break
                    fi
                done
                if ! $WAS_PROCESSED; then
                    ((TOTAL_SKIPPED++))
                fi
            fi
        done
    fi

    echo ""
    echo "================ Summary ================"
    echo "SUCCESS:       $TOTAL_SUCCESS"
    echo "GRASPFAILURE:  $TOTAL_GRASPFAILURE"
    echo "PLANFAILURE:   $TOTAL_PLANFAILURE"
    echo "SKIPPED:       $TOTAL_SKIPPED"
    echo "PROCESSED:     $TOTAL_PROCESSED / $TOTAL_DIRS"
    echo "=========================================="
    echo "Logs: $LOG_DIR"
}

# Distribute directories across workers
# Create arrays for each worker
for i in $(seq 0 $((NUM_WORKERS - 1))); do
    eval "WORKER_${i}_DIRS=()"
done

# Round-robin assignment
for i in "${!DIRS[@]}"; do
    WORKER_IDX=$((i % NUM_WORKERS))
    eval "WORKER_${WORKER_IDX}_DIRS+=(\"\${DIRS[$i]}\")"
done

# Print distribution
echo "Work distribution:"
for i in $(seq 0 $((NUM_WORKERS - 1))); do
    eval "COUNT=\${#WORKER_${i}_DIRS[@]}"
    echo "  Worker $i: $COUNT experiment(s)"
done
echo ""

# Launch workers in background
for i in $(seq 0 $((NUM_WORKERS - 1))); do
    eval "WORKER_DIRS=(\"\${WORKER_${i}_DIRS[@]}\")"
    if [ ${#WORKER_DIRS[@]} -gt 0 ]; then
        run_worker "$i" "${WORKER_DIRS[@]}" &
        WORKER_PIDS+=($!)
        echo "Started worker $i (PID ${WORKER_PIDS[-1]})"
    fi
done

echo ""
echo "All workers started. Waiting for completion..."
echo "(Press Ctrl+C to stop gracefully)"
echo ""

# Wait for all workers
for pid in "${WORKER_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
done

# Aggregate and print results
aggregate_results

echo ""
echo "Done!"
