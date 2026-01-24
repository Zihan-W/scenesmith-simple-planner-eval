#!/usr/bin/env bash
set -euo pipefail

# -------------------------
# Config / inputs
# -------------------------
export SCENE="${1:-}"
if [[ -z "$SCENE" ]]; then
  echo "Usage: $0 <SCENE_DIR>" >&2
  exit 64
fi

# WORKDIR allows parallel workers to write outputs to isolated directories
WORKDIR="${WORKDIR:-.}"

export TIMELIMIT=180m
export GRASPTIMELIMIT=30m
export GRASPNUMRUNS=10
export GRACEPERIOD=5s

echo "Running scene $SCENE"

# -------------------------
# Cleanup helpers + traps
# -------------------------
TO_PIDS=()
PY_PIDS=()

kill_children() {
  # Recursively TERM a pid and its descendants.
  # Requires pgrep (Ubuntu has it via procps).
  local pid="$1"
  local kids
  kids=$(pgrep -P "$pid" 2>/dev/null || true)
  for k in $kids; do
    kill_children "$k"
  done
  kill -TERM "$pid" 2>/dev/null || true
}

kill_all_tracked() {
  trap - TERM INT
  echo ""
  echo "Caught signal; killing tracked timeout/python processes..." >&2

  # Kill python first (it may have escaped process groups)
  for p in "${PY_PIDS[@]:-}"; do
    kill_children "$p" || true
  done

  # Then kill timeout wrappers
  for t in "${TO_PIDS[@]:-}"; do
    kill_children "$t" || true
  done

  # Escalate
  sleep 0.2
  for p in "${PY_PIDS[@]:-}"; do kill -KILL "$p" 2>/dev/null || true; done
  for t in "${TO_PIDS[@]:-}"; do kill -KILL "$t" 2>/dev/null || true; done

  exit 130
}

trap kill_all_tracked TERM INT

run_timed_python() {
  # Usage: run_timed_python <timelimit> <cmd...>
  local timelimit="$1"; shift

  timeout -k "$GRACEPERIOD" "$timelimit" "$@" &
  local to_pid=$!
  TO_PIDS+=("$to_pid")

  # Give timeout a moment to spawn its child, then track any direct children.
  # (Often exactly one python process; if multiple, track them all.)
  sleep 0.05
  local kids
  kids=$(pgrep -P "$to_pid" 2>/dev/null || true)
  for k in $kids; do
    PY_PIDS+=("$k")
  done

  wait "$to_pid"
  return $?
}

# -------------------------
# Start fresh outputs
# -------------------------
rm -f "$WORKDIR/robot_task.dmd.yaml"
rm -f "$WORKDIR/robot_waypoints.json"
rm -f "$WORKDIR/robot_plan_good.json"
rm -f "$WORKDIR/robot_plan_bad.json"
rm -f "$WORKDIR/simulation_good.html"
rm -f "$WORKDIR/simulation_bad.html"
rm -f "$WORKDIR/out_good.dmd.yaml"
rm -f "$WORKDIR/out_bad.dmd.yaml"

# -------------------------
# Build task directives
# -------------------------
run_timed_python "$TIMELIMIT" \
  python3 scripts/add_robot_to_directives.py \
    "$SCENE/combined_house/house_furniture_welded.dmd.yaml" \
    "$SCENE/combined_house/robot_commands.json" \
    "$WORKDIR/robot_task.dmd.yaml"

# -------------------------
# Compute grasp/placement waypoints with retries
# -------------------------
OUT_WAYPOINTS="$WORKDIR/robot_waypoints.json"

for ((i=1; i<=GRASPNUMRUNS; i++)); do
  if [[ -f "$OUT_WAYPOINTS" ]]; then
    break
  fi

  echo "Grasp attempt $i/$GRASPNUMRUNS (timeout: $GRASPTIMELIMIT)..."

  run_timed_python "$GRASPTIMELIMIT" \
    python3 scripts/compute_grasp_config_noninteractive.py \
      "$SCENE/combined_house/robot_commands.json" \
      "$WORKDIR/robot_task.dmd.yaml" \
      --package-xml "$SCENE/package.xml" \
      --package-xml models/iiwa/package.xml \
      --out-waypoints "$OUT_WAYPOINTS" \
    || true
done

if [[ ! -f "$OUT_WAYPOINTS" ]]; then
  echo "Failed to compute grasp or place configurations in the allotted time for scene $SCENE."
  exit 1
fi

# -------------------------
# Plan motion
# -------------------------
run_timed_python "$TIMELIMIT" \
  python3 scripts/plan_robot_waypoints_rrt_noninteractive.py \
    "$SCENE/combined_house/robot_commands.json" \
    "$WORKDIR/robot_task.dmd.yaml" \
    "$WORKDIR/robot_waypoints.json" \
    --package-xml "$SCENE/package.xml" \
    --package-xml models/iiwa/package.xml \
    --out-traj "$WORKDIR/robot_plan_good.json" \
    --shortcut-tries 100 \
    --gripper-clearance \
  || true

if [[ ! -f "$WORKDIR/robot_plan_good.json" ]]; then
  echo "Failed to compute good robot plan in the allotted time for scene $SCENE."
  exit 2
fi

run_timed_python "$TIMELIMIT" \
  python3 scripts/plan_robot_waypoints_rrt_noninteractive.py \
    "$SCENE/combined_house/robot_commands.json" \
    "$WORKDIR/robot_task.dmd.yaml" \
    "$WORKDIR/robot_waypoints.json" \
    --package-xml "$SCENE/package.xml" \
    --package-xml models/iiwa/package.xml \
    --out-traj "$WORKDIR/robot_plan_bad.json" \
    --shortcut-tries 100 \
  || true

if [[ ! -f "$WORKDIR/robot_plan_bad.json" ]]; then
  echo "Failed to compute bad robot plan in the allotted time for scene $SCENE."
  exit 2
fi

# -------------------------
# Simulate
# -------------------------
python3 scripts/simulate_noninteractive.py \
  "$WORKDIR/robot_task.dmd.yaml" \
  "$WORKDIR/robot_plan_good.json" \
  --friction-mult 10 \
  --package-xml "$SCENE/package.xml" \
  --package-xml models/iiwa/package.xml \
  --ee-vel 1 \
  --ee-accel 1 \
  --write-updated-scenario "$WORKDIR/out_good.dmd.yaml" \
  --record-html "$WORKDIR/simulation_good.html"

python3 scripts/simulate_noninteractive.py \
  "$WORKDIR/robot_task.dmd.yaml" \
  "$WORKDIR/robot_plan_bad.json" \
  --friction-mult 10 \
  --package-xml "$SCENE/package.xml" \
  --package-xml models/iiwa/package.xml \
  --write-updated-scenario "$WORKDIR/out_bad.dmd.yaml" \
  --record-html "$WORKDIR/simulation_bad.html"

echo "Successfully finished running scene $SCENE"
