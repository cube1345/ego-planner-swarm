#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

LABEL="run"
OUT_DIR="artifacts/fusion_eval"
DURATION=90
STARTUP_WAIT=12
WINDOW_SIZE=80
REPORT_EVERY=20
USE_MOCKAMAP="True"
MIN_PROBABILITY="0.30"
MIN_HITS="1"
ADAPTIVE_ENABLE="True"
ADAPTIVE_MIN="0.20"
ADAPTIVE_MAX="0.35"
ADAPTIVE_STEP="0.02"
NEAR_FIELD_RADIUS="4.0"
LIDAR_GROWTH="5.0"
ADAPTIVE_MIN_HITS_ENABLE="True"
ADAPTIVE_MIN_HITS_MIN="1"
ADAPTIVE_MIN_HITS_MAX="3"
ADAPTIVE_EVAL_RANGE="10.0"
ADAPTIVE_MIN_GT_VOXELS="40"
ADAPTIVE_SCORE_ALPHA="0.35"
INIT_X="-15.0"
INIT_Y="0.0"
INIT_Z="0.1"
POINT_NUM="1"
POINT0_X="15.0"
POINT0_Y="0.0"
POINT0_Z="1.0"

usage() {
  cat <<'EOF'
Usage: bash tools/compare_fusion.sh [options]

  --label NAME
  --out-dir DIR
  --duration SEC
  --startup-wait SEC
  --window-size N
  --report-every N
  --use-mockamap True|False
  --min-probability FLOAT
  --min-hits INT
  --adaptive-enable True|False
  --adaptive-min FLOAT
  --adaptive-max FLOAT
  --adaptive-step FLOAT
  --near-field-radius FLOAT
  --lidar-growth FLOAT
  --adaptive-min-hits-enable True|False
  --adaptive-min-hits-min INT
  --adaptive-min-hits-max INT
  --adaptive-eval-range FLOAT
  --adaptive-min-gt-voxels INT
  --adaptive-score-alpha FLOAT
  --init-x FLOAT
  --init-y FLOAT
  --init-z FLOAT
  --point-num INT
  --point0-x FLOAT
  --point0-y FLOAT
  --point0-z FLOAT
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --label) LABEL="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --startup-wait) STARTUP_WAIT="$2"; shift 2 ;;
    --window-size) WINDOW_SIZE="$2"; shift 2 ;;
    --report-every) REPORT_EVERY="$2"; shift 2 ;;
    --use-mockamap) USE_MOCKAMAP="$2"; shift 2 ;;
    --min-probability) MIN_PROBABILITY="$2"; shift 2 ;;
    --min-hits) MIN_HITS="$2"; shift 2 ;;
    --adaptive-enable) ADAPTIVE_ENABLE="$2"; shift 2 ;;
    --adaptive-min) ADAPTIVE_MIN="$2"; shift 2 ;;
    --adaptive-max) ADAPTIVE_MAX="$2"; shift 2 ;;
    --adaptive-step) ADAPTIVE_STEP="$2"; shift 2 ;;
    --near-field-radius) NEAR_FIELD_RADIUS="$2"; shift 2 ;;
    --lidar-growth) LIDAR_GROWTH="$2"; shift 2 ;;
    --adaptive-min-hits-enable) ADAPTIVE_MIN_HITS_ENABLE="$2"; shift 2 ;;
    --adaptive-min-hits-min) ADAPTIVE_MIN_HITS_MIN="$2"; shift 2 ;;
    --adaptive-min-hits-max) ADAPTIVE_MIN_HITS_MAX="$2"; shift 2 ;;
    --adaptive-eval-range) ADAPTIVE_EVAL_RANGE="$2"; shift 2 ;;
    --adaptive-min-gt-voxels) ADAPTIVE_MIN_GT_VOXELS="$2"; shift 2 ;;
    --adaptive-score-alpha) ADAPTIVE_SCORE_ALPHA="$2"; shift 2 ;;
    --init-x) INIT_X="$2"; shift 2 ;;
    --init-y) INIT_Y="$2"; shift 2 ;;
    --init-z) INIT_Z="$2"; shift 2 ;;
    --point-num) POINT_NUM="$2"; shift 2 ;;
    --point0-x) POINT0_X="$2"; shift 2 ;;
    --point0-y) POINT0_Y="$2"; shift 2 ;;
    --point0-z) POINT0_Z="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

mkdir -p "$OUT_DIR"

CSV_PATH="$OUT_DIR/${LABEL}.csv"
SUMMARY_PATH="$OUT_DIR/${LABEL}.summary.json"
LAUNCH_LOG="$OUT_DIR/${LABEL}.launch.log"
REPORT_LOG="$OUT_DIR/${LABEL}.report.log"
RUN_LOG_DIR="$OUT_DIR/${LABEL}_ros_logs"
SIM_STATS_DIR="$OUT_DIR/${LABEL}_sim_stats"
SIM_STATS_LOG="$OUT_DIR/${LABEL}.sim_stats.log"

cleanup_existing_ros_processes() {
  pkill -TERM -f 'single_run_in_sim_fusion.launch.py|single_run_in_sim.launch.py|ego_planner_node|traj_server|poscmd_2_odom|odom_visualization|pcl_render_node|simulated_lidar_cloud.py|ros2_lidar_depth_fusion_node.py|mockamap_node|random_forest' 2>/dev/null || true
  sleep 2
  pkill -KILL -f 'single_run_in_sim_fusion.launch.py|single_run_in_sim.launch.py|ego_planner_node|traj_server|poscmd_2_odom|odom_visualization|pcl_render_node|simulated_lidar_cloud.py|ros2_lidar_depth_fusion_node.py|mockamap_node|random_forest' 2>/dev/null || true
}

stop_pid() {
  local pid="$1"
  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    return 0
  fi

  kill -INT "$pid" 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 1
  done

  kill -TERM "$pid" 2>/dev/null || true
  for _ in 1 2 3; do
    if ! kill -0 "$pid" 2>/dev/null; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 1
  done

  kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  set +e
  if [[ -n "${SIM_STATS_PID:-}" ]]; then
    stop_pid "$SIM_STATS_PID"
  fi
  if [[ -n "${REPORT_PID:-}" ]]; then
    stop_pid "$REPORT_PID"
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    stop_pid "$LAUNCH_PID"
  fi
  pkill -TERM -f 'fusion_benefit_report.py' 2>/dev/null || true
  sleep 1
  pkill -KILL -f 'fusion_benefit_report.py' 2>/dev/null || true
  cleanup_existing_ros_processes
}

trap cleanup EXIT

set +u
source install/setup.bash
set -u

cleanup_existing_ros_processes

rm -f "$CSV_PATH" "$SUMMARY_PATH" "$LAUNCH_LOG" "$REPORT_LOG" "$SIM_STATS_LOG"
rm -rf "$SIM_STATS_DIR"
rm -rf "$RUN_LOG_DIR"
mkdir -p "$RUN_LOG_DIR"
export ROS_LOG_DIR="$RUN_LOG_DIR"

echo "[compare_fusion] launch label=$LABEL out_dir=$OUT_DIR"
echo "[compare_fusion] ros_log_dir=$RUN_LOG_DIR"
ros2 launch ego_planner single_run_in_sim_fusion.launch.py \
  use_fusion:=True \
  use_mockamap:="$USE_MOCKAMAP" \
  init_x:="$INIT_X" \
  init_y:="$INIT_Y" \
  init_z:="$INIT_Z" \
  point_num:="$POINT_NUM" \
  point0_x:="$POINT0_X" \
  point0_y:="$POINT0_Y" \
  point0_z:="$POINT0_Z" \
  near_field_radius:="$NEAR_FIELD_RADIUS" \
  lidar_growth:="$LIDAR_GROWTH" \
  min_probability:="$MIN_PROBABILITY" \
  min_hits:="$MIN_HITS" \
  adaptive_min_probability_enable:="$ADAPTIVE_ENABLE" \
  adaptive_min_probability_min:="$ADAPTIVE_MIN" \
  adaptive_min_probability_max:="$ADAPTIVE_MAX" \
  adaptive_min_probability_step:="$ADAPTIVE_STEP" \
  adaptive_min_hits_enable:="$ADAPTIVE_MIN_HITS_ENABLE" \
  adaptive_min_hits_min:="$ADAPTIVE_MIN_HITS_MIN" \
  adaptive_min_hits_max:="$ADAPTIVE_MIN_HITS_MAX" \
  adaptive_eval_range:="$ADAPTIVE_EVAL_RANGE" \
  adaptive_min_gt_voxels:="$ADAPTIVE_MIN_GT_VOXELS" \
  adaptive_score_alpha:="$ADAPTIVE_SCORE_ALPHA" \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

sleep "$STARTUP_WAIT"

/usr/bin/python3 src/planner/plan_manage/scripts/fusion_benefit_report.py \
  --ros-args \
  -p report_every:="$REPORT_EVERY" \
  -p window_size:="$WINDOW_SIZE" \
  -p csv_path:="$CSV_PATH" \
  >"$REPORT_LOG" 2>&1 &
REPORT_PID=$!

/usr/bin/python3 tools/sim_flight_stats_report.py \
  --duration-sec "$DURATION" \
  --output-dir "$SIM_STATS_DIR" \
  --global-cloud-topic /map_generator/global_cloud \
  --occupancy-topic /drone_0_grid/grid_map/occupancy_inflate \
  --odom-topic /drone_0_visual_slam/odom \
  --launch-log-path "$LAUNCH_LOG" \
  --goal-x "$POINT0_X" \
  --goal-y "$POINT0_Y" \
  --goal-z "$POINT0_Z" \
  --goal-exit-margin 1.0 \
  >"$SIM_STATS_LOG" 2>&1 &
SIM_STATS_PID=$!

sleep "$DURATION"

stop_pid "$LAUNCH_PID"
unset LAUNCH_PID

stop_pid "$SIM_STATS_PID"
unset SIM_STATS_PID

stop_pid "$REPORT_PID"
unset REPORT_PID

if [[ ! -s "$CSV_PATH" ]]; then
  echo "[compare_fusion] csv not generated: $CSV_PATH" >&2
  exit 2
fi

/usr/bin/python3 - "$CSV_PATH" "$SUMMARY_PATH" <<'PY'
import csv
import json
import math
import sys
from pathlib import Path

csv_path = Path(sys.argv[1])
summary_path = Path(sys.argv[2])
with csv_path.open("r", newline="", encoding="utf-8") as handle:
    reader = csv.DictReader(handle)
    rows = list(reader)

if not rows:
    raise SystemExit("empty csv")

keys = [
    "fusion_recall",
    "fusion_f1",
    "fusion_recall_gain_vs_best_single",
    "fusion_f1_gain_vs_best_single",
    "fusion_voxels",
    "depth_recall",
    "lidar_recall",
    "depth_f1",
    "lidar_f1",
]

def mean_of(name: str) -> float:
    values = []
    for row in rows:
        try:
            value = float(row[name])
        except Exception:
            continue
        if math.isfinite(value):
            values.append(value)
    return sum(values) / len(values) if values else float("nan")

def positive_ratio(name: str) -> float:
    values = []
    for row in rows:
        try:
            value = float(row[name])
        except Exception:
            continue
        if math.isfinite(value):
            values.append(value)
    return (sum(value > 0.0 for value in values) / len(values)) if values else float("nan")

summary = {"rows": len(rows)}
for key in keys:
    summary[key] = mean_of(key)
summary["positive_ratio_recall_gain"] = positive_ratio("fusion_recall_gain_vs_best_single")
summary["positive_ratio_f1_gain"] = positive_ratio("fusion_f1_gain_vs_best_single")

summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
print(json.dumps(summary, indent=2, ensure_ascii=False))
PY
