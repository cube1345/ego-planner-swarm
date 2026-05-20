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
ADAPTIVE_NEAR_FIELD_RADIUS_ENABLE="False"
ADAPTIVE_NEAR_FIELD_RADIUS_MIN="3.0"
ADAPTIVE_NEAR_FIELD_RADIUS_MAX="5.0"
ADAPTIVE_NEAR_FIELD_RADIUS_STEP="1.0"
LIDAR_GROWTH="5.0"
DEPTH_DECAY="4.5"
ADAPTIVE_DEPTH_DECAY_ENABLE="False"
ADAPTIVE_DEPTH_DECAY_MIN="3.5"
ADAPTIVE_DEPTH_DECAY_MAX="5.5"
ADAPTIVE_DEPTH_DECAY_STEP="1.0"
FUSION_Z_MAX="3.5"
ADAPTIVE_Z_MAX_ENABLE="False"
ADAPTIVE_Z_MAX_MIN="3.0"
ADAPTIVE_Z_MAX_MAX="3.5"
ADAPTIVE_Z_MAX_STEP="0.25"
ADAPTIVE_MIN_HITS_ENABLE="True"
ADAPTIVE_MIN_HITS_MIN="1"
ADAPTIVE_MIN_HITS_MAX="3"
ADAPTIVE_RETENTION_ENABLE="False"
ADAPTIVE_RETENTION_TARGET="0.30"
ADAPTIVE_RETENTION_BAND="0.05"
ADAPTIVE_LIDAR_GROWTH_ENABLE="False"
ADAPTIVE_LIDAR_GROWTH_MIN="3.8"
ADAPTIVE_LIDAR_GROWTH_MAX="4.2"
ADAPTIVE_LIDAR_GROWTH_STEP="0.2"
ADAPTIVE_DUAL_BONUS_ENABLE="True"
ADAPTIVE_DUAL_BONUS_MIN="0.0"
ADAPTIVE_DUAL_BONUS_MAX="0.2"
ADAPTIVE_DUAL_BONUS_STEP="0.1"
ADAPTIVE_EVAL_RANGE="10.0"
ADAPTIVE_MIN_GT_VOXELS="40"
ADAPTIVE_SCORE_ALPHA="0.35"
CLOSED_LOOP_FEEDBACK_ENABLE="True"
CLOSED_LOOP_FEEDBACK_WEIGHT="0.05"
CLOSED_LOOP_OPTIMIZER_ENABLE="False"
CLOSED_LOOP_LOCAL_SCORE_WEIGHT="1.0"
CLOSED_LOOP_CANDIDATE_SCORE_ALPHA="0.30"
CLOSED_LOOP_ACTION_DELAY_SEC="3.0"
CLOSED_LOOP_ACTION_HISTORY_SEC="20.0"
CLOSED_LOOP_WINDOW_SEC="8.0"
CLOSED_LOOP_DS_FEEDBACK_ENABLE="True"
CLOSED_LOOP_DS_UNKNOWN_WEIGHT="0.002"
CLOSED_LOOP_DS_CONFLICT_WEIGHT="0.004"
CLOSED_LOOP_DS_UNKNOWN_REF="0.50"
CLOSED_LOOP_DS_CONFLICT_REF="0.08"
DS_EVIDENCE_ENABLE="True"
DS_UNKNOWN_FLOOR="0.10"
DS_FREE_SCALE="0.35"
ADAPTIVE_DS_SCORE_ENABLE="True"
ADAPTIVE_DS_UNKNOWN_WEIGHT="0.002"
ADAPTIVE_DS_CONFLICT_WEIGHT="0.004"
ADAPTIVE_DS_UNKNOWN_REF="0.50"
ADAPTIVE_DS_CONFLICT_REF="0.08"
DYNAMIC_OBSTACLES_ENABLE="False"
DYNAMIC_OBSTACLES_SPECS="0.0,1.8,0.75,0.30,1.30,0.0,1.2,9.0,0.0;5.0,-1.6,0.75,0.28,1.20,0.0,1.0,8.0,1.57;-5.5,2.2,0.75,0.26,1.10,0.8,0.7,10.0,3.14;9.0,-2.4,0.75,0.24,1.10,-0.7,0.9,11.0,0.78"
DYNAMIC_OBSTACLES_RATE="15.0"
DYNAMIC_OBSTACLES_SPACING="0.12"
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
  --adaptive-near-field-radius-enable True|False
  --adaptive-near-field-radius-min FLOAT
  --adaptive-near-field-radius-max FLOAT
  --adaptive-near-field-radius-step FLOAT
  --lidar-growth FLOAT
  --depth-decay FLOAT
  --adaptive-depth-decay-enable True|False
  --adaptive-depth-decay-min FLOAT
  --adaptive-depth-decay-max FLOAT
  --adaptive-depth-decay-step FLOAT
  --fusion-z-max FLOAT
  --adaptive-z-max-enable True|False
  --adaptive-z-max-min FLOAT
  --adaptive-z-max-max FLOAT
  --adaptive-z-max-step FLOAT
  --adaptive-min-hits-enable True|False
  --adaptive-min-hits-min INT
  --adaptive-min-hits-max INT
  --adaptive-retention-enable True|False
  --adaptive-retention-target FLOAT
  --adaptive-retention-band FLOAT
  --adaptive-lidar-growth-enable True|False
  --adaptive-lidar-growth-min FLOAT
  --adaptive-lidar-growth-max FLOAT
  --adaptive-lidar-growth-step FLOAT
  --adaptive-dual-bonus-enable True|False
  --adaptive-dual-bonus-min FLOAT
  --adaptive-dual-bonus-max FLOAT
  --adaptive-dual-bonus-step FLOAT
  --adaptive-eval-range FLOAT
  --adaptive-min-gt-voxels INT
  --adaptive-score-alpha FLOAT
  --closed-loop-feedback-enable True|False
  --closed-loop-feedback-weight FLOAT
  --closed-loop-optimizer-enable True|False
  --closed-loop-local-score-weight FLOAT
  --closed-loop-candidate-score-alpha FLOAT
  --closed-loop-action-delay-sec FLOAT
  --closed-loop-action-history-sec FLOAT
  --closed-loop-window-sec FLOAT
  --closed-loop-ds-feedback-enable True|False
  --closed-loop-ds-unknown-weight FLOAT
  --closed-loop-ds-conflict-weight FLOAT
  --closed-loop-ds-unknown-ref FLOAT
  --closed-loop-ds-conflict-ref FLOAT
  --ds-evidence-enable True|False
  --ds-unknown-floor FLOAT
  --ds-free-scale FLOAT
  --adaptive-ds-score-enable True|False
  --adaptive-ds-unknown-weight FLOAT
  --adaptive-ds-conflict-weight FLOAT
  --adaptive-ds-unknown-ref FLOAT
  --adaptive-ds-conflict-ref FLOAT
  --dynamic-obstacles-enable True|False
  --dynamic-obstacles-specs SPEC
  --dynamic-obstacles-rate FLOAT
  --dynamic-obstacles-spacing FLOAT
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
    --adaptive-near-field-radius-enable) ADAPTIVE_NEAR_FIELD_RADIUS_ENABLE="$2"; shift 2 ;;
    --adaptive-near-field-radius-min) ADAPTIVE_NEAR_FIELD_RADIUS_MIN="$2"; shift 2 ;;
    --adaptive-near-field-radius-max) ADAPTIVE_NEAR_FIELD_RADIUS_MAX="$2"; shift 2 ;;
    --adaptive-near-field-radius-step) ADAPTIVE_NEAR_FIELD_RADIUS_STEP="$2"; shift 2 ;;
    --lidar-growth) LIDAR_GROWTH="$2"; shift 2 ;;
    --depth-decay) DEPTH_DECAY="$2"; shift 2 ;;
    --adaptive-depth-decay-enable) ADAPTIVE_DEPTH_DECAY_ENABLE="$2"; shift 2 ;;
    --adaptive-depth-decay-min) ADAPTIVE_DEPTH_DECAY_MIN="$2"; shift 2 ;;
    --adaptive-depth-decay-max) ADAPTIVE_DEPTH_DECAY_MAX="$2"; shift 2 ;;
    --adaptive-depth-decay-step) ADAPTIVE_DEPTH_DECAY_STEP="$2"; shift 2 ;;
    --fusion-z-max) FUSION_Z_MAX="$2"; shift 2 ;;
    --adaptive-z-max-enable) ADAPTIVE_Z_MAX_ENABLE="$2"; shift 2 ;;
    --adaptive-z-max-min) ADAPTIVE_Z_MAX_MIN="$2"; shift 2 ;;
    --adaptive-z-max-max) ADAPTIVE_Z_MAX_MAX="$2"; shift 2 ;;
    --adaptive-z-max-step) ADAPTIVE_Z_MAX_STEP="$2"; shift 2 ;;
    --adaptive-min-hits-enable) ADAPTIVE_MIN_HITS_ENABLE="$2"; shift 2 ;;
    --adaptive-min-hits-min) ADAPTIVE_MIN_HITS_MIN="$2"; shift 2 ;;
    --adaptive-min-hits-max) ADAPTIVE_MIN_HITS_MAX="$2"; shift 2 ;;
    --adaptive-retention-enable) ADAPTIVE_RETENTION_ENABLE="$2"; shift 2 ;;
    --adaptive-retention-target) ADAPTIVE_RETENTION_TARGET="$2"; shift 2 ;;
    --adaptive-retention-band) ADAPTIVE_RETENTION_BAND="$2"; shift 2 ;;
    --adaptive-lidar-growth-enable) ADAPTIVE_LIDAR_GROWTH_ENABLE="$2"; shift 2 ;;
    --adaptive-lidar-growth-min) ADAPTIVE_LIDAR_GROWTH_MIN="$2"; shift 2 ;;
    --adaptive-lidar-growth-max) ADAPTIVE_LIDAR_GROWTH_MAX="$2"; shift 2 ;;
    --adaptive-lidar-growth-step) ADAPTIVE_LIDAR_GROWTH_STEP="$2"; shift 2 ;;
    --adaptive-dual-bonus-enable) ADAPTIVE_DUAL_BONUS_ENABLE="$2"; shift 2 ;;
    --adaptive-dual-bonus-min) ADAPTIVE_DUAL_BONUS_MIN="$2"; shift 2 ;;
    --adaptive-dual-bonus-max) ADAPTIVE_DUAL_BONUS_MAX="$2"; shift 2 ;;
    --adaptive-dual-bonus-step) ADAPTIVE_DUAL_BONUS_STEP="$2"; shift 2 ;;
    --adaptive-eval-range) ADAPTIVE_EVAL_RANGE="$2"; shift 2 ;;
    --adaptive-min-gt-voxels) ADAPTIVE_MIN_GT_VOXELS="$2"; shift 2 ;;
    --adaptive-score-alpha) ADAPTIVE_SCORE_ALPHA="$2"; shift 2 ;;
    --closed-loop-feedback-enable) CLOSED_LOOP_FEEDBACK_ENABLE="$2"; shift 2 ;;
    --closed-loop-feedback-weight) CLOSED_LOOP_FEEDBACK_WEIGHT="$2"; shift 2 ;;
    --closed-loop-optimizer-enable) CLOSED_LOOP_OPTIMIZER_ENABLE="$2"; shift 2 ;;
    --closed-loop-local-score-weight) CLOSED_LOOP_LOCAL_SCORE_WEIGHT="$2"; shift 2 ;;
    --closed-loop-candidate-score-alpha) CLOSED_LOOP_CANDIDATE_SCORE_ALPHA="$2"; shift 2 ;;
    --closed-loop-action-delay-sec) CLOSED_LOOP_ACTION_DELAY_SEC="$2"; shift 2 ;;
    --closed-loop-action-history-sec) CLOSED_LOOP_ACTION_HISTORY_SEC="$2"; shift 2 ;;
    --closed-loop-window-sec) CLOSED_LOOP_WINDOW_SEC="$2"; shift 2 ;;
    --closed-loop-ds-feedback-enable) CLOSED_LOOP_DS_FEEDBACK_ENABLE="$2"; shift 2 ;;
    --closed-loop-ds-unknown-weight) CLOSED_LOOP_DS_UNKNOWN_WEIGHT="$2"; shift 2 ;;
    --closed-loop-ds-conflict-weight) CLOSED_LOOP_DS_CONFLICT_WEIGHT="$2"; shift 2 ;;
    --closed-loop-ds-unknown-ref) CLOSED_LOOP_DS_UNKNOWN_REF="$2"; shift 2 ;;
    --closed-loop-ds-conflict-ref) CLOSED_LOOP_DS_CONFLICT_REF="$2"; shift 2 ;;
    --ds-evidence-enable) DS_EVIDENCE_ENABLE="$2"; shift 2 ;;
    --ds-unknown-floor) DS_UNKNOWN_FLOOR="$2"; shift 2 ;;
    --ds-free-scale) DS_FREE_SCALE="$2"; shift 2 ;;
    --adaptive-ds-score-enable) ADAPTIVE_DS_SCORE_ENABLE="$2"; shift 2 ;;
    --adaptive-ds-unknown-weight) ADAPTIVE_DS_UNKNOWN_WEIGHT="$2"; shift 2 ;;
    --adaptive-ds-conflict-weight) ADAPTIVE_DS_CONFLICT_WEIGHT="$2"; shift 2 ;;
    --adaptive-ds-unknown-ref) ADAPTIVE_DS_UNKNOWN_REF="$2"; shift 2 ;;
    --adaptive-ds-conflict-ref) ADAPTIVE_DS_CONFLICT_REF="$2"; shift 2 ;;
    --dynamic-obstacles-enable) DYNAMIC_OBSTACLES_ENABLE="$2"; shift 2 ;;
    --dynamic-obstacles-specs) DYNAMIC_OBSTACLES_SPECS="$2"; shift 2 ;;
    --dynamic-obstacles-rate) DYNAMIC_OBSTACLES_RATE="$2"; shift 2 ;;
    --dynamic-obstacles-spacing) DYNAMIC_OBSTACLES_SPACING="$2"; shift 2 ;;
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
CLOSED_LOOP_PATH="$OUT_DIR/${LABEL}.closed_loop.json"
LAUNCH_LOG="$OUT_DIR/${LABEL}.launch.log"
REPORT_LOG="$OUT_DIR/${LABEL}.report.log"
RUN_LOG_DIR="$OUT_DIR/${LABEL}_ros_logs"
SIM_STATS_DIR="$OUT_DIR/${LABEL}_sim_stats"
SIM_STATS_LOG="$OUT_DIR/${LABEL}.sim_stats.log"

cleanup_existing_ros_processes() {
  pkill -TERM -f 'single_run_in_sim_fusion.launch.py|single_run_in_sim.launch.py|ego_planner_node|traj_server|poscmd_2_odom|odom_visualization|pcl_render_node|simulated_lidar_cloud.py|dynamic_obstacle_cloud.py|ros2_lidar_depth_fusion_node.py|mockamap_node|random_forest' 2>/dev/null || true
  sleep 2
  pkill -KILL -f 'single_run_in_sim_fusion.launch.py|single_run_in_sim.launch.py|ego_planner_node|traj_server|poscmd_2_odom|odom_visualization|pcl_render_node|simulated_lidar_cloud.py|dynamic_obstacle_cloud.py|ros2_lidar_depth_fusion_node.py|mockamap_node|random_forest' 2>/dev/null || true
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

rm -f "$CSV_PATH" "$SUMMARY_PATH" "$CLOSED_LOOP_PATH" "$LAUNCH_LOG" "$REPORT_LOG" "$SIM_STATS_LOG"
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
  adaptive_near_field_radius_enable:="$ADAPTIVE_NEAR_FIELD_RADIUS_ENABLE" \
  adaptive_near_field_radius_min:="$ADAPTIVE_NEAR_FIELD_RADIUS_MIN" \
  adaptive_near_field_radius_max:="$ADAPTIVE_NEAR_FIELD_RADIUS_MAX" \
  adaptive_near_field_radius_step:="$ADAPTIVE_NEAR_FIELD_RADIUS_STEP" \
  lidar_growth:="$LIDAR_GROWTH" \
  depth_decay:="$DEPTH_DECAY" \
  adaptive_depth_decay_enable:="$ADAPTIVE_DEPTH_DECAY_ENABLE" \
  adaptive_depth_decay_min:="$ADAPTIVE_DEPTH_DECAY_MIN" \
  adaptive_depth_decay_max:="$ADAPTIVE_DEPTH_DECAY_MAX" \
  adaptive_depth_decay_step:="$ADAPTIVE_DEPTH_DECAY_STEP" \
  fusion_z_max:="$FUSION_Z_MAX" \
  adaptive_z_max_enable:="$ADAPTIVE_Z_MAX_ENABLE" \
  adaptive_z_max_min:="$ADAPTIVE_Z_MAX_MIN" \
  adaptive_z_max_max:="$ADAPTIVE_Z_MAX_MAX" \
  adaptive_z_max_step:="$ADAPTIVE_Z_MAX_STEP" \
  min_probability:="$MIN_PROBABILITY" \
  min_hits:="$MIN_HITS" \
  adaptive_min_probability_enable:="$ADAPTIVE_ENABLE" \
  adaptive_min_probability_min:="$ADAPTIVE_MIN" \
  adaptive_min_probability_max:="$ADAPTIVE_MAX" \
  adaptive_min_probability_step:="$ADAPTIVE_STEP" \
  adaptive_min_hits_enable:="$ADAPTIVE_MIN_HITS_ENABLE" \
  adaptive_min_hits_min:="$ADAPTIVE_MIN_HITS_MIN" \
  adaptive_min_hits_max:="$ADAPTIVE_MIN_HITS_MAX" \
  adaptive_retention_enable:="$ADAPTIVE_RETENTION_ENABLE" \
  adaptive_target_retention:="$ADAPTIVE_RETENTION_TARGET" \
  adaptive_retention_band:="$ADAPTIVE_RETENTION_BAND" \
  adaptive_lidar_growth_enable:="$ADAPTIVE_LIDAR_GROWTH_ENABLE" \
  adaptive_lidar_growth_min:="$ADAPTIVE_LIDAR_GROWTH_MIN" \
  adaptive_lidar_growth_max:="$ADAPTIVE_LIDAR_GROWTH_MAX" \
  adaptive_lidar_growth_step:="$ADAPTIVE_LIDAR_GROWTH_STEP" \
  adaptive_dual_bonus_enable:="$ADAPTIVE_DUAL_BONUS_ENABLE" \
  adaptive_dual_bonus_min:="$ADAPTIVE_DUAL_BONUS_MIN" \
  adaptive_dual_bonus_max:="$ADAPTIVE_DUAL_BONUS_MAX" \
  adaptive_dual_bonus_step:="$ADAPTIVE_DUAL_BONUS_STEP" \
  adaptive_eval_range:="$ADAPTIVE_EVAL_RANGE" \
  adaptive_min_gt_voxels:="$ADAPTIVE_MIN_GT_VOXELS" \
  adaptive_score_alpha:="$ADAPTIVE_SCORE_ALPHA" \
  closed_loop_feedback_enable:="$CLOSED_LOOP_FEEDBACK_ENABLE" \
  closed_loop_feedback_weight:="$CLOSED_LOOP_FEEDBACK_WEIGHT" \
  closed_loop_optimizer_enable:="$CLOSED_LOOP_OPTIMIZER_ENABLE" \
  closed_loop_local_score_weight:="$CLOSED_LOOP_LOCAL_SCORE_WEIGHT" \
  closed_loop_candidate_score_alpha:="$CLOSED_LOOP_CANDIDATE_SCORE_ALPHA" \
  closed_loop_action_delay_sec:="$CLOSED_LOOP_ACTION_DELAY_SEC" \
  closed_loop_action_history_sec:="$CLOSED_LOOP_ACTION_HISTORY_SEC" \
  closed_loop_window_sec:="$CLOSED_LOOP_WINDOW_SEC" \
  closed_loop_ds_feedback_enable:="$CLOSED_LOOP_DS_FEEDBACK_ENABLE" \
  closed_loop_ds_unknown_weight:="$CLOSED_LOOP_DS_UNKNOWN_WEIGHT" \
  closed_loop_ds_conflict_weight:="$CLOSED_LOOP_DS_CONFLICT_WEIGHT" \
  closed_loop_ds_unknown_ref:="$CLOSED_LOOP_DS_UNKNOWN_REF" \
  closed_loop_ds_conflict_ref:="$CLOSED_LOOP_DS_CONFLICT_REF" \
  ds_evidence_enable:="$DS_EVIDENCE_ENABLE" \
  ds_unknown_floor:="$DS_UNKNOWN_FLOOR" \
  ds_free_scale:="$DS_FREE_SCALE" \
  adaptive_ds_score_enable:="$ADAPTIVE_DS_SCORE_ENABLE" \
  adaptive_ds_unknown_weight:="$ADAPTIVE_DS_UNKNOWN_WEIGHT" \
  adaptive_ds_conflict_weight:="$ADAPTIVE_DS_CONFLICT_WEIGHT" \
  adaptive_ds_unknown_ref:="$ADAPTIVE_DS_UNKNOWN_REF" \
  adaptive_ds_conflict_ref:="$ADAPTIVE_DS_CONFLICT_REF" \
  dynamic_obstacles_enable:="$DYNAMIC_OBSTACLES_ENABLE" \
  dynamic_obstacles_specs:="$DYNAMIC_OBSTACLES_SPECS" \
  dynamic_obstacles_rate:="$DYNAMIC_OBSTACLES_RATE" \
  dynamic_obstacles_spacing:="$DYNAMIC_OBSTACLES_SPACING" \
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
  --ds-metrics-topic /drone_0_fusion/ds_metrics \
  --dynamic-obstacle-topic /drone_0_dynamic_obstacles/cloud \
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

SIM_SUMMARY_PATH="$SIM_STATS_DIR/sim_stats_summary.json"
if [[ -s "$SIM_SUMMARY_PATH" ]]; then
  /usr/bin/python3 tools/closed_loop_score.py \
    --fusion-summary "$SUMMARY_PATH" \
    --sim-summary "$SIM_SUMMARY_PATH" \
    --output "$CLOSED_LOOP_PATH" \
    --update-fusion-summary
else
  echo "[compare_fusion] sim stats summary not generated: $SIM_SUMMARY_PATH" >&2
fi
