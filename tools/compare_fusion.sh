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
MIN_PROBABILITY="0.55"
ADAPTIVE_ENABLE="False"
ADAPTIVE_MIN="0.35"
ADAPTIVE_MAX="0.60"
ADAPTIVE_STEP="0.02"
ADAPTIVE_EVAL_RANGE="10.0"
ADAPTIVE_MIN_GT_VOXELS="40"
ADAPTIVE_SCORE_ALPHA="0.25"

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
  --adaptive-enable True|False
  --adaptive-min FLOAT
  --adaptive-max FLOAT
  --adaptive-step FLOAT
  --adaptive-eval-range FLOAT
  --adaptive-min-gt-voxels INT
  --adaptive-score-alpha FLOAT
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
    --adaptive-enable) ADAPTIVE_ENABLE="$2"; shift 2 ;;
    --adaptive-min) ADAPTIVE_MIN="$2"; shift 2 ;;
    --adaptive-max) ADAPTIVE_MAX="$2"; shift 2 ;;
    --adaptive-step) ADAPTIVE_STEP="$2"; shift 2 ;;
    --adaptive-eval-range) ADAPTIVE_EVAL_RANGE="$2"; shift 2 ;;
    --adaptive-min-gt-voxels) ADAPTIVE_MIN_GT_VOXELS="$2"; shift 2 ;;
    --adaptive-score-alpha) ADAPTIVE_SCORE_ALPHA="$2"; shift 2 ;;
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
  if [[ -n "${REPORT_PID:-}" ]]; then
    stop_pid "$REPORT_PID"
  fi
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    stop_pid "$LAUNCH_PID"
  fi
  pkill -TERM -f 'single_run_in_sim_fusion.launch.py' 2>/dev/null || true
  pkill -TERM -f 'fusion_benefit_report.py' 2>/dev/null || true
  sleep 1
  pkill -KILL -f 'single_run_in_sim_fusion.launch.py' 2>/dev/null || true
  pkill -KILL -f 'fusion_benefit_report.py' 2>/dev/null || true
}

trap cleanup EXIT

set +u
source install/setup.bash
set -u

rm -f "$CSV_PATH" "$SUMMARY_PATH" "$LAUNCH_LOG" "$REPORT_LOG"

echo "[compare_fusion] launch label=$LABEL out_dir=$OUT_DIR"
ros2 launch ego_planner single_run_in_sim_fusion.launch.py \
  use_fusion:=True \
  use_mockamap:="$USE_MOCKAMAP" \
  min_probability:="$MIN_PROBABILITY" \
  adaptive_min_probability_enable:="$ADAPTIVE_ENABLE" \
  adaptive_min_probability_min:="$ADAPTIVE_MIN" \
  adaptive_min_probability_max:="$ADAPTIVE_MAX" \
  adaptive_min_probability_step:="$ADAPTIVE_STEP" \
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

sleep "$DURATION"

stop_pid "$REPORT_PID"
unset REPORT_PID

stop_pid "$LAUNCH_PID"
unset LAUNCH_PID

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
