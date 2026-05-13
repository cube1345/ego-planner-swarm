#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

OUT_DIR="artifacts/headless_eval"
DURATION=24
STARTUP_WAIT=8
LABEL_PREFIX="adaptive_near_field_radius_eval"
RADIUS_MIN="3.0"
RADIUS_MAX="5.0"
RADIUS_STEP="1.0"

usage() {
  cat <<'EOF'
Usage: bash tools/evaluate_adaptive_near_field_radius.sh [options]

  --out-dir DIR
  --duration SEC
  --startup-wait SEC
  --label-prefix NAME
  --radius-min FLOAT
  --radius-max FLOAT
  --radius-step FLOAT
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --startup-wait) STARTUP_WAIT="$2"; shift 2 ;;
    --label-prefix) LABEL_PREFIX="$2"; shift 2 ;;
    --radius-min) RADIUS_MIN="$2"; shift 2 ;;
    --radius-max) RADIUS_MAX="$2"; shift 2 ;;
    --radius-step) RADIUS_STEP="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

mkdir -p "$OUT_DIR"

OFF_LABEL="${LABEL_PREFIX}_off"
ON_LABEL="${LABEL_PREFIX}_on"
RESULT_JSON="${OUT_DIR}/${LABEL_PREFIX}.comparison.json"

echo "[adaptive_near_field_radius_eval] running OFF baseline..."
bash tools/compare_fusion.sh \
  --label "$OFF_LABEL" \
  --out-dir "$OUT_DIR" \
  --duration "$DURATION" \
  --startup-wait "$STARTUP_WAIT" \
  --adaptive-near-field-radius-enable False

echo "[adaptive_near_field_radius_eval] running ON baseline..."
bash tools/compare_fusion.sh \
  --label "$ON_LABEL" \
  --out-dir "$OUT_DIR" \
  --duration "$DURATION" \
  --startup-wait "$STARTUP_WAIT" \
  --adaptive-near-field-radius-enable True \
  --adaptive-near-field-radius-min "$RADIUS_MIN" \
  --adaptive-near-field-radius-max "$RADIUS_MAX" \
  --adaptive-near-field-radius-step "$RADIUS_STEP"

/usr/bin/python3 - "$OUT_DIR/${OFF_LABEL}.summary.json" "$OUT_DIR/${ON_LABEL}.summary.json" "$RESULT_JSON" <<'PY'
import json
import sys
from pathlib import Path

off_path = Path(sys.argv[1])
on_path = Path(sys.argv[2])
result_path = Path(sys.argv[3])

off_data = json.loads(off_path.read_text(encoding="utf-8"))
on_data = json.loads(on_path.read_text(encoding="utf-8"))

delta = {
    "fusion_recall": on_data["fusion_recall"] - off_data["fusion_recall"],
    "fusion_f1": on_data["fusion_f1"] - off_data["fusion_f1"],
    "fusion_recall_gain_vs_best_single": on_data["fusion_recall_gain_vs_best_single"] - off_data["fusion_recall_gain_vs_best_single"],
    "fusion_f1_gain_vs_best_single": on_data["fusion_f1_gain_vs_best_single"] - off_data["fusion_f1_gain_vs_best_single"],
    "positive_ratio_recall_gain": on_data["positive_ratio_recall_gain"] - off_data["positive_ratio_recall_gain"],
    "positive_ratio_f1_gain": on_data["positive_ratio_f1_gain"] - off_data["positive_ratio_f1_gain"],
}

if (
    delta["fusion_f1"] >= 0.002
    and delta["fusion_recall"] >= 0.0008
    and delta["fusion_f1_gain_vs_best_single"] >= 0.0005
):
    verdict = "keep"
elif delta["fusion_f1"] <= -0.002 or delta["fusion_recall"] <= -0.0008:
    verdict = "disable"
else:
    verdict = "neutral"

comparison = {
    "off_label": off_path.stem.replace(".summary", ""),
    "on_label": on_path.stem.replace(".summary", ""),
    "off": off_data,
    "on": on_data,
    "delta": delta,
    "verdict": verdict,
}
result_path.write_text(json.dumps(comparison, indent=2), encoding="utf-8")
print(json.dumps(comparison, indent=2))
PY
