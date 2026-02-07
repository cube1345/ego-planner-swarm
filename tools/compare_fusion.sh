#!/usr/bin/env bash
set -euo pipefail

DURATION_SEC="${1:-90}"
if [[ "$DURATION_SEC" -gt 120 ]]; then
  echo "[WARN] Duration capped to 120s (was $DURATION_SEC)"
  DURATION_SEC=120
fi
if [[ "$DURATION_SEC" -lt 30 ]]; then
  echo "[WARN] Duration raised to 30s (was $DURATION_SEC)"
  DURATION_SEC=30
fi
WORKDIR="/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm"
LAUNCH_CMD="ros2 launch ego_planner single_run_in_sim.launch.py"

DEPTH_LOG="/tmp/run_depth_only.log"
FUSION_LOG="/tmp/run_fusion.log"
DEPTH_CSV="/tmp/grid_map_fusion_stats_depth.csv"
FUSION_CSV="/tmp/grid_map_fusion_stats_fusion.csv"
STATS_CSV="/tmp/grid_map_fusion_stats.csv"
SUMMARY_OUT="/tmp/compare_fusion_summary.txt"

set +u
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
  source "/opt/ros/humble/setup.bash"
fi
source "$WORKDIR/install/setup.bash"
set -u

run_case() {
  local name="$1"
  local use_lidar="$2"
  local hit_scale="$3"
  local miss_scale="$4"
  local conflict_scale="$5"
  local log_file="$6"
  local csv_out="$7"

  rm -f "$log_file" "$STATS_CSV"

  echo "[INFO] Start $name for ${DURATION_SEC}s"
  timeout --signal=INT --kill-after=10s "${DURATION_SEC}s" $LAUNCH_CMD 2>&1 | tee "$log_file" &
  local launch_pid=$!

  # Wait a bit for node to come up.
  sleep 5

  ros2 param set /drone_0_ego_planner_node grid_map/use_lidar_buffer "$use_lidar" >/dev/null
  ros2 param set /drone_0_ego_planner_node grid_map/lidar_sync_tolerance 0.1 >/dev/null
  ros2 param set /drone_0_ego_planner_node grid_map/lidar_fallback_timeout 0.2 >/dev/null
  ros2 param set /drone_0_ego_planner_node grid_map/lidar_hit_scale "$hit_scale" >/dev/null
  ros2 param set /drone_0_ego_planner_node grid_map/lidar_miss_scale "$miss_scale" >/dev/null
  ros2 param set /drone_0_ego_planner_node grid_map/lidar_max_range 6.0 >/dev/null
  ros2 param set /drone_0_ego_planner_node grid_map/fusion_conflict_scale "$conflict_scale" >/dev/null
  ros2 param set /drone_0_ego_planner_node grid_map/enable_fusion_stats true >/dev/null

  wait "$launch_pid" >/dev/null 2>&1 || true

  if [[ -f "$STATS_CSV" ]]; then
    cp "$STATS_CSV" "$csv_out"
  else
    echo "[WARN] No stats file produced for $name"
  fi
}

run_case "depth-only" false 0.0 0.0 1.0 "$DEPTH_LOG" "$DEPTH_CSV"
run_case "fusion" true 0.4 0.4 0.9 "$FUSION_LOG" "$FUSION_CSV"

grep -c "plan_success=1" "$FUSION_LOG" 2>/dev/null || true
grep -c "traj .* failed" "$FUSION_LOG" 2>/dev/null || true
grep -c "collided, keep optimizing" "$FUSION_LOG" 2>/dev/null || true
grep -c "rebound" "$FUSION_LOG" 2>/dev/null || true
{
  echo "[INFO] Done. Logs and CSVs:"
  ls -l "$DEPTH_LOG" "$FUSION_LOG" "$DEPTH_CSV" "$FUSION_CSV" 2>/dev/null || true

  if [[ -f "$DEPTH_CSV" ]]; then
    tail -n +2 "$DEPTH_CSV" | awk -F, '{sum+=$4; n+=1} END{print "depth avg_conflict_ratio=", (n?sum/n:0)}'
  else
    echo "depth avg_conflict_ratio= N/A"
  fi
  if [[ -f "$FUSION_CSV" ]]; then
    tail -n +2 "$FUSION_CSV" | awk -F, '{sum+=$4; n+=1} END{print "fusion avg_conflict_ratio=", (n?sum/n:0)}'
  else
    echo "fusion avg_conflict_ratio= N/A"
  fi

  depth_success=$(grep -c "plan_success=1" "$DEPTH_LOG" 2>/dev/null || true)
  fusion_success=$(grep -c "plan_success=1" "$FUSION_LOG" 2>/dev/null || true)
  depth_fail=$(grep -c "plan_success=0" "$DEPTH_LOG" 2>/dev/null || true)
  fusion_fail=$(grep -c "plan_success=0" "$FUSION_LOG" 2>/dev/null || true)

  depth_rate=$(awk -v s="$depth_success" -v f="$depth_fail" 'BEGIN{t=s+f; if(t>0){printf("%.3f", s/t)} else {printf("N/A")}}')
  fusion_rate=$(awk -v s="$fusion_success" -v f="$fusion_fail" 'BEGIN{t=s+f; if(t>0){printf("%.3f", s/t)} else {printf("N/A")}}')

  printf "depth success_rate=%s (success=%s, fail=%s)\n" "$depth_rate" "$depth_success" "$depth_fail"
  printf "fusion success_rate=%s (success=%s, fail=%s)\n" "$fusion_rate" "$fusion_success" "$fusion_fail"

  echo "depth traj failed count="$(grep -c "traj .* failed" "$DEPTH_LOG" 2>/dev/null || true)
  echo "fusion traj failed count="$(grep -c "traj .* failed" "$FUSION_LOG" 2>/dev/null || true)
  echo "depth collided count="$(grep -c "collided, keep optimizing" "$DEPTH_LOG" 2>/dev/null || true)
  echo "fusion collided count="$(grep -c "collided, keep optimizing" "$FUSION_LOG" 2>/dev/null || true)
  echo "depth rebound count="$(grep -c "rebound" "$DEPTH_LOG" 2>/dev/null || true)
  echo "fusion rebound count="$(grep -c "rebound" "$FUSION_LOG" 2>/dev/null || true)
} | tee "$SUMMARY_OUT"

echo "[INFO] Summary saved to $SUMMARY_OUT"
