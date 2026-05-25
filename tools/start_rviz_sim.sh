#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

MODE="fusion"
WITH_RVIZ="true"
KILL_ONLY="false"
USE_DYNAMIC="False"
USE_MOCKAMAP="False"
POINT_NUM="1"
POINT0_X="15.0"
POINT0_Y="0.0"
POINT0_Z="1.0"
MIN_PROBABILITY="0.30"
CLOSED_LOOP_FEEDBACK_ENABLE="True"
RVIZ_CONFIG="src/planner/plan_manage/launch/drone0_clean.rviz"
LOG_DIR="/tmp/ego_planner_rviz"
STARTUP_WAIT="5"

usage() {
  cat <<'EOF'
Usage: bash tools/start_rviz_sim.sh [options]

One-key RViz simulation launcher. It kills stale simulation/RViz processes first,
then starts the selected simulation chain and optionally RViz.

Options:
  --mode fusion|plain              Simulation chain. Default: fusion
  --rviz                           Open RViz. Default
  --no-rviz                        Start simulation only
  --kill-only                      Kill related processes and exit
  --use-dynamic True|False         Enable dynamic obstacles. Default: False
  --use-mockamap True|False        Use mockamap instead of random forest. Default: False
  --point-num N                    Number of goal points. Default: 1
  --point0-x X                     First goal x. Default: 15.0
  --point0-y Y                     First goal y. Default: 0.0
  --point0-z Z                     First goal z. Default: 1.0
  --min-probability P              Fusion min probability. Default: 0.30
  --closed-loop-feedback True|False
                                   Enable feedback metrics node. Default: True
  --rviz-config PATH               RViz config. Default: drone0_clean.rviz
  --log-dir DIR                    Log directory. Default: /tmp/ego_planner_rviz
  --startup-wait SEC               Wait before opening RViz/checking topics. Default: 5
  -h, --help                       Show this help

Examples:
  bash tools/start_rviz_sim.sh
  bash tools/start_rviz_sim.sh --mode plain
  bash tools/start_rviz_sim.sh --no-rviz
  bash tools/start_rviz_sim.sh --kill-only
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --rviz)
      WITH_RVIZ="true"
      shift
      ;;
    --no-rviz)
      WITH_RVIZ="false"
      shift
      ;;
    --kill-only)
      KILL_ONLY="true"
      shift
      ;;
    --use-dynamic)
      USE_DYNAMIC="$2"
      shift 2
      ;;
    --use-mockamap)
      USE_MOCKAMAP="$2"
      shift 2
      ;;
    --point-num)
      POINT_NUM="$2"
      shift 2
      ;;
    --point0-x)
      POINT0_X="$2"
      shift 2
      ;;
    --point0-y)
      POINT0_Y="$2"
      shift 2
      ;;
    --point0-z)
      POINT0_Z="$2"
      shift 2
      ;;
    --min-probability)
      MIN_PROBABILITY="$2"
      shift 2
      ;;
    --closed-loop-feedback)
      CLOSED_LOOP_FEEDBACK_ENABLE="$2"
      shift 2
      ;;
    --rviz-config)
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    --log-dir)
      LOG_DIR="$2"
      shift 2
      ;;
    --startup-wait)
      STARTUP_WAIT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[start_rviz_sim] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$MODE" != "fusion" && "$MODE" != "plain" ]]; then
  echo "[start_rviz_sim] --mode must be fusion or plain, got: $MODE" >&2
  exit 2
fi

cleanup_existing() {
  local pattern
  pattern='[s]ingle_run_in_sim_fusion.launch.py|[s]ingle_run_in_sim.launch.py|[e]go_planner_node|[t]raj_server|[p]oscmd_2_odom|[o]dom_visualization|[p]cl_render_node|[s]imulated_lidar_cloud.py|[r]os2_lidar_depth_fusion_node.py|[c]losed_loop_feedback_node.py|[d]ynamic_obstacles_node.py|[r]viz2'
  echo "[start_rviz_sim] stopping stale simulation/RViz processes..."
  pkill -TERM -f "$pattern" 2>/dev/null || true
  sleep 2
  pkill -KILL -f "$pattern" 2>/dev/null || true
}

source_ros_env() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "[start_rviz_sim] missing /opt/ros/humble/setup.bash" >&2
    exit 1
  fi
  if [[ ! -f install/setup.bash ]]; then
    echo "[start_rviz_sim] missing install/setup.bash; run colcon build first" >&2
    exit 1
  fi

  set +u
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  set -u

  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
}

start_detached() {
  local log_file="$1"
  shift
  setsid "$@" > "$log_file" 2>&1 < /dev/null &
  echo $!
}

mkdir -p "$LOG_DIR"
cleanup_existing

if [[ "$KILL_ONLY" == "true" ]]; then
  echo "[start_rviz_sim] cleanup complete."
  exit 0
fi

source_ros_env

LAUNCH_LOG="$LOG_DIR/${MODE}_launch.log"
RVIZ_LOG="$LOG_DIR/rviz.log"

if [[ "$MODE" == "fusion" ]]; then
  LAUNCH_CMD=(
    ros2 launch ego_planner single_run_in_sim_fusion.launch.py
    use_fusion:=True
    use_dynamic:="$USE_DYNAMIC"
    use_mockamap:="$USE_MOCKAMAP"
    point_num:="$POINT_NUM"
    point0_x:="$POINT0_X"
    point0_y:="$POINT0_Y"
    point0_z:="$POINT0_Z"
    min_probability:="$MIN_PROBABILITY"
    closed_loop_feedback_enable:="$CLOSED_LOOP_FEEDBACK_ENABLE"
  )
else
  LAUNCH_CMD=(
    ros2 launch ego_planner single_run_in_sim.launch.py
    use_dynamic:="$USE_DYNAMIC"
    use_mockamap:="$USE_MOCKAMAP"
    point_num:="$POINT_NUM"
    point0_x:="$POINT0_X"
    point0_y:="$POINT0_Y"
    point0_z:="$POINT0_Z"
  )
fi

echo "[start_rviz_sim] starting $MODE simulation..."
LAUNCH_PID="$(start_detached "$LAUNCH_LOG" "${LAUNCH_CMD[@]}")"
echo "[start_rviz_sim] launch pid: $LAUNCH_PID"
echo "[start_rviz_sim] launch log: $LAUNCH_LOG"

sleep "$STARTUP_WAIT"

if [[ "$WITH_RVIZ" == "true" ]]; then
  if [[ ! -f "$RVIZ_CONFIG" ]]; then
    echo "[start_rviz_sim] missing RViz config: $RVIZ_CONFIG" >&2
    exit 1
  fi
  echo "[start_rviz_sim] opening RViz: $RVIZ_CONFIG"
  RVIZ_PID="$(start_detached "$RVIZ_LOG" rviz2 -d "$RVIZ_CONFIG")"
  echo "[start_rviz_sim] rviz pid: $RVIZ_PID"
  echo "[start_rviz_sim] rviz log: $RVIZ_LOG"
fi

echo "[start_rviz_sim] active processes:"
ps -eo pid,cmd | rg '[r]viz2|[s]ingle_run_in_sim_fusion|[s]ingle_run_in_sim.launch.py|[e]go_planner_node|[t]raj_server|[p]oscmd_2_odom|[o]dom_visualization|[p]cl_render_node|[s]imulated_lidar_cloud|[r]os2_lidar_depth_fusion_node|[c]losed_loop_feedback_node' || true

if command -v ros2 >/dev/null 2>&1; then
  echo "[start_rviz_sim] key topics:"
  ros2 topic list 2>/dev/null | rg 'drone_0_vis/robot|drone_0_vis/robot_body|drone_0_vis/path|drone_0_fusion/fused_cloud|drone_0_grid/grid_map/occupancy_inflate|drone_0_visual_slam/odom' || true
fi

echo "[start_rviz_sim] done."
