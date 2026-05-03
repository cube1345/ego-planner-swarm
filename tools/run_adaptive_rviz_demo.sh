#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

cleanup_existing() {
  pkill -TERM -f 'single_run_in_sim_fusion.launch.py|single_run_in_sim.launch.py|ego_planner_node|traj_server|poscmd_2_odom|odom_visualization|pcl_render_node|simulated_lidar_cloud.py|ros2_lidar_depth_fusion_node.py|rviz2' 2>/dev/null || true
  sleep 2
  pkill -KILL -f 'single_run_in_sim_fusion.launch.py|single_run_in_sim.launch.py|ego_planner_node|traj_server|poscmd_2_odom|odom_visualization|pcl_render_node|simulated_lidar_cloud.py|ros2_lidar_depth_fusion_node.py|rviz2' 2>/dev/null || true
}

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

cleanup_existing

echo "[adaptive_rviz_demo] starting clean adaptive fusion simulation..."
ros2 launch ego_planner single_run_in_sim_fusion.launch.py \
  use_fusion:=True \
  use_dynamic:=False \
  use_mockamap:=False \
  point_num:=1 \
  point0_x:=15.0 \
  point0_y:=0.0 \
  point0_z:=1.0 \
  min_probability:=0.30 &

sleep 5

echo "[adaptive_rviz_demo] opening rviz with drone0_clean.rviz..."
rviz2 -d src/planner/plan_manage/launch/drone0_clean.rviz
