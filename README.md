# EGO-Planner ROS 2 UAV Avoidance Stack

English | [中文](README.zh-CN.md)

## What This Project Does

This project is a ROS 2 based UAV local obstacle-avoidance simulation stack built on top of EGO-Planner.

It provides:

- A baseline EGO-Planner simulation path.
- A multimodal fusion simulation path using depth cloud and simulated LiDAR cloud.
- A Dempster-Shafer evidence metric layer for fusion uncertainty and sensor-conflict diagnostics.
- A clean RViz view for single-drone obstacle-avoidance demonstration.
- Headless evaluation scripts for collecting simulation and avoidance metrics.
- Helper scripts to start or stop simulation processes safely.

The project is intended for local planning, obstacle avoidance, RViz demonstration, and batch evaluation in simulation.

## Environment

Tested environment:

- Ubuntu 22.04
- ROS 2 Humble
- `rmw_fastrtps_cpp`

Before running commands, enter the workspace:

```bash
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
```

Source ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/ros_logs
```

If `install/setup.bash` prints a warning about `drone_detect/local_setup.bash not found`, it does not block the current simulation chain.

## Build

Build the main packages:

```bash
colcon build --packages-select ego_planner odom_visualization
```

If you need to rebuild everything:

```bash
colcon build
```

After building, source the workspace again:

```bash
source install/setup.bash
```

## One-Command RViz Simulation

The recommended launcher is:

```bash
bash tools/start_rviz_sim.sh
```

By default, it:

- Kills stale simulation and RViz processes.
- Starts the fusion simulation.
- Opens RViz with `src/planner/plan_manage/launch/drone0_clean.rviz`.
- Writes logs to `/tmp/ego_planner_rviz`.

Start the baseline non-fusion simulation:

```bash
bash tools/start_rviz_sim.sh --mode plain
```

Start the fusion simulation:

```bash
bash tools/start_rviz_sim.sh --mode fusion
```

Start the fusion simulation with dynamic obstacles:

```bash
bash tools/start_rviz_sim.sh --mode fusion --use-dynamic True
```

Dynamic mode automatically uses a larger local map update range of `6.5m x 6.5m` so moving obstacles enter the local map and planner earlier.

Start simulation without RViz:

```bash
bash tools/start_rviz_sim.sh --no-rviz
```

Only stop existing simulation and RViz processes:

```bash
bash tools/start_rviz_sim.sh --kill-only
```

Show all launcher options:

```bash
bash tools/start_rviz_sim.sh --help
```

## Manual Launch

Baseline simulation:

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  use_dynamic:=False \
  use_mockamap:=False \
  point_num:=1 \
  point0_x:=15.0 \
  point0_y:=0.0 \
  point0_z:=1.0
```

Fusion simulation:

```bash
ros2 launch ego_planner single_run_in_sim_fusion.launch.py \
  use_fusion:=True \
  use_dynamic:=False \
  use_mockamap:=False \
  point_num:=1 \
  point0_x:=15.0 \
  point0_y:=0.0 \
  point0_z:=1.0 \
  min_probability:=0.30 \
  closed_loop_feedback_enable:=True
```

Open RViz manually:

```bash
rviz2 -d src/planner/plan_manage/launch/drone0_clean.rviz
```

## Useful RViz Topics

Common topics shown in RViz:

- `/drone_0_vis/robot`: UAV mesh marker.
- `/drone_0_vis/path`: executed flight path.
- `/drone_0_plan_vis/optimal_list`: current planned trajectory.
- `/drone_0_grid/grid_map/occupancy_inflate`: inflated local obstacle map.
- `/drone_0_pcl_render_node/cloud`: depth camera cloud.
- `/drone_0_lidar/points`: simulated LiDAR cloud.
- `/drone_0_fusion/fused_cloud`: fused cloud used by the planner in fusion mode.
- `/drone_0_fusion/ds_metrics`: D-S evidence metrics, including occupied belief, unknown, and conflict.

## Current Fusion Chain

Fusion mode builds voxel-level Log-odds occupancy evidence from the depth cloud and simulated LiDAR cloud, then feeds `/drone_0_fusion/fused_cloud` to EGO-Planner.

Dempster-Shafer is currently used as a metric layer and a lightweight adaptive-candidate penalty, not as the main output layer:

- `ds_evidence_enable=True` is enabled by default.
- `/drone_0_fusion/ds_metrics` publishes `belief_occupied_mean`, `unknown_mean`, `conflict_mean`, and related diagnostics.
- When `adaptive_ds_score_enable=True`, `unknown/conflict` lightly affects online candidate ranking.
- D-S does not replace Log-odds and does not directly replace the `/drone_0_fusion/fused_cloud` selection rule.

The default online adaptive parameters are `min_probability`, `min_hits`, and `dual_bonus`. `closed_loop_feedback` publishes metrics by default, but `closed_loop_optimizer_enable=False`, so the closed-loop optimizer does not take over online parameter selection by default.

## Headless Evaluation

Run a short headless evaluation:

```bash
bash tools/compare_fusion.sh \
  --label headless_eval_run \
  --out-dir artifacts/headless_eval \
  --duration 20 \
  --startup-wait 6 \
  --window-size 30 \
  --report-every 10 \
  --use-mockamap False \
  --point-num 1 \
  --point0-x 15.0 \
  --point0-y 0.0 \
  --point0-z 1.0 \
  --min-probability 0.30 \
  --lambda-collision 0.65
```

Dynamic-obstacle evaluation:

```bash
bash tools/compare_fusion.sh \
  --label dynamic_eval_run \
  --out-dir artifacts/headless_eval \
  --duration 45 \
  --startup-wait 8 \
  --window-size 80 \
  --report-every 20 \
  --dynamic-obstacles-enable True
```

When dynamic obstacles are enabled, the script raises `local_update_range_x/y` to `6.5` by default unless `--local-update-range-x/y` is supplied.
The default dynamic scene uses one softened moving cylinder so RViz still shows motion while the headless metrics remain stable. The latest validated 45 s run reached the goal in about 25 s with zero dynamic safety violations.

Main outputs:

- `artifacts/headless_eval/<label>.summary.json`
- `artifacts/headless_eval/<label>.closed_loop.json`
- `artifacts/headless_eval/<label>_sim_stats/sim_stats_summary.json`
- `artifacts/headless_eval/<label>.launch.log`

## Stop Simulation

Recommended:

```bash
bash tools/start_rviz_sim.sh --kill-only
```

Manual cleanup:

```bash
pkill -f "single_run_in_sim_fusion.launch.py"
pkill -f "single_run_in_sim.launch.py"
pkill -f "rviz2"
```
