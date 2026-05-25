# EGO-Planner ROS 2 无人机避障仿真项目

[English](README.md) | 中文

## 项目作用

本项目是一个基于 ROS 2 和 EGO-Planner 的无人机局部避障仿真工程。

它提供：

- 原版 EGO-Planner 避障仿真链路。
- 基于深度点云和模拟 LiDAR 点云的融合仿真链路。
- 面向单无人机演示的干净 RViz 配置。
- 用于采集仿真和避障指标的 headless 评测脚本。
- 一键启动、清理和停止仿真的辅助脚本。

本项目主要用于局部路径规划、避障仿真、RViz 演示和批量指标评测。

## 环境

当前项目环境：

- Ubuntu 22.04
- ROS 2 Humble
- `rmw_fastrtps_cpp`

进入工作空间：

```bash
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
```

加载 ROS 和工作空间环境：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/ros_logs
```

如果 `install/setup.bash` 提示 `drone_detect/local_setup.bash not found`，当前仿真链路不受影响。

## 编译

编译主要包：

```bash
colcon build --packages-select ego_planner odom_visualization
```

如果需要完整重编译：

```bash
colcon build
```

编译后重新加载环境：

```bash
source install/setup.bash
```

## 一键启动 RViz 仿真

推荐使用：

```bash
bash tools/start_rviz_sim.sh
```

默认行为：

- 启动前清理旧的仿真和 RViz 进程。
- 启动融合版仿真。
- 使用 `src/planner/plan_manage/launch/drone0_clean.rviz` 打开 RViz。
- 日志写入 `/tmp/ego_planner_rviz`。

启动原版非融合仿真：

```bash
bash tools/start_rviz_sim.sh --mode plain
```

启动融合版仿真：

```bash
bash tools/start_rviz_sim.sh --mode fusion
```

只启动仿真，不打开 RViz：

```bash
bash tools/start_rviz_sim.sh --no-rviz
```

只清理当前仿真和 RViz 进程：

```bash
bash tools/start_rviz_sim.sh --kill-only
```

查看所有启动参数：

```bash
bash tools/start_rviz_sim.sh --help
```

## 手动启动

原版非融合仿真：

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  use_dynamic:=False \
  use_mockamap:=False \
  point_num:=1 \
  point0_x:=15.0 \
  point0_y:=0.0 \
  point0_z:=1.0
```

融合版仿真：

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

手动打开 RViz：

```bash
rviz2 -d src/planner/plan_manage/launch/drone0_clean.rviz
```

## 常用 RViz 话题

RViz 中常用显示话题：

- `/drone_0_vis/robot`：无人机模型。
- `/drone_0_vis/path`：已飞行历史轨迹。
- `/drone_0_plan_vis/optimal_list`：当前规划轨迹。
- `/drone_0_grid/grid_map/occupancy_inflate`：膨胀后的局部障碍地图。
- `/drone_0_pcl_render_node/cloud`：深度相机点云。
- `/drone_0_lidar/points`：模拟 LiDAR 点云。
- `/drone_0_fusion/fused_cloud`：融合模式下输入给规划器的融合点云。

## Headless 评测

运行一次短时无界面评测：

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
  --min-probability 0.30
```

主要输出：

- `artifacts/headless_eval/<label>.summary.json`
- `artifacts/headless_eval/<label>.closed_loop.json`
- `artifacts/headless_eval/<label>_sim_stats/sim_stats_summary.json`
- `artifacts/headless_eval/<label>.launch.log`

## 停止仿真

推荐方式：

```bash
bash tools/start_rviz_sim.sh --kill-only
```

手动清理：

```bash
pkill -f "single_run_in_sim_fusion.launch.py"
pkill -f "single_run_in_sim.launch.py"
pkill -f "rviz2"
```
