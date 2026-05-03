# EGO-Planner ROS2 Project

## English

### Overview

This repository contains a ROS 2 based UAV local obstacle-avoidance stack built around EGO-Planner.  
The current project supports two main simulation paths:

- Original non-fusion simulation:
  `single_run_in_sim.launch.py`
- Adaptive multimodal fusion simulation:
  `single_run_in_sim_fusion.launch.py`

The fusion path adds:

- depth cloud: `/drone_0_pcl_render_node/cloud`
- simulated lidar cloud: `/drone_0_lidar/points`
- fused cloud: `/drone_0_fusion/fused_cloud`

The current online adaptive parameter is:

- fusion-layer `min_probability`

The current clean RViz configuration is:

- `src/planner/plan_manage/launch/drone0_clean.rviz`

### Recommended Environment

Run in each terminal:

```bash
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Notes:

- `install/setup.bash` may warn about `drone_detect/local_setup.bash not found`; this does not block the current simulation chain.
- In the current project environment, `rmw_fastrtps_cpp` is the recommended middleware for simulation and evaluation.

### Build

```bash
colcon build --packages-select ego_planner
```

### Quick Start

#### 1. Original non-fusion simulation

Terminal 1:

```bash
ros2 launch ego_planner single_run_in_sim.launch.py use_dynamic:=False use_mockamap:=False
```

Terminal 2:

```bash
ros2 launch ego_planner rviz.launch.py
```

#### 2. Adaptive fusion simulation

Recommended one-command demo:

```bash
bash tools/run_adaptive_rviz_demo.sh
```

This script will:

- kill stale simulation / RViz processes
- start one clean adaptive fusion simulation
- open RViz with the clean `drone0_clean.rviz` view

Manual launch is also supported:

Terminal 1:

```bash
ros2 launch ego_planner single_run_in_sim_fusion.launch.py \
  use_fusion:=True \
  use_dynamic:=False \
  use_mockamap:=False \
  point_num:=1 \
  point0_x:=15.0 \
  point0_y:=0.0 \
  point0_z:=1.0 \
  min_probability:=0.30
```

Terminal 2:

```bash
ros2 launch ego_planner rviz.launch.py
```

### Headless Evaluation

Use the current batch evaluation tool:

```bash
bash tools/compare_fusion.sh \
  --label headless_eval_run \
  --out-dir artifacts/headless_eval \
  --duration 20 \
  --startup-wait 6 \
  --window-size 30 \
  --report-every 10 \
  --use-mockamap False \
  --init-x -15.0 \
  --init-y 0.0 \
  --init-z 0.1 \
  --point-num 1 \
  --point0-x 15.0 \
  --point0-y 0.0 \
  --point0-z 1.0 \
  --min-probability 0.30 \
  --adaptive-enable True
```

Key outputs:

- `<label>.summary.json`: fusion benefit summary
- `<label>_sim_stats/sim_stats_summary.json`: avoidance process statistics
- `<label>.launch.log`: final replan counting reference

### Project Status

Current validated engineering state:

- Original non-fusion simulation is available and stable for RViz demonstration.
- Adaptive fusion simulation is available and should be started from a clean process state.
- Default RViz launch now points to `drone0_clean.rviz` to avoid clutter from multi-drone display leftovers.
- Headless metrics chain has been repaired for clean shutdown and correct `replan_count`.

### Documentation

- `docs/latest_operation_guide.md`
- `docs/multimodal_fusion_code_walkthrough.md`
- `docs/multimodal_fusion_paper_style.md`
- `CHANGELOG.md`

## 中文

### 项目概览

本仓库是一个基于 ROS 2 的无人机局部避障工程，核心规划器为 EGO-Planner。  
当前项目支持两条主要仿真链路：

- 原版非融合仿真：
  `single_run_in_sim.launch.py`
- 自适应多模态融合仿真：
  `single_run_in_sim_fusion.launch.py`

融合链路额外引入：

- 深度点云：`/drone_0_pcl_render_node/cloud`
- 模拟 LiDAR 点云：`/drone_0_lidar/points`
- 融合输出：`/drone_0_fusion/fused_cloud`

当前真正在线自适应的参数只有一个：

- 融合层 `min_probability`

当前推荐的干净 RViz 配置是：

- `src/planner/plan_manage/launch/drone0_clean.rviz`

### 推荐环境

每个终端执行：

```bash
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

说明：

- `install/setup.bash` 可能提示 `drone_detect/local_setup.bash not found`，当前仿真链路不受此提示阻塞。
- 当前项目环境下，`rmw_fastrtps_cpp` 是推荐中间件。

### 编译

```bash
colcon build --packages-select ego_planner
```

### 快速开始

#### 1. 原版非融合仿真

终端 1：

```bash
ros2 launch ego_planner single_run_in_sim.launch.py use_dynamic:=False use_mockamap:=False
```

终端 2：

```bash
ros2 launch ego_planner rviz.launch.py
```

#### 2. 自适应融合仿真

推荐直接使用一键脚本：

```bash
bash tools/run_adaptive_rviz_demo.sh
```

该脚本会：

- 清理旧的仿真与 RViz 残留进程
- 启动一套干净的自适应融合仿真
- 用 `drone0_clean.rviz` 打开 RViz

也可以手动启动：

终端 1：

```bash
ros2 launch ego_planner single_run_in_sim_fusion.launch.py \
  use_fusion:=True \
  use_dynamic:=False \
  use_mockamap:=False \
  point_num:=1 \
  point0_x:=15.0 \
  point0_y:=0.0 \
  point0_z:=1.0 \
  min_probability:=0.30
```

终端 2：

```bash
ros2 launch ego_planner rviz.launch.py
```

### 无界面批量评测

```bash
bash tools/compare_fusion.sh \
  --label headless_eval_run \
  --out-dir artifacts/headless_eval \
  --duration 20 \
  --startup-wait 6 \
  --window-size 30 \
  --report-every 10 \
  --use-mockamap False \
  --init-x -15.0 \
  --init-y 0.0 \
  --init-z 0.1 \
  --point-num 1 \
  --point0-x 15.0 \
  --point0-y 0.0 \
  --point0-z 1.0 \
  --min-probability 0.30 \
  --adaptive-enable True
```

关键产物：

- `<label>.summary.json`：融合收益统计
- `<label>_sim_stats/sim_stats_summary.json`：避障过程统计
- `<label>.launch.log`：最终 replan 对账日志

### 当前项目状态

当前已经验证过的工程状态：

- 原版非融合仿真可稳定用于 RViz 演示
- 自适应融合仿真可用，但必须先清理旧进程再启动
- `rviz.launch.py` 默认已经切到 `drone0_clean.rviz`
- headless 指标链路已经修复重复 shutdown 和 `replan_count` 误计数问题

### 相关文档

- `docs/latest_operation_guide.md`
- `docs/multimodal_fusion_code_walkthrough.md`
- `docs/multimodal_fusion_paper_style.md`
- `CHANGELOG.md`
