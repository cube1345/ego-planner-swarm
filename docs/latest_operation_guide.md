# Ego-Planner 当前操作指南

更新时间：2026-05-03

## 1. 适用范围

本指南对应当前仓库已经验证可用的三类链路：

- 原版非融合 RViz 仿真：
  `single_run_in_sim.launch.py`
- 自适应融合 RViz 仿真：
  `single_run_in_sim_fusion.launch.py`
- 无界面 headless 融合评测：
  `tools/compare_fusion.sh`

当前默认 RViz 配置：

- `drone0_clean.rviz`

## 2. 环境准备

每个终端执行：

```bash
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

说明：

- `install/setup.bash` 可能提示 `drone_detect/local_setup.bash not found`，当前仿真链路不受该提示阻塞。
- 当前项目环境推荐使用 `rmw_fastrtps_cpp`。

## 3. 编译

```bash
colcon build --packages-select ego_planner
```

## 4. 原版非融合仿真

### 4.1 启动仿真

```bash
ros2 launch ego_planner single_run_in_sim.launch.py use_dynamic:=False use_mockamap:=False
```

### 4.2 打开 RViz

```bash
ros2 launch ego_planner rviz.launch.py
```

说明：

- 当前 `rviz.launch.py` 默认加载 `drone0_clean.rviz`。
- 这套链路不使用融合节点，`grid_map/cloud` 直接来自深度点云。

## 5. 自适应融合仿真

### 5.1 推荐启动方式

```bash
bash tools/run_adaptive_rviz_demo.sh
```

该脚本会自动：

- 清理旧的 `single_run_in_sim*`、`ego_planner_node`、`rviz2` 等残留进程
- 启动一套干净的自适应融合仿真
- 打开 `drone0_clean.rviz`

### 5.2 手动启动方式

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

说明：

- 当前默认已开启自适应 `min_probability`
- 当前默认搜索区间为 `0.20 ~ 0.35`
- 当前默认候选步长为 `0.02`
- 当前演示模式为单目标点、非动态障碍、融合避障

## 6. 如何判断 RViz 现象正常

以下现象应当同时成立：

- 能看到 `drone_0` 的机器人 marker 持续运动
- 能看到 `drone_0_vis/path` 持续累积历史轨迹
- 能看到 `drone_0_plan_vis/optimal_list` 持续刷新
- 能看到 `/drone_0_grid/grid_map/occupancy_inflate` 与 `/map_generator/global_cloud`
- 飞行器不会长期原地不动，也不会长时间直线穿障

如果你看到“无人机停在原地”，优先检查：

- 是否误开了多套 `/drone_0` 仿真
- 是否仍在使用旧的 `default.rviz`
- 是否使用了 `tools/run_adaptive_rviz_demo.sh` 先做清场

## 7. 如何确认融合已经生效

### 7.1 看日志

融合节点会周期性输出：

- `depth_pts=...`
- `lidar_pts=...`
- `fused_voxels=...`
- `depth_only=...`
- `lidar_only=...`
- `dual=...`
- `min_prob=...`

其中：

- `dual > 0` 说明同一帧中两路传感器都对障碍体素有贡献
- `min_prob` 会显示当前在线使用的阈值

### 7.2 看话题

```bash
ros2 topic echo --once /drone_0_fusion/fused_cloud --field width
ros2 topic echo --once /drone_0_grid/grid_map/occupancy_inflate --field width
```

推荐判据：

- `fused_cloud.width > 0`
- `occupancy_inflate.width > 0`

## 8. Headless 指标检测

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

- `artifacts/headless_eval/<label>.summary.json`
- `artifacts/headless_eval/<label>_sim_stats/sim_stats_summary.json`
- `artifacts/headless_eval/<label>.launch.log`

当前版本已经修复：

- `fusion_benefit_report.py` 重复 `rclpy.shutdown()` 报错
- `sim_flight_stats_report.py` 的 `replan_count` 漏算问题

## 9. 当前自适应参数说明

当前真正在线自适应的参数只有一个：

- `min_probability`

其工作方式：

- 对每帧同步的 depth / lidar 数据构建共享体素证据表
- 在 `adaptive_min_probability_min ~ adaptive_min_probability_max` 范围内枚举候选阈值
- 把每个候选阈值生成的局部障碍集合与局部 GT 体素比较
- 评分目标为：
  `utility = gain_f1 + 0.35 * gain_recall`
- 对各候选 utility 做 EMA 平滑
- 选择得分最高的阈值作为当前帧使用的 `current_min_probability`

当前用于控制自适应过程的超参数包括：

- `adaptive_min_probability_enable`
- `adaptive_min_probability_min`
- `adaptive_min_probability_max`
- `adaptive_min_probability_step`
- `adaptive_score_alpha`
- `adaptive_eval_range`
- `adaptive_min_gt_voxels`

说明：

- `adaptive_target_retention`
- `adaptive_retention_band`

这两个参数当前已声明，但没有真正参与当前在线决策逻辑。

## 10. 停止仿真

普通停止：

- 在各终端按 `Ctrl+C`

如需强制清理：

```bash
pkill -f "single_run_in_sim_fusion.launch.py"
pkill -f "single_run_in_sim.launch.py"
pkill -f "rviz2"
```
