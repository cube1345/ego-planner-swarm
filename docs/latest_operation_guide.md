# Ego-Planner 融合版最新操作指南

更新时间：2026-05-03

## 1. 适用范围

本指南对应当前仓库已验证可用的 ROS2 融合仿真链路：

- 启动文件：`single_run_in_sim_fusion.launch.py`
- 融合节点：`ros2_lidar_depth_fusion_node.py`
- RViz 配置：`default.rviz`
- 地图源：`random_forest`
- 演示模式：单目标点、非动态障碍、融合避障

## 2. 环境准备

在每个终端执行：

```bash
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

说明：
- `install/setup.bash` 可能提示 `drone_detect/local_setup.bash not found`，当前融合仿真和 RViz 演示不受该提示阻塞。
- 在受限环境下，`rmw_fastrtps_cpp` 比 CycloneDDS 更稳定。

## 3. 构建（修改代码后）

```bash
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
colcon build --packages-select ego_planner
```

## 4. 启动当前推荐的 RViz 演示仿真

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
  adaptive_min_probability_enable:=False
```

说明：
- 这是当前已经在 RViz 中确认“现象合适”的演示配置。
- 当前演示使用单个目标点，便于稳定观察规划轨迹、历史轨迹和绕障过程。
- `use_dynamic:=False` 表示关闭动态障碍物实验链路，避免把未验证稳定的动态效果混入当前演示。

## 5. 启动 RViz

新开一个终端，执行环境准备后：

```bash
rviz2 -d src/planner/plan_manage/launch/default.rviz
```

说明：
- `default.rviz` 是当前已清理过的显示配置，适合直接看规划路线、历史轨迹、障碍物地图和飞行器状态。
- 如果后续又出现杂乱旧轨迹，优先检查是否误开了其他 RViz 配置文件。

## 6. 如何确认当前 RViz 现象正常

推荐先看以下几个现象是否同时成立：

- 能看到飞行器模型或位姿标记持续运动，而不是停在原地。
- 能看到局部规划轨迹持续刷新，而不是只有一小段红线后停止。
- 能看到历史轨迹逐步累积，而不是整张地图被异常乱码轨迹覆盖。
- 能看到全局障碍地图和局部感知障碍共同存在，而不是只剩局部稀疏障碍点。
- 飞行器不会长期直线穿障，而是会在障碍前出现明显绕行动作。

## 7. 如何确认“数据融合已生效”

### 7.1 看融合节点日志

融合节点会输出类似统计：

- `depth_pts=...`
- `lidar_pts=...`
- `fused_voxels=...`
- `depth_only=...`
- `lidar_only=...`
- `dual=...`

当 `dual > 0` 时，表示同一融合帧中两类传感器共同贡献了障碍体素。

### 7.2 看话题点数

```bash
ros2 topic echo --once /drone_0_fusion/fused_cloud --field width
ros2 topic echo --once /drone_0_grid/grid_map/occupancy_inflate --field width
```

推荐判据：
- `fused_cloud.width > 0`
- `occupancy_inflate.width > 0`

### 7.3 在 RViz 做可视化确认

同时显示并区分颜色：
- `/drone_0_pcl_render_node/cloud`（深度点云）
- `/drone_0_lidar/points`（模拟 LiDAR）
- `/drone_0_fusion/fused_cloud`（融合结果）

## 8. Headless 指标检测

当前仓库已经补齐 headless 检测链路，适合在不打开 RViz 的情况下检查融合收益和避障统计。

执行：

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
  --adaptive-enable False
```

关键产物：
- `artifacts/headless_eval/<label>.summary.json`：融合收益统计。
- `artifacts/headless_eval/<label>_sim_stats/sim_stats_summary.json`：避障过程统计。
- `artifacts/headless_eval/<label>.launch.log`：最终 replan 对账基准日志。

当前版本已修复两类统计问题：
- `fusion_benefit_report.py` 不再因重复 `rclpy.shutdown()` 报错。
- `sim_flight_stats_report.py` 的 `replan_count` 已改为基于最终 `launch.log` 对账，避免漏算后段重规划。

## 9. 关键实现说明（当前版本）

- 为兼容当前仿真链路中深度点云时间戳为 `0` 的现象，
  `simulated_lidar_cloud.py` 增加了参数 `force_zero_stamp`。
- `single_run_in_sim_fusion.launch.py` 中对模拟 LiDAR 设定：
  `force_zero_stamp:=True`，保证与深度云时间对齐，触发同步融合。
- `single_run_in_sim_fusion.launch.py` 已支持显式设置 `init_x/init_y/init_z` 与 `point_num/point0_*`，便于 headless 评测和 RViz 演示共用同一条飞行链路。

## 10. 常见问题

### 10.1 无人机只走直线，不避障

优先检查：
- `/drone_0_fusion/fused_cloud` 是否为空
- `/drone_0_grid/grid_map/occupancy_inflate` 是否为空
- 融合节点是否打印 `fusion frame=...` 统计

### 10.2 轨迹能规划但飞行器“扎进障碍后出不来”

这是规划代价、地图膨胀、控制跟踪共同作用的综合问题。
建议先保持融合链路稳定，再逐步调参：
- `grid_map/obstacles_inflation`
- `optimization/lambda_collision`
- `manager/max_vel`, `manager/max_acc`

### 10.3 Headless 报告里 `replan_count` 明显偏小

优先确认：
- 是否通过 `tools/compare_fusion.sh` 启动，而不是手动只跑一部分节点。
- 是否保留了脚本默认的停止顺序，即先停 launch，再停 stats。
- 是否读取的是同一批次标签下的 `launch.log` 与 `sim_stats_summary.json`。

## 11. 停止仿真

在各终端按 `Ctrl+C`。

如需清理残留：

```bash
pkill -f "single_run_in_sim_fusion.launch.py"
pkill -f "rviz.launch.py"
```
