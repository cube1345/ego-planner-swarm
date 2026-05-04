# Ego-Planner 融合版最新操作指南

更新时间：2026-03-31

## 1. 适用范围

本指南对应当前仓库的 ROS2 融合仿真链路：

- 启动文件：`single_run_in_sim_fusion.launch.py`
- 融合节点：`ros2_lidar_depth_fusion_node.py`
- RViz 配置：`fusion_compare.rviz`

## 2. 环境准备

在每个终端执行：

```bash
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_LOG_DIR=/tmp/ros_logs
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

说明：
- `install/local_setup.bash` 可能提示 `drone_detect/local_setup.bash not found`，当前融合仿真不受该提示阻塞。
- 在受限环境下，`rmw_fastrtps_cpp` 比 CycloneDDS 更稳定。

## 3. 构建（修改代码后）

```bash
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
colcon build --packages-select ego_planner
```

## 4. 启动融合仿真

```bash
ros2 launch ego_planner single_run_in_sim_fusion.launch.py use_fusion:=True
```

## 5. 启动 RViz（融合显示）

新开一个终端，执行环境准备后：

```bash
ros2 launch ego_planner rviz.launch.py rviz_config:=fusion_compare.rviz
```

## 6. 如何确认“数据融合已生效”

### 6.1 看融合节点日志

融合节点会输出类似统计：

- `depth_pts=...`
- `lidar_pts=...`
- `fused_voxels=...`
- `depth_only=...`
- `lidar_only=...`
- `dual=...`

当 `dual > 0` 时，表示同一融合帧中两类传感器共同贡献了障碍体素。

### 6.2 看话题点数

```bash
ros2 topic echo --once /drone_0_fusion/fused_cloud --field width
ros2 topic echo --once /drone_0_grid/grid_map/occupancy_inflate --field width
```

推荐判据：
- `fused_cloud.width > 0`
- `occupancy_inflate.width > 0`

### 6.3 在 RViz 做可视化确认

同时显示并区分颜色：
- `/drone_0_pcl_render_node/cloud`（深度点云）
- `/drone_0_lidar/points`（模拟 LiDAR）
- `/drone_0_fusion/fused_cloud`（融合结果）

## 7. 关键实现说明（当前版本）

- 为兼容当前仿真链路中深度点云时间戳为 `0` 的现象，
  `simulated_lidar_cloud.py` 增加了参数 `force_zero_stamp`。
- `single_run_in_sim_fusion.launch.py` 中对模拟 LiDAR 设定：
  `force_zero_stamp:=True`，保证与深度云时间对齐，触发同步融合。

## 8. 常见问题

### 8.1 无人机只走直线，不避障

优先检查：
- `/drone_0_fusion/fused_cloud` 是否为空
- `/drone_0_grid/grid_map/occupancy_inflate` 是否为空
- 融合节点是否打印 `fusion frame=...` 统计

### 8.2 轨迹能规划但飞行器“扎进障碍后出不来”

这是规划代价、地图膨胀、控制跟踪共同作用的综合问题。
建议先保持融合链路稳定，再逐步调参：
- `grid_map/obstacles_inflation`
- `optimization/lambda_collision`
- `manager/max_vel`, `manager/max_acc`

## 9. 停止仿真

在各终端按 `Ctrl+C`。

如需清理残留：

```bash
pkill -f "single_run_in_sim_fusion.launch.py"
pkill -f "rviz.launch.py"
```
