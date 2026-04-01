# EGO-Planner 多模态融合代码说明（ROS2）

更新时间：2026-03-31

## 1. 先看结论：当前仓库“融合”是怎么接上的

当前仿真链路里，多模态融合是通过一个 Python 节点实现的：

- 深度点云：`/drone_0_pcl_render_node/cloud`
- 模拟 LiDAR 点云：`/drone_0_lidar/points`
- 融合输出：`/drone_0_fusion/fused_cloud`
- 规划器输入重映射到：`grid_map/cloud`

关键代码入口在：


- `src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py`
- `src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py`
- `src/planner/plan_manage/launch/advanced_param.launch.py`

## 2. 数据流（Data Flow）

```text
map_generator/global_cloud
          |
          v
simulated_lidar_cloud.py --------------------> /drone_0_lidar/points
                                               (LiDAR)
pcl_render_node/cloud -----------------------> /drone_0_pcl_render_node/cloud
                                               (Depth cloud)

                      +------------------------------+
                      | ros2_lidar_depth_fusion_node |
                      |  - ApproximateTime sync      |
                      |  - filter(z/range)           |
                      |  - voxel evidence fusion     |
                      +---------------+--------------+
                                      |
                                      v
                         /drone_0_fusion/fused_cloud
                                      |
                                      v
                    advanced_param.launch.py: grid_map/cloud
                                      |
                                      v
                             EGO-Planner grid_map
```

## 3. 代码级拆解

### 3.1 启动文件如何切换到融合点云

文件：`src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py`

1. `use_fusion` 决定规划器吃哪路点云。
   - 第 32 行：`cloud_topic = 'fusion/fused_cloud' if use_fusion_value else 'pcl_render_node/cloud'`
2. 这个 `cloud_topic` 传到 planner 参数启动文件。
   - 第 84-95 行：`advanced_param_include` 把 `cloud_topic` 继续下传。
3. 启动模拟 LiDAR。
   - 第 151-173 行：`simulated_lidar_cloud.py`
4. 启动融合节点进程。
   - 第 175-191 行：`ExecuteProcess` 拉起 `ros2_lidar_depth_fusion_node.py`

### 3.2 规划器最终从哪里接收融合结果

文件：`src/planner/plan_manage/launch/advanced_param.launch.py`

- 第 109 行：`('grid_map/cloud', ['drone_', drone_id, '_', cloud_topic])`

当 `cloud_topic=fusion/fused_cloud` 时，EGO-Planner 的 `grid_map/cloud` 实际订阅：

- `/drone_<id>_fusion/fused_cloud`

这就是融合结果进入局部占据地图（occupancy / inflate）的入口。

### 3.3 融合节点核心逻辑（逐函数）

文件：`src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py`

1. 参数与运行阈值（第 57-73 行）

   - 输入 topic、输出 topic、体素分辨率、z 高度裁剪、最大范围、同步窗口、概率阈值等。
2. 时间同步（第 100-108 行）

   - `message_filters.ApproximateTimeSynchronizer`
   - `queue_size=sync_queue`，`slop=sync_slop`
   - 融合回调入口：`sync_callback`
3. 同步回调（第 124-147 行）

   - 点云解码：`cloud_to_xyz()`（第 149-156 行）
   - 基础过滤：`filter_points()`（第 158-170 行）
   - 融合主流程：`fuse_clouds()`（第 216-251 行）
   - 发布输出：`xyz_to_cloud()`（第 253-257 行）
   - 调试统计输出：`depth_only / lidar_only / dual`
4. 证据融合策略（第 172-251 行）

   - 深度概率模型：`depth_probability()`（近场权重大）
   - LiDAR 概率模型：`lidar_probability()`（中远场权重大）
   - `accumulate_voxels()` 把两路点投到同一体素格，累加 log-odds 证据
   - `fuse_clouds()` 按 `min_probability`、`min_hits` 进行体素保留判定

> 当前版本是“体素证据级融合”，不是直接拼接两路点云。

### 3.4 为什么加 `force_zero_stamp`

文件：`src/planner/plan_manage/scripts/simulated_lidar_cloud.py`

- 第 33 行：增加参数 `force_zero_stamp`
- 第 145-156 行：发布时若开启则强制时间戳为 0

对应启动文件：

- `single_run_in_sim_fusion.launch.py` 第 170 行：`force_zero_stamp=True`

原因：当前仿真路径下 depth 云时间戳为 0，若 LiDAR 用真实时钟，`ApproximateTimeSynchronizer` 很难配对，融合回调触发频率会明显下降。

## 4. 当前版本“坐标变换”现状

当前仿真里两路点云都在 `world` 下发布（launch 第 160 行 `frame_id=world`，融合输出第 184 行 `output_frame=world`），因此没有单独 TF 外参求解流程。

这意味着：

- 仿真可跑通；
- 真机接入时必须补齐 `camera_frame -> body -> lidar_frame -> world` 的 TF 变换与时间对齐。

## 5. 真机接入时建议改动点（TODO）

1. 在融合节点增加 TF2 变换层

   - 在 `sync_callback()` 内，对两路点云先做 `doTransform` 到统一坐标系再 `filter_points()`/`fuse_clouds()`。
   - TODO：标定并固化 `T_body_camera`、`T_body_lidar`。
2. 将“仿真时间戳 hack”替换为真实时间同步

   - 关闭 `force_zero_stamp`；
   - 使用硬件时间戳 + 触发同步（或更小 `sync_slop` + 延迟补偿）。
3. 若转为 C++ 高性能实现

   - 保持 topic 契约不变（输入两路云 + 里程计，输出 `/fusion_cloud`）；
   - 用 PCL + `pcl::VoxelGrid` + `tf2_ros::Buffer` 复现同样模块边界，避免影响上层 planner。

## 6. 如何确认“确实在做多模态融合”

1. 看融合节点日志（`publish_debug_stats_every`）

   - 持续出现 `dual > 0`，表示同一帧体素同时被 depth 和 lidar 观测到。
2. 看话题链路

   - `ros2 topic echo --once /drone_0_fusion/fused_cloud --field width`
   - `ros2 topic echo --once /drone_0_grid/grid_map/occupancy_inflate --field width`
3. RViz 同时显示三路点云

   - 深度：`/drone_0_pcl_render_node/cloud`
   - LiDAR：`/drone_0_lidar/points`
   - 融合：`/drone_0_fusion/fused_cloud`

## 7. 关键调参建议（先保证可飞再提性能）

- 同步：`sync_slop`（第 72/105 行），先保证回调稳定触发。
- 过滤：`z_min/z_max/max_range`（第 63-66 行），减少离群点。
- 融合偏好：`near_field_radius/depth_decay/lidar_growth`（第 66-68 行）。
- 占据判定：`min_probability/min_hits`（第 69-70 行），控制漏检与误检平衡。

建议流程：先固定 `resolution` 与同步，再调概率模型，最后调 `min_probability`。
