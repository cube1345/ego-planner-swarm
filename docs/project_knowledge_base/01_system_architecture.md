# 系统架构与节点关系

## 主入口

当前推荐从下面的 launch 文件看完整链路：

```text
src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py
```

它负责启动地图、规划、轨迹执行、仿真、传感器模拟、融合和闭环反馈。

## 闭环数据流

```text
random_forest / mockamap
    -> /map_generator/global_cloud

pcl_render_node
    -> /drone_0_pcl_render_node/cloud
    -> /drone_0_pcl_render_node/depth

simulated_lidar_cloud.py
    -> /drone_0_lidar/points

simulated_mmwave_radar_cloud.py
    -> /drone_0_radar/points

ros2_lidar_depth_fusion_node.py
    subscribes:
        /drone_0_pcl_render_node/cloud
        /drone_0_lidar/points
        /drone_0_radar/points
        /drone_0_visual_slam/odom
        /map_generator/global_cloud
    publishes:
        /drone_0_fusion/fused_cloud
        /drone_0_fusion/ds_metrics

EGO-Planner grid_map
    uses fused cloud as obstacle input

ego_planner_node
    -> B-spline trajectory

traj_server
    -> /drone_0_planning/pos_cmd

simulator
    -> /drone_0_visual_slam/odom

odom feedback returns to planner and sensor simulators
```

## 核心节点说明

### random_forest / mockamap

地图源。`random_forest` 生成随机森林障碍环境；`mockamap` 在开关启用时使用。它们共同服务于后续感知仿真和指标评价，关键输出是：

```text
/map_generator/global_cloud
```

### pcl_render_node

模拟深度相机。输入全局地图和无人机 odom，根据当前视角生成局部深度点云和深度图。

主要输出：

```text
/drone_0_pcl_render_node/cloud
/drone_0_pcl_render_node/depth
```

### simulated_lidar_cloud.py

模拟 LiDAR 点云。它根据全局地图、动态障碍物点云和 odom，生成稀疏但中远距离较稳定的 LiDAR 点云。

主要输出：

```text
/drone_0_lidar/points
```

### simulated_mmwave_radar_cloud.py

模拟毫米波雷达点云。它从全局地图和动态障碍物中筛选雷达视场内的点，加入测距/横向噪声，并进行 voxel downsample。当前用于提供第三模态。

主要输出：

```text
/drone_0_radar/points
```

### ros2_lidar_depth_fusion_node.py

几何融合核心节点。它将 depth、LiDAR、radar 的点云转成 voxel evidence，使用概率和 Dempster-Shafer evidence 思路输出融合点云。

主要输出：

```text
/drone_0_fusion/fused_cloud
/drone_0_fusion/ds_metrics
```

### ego_planner_node

局部规划主节点。它读取 odom、目标点和 grid_map 中的障碍信息，进行 ESDF-Free 的局部轨迹优化，输出 B-spline 轨迹。

### traj_server

轨迹执行中间层。它订阅 planner 输出的 B-spline 轨迹，按时间采样并发布位置控制命令。

### simulator

仿真无人机运动。关闭动力学模式时常用 `poscmd_2_odom` 直接将位置指令转成 odom；打开动力学模式时使用更接近真实动力学的 SO3 控制和四旋翼模拟器。

### closed_loop_feedback_node.py

闭环评价节点。它综合路径长度、重规划强度、collision risk、occupancy jitter、平滑性、D-S unknown/conflict、动态障碍距离等指标，给 fusion/规划参数优化提供反馈。

## 设计原则

- 感知融合节点输出统一点云，避免侵入 EGO-Planner 核心代码。
- launch 层负责 topic 接线和参数传递。
- depth/LiDAR 用 ApproximateTimeSynchronizer 同步，radar 使用最新帧缓存，避免三路同步卡住。
- radar 默认作为辅助证据，不应让单独 radar 噪声直接支配障碍物发布。
