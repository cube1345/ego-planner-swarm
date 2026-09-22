# 项目目标与当前状态

## 项目定位

本项目基于 ROS 2 Humble 和 EGO-Planner，目标是在仿真环境中构建一套无人机闭环避障系统，并在原有深度相机和 LiDAR 融合基础上，引入 mmWave radar，形成三模态感知融合链路。

项目核心不是单独做一个传感器节点，而是让感知结果稳定进入 EGO-Planner 的 `grid_map`，最终影响局部轨迹规划和避障行为。

## 总体目标

1. 建立可运行的闭环避障仿真链路。
2. 将 depth cloud、LiDAR cloud、mmWave radar cloud 统一融合为占据点云。
3. 用融合点云驱动 EGO-Planner 的局部地图和轨迹规划。
4. 建立动态障碍物场景下的评价指标和调参闭环。
5. 让项目知识沉淀为可持续维护的文档，而不是只存在于临时对话中。

## 当前已完成

- 主 ROS 包名确认：`ego_planner`。
  - `src/planner/plan_manage/package.xml` 中 `<name>ego_planner</name>`。
  - `src/planner/plan_manage/CMakeLists.txt` 中 `project(ego_planner)`。
- 主入口 launch：`src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py`。
- 已有 depth cloud 和 LiDAR cloud 融合节点：
  - `src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py`
- 已新增 mmWave radar 仿真节点：
  - `src/planner/plan_manage/scripts/simulated_mmwave_radar_cloud.py`
- 主 launch 已接入 radar 仿真节点：
  - `simulated_mmwave_radar_node`
  - 输出 `/drone_0_radar/points`
- fusion 节点已接收 radar：
  - 参数 `radar_cloud_topic`
  - 最新帧缓存 `latest_radar_points`
  - 回调 `radar_cloud_callback`
  - 新鲜度过滤 `fresh_radar_points`
- fusion 输出已验证非空：
  - `/drone_0_radar/points`：用户验证 `width = 2354`
  - `/drone_0_fusion/fused_cloud`：用户验证 `width = 12199`
- radar 贡献统计已进入 debug log：
  - `radar_points`
  - `radar_only_voxels`
  - `depth_radar_voxels`
  - `lidar_radar_voxels`
  - `tri_modal_voxels`
  - `multi_modal_voxels`
- 最近一次日志验证显示 radar 已实际参与融合：
  - `radar_pts=2276`
  - `depth_radar=264`
  - `lidar_radar=126`
  - `tri_modal=103`
  - `multi=1737`

## 当前待完成

1. 将 radar 贡献统计写入 `/drone_0_fusion/ds_metrics`。
2. 对比二模态和三模态的指标变化。
3. 继续优化动态避障参数和 fusion 参数。
4. 整理更完整的实验表，包括配置、指标、结论。

## 当前已知环境问题

当前 zsh 环境应使用：

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

不要在 zsh 中使用：

```zsh
source install/setup.bash
```

曾遇到 `drone_detect` 安装产物不完整或链接失败问题。当前三模态避障主线不依赖 `drone_detect`，但如果全工作区 source 或全量 build 被它阻塞，需要单独处理该包。

曾遇到 Homebrew linker 污染导致 C++ 链接失败。若 `ld` 路径指向 `/home/linuxbrew/.linuxbrew/.../ld` 并出现大量 `libgdal/libcurl/opencv/armadillo` undefined reference，应使用干净 `PATH` 构建。
