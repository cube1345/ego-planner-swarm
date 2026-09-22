# 进度日志

## 2026-06-21

### 建立公共知识库

新增目录：

```text
docs/project_knowledge_base/
```

目的：

- 记录项目目标、架构、算法、运行手册和当前进度。
- 后续每次代码改动、参数调整、仿真验证都应同步更新。

### 三模态融合链路接通

当前三模态：

```text
Depth camera cloud
LiDAR cloud
mmWave radar cloud
```

新增或接入的关键文件：

```text
src/planner/plan_manage/scripts/simulated_mmwave_radar_cloud.py
src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py
src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py
src/planner/plan_manage/CMakeLists.txt
```

已确认：

- `simulated_mmwave_radar_cloud.py` 已安装到 `ego_planner` 包。
- 主 launch 已定义 `simulated_mmwave_radar_node`。
- 主 launch 已把 `radar_cloud_topic` 传给 fusion 节点。
- fusion 节点已缓存最新 radar 点云，而不是把 radar 加入三路同步。

用户验证结果：

```text
ros2 topic list | grep radar
-> /drone_0_radar/points
```

```text
ros2 topic echo --once /drone_0_radar/points --field width
-> 2354
```

```text
ros2 topic echo --once /drone_0_fusion/fused_cloud --field width
-> 12199
```

结论：

```text
radar 发布端已接通
fusion 输出非空
可以进入 radar 贡献统计与指标分析阶段
```

### 已修复的明显笔误

已修复：

- `self.self.radar_growth` -> `self.radar_growth`
- `rader` -> `radar`
- `belief_occuiped` -> `belief_occupied`
- `combined` 未定义问题
- `srcipts/simulated_mmwave_radar_cloud.py` -> `scripts/simulated_mmwave_radar_cloud.py`
- `auto_tune_parameters()` 中 `compute_fusion_output()` 调用补传 `radar_state`

验证：

```text
python3 -m py_compile 通过
colcon build --packages-select ego_planner --symlink-install 通过
```

### 当前下一步

补充 radar 贡献统计已完成 debug log 输出：

- `radar_points`
- `radar_only_voxels`
- `depth_radar_voxels`
- `lidar_radar_voxels`
- `tri_modal_voxels`
- `multi_modal_voxels`

已验证日志：

```text
fusion frame=40 depth_pts=5135 lidar_pts=6832 radar_pts=2276
fused_voxels=10527
depth_only=3524 lidar_only=5266 radar_only=0
depth_lidar=1244 depth_radar=264 lidar_radar=126 tri_modal=103
multi=1737 dual=1634 retention=0.856
min_prob=0.280 near_radius=4.00 lidar_growth=5.00
ds_occ=0.692 ds_unknown=0.193 ds_conflict=0.008
```

结论：

```text
radar 已实际参与融合统计。
radar_only=0 是预期现象，因为 radar_requires_geometry_support=True。
depth_radar/lidar_radar/tri_modal 均非 0，说明 radar 对融合结果有可观测贡献。
```

当前下一步：

```text
把 radar 贡献字段写入 /drone_0_fusion/ds_metrics，方便后续实验脚本记录。
```

### 本轮验证中遇到的环境问题

1. `colcon build --packages-select ego_planner --symlink-install` 曾因 Homebrew linker 污染失败。
   - 失败表现：`/home/linuxbrew/.linuxbrew/.../ld` 参与链接，并出现大量 `libgdal/libcurl/opencv/armadillo` undefined reference。
   - 处理：使用干净 `PATH` 后构建通过。
2. 工具沙箱中 `ros2 topic echo/list` 创建 DDS participant 或 ROS daemon socket 受限。
   - 处理：改用 launch 日志判断节点运行和 fusion debug 输出。
3. launch 在沙箱中默认写 `/home/cube/.ros/log` 会遇到只读限制。
   - 处理：设置 `ROS_LOG_DIR=/tmp/ego_planner_ros_logs`。
4. 默认 CycloneDDS 在沙箱中枚举 UDP 网卡失败。
   - 处理：使用 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，节点可继续启动并输出日志。

## 待补实验记录模板

每次实验建议按下面格式追加：

```text
日期：
分支：
commit：
启动命令：
关键参数：
场景：
指标：
结论：
下一步：
```
