# 运行、验证与排错手册

## 基本环境

当前环境：

```text
Ubuntu 22.04
ROS 2 Humble
shell: zsh
```

在 zsh 中推荐使用：

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

不要在 zsh 中使用：

```zsh
source install/setup.bash
```

如果必须用 bash，则新开 bash 后使用：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 构建 ego_planner 包

只构建主包：

```zsh
source /opt/ros/humble/setup.zsh
colcon build --packages-select ego_planner --symlink-install
```

如果构建时出现 Homebrew linker 污染，例如日志中出现：

```text
/home/linuxbrew/.linuxbrew/Cellar/binutils/.../bin/ld
undefined reference to ...
```

用干净 `PATH` 构建：

```bash
bash --noprofile --norc -lc '
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
unset LIBRARY_PATH CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH
source /opt/ros/humble/setup.bash
colcon build --packages-select ego_planner --symlink-install
'
```

如果遇到全工作区构建被 `drone_detect` 阻塞，可先跳过该包：

```zsh
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-skip drone_detect
```

## 启动三模态融合仿真

终端 1：

```zsh
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch ego_planner single_run_in_sim_fusion.launch.py use_fusion:=True
```

如果运行环境不能写默认 ROS 日志目录，指定日志目录：

```zsh
export ROS_LOG_DIR=/tmp/ego_planner_ros_logs
```

如果 CycloneDDS 报网卡或 UDP 枚举问题，可尝试切换 RMW：

```zsh
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

组合启动示例：

```zsh
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
export ROS_LOG_DIR=/tmp/ego_planner_ros_logs
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch ego_planner single_run_in_sim_fusion.launch.py use_fusion:=True
```

启用动态障碍物：

```zsh
ros2 launch ego_planner single_run_in_sim_fusion.launch.py use_fusion:=True dynamic_obstacles_enable:=True
```

## 检查 radar 发布端

终端 2：

```zsh
cd /home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 topic list | grep radar
```

期望看到：

```text
/drone_0_radar/points
```

检查点云宽度：

```zsh
ros2 topic echo --once /drone_0_radar/points --field width
```

如果 width 大于 0，说明 radar 发布端正常。

## 检查 fusion 输出

```zsh
ros2 topic echo --once /drone_0_fusion/fused_cloud --field width
```

如果 width 大于 0，说明 fusion 输出非空。

检查 D-S metrics：

```zsh
ros2 topic echo --once /drone_0_fusion/ds_metrics
```

## 检查 topic 类型

```zsh
ros2 topic info /drone_0_radar/points
ros2 topic info /drone_0_fusion/fused_cloud
```

期望类型：

```text
sensor_msgs/msg/PointCloud2
```

## 常见问题

### source install/setup.bash 报错

现象：

```text
not found: ".../local_setup.bash"
```

原因：当前 shell 是 zsh，却 source 了 bash setup；或者旧 overlay 残留导致路径链混乱。

处理：

```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```

必要时新开一个干净终端再执行。

### source install/setup.zsh 报 drone_detect 缺 local_setup.zsh

现象：

```text
not found: ".../install/drone_detect/share/drone_detect/local_setup.zsh"
```

原因：`drone_detect` 安装产物不完整或该包构建失败。

当前主线不依赖 `drone_detect`，可以先跳过该包完成三模态避障验证。若需要彻底修复，需要单独处理 `drone_detect` 的链接依赖。

### drone_detect 链接失败

曾观察到链接错误：

```text
undefined reference to curl_*@CURL_OPENSSL_4
```

这通常和 `libgdal`、`libnetcdf`、`libcurl` 的系统/Conda 库混用有关。该问题不属于当前三模态 fusion 主线，建议单独记录和修复。

### ros2 topic 在受限环境中无法监视

在某些工具沙箱或受限环境中，`ros2 topic list/echo` 可能因为 socket 权限失败：

```text
PermissionError: [Errno 1] Operation not permitted
Error creating socket: Operation not permitted
```

这不一定代表仿真节点失败。可以先看 launch 日志中是否出现：

```text
simulated mmwave radar ready
Fusion node ready
fusion frame=... radar_pts=... depth_radar=... lidar_radar=... tri_modal=...
```

如果这些日志存在，说明节点内部链路已经在运行。

### radar topic 存在但 width 为 0

排查顺序：

1. `/map_generator/global_cloud` 是否有输出。
2. `/drone_0_visual_slam/odom` 是否有输出。
3. `simulated_mmwave_radar_cloud.py` 是否打印 ready 日志。
4. radar 的 FOV、range、height 过滤是否过窄。
5. `dynamic_obstacles_enable` 关闭时，动态点云为空是正常的，但静态地图仍应产生 radar 点。

### fusion 输出为空

排查顺序：

1. depth cloud 是否有点。
2. LiDAR cloud 是否有点。
3. fusion 节点是否启动。
4. `min_probability` 和 `min_hits` 是否过严。
5. `z_min/z_max` 和 `max_range` 是否过滤掉了大部分点。
6. radar-only 是否被 `radar_requires_geometry_support=True` gate 掉，这是预期行为。

## 修改后验证清单

每次修改 Python 节点或 launch 后：

```zsh
python3 -m py_compile \
  src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py \
  src/planner/plan_manage/scripts/simulated_mmwave_radar_cloud.py \
  src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py
```

构建：

```zsh
source /opt/ros/humble/setup.zsh
colcon build --packages-select ego_planner --symlink-install
```

运行后检查：

```zsh
ros2 topic list | grep -E 'radar|fusion|lidar|pcl_render'
ros2 topic echo --once /drone_0_radar/points --field width
ros2 topic echo --once /drone_0_fusion/fused_cloud --field width
```
