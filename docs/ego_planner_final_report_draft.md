# 项目《基于多模态融合与 EGO-Planner 的无人机局部避障仿真系统》结题报告（草稿）

申请人：待填写  
指导教师：待填写  
项目编号：待填写  
所属学院：待填写  

## 第一章 立项背景与国内外研究现状

### 1.1 国内外研究现状

随着低空经济、智能巡检、无人配送和应急救援等应用场景的发展，无人机需要在复杂、未知、动态环境中具备稳定的自主避障能力。传统无人机避障系统通常依赖单一视觉、超声波或激光测距传感器，在简单环境中能够完成基础避障，但在光照变化、弱纹理区域、动态障碍物、窄通道和高密度障碍场景下容易出现感知不稳定、局部地图缺失和轨迹重规划频繁等问题。

在无人机局部规划方向，EGO-Planner 以 ESDF-free 的局部轨迹优化框架为代表，通过前端路径搜索和后端 B-spline 轨迹优化实现快速局部避障。相比依赖全局欧式符号距离场的方法，该类方法减少了距离场维护开销，更适合实时局部重规划任务。与此同时，多模态感知融合逐渐成为无人机避障系统的重要方向。深度相机能够提供较稠密的近场几何信息，但在远距离和退化视觉条件下容易不稳定；LiDAR 点云通常空间结构更稳定，但点云稀疏、视场和密度有限。将两类点云统一到体素地图中进行概率占据建模，可以在近场细节和远场稳定性之间取得更好的平衡。

当前工程实践中，围绕 ROS 2 的无人机避障系统通常需要同时解决数据链路、局部地图、规划器输入、可视化和自动化评测等问题。仅完成单一算法模块并不足以说明系统可用性，必须构建从感知输入、地图更新、轨迹规划、轨迹执行到指标反馈的完整闭环，并通过 RViz 和 headless 仿真验证不同参数与策略的影响。

### 1.2 研究意义

无人机局部避障系统的核心问题不只是“能否规划一条路径”，还包括感知输入是否稳定、局部地图是否及时、轨迹是否平滑、动态障碍物是否能触发有效重规划，以及不同融合策略是否能被客观评测。针对这些问题，本项目基于 ROS 2 Humble、C++17 和 Python3，对 EGO-Planner 进行工程化集成与扩展，构建了面向仿真验证的无人机局部避障闭环系统。

本项目的研究意义主要体现在以下方面：

1. 在保留 EGO-Planner 轻量化局部规划优势的基础上，引入深度点云与模拟 LiDAR 点云的融合输入，提升局部地图对复杂障碍环境的描述能力。
2. 通过体素级 Log-odds 证据累积和距离自适应权重，增强近场与远场障碍物观测的平衡性。
3. 通过在线候选参数评估机制，将融合参数调节从经验手动调参转化为可度量的候选评分选择过程。
4. 通过动态障碍物注入、RViz 分色显示和 headless 批量评测，形成可复现、可观察、可对比的实验流程。

### 1.3 本项目主要创新点

本项目围绕“多模态感知融合 + EGO-Planner 局部规划 + 自动化评测”开展，主要工作如下。

1. 构建了 ROS 2 下无人机局部避障闭环仿真系统  
   系统包含环境点云生成、深度相机点云渲染、模拟 LiDAR 点云生成、多模态融合、Occupancy Grid Map 局部建图、A* 前端搜索、B-spline 后端轨迹优化、轨迹执行与 RViz/headless 评测链路。各模块通过 ROS 2 topic 和 launch remap 连接，形成可运行的完整闭环。

2. 实现了深度点云与模拟 LiDAR 点云的体素级几何融合  
   融合节点订阅 `/drone_0_pcl_render_node/cloud` 与 `/drone_0_lidar/points`，通过时间同步、空间裁剪、体素化和 Log-odds 证据累积生成 `/drone_0_fusion/fused_cloud`，并将其作为 EGO-Planner 的 `grid_map/cloud` 输入。

3. 引入在线自适应融合参数优化机制  
   围绕 `min_probability`、`min_hits` 和 `dual_bonus` 构造候选集合，根据局部 GT 体素对融合结果进行评分，自动选择当前帧更合适的融合阈值与保留策略，提高不同场景下的融合适应性。

4. 增加 Dempster-Shafer 证据指标层用于融合质量诊断  
   在不替代主链路 Log-odds 输出的前提下，将 depth 与 LiDAR 的体素级证据映射为 `occupied/free/unknown` 质量，并发布 `belief_occupied_mean`、`unknown_mean` 和 `conflict_mean`，用于观测融合置信度、不确定性和传感器冲突。

5. 构建动态障碍物仿真与可视化评测机制  
   通过动态障碍物点云节点生成移动圆柱障碍物，并注入深度相机和模拟 LiDAR 感知链路；RViz 中使用不同颜色区分 depth、LiDAR、融合点云、膨胀栅格和动态障碍物，提高动态避障过程的可解释性。

## 第二章 多模态感知融合模型

本章介绍本项目中深度相机点云与模拟 LiDAR 点云的几何级融合方法。该方法不采用简单点云拼接，而是将两类传感器观测统一映射到体素网格，通过距离自适应概率模型生成占据证据，并以 Log-odds 方式进行累积，最终输出供 EGO-Planner 使用的融合点云。

### 2.1 数据来源与坐标统一

本项目仿真链路中的主要感知输入包括：

- 深度相机点云：`/drone_0_pcl_render_node/cloud`
- 模拟 LiDAR 点云：`/drone_0_lidar/points`
- 融合点云输出：`/drone_0_fusion/fused_cloud`
- D-S 证据指标输出：`/drone_0_fusion/ds_metrics`

深度点云由局部感知渲染节点根据地图和无人机里程计生成，模拟 LiDAR 点云由 `simulated_lidar_cloud.py` 根据全局点云、动态障碍物点云与无人机位姿裁剪生成。两类点云均在 `world` 坐标系下进行处理，融合节点通过近似时间同步机制获得同一时刻附近的 depth/LiDAR 点云帧。

### 2.2 点云预处理

融合节点首先对输入点云进行基础过滤，包括：

1. 剔除非有限值点；
2. 按 `z_min` 与 `z_max` 限制高度范围；
3. 按 `max_range` 限制无人机周围有效感知距离；
4. 将保留点映射到统一分辨率的体素网格。

该预处理过程减少了无效点和远距离噪声对局部地图的影响，也为后续体素级证据融合提供统一数据结构。

### 2.3 距离自适应观测权重

深度相机和 LiDAR 在不同距离上的优势不同。深度点云在近场通常更稠密，有利于刻画障碍物表面细节；模拟 LiDAR 点云在中远距离上更稳定，有利于提供较可靠的几何约束。因此，本项目在融合节点中分别构建 depth 与 LiDAR 的距离相关占据概率：

- depth 权重在近场较高，随距离增大逐渐衰减；
- LiDAR 权重随距离增长逐渐增强，并在超过近场半径后给予额外加权；
- 对同一体素，如果两类传感器均观测到，可通过 `dual_bonus` 提高其保留优先级。

该机制使融合结果不会简单偏向点数更多的深度点云，也不会完全依赖稀疏 LiDAR，而是在不同距离范围内动态调整传感器贡献。

### 2.4 体素级 Log-odds 证据融合

融合节点将每个体素作为基本融合单元，统计 depth 与 LiDAR 对该体素的命中情况，并基于观测概率计算 Log-odds 证据增量。对每个体素记录以下信息：

- 累积 Log-odds 证据；
- 命中次数；
- 是否由 depth 观测到；
- 是否由 LiDAR 观测到；
- D-S 证据指标所需的 occupied/free/unknown 质量。

最终，体素是否进入融合点云由以下条件共同决定：

1. 体素占据概率不低于当前 `min_probability`；
2. 体素命中次数不低于当前 `min_hits`；
3. 如 depth 与 LiDAR 同时命中，则可获得 `dual_bonus` 的保留加成。

融合输出以 PointCloud2 形式发布，并通过 launch remap 接入 EGO-Planner 的 `grid_map/cloud`。

### 2.5 Dempster-Shafer 证据指标层

为增强融合过程的可解释性，本项目在融合节点中加入 Dempster-Shafer evidence metrics。该层并不替代主链路的 Log-odds 融合结果，而是在每个体素上额外计算：

- `belief_occupied`：占据置信度；
- `belief_free`：空闲置信度；
- `unknown`：未知或证据不足程度；
- `conflict`：depth 与 LiDAR 证据冲突程度。

聚合后的在线指标通过 `/drone_0_fusion/ds_metrics` 发布。该指标可用于诊断融合质量，也可作为后续闭环参数优化的扩展反馈信号。

## 第三章 融合地图驱动的 EGO-Planner 局部规划

本章介绍融合点云如何驱动 EGO-Planner 完成局部建图、前端搜索、后端优化与轨迹执行。

### 3.1 系统规划框架

本项目沿用 EGO-Planner 的局部规划框架。无人机当前里程计由 `/drone_0_visual_slam/odom` 提供，融合点云进入 `grid_map/cloud` 后由 Occupancy Grid Map 构建局部占据地图。规划器在该地图上进行前端路径搜索和后端轨迹优化，并输出位置指令给执行层。

整体数据链路如下：

```text
depth cloud + simulated lidar cloud
        |
        v
voxel log-odds fusion
        |
        v
/drone_0_fusion/fused_cloud
        |
        v
Occupancy Grid Map
        |
        v
A* front-end + B-spline optimization
        |
        v
/drone_0_planning/pos_cmd
        |
        v
poscmd_2_odom / SO3 simulation
```

### 3.2 Occupancy Grid Map 局部建图

局部建图模块根据融合点云更新局部占据栅格，并发布膨胀后的占据点云 `/drone_0_grid/grid_map/occupancy_inflate`。膨胀栅格用于为规划器提供安全边界，避免轨迹过于贴近障碍物。

### 3.3 A* 前端路径搜索

前端 A* 在局部占据地图中搜索从当前状态到目标点的拓扑可行路径。融合点云提升了局部地图的障碍物完整性，使 A* 在稠密障碍场景下获得更稳定的初始路径。该路径随后作为 B-spline 后端优化的初始引导。

### 3.4 B-spline 后端轨迹优化

后端采用 B-spline 轨迹表示，并围绕平滑性、动力学可行性和碰撞约束进行优化。EGO-Planner 的核心优势在于不依赖全局 ESDF，而是通过局部地图与碰撞惩罚实现轻量化轨迹优化。最终输出的轨迹由 `traj_server` 转换为位置指令，再由仿真执行层更新无人机状态。

### 3.5 动态障碍物场景下的在线重规划

本项目新增动态障碍物点云节点，用于生成移动圆柱障碍物，并发布：

- `/drone_0_dynamic_obstacles/cloud`
- `/drone_0_dynamic_obstacles/markers`

动态障碍物点云被注入深度相机渲染链路和模拟 LiDAR 链路，使规划器能够在局部地图变化时触发重规划。当前动态避障并未引入新的规划算法，而是通过动态感知输入驱动 EGO-Planner 原有的在线重规划机制。

## 第四章 系统实现、可视化与评测

### 4.1 ROS 2 系统实现

本项目基于 ROS 2 Humble 构建，核心代码由 C++17 与 Python3 实现。主要模块包括：

- `pcl_render_node.cpp`：深度相机点云/深度图仿真；
- `simulated_lidar_cloud.py`：模拟 LiDAR 点云生成；
- `ros2_lidar_depth_fusion_node.py`：depth/LiDAR 几何融合与自适应参数选择；
- `dynamic_obstacle_cloud.py`：动态障碍物点云与 Marker 发布；
- `closed_loop_feedback_node.py`：闭环反馈指标采集；
- `compare_fusion.sh`：headless 批量评测入口；
- `sim_flight_stats_report.py`：轨迹、避障、重规划和地图指标统计。

### 4.2 RViz 可视化设计

RViz 配置文件 `drone0_clean.rviz` 对不同类型感知与规划结果进行分色显示：

- depth camera cloud：橙色；
- simulated LiDAR cloud：蓝色；
- fused cloud：紫色；
- occupancy inflate：绿色；
- dynamic obstacles：按障碍物编号分色；
- drone path 与 robot marker：显示无人机轨迹与当前位置。

通过该可视化配置，可以直接观察深度相机、LiDAR、融合地图和动态障碍物对规划过程的影响。

### 4.3 Headless 批量评测工具链

为避免只依赖 RViz 主观观察，本项目实现了 headless 批量评测工具链。`tools/compare_fusion.sh` 支持无人值守启动仿真、采集融合指标、统计轨迹执行指标，并导出 JSON/CSV/Markdown/SVG 等结果。

当前评测指标包括：

- `fusion_f1`
- `fusion_recall`
- `fusion_f1_gain_vs_best_single`
- `fusion_recall_gain_vs_best_single`
- `replan_count`
- `path_length_m`
- `accel_rms_mps2`
- `occupancy_jitter_ratio`
- `min_dynamic_obstacle_distance_m`
- `dynamic_safety_violation_ratio`

### 4.4 代表性实验结果

在自适应融合评测中，代表性结果显示：

- `fusion_f1` 达到 0.2893；
- `fusion_recall` 达到 0.1693；
- 相较最佳单传感器，`fusion_f1` 提升约 0.1000；
- 相较最佳单传感器，`fusion_recall` 提升约 0.0647；
- 闭环优化后，路径长度由 23.38 m 降至 23.06 m；
- 加速度 RMS 由 9.10 降至 6.28。

在四动态障碍物短时评测中，系统统计到：

- 动态障碍物点云最新点数：617；
- 重规划次数：98；
- 平均重规划间隔：0.304 s；
- 最近动态障碍距离：0.355 m；
- 动态安全违规比例：0；
- 动态碰撞风险分数：0。

结果表明，当前动态障碍物配置能够对规划器形成持续扰动，并触发在线重规划，同时在动态障碍物安全半径指标上保持可接受表现。

### 4.5 当前不足与后续优化方向

当前系统仍以仿真验证为主，距离真机部署仍需进一步完善：

1. 需要补充真实传感器外参标定、时间同步和 TF 树约束；
2. 需要接入 PX4/ArduPilot 或实际飞控桥接层，增加限幅、悬停和失效降级策略；
3. 动态障碍物模型仍较规则，后续可扩展非周期运动、突然出现/消失、速度变化和遮挡场景；
4. 当前 D-S evidence 主要用于诊断，后续可进一步纳入闭环参数优化目标；
5. 需要在多随机种子和长时段场景下统计成功率、碰撞率和到达率，提高实验结论稳定性。

## 第五章 总结

本项目基于 ROS 2 Humble、C++17 和 Python3，围绕 EGO-Planner 构建了无人机局部避障仿真系统，实现了从多模态感知、局部建图、轨迹规划、轨迹执行到 RViz/headless 评测的完整闭环。项目重点完成了深度相机点云与模拟 LiDAR 点云的体素级 Log-odds 融合、在线自适应融合参数选择、D-S 证据指标诊断、动态障碍物注入与分色可视化、以及自动化批量评测工具链。

实验结果表明，自适应融合相对最佳单传感器具有稳定正收益，动态障碍物场景能够触发 EGO-Planner 的在线重规划机制，headless 工具链能够对融合质量、轨迹执行、安全距离和地图稳定性进行统一评估。本项目为后续真机接入、飞控控制层适配和更复杂动态场景评测奠定了工程基础。

## 参考文献建议

[1] EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors.  
[2] EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments.  
[3] ROS 2 Humble Documentation.  
[4] OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees.  
[5] Dempster-Shafer Evidence Theory 相关不确定性建模文献。  

## 致谢

本项目的完成离不开指导教师、学院平台和团队成员的支持。在项目实施过程中，围绕 ROS 2 仿真环境搭建、EGO-Planner 代码阅读、多模态点云融合、动态障碍物建模、RViz 可视化和 headless 自动化评测等内容进行了持续调试与验证。感谢指导教师在项目方向、技术路线和报告撰写方面给予的指导，感谢学院提供的实验环境与学习资源，感谢团队成员在代码实现、实验验证和文档整理过程中的协作与支持。

