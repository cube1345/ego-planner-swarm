# 一种面向 EGO-Planner 的深度相机与 LiDAR 多模态融合局部感知方法

## 摘要

针对四旋翼无人机在未知障碍环境中的局部避障问题，本文基于当前 ROS2 版 EGO-Planner 工程实现，提出并实现了一种深度相机与 LiDAR 的多模态几何融合方法。该方法以深度点云和 LiDAR 点云作为输入，首先通过近似时间同步保证两路观测在同一局部时刻内对齐；随后在统一世界坐标系下完成点云裁剪与体素化表达；最后采用距离自适应的概率占据证据融合策略，对近场和中远场观测赋予不同权重，在体素级别累积多源占据证据，输出可直接送入 EGO-Planner 局部地图构建模块的融合点云。与单一深度点云或单一 LiDAR 点云方案相比，当前仿真统计结果表明，该方法在平均 Recall 和平均 F1 指标上均获得稳定增益，其中相对于最优单模态的平均 Recall 增益为 0.0571，平均 F1 增益为 0.0841，且在当前评估窗口内正增益比例达到 100%。实验结果说明，该方法能够在保持工程实现简洁性的前提下，提高局部障碍感知的完整性与鲁棒性。

## 关键词

无人机避障；EGO-Planner；多模态融合；深度相机；LiDAR；体素占据融合；ROS2

## 1. 引言

在基于局部地图的无人机避障系统中，感知模块的可靠性直接决定了轨迹优化结果的可行性与安全性。单目深度相机或深度渲染点云通常具有较高的近距离空间分辨率，但在远距离、边缘区域及遮挡条件下容易出现观测不完整的问题；LiDAR 具有更稳定的几何测距能力，但在稀疏采样、近距离密集结构表达方面又存在局限。因此，将二者进行合理融合，是提升 EGO-Planner 局部地图质量的一条务实路径。

当前工程并未采用端到端学习式融合或全局状态估计融合，而是围绕局部占据表达，设计了一套轻量化、实时友好的几何证据融合方法。该设计的目标不是替代 EGO-Planner 本身的轨迹优化逻辑，而是为其 `grid_map/cloud` 输入提供更稳定、更完整的障碍观测。

## 2. 系统总体架构

当前仿真链路的多模态融合入口定义在 [single_run_in_sim_fusion.launch.py](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py)。当 `use_fusion=True` 时，规划器输入点云话题由原始深度点云切换为融合点云，其中 `cloud_topic` 被设置为 `fusion/fused_cloud`，见 [single_run_in_sim_fusion.launch.py:32](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py:32)。随后该话题通过 [advanced_param.launch.py:108](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/advanced_param.launch.py:108) 到 [advanced_param.launch.py:109](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/advanced_param.launch.py:109) 接入 EGO-Planner 的 `grid_map/cloud`。

系统中的两路感知输入分别为：

- 深度点云：`/drone_0_pcl_render_node/cloud`
- 模拟 LiDAR 点云：`/drone_0_lidar/points`

融合节点由 [single_run_in_sim_fusion.launch.py:175](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py:175) 到 [single_run_in_sim_fusion.launch.py:190](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py:190) 启动，实际执行脚本为 [ros2_lidar_depth_fusion_node.py](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py)。在仿真条件下，两路点云统一工作在 `world` 坐标系，因此当前版本未引入显式 TF 外参解算。

## 3. 多模态融合方法

在当前工程中，所采用的融合算法并非简单点云拼接，也不是基于滤波器的状态融合，而是一种面向局部障碍表达的“距离自适应体素占据证据融合”方法。其基本思想是：将深度相机与 LiDAR 对同一空间体素的占据判断视为两类独立观测证据，并根据观测距离对二者赋予不同可信度，再通过 log-odds 累积方式获得体素最终占据概率。该设计直接服务于 EGO-Planner 的局部地图构建模块，因此关注的核心目标不是全局环境重建，而是在局部规划尺度上获得更稳定、更完整的障碍几何表达。

### 3.1 问题建模

设深度点云观测集合为

$$
\mathcal{P}_d = \{ \mathbf{p}_i^d \in \mathbb{R}^3 \},
$$

LiDAR 点云观测集合为

$$
\mathcal{P}_l = \{ \mathbf{p}_j^l \in \mathbb{R}^3 \}.
$$

目标是在统一坐标系下构造融合后的占据体素集合

$$
\mathcal{V}_f = \{ \mathbf{v}_k \},
$$

并将其发布为融合点云，作为局部地图更新输入。

### 3.2 时间同步

融合节点使用 `ApproximateTimeSynchronizer` 对深度点云与 LiDAR 点云进行近似时间同步，见 [ros2_lidar_depth_fusion_node.py:100](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:100) 到 [ros2_lidar_depth_fusion_node.py:108](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:108)。当前默认参数为：

- `sync_queue = 10`
- `sync_slop = 0.08`

其设计意图是在实时性与同步成功率之间取得平衡。由于仿真链路中深度点云时间戳为零，系统在模拟 LiDAR 端引入了 `force_zero_stamp=True` 机制，以便触发同步回调，见 [single_run_in_sim_fusion.launch.py:168](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py:168) 到 [single_run_in_sim_fusion.launch.py:170](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py:170) 以及 [simulated_lidar_cloud.py:145](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/simulated_lidar_cloud.py:145) 到 [simulated_lidar_cloud.py:156](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/simulated_lidar_cloud.py:156)。

### 3.3 数据预处理

在进入融合前，两路点云均执行相同的基础预处理，见 [ros2_lidar_depth_fusion_node.py:158](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:158) 到 [ros2_lidar_depth_fusion_node.py:170](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:170)。其处理步骤包括：

1. 非有限值去除；
2. 垂向高度裁剪；
3. 基于无人机当前位置的最大感知半径裁剪。

在当前实现中，默认参数为：

- `z_min = -0.10`
- `z_max = 3.50`
- `max_range = 12.0`
- `resolution = 0.10`

该处理使后续融合只针对与局部规划相关的障碍观测执行，避免远距离点和异常点对局部占据判断造成污染。

### 3.4 距离自适应概率模型

当前融合方法的关键在于：并不将深度和 LiDAR 视作等权传感器，而是根据观测距离动态调整其占据证据强度。

对深度点云，当前实现采用如下近场偏置模型，见 [ros2_lidar_depth_fusion_node.py:172](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:172) 到 [ros2_lidar_depth_fusion_node.py:176](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:176)：

$$
p_d(r) = \mathrm{clip}\left(0.25 + 0.55 e^{-r / \lambda_d} + \mathbb{I}(r \le r_n)\cdot 0.10 \right),
$$

其中：

- $r$ 为体素中心到当前无人机位置的距离；
- $\lambda_d$ 对应 `depth_decay`；
- $r_n$ 对应 `near_field_radius`。

对 LiDAR 点云，采用如下中远场增强模型，见 [ros2_lidar_depth_fusion_node.py:178](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:178) 到 [ros2_lidar_depth_fusion_node.py:182](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:182)：

$$
p_l(r) = \mathrm{clip}\left(0.35 + 0.45 \left(1 - e^{-r / \lambda_l}\right) + \mathbb{I}(r > r_n)\cdot 0.10 \right),
$$

其中 $\lambda_l$ 对应 `lidar_growth`。

该设计体现了工程上的直观假设：近场结构更信任深度相机，远场稀疏几何更信任 LiDAR。

从方法本质上看，这一步决定了当前融合算法区别于“等权融合”的关键特征。若对两类传感器赋予完全相同的证据强度，则无法反映深度相机在近场密集观测中的优势，也无法体现 LiDAR 在中远场测距稳定性上的优势。当前工程所采用的距离自适应策略，本质上是在不引入复杂学习模型的前提下，将传感器物理特性直接编码进融合权重函数之中。

### 3.5 体素证据融合

预处理后，点云按照体素分辨率进行量化，见 [ros2_lidar_depth_fusion_node.py:192](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:192) 到 [ros2_lidar_depth_fusion_node.py:205](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:205)。对于每个体素，先统计命中次数 $n_k$，再构造命中增强项：

$$
w_k = \min(\log(1+n_k), 1.6).
$$

接着把概率映射为 log-odds 形式：

$$
\ell_k = \log \frac{p_k}{1-p_k},
$$

并得到该模态对体素的证据贡献：

$$
\Delta L_k = w_k \cdot \ell_k.
$$

深度与 LiDAR 的证据在同一体素内累加为：

$$
L_k = \sum \Delta L_k^{(d)} + \sum \Delta L_k^{(l)}.
$$

最终利用 sigmoid 函数恢复为占据概率：

$$
P_k = \sigma(L_k)=\frac{1}{1+e^{-L_k}}.
$$

若满足

$$
P_k \ge P_{\min}, \quad n_k \ge n_{\min},
$$

则该体素被保留为融合障碍体素，见 [ros2_lidar_depth_fusion_node.py:226](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:226) 到 [ros2_lidar_depth_fusion_node.py:251](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py:251)。

因此，当前方法本质上是一种“距离自适应的体素级概率占据证据融合”，可视为 OctoMap 风格 log-odds 思路在双模态局部障碍感知中的工程化实现。

进一步地，可以将当前融合算法概括为如下三条核心原则：

1. 统一空间表达：所有观测首先投影到相同体素网格中，避免直接在原始点级别做不稳定的数据拼接。
2. 区分传感器优势：通过距离自适应概率函数，让深度相机主导近场，让 LiDAR 主导中远场。
3. 以占据证据为融合对象：输出的不是“原始点并集”，而是经过概率判别后的高置信障碍体素集合。

这意味着当前算法的最终输出更适合作为 EGO-Planner 的 `grid_map/cloud` 输入，因为它直接对应规划所需的局部障碍占据表达，而不是视觉重建意义上的稠密场景模型。

### 3.6 当前融合算法的重点概括

为了便于在论文中快速说明，当前融合算法可以概括为：

“一种基于 log-odds 的深度自适应双模态体素占据融合方法。”

其要点包括：

- 融合对象是“障碍占据证据”，不是原始点云坐标本身。
- 融合层级是“体素层”，不是像素层，也不是轨迹层。
- 权重依据是“距离”，不是人工固定常数。
- 输出目标是“供局部规划直接使用的融合障碍点云”。

若进一步压缩为一句工程化表述，则可写为：

“该方法通过对深度点云和 LiDAR 点云在统一体素网格中进行距离自适应的概率证据累积，生成高置信障碍体素集合，从而提升 EGO-Planner 局部地图输入的完整性与鲁棒性。”

## 4. 仿真 LiDAR 的构造与意义

为了在当前仿真环境中得到与真实 LiDAR 类似的几何观测，系统额外实现了 [simulated_lidar_cloud.py](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/simulated_lidar_cloud.py)。该节点从全局障碍云中，根据无人机当前位置和航向，模拟 LiDAR 的：

- 最大量程约束；
- 水平视场裁剪；
- 垂直视场裁剪；
- 随机稀疏采样；
- 体素级去重。

其关键实现位于 [simulated_lidar_cloud.py:95](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/simulated_lidar_cloud.py:95) 到 [simulated_lidar_cloud.py:143](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/scripts/simulated_lidar_cloud.py:143)。该节点的存在使得当前多模态融合方案在不改变规划框架的前提下，能够形成“深度相机 + LiDAR”的双几何传感器输入。

## 5. 与 EGO-Planner 的接口关系

融合节点输出的话题为 `/drone_0_fusion/fused_cloud`，对应 `world` 坐标系，见 [single_run_in_sim_fusion.launch.py:183](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py:183) 到 [single_run_in_sim_fusion.launch.py:184](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py:184)。在规划器启动配置中，该融合输出被接到 `grid_map/cloud`，见 [advanced_param.launch.py:108](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/advanced_param.launch.py:108) 到 [advanced_param.launch.py:109](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/src/planner/plan_manage/launch/advanced_param.launch.py:109)。

因此，融合模块与 EGO-Planner 的关系不是“直接修改轨迹优化器”，而是“提高局部占据输入质量”。从系统职责上看，这是一种感知层增强而非规划层重构。

## 6. 实验设置与结果

### 6.1 实验设置

当前实验基于工程内仿真结果文件：

- 融合收益统计：[fusion_benefit_summary.json](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/artifacts/fusion_metrics/plots/fusion_benefit_summary.json)
- 环境统计：[sim_stats_summary.json](/home/cube/WorkSpace/ROS/Ego_Planner/ego-planner-swarm/artifacts/sim_stats/sim_stats_summary.json)

环境障碍规模统计如下：

- 全局障碍点数：152503
- 全局障碍体素数：37881
- 障碍簇估计数：157
- 局部膨胀占据图峰值点数：31547

### 6.2 融合收益

在 72 帧融合评估窗口内，当前方法取得如下平均结果：

| 指标 | Depth | LiDAR | Fusion |
|---|---:|---:|---:|
| Recall | 0.1108 | 0.1371 | 0.1942 |
| F1 | 0.1994 | 0.2412 | 0.3253 |

相对于最佳单模态，融合结果的提升为：

- 平均 Recall 增益：`+0.0571`
- 平均 F1 增益：`+0.0841`
- Recall 正增益比例：`100%`
- F1 正增益比例：`100%`

这说明当前距离自适应证据融合方法在当前仿真设置下，不仅优于单独深度相机，也优于单独 LiDAR，并且这种优势具有稳定性，而非偶然出现在少数帧中。

### 6.3 结果分析

从结果上看，深度点云在近场密集结构表达方面仍有价值，但其单独使用时 Recall 和 F1 均明显低于融合结果；LiDAR 作为单模态时表现优于深度点云，但融合结果依然进一步提升，表明两种传感器在体素层面具有互补性。

需要指出的是，当前收益主要体现在局部障碍观测完整性上，而不是直接体现在轨迹性能指标上。换言之，本文方法首先改善的是“地图输入质量”，进而为后续规划性能提升创造条件。

## 7. 讨论

### 7.1 当前方法的优点

当前方法具有以下工程优势：

- 实现轻量，能够直接运行在现有 ROS2 EGO-Planner 工程中；
- 不依赖深度学习模型或额外训练数据；
- 输出保持为标准 `PointCloud2`，便于与现有 `grid_map/cloud` 接口兼容；
- 参数含义清晰，便于按近场、远场和噪声水平进行调节。

### 7.2 当前方法的局限性

该方法仍存在以下局限：

- 当前仿真默认两路点云均在 `world` 坐标系下，未体现真实系统中的外参与时延误差；
- 融合策略只使用几何信息，未引入语义可信度；
- 融合结果本质上仍是占据点云，尚未扩展为更丰富的置信地图或 ESDF 置信权重；
- 当前统计结果主要验证感知质量增益，尚未系统量化轨迹平滑性、最小安全距离与任务完成时间。

## 8. 结论

本文围绕当前 ROS2 EGO-Planner 工程，介绍并总结了一种已实际接入仿真链路的多模态融合方法。该方法采用近似时间同步、统一坐标系下的点云预处理，以及基于 log-odds 的距离自适应体素证据融合，将深度相机与 LiDAR 的优势结合为单一融合障碍点云，再输入 EGO-Planner 的局部地图模块。

现有实验结果表明，该方法在当前工程环境中可稳定提高障碍感知的 Recall 和 F1 指标，并具有明确的工程可扩展性。对于后续工作，可进一步引入真实 TF 外参、时间延迟补偿、C++ 高性能实现，以及语义信息与 ESDF 置信建模，从而提升该方法在真实无人机平台中的实用价值。
