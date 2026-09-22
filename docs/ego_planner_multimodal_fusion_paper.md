# 基于多模态证据融合与在线自适应参数优化的 EGO-Planner 无人机局部避障系统研究

## 摘要

EGO-Planner 是四旋翼无人机局部避障领域中具有代表性的 ESDF-free 轨迹优化方法。其核心优势在于不依赖完整全局欧式符号距离场，而是通过局部占据地图、前端路径搜索和 B-spline 轨迹优化实现实时重规划。原始 EGO-Planner 工程通常以单一深度点云作为局部建图输入，在仿真和实际部署中具有较好的实时性，但单传感器链路也存在明显局限：深度相机在远距离、弱纹理、遮挡、反光或稀疏障碍物场景中容易产生观测缺失；单一局部点云输入也难以表达不同传感器之间的置信度差异和冲突关系。针对上述问题，本项目在保留 EGO-Planner 原有规划框架的基础上，围绕 ROS 2 Humble 构建了一个包含深度相机点云、模拟 LiDAR 点云、体素级概率证据融合、Dempster-Shafer 证据指标、自适应融合参数、闭环反馈评测和 RViz/headless 自动化验证的无人机局部避障系统。

本文重点分析本项目相对于原 EGO-Planner 的优化设计。首先，项目新增 `simulated_lidar_cloud.py` 与 `ros2_lidar_depth_fusion_node.py`，将深度点云 `/drone_0_pcl_render_node/cloud` 与模拟 LiDAR 点云 `/drone_0_lidar/points` 在 `world` 坐标系下进行同步、裁剪、体素化和 Log-odds 证据累积，输出 `/drone_0_fusion/fused_cloud` 并通过 launch remap 接入 EGO-Planner 的 `grid_map/cloud`。其次，项目引入距离自适应传感器置信模型，在近场提高 depth 权重，在中远场提高 LiDAR 权重，从而避免简单点云拼接导致的噪声扩散或单模态偏置。再次，项目实现 Dempster-Shafer evidence metrics，对每个体素估计 occupied、free、unknown 和 conflict 质量，并通过 `/drone_0_fusion/ds_metrics` 发布在线诊断指标，使融合质量从“可视化观察”扩展为“可解释度量”。最后，项目围绕 `min_probability`、`min_hits` 和 `dual_bonus` 构建在线候选参数搜索机制，并通过局部 GT 体素、F1、Recall、EMA 平滑和 D-S 惩罚项实现融合层自适应优化。实验数据表明，多模态融合链路在多个 batch 中相对最佳单传感器获得正向 F1 与 Recall 增益；`dual_bonus` 长时段评测中使 `fusion_recall` 从 `0.166402` 提升到 `0.169008`，`fusion_f1` 从 `0.285162` 提升到 `0.288916`；近期默认参数评测中 `fusion_recall_gain_vs_best_single=0.014841`，`fusion_f1_gain_vs_best_single=0.023261`。同时，本文也指出闭环 sliding-window optimizer 当前默认关闭，原因是完整 batch 结果显示其开启后长期指标下降，因此其工程定位应为实验功能而非默认收益点。

关键词：EGO-Planner；无人机避障；多模态融合；深度点云；LiDAR；Dempster-Shafer 证据理论；在线自适应；ROS 2；局部规划

## 1 引言

无人机自主避障系统通常由感知、建图、规划和控制四个环节组成。在静态、结构简单且传感器条件良好的环境中，单一深度相机或单一 LiDAR 输入往往能够支撑基础避障任务。但当环境中存在复杂障碍物、稀疏结构、遮挡、远距离障碍、传感器噪声或局部地图不稳定时，规划器的表现不仅取决于轨迹优化算法本身，也强烈依赖上游感知输入的完整性和稳定性。对 EGO-Planner 这类 ESDF-free 局部规划器而言，这一点尤为明显：规划器不维护高成本全局 ESDF，而是依赖局部占据地图和局部梯度优化。因此，若 `grid_map/cloud` 输入缺失关键障碍点、包含过多噪声，或在连续帧之间剧烈抖动，规划器就可能出现路径贴障、重规划频繁、轨迹不稳定甚至局部失败。

原始 EGO-Planner 的工程重点在于快速局部规划。它通过局部地图、A* 前端搜索和 B-spline 后端优化，在较低计算开销下实现四旋翼实时避障。该设计对于资源受限平台具有重要价值，但其标准仿真链路通常以单一深度感知输入为主，缺少对不同传感器互补性的建模。深度相机具有近场稠密、结构细节丰富的优势，但远距离深度噪声和视场边缘误差较明显；LiDAR 或雷达类传感器通常距离测量更稳定，但点云稀疏、分辨率有限，近距离表面细节可能不如深度图丰富。因此，将 depth 与 LiDAR 在几何层融合，可以在保留 EGO-Planner 轻量规划优势的同时，提高局部地图输入质量。

本项目并没有重写 EGO-Planner 的核心优化器，而是选择从工程系统角度增强其上游感知与评测链路。这个选择具有明确的工程动机：一方面，EGO-Planner 的规划框架已经具备成熟的实时重规划能力，直接改动其优化器可能引入大量不可控因素；另一方面，在实际无人机系统中，许多规划异常并不是后端优化器本身导致，而是感知点云不稳定、局部地图输入不一致、参数阈值不适配场景造成。因此，本项目优先优化 `grid_map/cloud` 的输入质量，并建立自动化指标评测体系，使优化结果能够被量化验证。

本文围绕当前仓库中的实际代码和实验数据展开，不把未验证设计写成已验证结论。项目当前已默认启用的核心优化包括：多模态几何融合、体素级 Log-odds 证据累积、在线 `min_probability`、在线 `min_hits`、在线 `dual_bonus`、D-S evidence metrics 和闭环反馈观测。项目当前保留但不默认启用的实验功能包括：closed-loop optimizer、过宽 near-field radius 自适应、adaptive lidar growth、adaptive z_max、adaptive depth_decay 等。这样的区分有助于保证论文结论与工程事实一致。

## 2 原 EGO-Planner 链路及其局限

### 2.1 原规划框架

EGO-Planner 的主要思想是使用局部占据地图和轨迹优化替代高成本 ESDF 维护。典型链路可以概括为：

```text
depth cloud
    -> local occupancy grid map
    -> front-end path search
    -> B-spline trajectory optimization
    -> position command
    -> simulator / controller
```

在本仓库的原版单传感器仿真中，规划节点通过 launch remap 将 `grid_map/cloud` 接到 `pcl_render_node/cloud`。也就是说，局部建图主要由深度相机渲染点云驱动。其优点是链路简单、延迟低、调试直接；缺点是感知输入的鲁棒性完全依赖单一路径。当深度点云局部缺失时，规划器可能低估障碍物；当深度点云噪声较大时，局部占据地图可能出现抖动，进而触发不必要重规划。

### 2.2 单传感器输入的工程问题

原链路在工程上至少存在四类问题。第一，单一 depth cloud 的有效距离有限，在远距离障碍物或稀疏障碍物附近，点云密度不足会降低局部地图对未来路径的预警能力。第二，深度相机对几何边界、遮挡边缘和视角变化较敏感，障碍物体素可能在连续帧中出现“忽有忽无”的现象。第三，原链路缺少传感器置信度建模，不区分近场高可信深度点、远场低可信深度点和 LiDAR 稳定测距点。第四，原链路缺少融合质量诊断指标，工程调参主要依赖 RViz 观察和最终轨迹表现，难以判断问题来自感知、地图还是规划。

这些问题决定了本项目的优化重点不是简单增加一个传感器 topic，而是建立一套能够被 EGO-Planner 直接使用、能够评估质量、能够在线调参的融合输入层。

## 3 系统总体设计

### 3.1 改造目标

本项目对原 EGO-Planner 的改造目标可以概括为五点：

1. 在不破坏原规划器核心接口的前提下，将单一 depth cloud 输入升级为 depth + LiDAR 多模态融合输入。
2. 使用体素级概率证据模型，而不是直接拼接两路点云。
3. 用 Dempster-Shafer 证据理论补充不确定性和冲突度诊断。
4. 将融合阈值从固定经验参数升级为在线候选参数选择。
5. 建立 RViz 与 headless 双评测链路，让融合收益能够被可视化和批量指标共同验证。

### 3.2 当前系统数据流

当前融合版仿真的核心数据流如下：

```text
/map_generator/global_cloud
        |
        +--> pcl_render_node -----------------> /drone_0_pcl_render_node/cloud
        |
        +--> simulated_lidar_cloud.py --------> /drone_0_lidar/points

/drone_0_pcl_render_node/cloud
/drone_0_lidar/points
        |
        v
ros2_lidar_depth_fusion_node.py
        |
        +--> /drone_0_fusion/fused_cloud
        +--> /drone_0_fusion/ds_metrics
        |
        v
EGO-Planner grid_map/cloud
        |
        v
occupancy_inflate -> A* -> B-spline optimization -> pos_cmd -> odom
```

该设计的重要特征是：融合层通过 launch remap 接入原规划器输入，而不是修改 EGO-Planner 内部规划接口。这样做的优点是保持算法模块边界清晰，降低引入新 bug 的概率，也便于在 `--mode plain` 与 `--mode fusion` 之间做 A/B 对照。

### 3.3 关键文件与模块

当前项目中与本文分析直接相关的关键文件包括：

- `src/planner/plan_manage/launch/single_run_in_sim.launch.py`：原版单传感器仿真入口。
- `src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py`：融合版仿真入口。
- `src/planner/plan_manage/launch/advanced_param.launch.py`：EGO-Planner 规划和地图参数入口。
- `src/planner/plan_manage/launch/simulator.launch.py`：仿真、里程计、可视化和深度渲染节点入口。
- `src/planner/plan_manage/scripts/simulated_lidar_cloud.py`：模拟 LiDAR 点云生成。
- `src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py`：多模态融合、自适应参数和 D-S 指标核心节点。
- `src/planner/plan_manage/scripts/closed_loop_feedback_node.py`：闭环反馈指标节点。
- `tools/compare_fusion.sh`：headless 批量评测入口。
- `tools/sim_flight_stats_report.py`：飞行统计、规划质量、安全距离和地图抖动统计。
- `tools/start_rviz_sim.sh`：一键启动/清理 RViz 仿真脚本。

这些文件构成了从仿真启动、感知输入、融合输出、规划执行到指标采集的完整链路。

## 4 多模态几何融合方法

### 4.1 为什么不是简单点云拼接

将 depth cloud 与 LiDAR cloud 直接做并集是最容易实现的融合方式，但它并不能解决真正的融合问题。直接拼接会把两类传感器的噪声、重复点和尺度差异全部交给后端地图处理，可能导致局部地图点数膨胀、障碍物边界变厚、远场噪声进入规划范围。更重要的是，直接拼接无法表达“同一体素被两类传感器同时支持”和“同一体素只有单一传感器弱支持”之间的差异。

因此，本项目采用体素级证据融合。融合节点先将两路点云投影到统一分辨率体素网格中，再对每个体素累计概率证据。每个体素都记录命中次数、depth 是否观测到、LiDAR 是否观测到、Log-odds 累积值以及 D-S 证据质量。最终是否输出该体素，不由原始点数直接决定，而由占据概率、命中次数和双传感器一致性共同决定。

为便于后续推导，定义两路输入点云分别为：

```text
P_d = {p_i^d | p_i^d in R^3}
P_l = {p_j^l | p_j^l in R^3}
```

其中 `P_d` 表示深度相机点云，`P_l` 表示模拟 LiDAR 点云。融合节点的目标不是求简单并集 `P_d union P_l`，而是估计体素集合 `V` 上的占据概率：

```text
p_occ(v) = P(v is occupied | P_d, P_l, x_uav)
```

随后根据占据概率、命中次数和跨模态一致性筛选输出点云：

```text
P_f = {c(v) | v in V, p_eff(v) >= tau, h(v) >= h_min}
```

其中 `c(v)` 是体素中心，`p_eff(v)` 是加入双传感器一致性奖励后的有效概率，`tau` 对应当前 `current_min_probability`，`h_min` 对应当前 `current_min_hits`。

|     符号     |               含义               | 代码/参数对应                      |
| :----------: | :------------------------------: | ---------------------------------- |
|   `P_d`   |       depth cloud 输入点集       | `/drone_0_pcl_render_node/cloud` |
|   `P_l`   |       LiDAR cloud 输入点集       | `/drone_0_lidar/points`          |
|   `P_f`   |           融合输出点集           | `/drone_0_fusion/fused_cloud`    |
|   `p_i`   |            单个三维点            | `PointCloud2` 中的 `(x,y,z)`   |
|    `v`    |             体素索引             | `floor(p / resolution)`          |
|   `c(v)`   |             体素中心             | `(v + 0.5) * resolution`         |
|   `r(v)`   |         体素到无人机距离         | `norm(c(v)-x_uav)`               |
| `p_occ(v)` |           体素占据概率           | `sigmoid(logit_sum)`             |
| `p_eff(v)` | 加入 `dual_bonus` 后的有效概率 | `effective_probability()`        |
|   `h(v)`   |           体素命中次数           | `VoxelEvidence.hits`             |
|   `tau`   |           输出概率阈值           | `current_min_probability`        |
|  `h_min`  |           输出命中阈值           | `current_min_hits`               |
|    `b`    |        双传感器一致性奖励        | `current_dual_bonus`             |

### 4.2 点云预处理

融合节点的预处理主要由 `cloud_to_xyz()` 和 `filter_points()` 完成。处理步骤包括：

1. 从 `PointCloud2` 中读取 `x,y,z` 字段并过滤 NaN。
2. 根据 `z_min` 与当前 `z_max` 过滤高度范围。
3. 根据无人机当前位置和 `max_range` 过滤有效感知半径。
4. 将剩余点映射到分辨率为 `resolution` 的体素网格。

这一预处理环节看似简单，但对稳定性非常关键。高度裁剪可以避免地面以下或过高点云污染局部地图；距离裁剪可以减少远场低质量点对规划器的干扰；体素化可以将不同密度的点云统一到相同空间尺度，防止深度点云因点数多而天然压制 LiDAR。

预处理可以形式化表示为一个过滤算子 `F(·)`。对任意输入点 `p=(x,y,z)`，若满足：

```text
isfinite(p) = true
z_min <= z <= z_max
||p - x_uav||_2 <= R_max
```

则该点被保留，否则被剔除。过滤后的点被映射到体素索引：

```text
v(p) = floor(p / rho)
```

其中 `rho` 是体素分辨率，对应代码中的 `resolution`。同一体素内可能包含多个原始点，融合节点只保留唯一体素索引和体素内点数 `n(v)`，从而把“点级别处理”转化为“体素级别处理”。这样做有三个好处：一是降低点云规模，二是削弱不同传感器点密度差异，三是让后续概率证据、D-S 证据和命中次数都可以在统一空间单元上定义。

### 4.3 距离自适应传感器置信模型

项目在 `depth_probability()` 和 `lidar_probability()` 中显式表达两类传感器的距离特性。depth 的基础概率随距离指数衰减，并在近场半径内获得额外加成；LiDAR 的基础概率随距离增长而提升，并在超过近场半径后获得远场加成。其工程含义如下：

- 近距离区域更依赖 depth，因为深度点云密集，能够提供障碍物表面细节。
- 中远距离区域更依赖 LiDAR，因为测距更稳定，更适合提前发现路径前方障碍。
- 同一体素被两类传感器同时观测时，后续 `dual_bonus` 会提高其有效概率。

这种设计不是追求理论上最复杂的传感器模型，而是在 EGO-Planner 仿真链路中采用足够轻量、可解释、可在线运行的概率近似。

从代码实现看，depth 与 LiDAR 的距离相关概率可以抽象为：

```text
p_d(r) = clamp(0.25 + 0.55 * exp(-r / lambda_d) + I(r <= R_near) * beta_d)
```

```text
p_l(r) = clamp(0.35 + 0.45 * (1 - exp(-r / lambda_l)) + I(r > R_near) * beta_l)
```

其中 `lambda_d` 对应 `current_depth_decay`，`lambda_l` 对应 `current_lidar_growth`，`R_near` 对应 `current_near_field_radius`，`beta_d` 与 `beta_l` 是近场/远场加成项。当前实现中，depth 近场加成约为 `0.10`，LiDAR 远场加成约为 `0.10`。`clamp(·)` 用于把概率限制在 `(eps, 1-eps)` 区间，避免后续 Log-odds 计算出现无穷值。

该模型体现了一个分段信任策略：当 `r` 较小时，`exp(-r/lambda_d)` 较大，depth 的占据证据更强；当 `r` 增大时，LiDAR 的 `1-exp(-r/lambda_l)` 项逐渐增大，LiDAR 对中远场障碍的贡献更明显。相比固定权重融合，该模型使传感器权重随无人机与障碍物距离变化，更贴近无人机局部避障任务的实际需求。

### 4.4 Log-odds 证据累积

融合节点将传感器概率转换为 Log-odds 增量，并以体素为单位进行累积。对于每个体素，节点维护 `VoxelEvidence` 数据结构，其中包括：

- `logit_sum`：累积 Log-odds 证据；
- `hits`：有效命中次数；
- `seen_depth`：是否被 depth 观测到；
- `seen_lidar`：是否被 LiDAR 观测到；
- depth 与 LiDAR 各自的 occupied/free/unknown 质量。

点云密度差异通过 `hit_boost = min(log1p(counts), 1.6)` 做有限增强。也就是说，一个体素内点越多，其证据会增强，但增强上限受到限制，避免密集 depth 点云无限放大。该处理在工程上很重要，因为 depth cloud 通常比模拟 LiDAR 更密，如果不限制密度优势，融合会退化成“深度点云主导”的伪融合。

设某体素 `v` 被传感器 `s` 观测到，其中 `s in {d,l}`。传感器给出的占据概率为 `p_s(v)`，则其 Log-odds 形式为：

```text
L_s(v) = log(p_s(v) / (1 - p_s(v)))
```

考虑同一体素内点数 `n_s(v)` 后，引入密度增强项：

```text
g_s(v) = min(log(1 + n_s(v)), 1.6)
```

因此传感器 `s` 对体素 `v` 的证据增量为：

```text
Delta L_s(v) = g_s(v) * L_s(v)
```

两路传感器的 Log-odds 证据累积为：

```text
L(v) = sum_{s in {d,l}} I_s(v) * Delta L_s(v)
```

其中 `I_s(v)` 表示体素是否被传感器 `s` 观测到。最终占据概率由 sigmoid 函数给出：

```text
p_occ(v) = sigma(L(v)) = 1 / (1 + exp(-L(v)))
```

这个过程等价于把两类传感器的占据证据在 logit 域相加。相比直接在概率域平均，Log-odds 累积更适合表达“多次独立观测会增强占据置信度”的思想，同时实现简单、计算开销低，适合在线 ROS 2 节点运行。

体素最终占据概率通过 sigmoid 从 `logit_sum` 映射得到。输出阶段调用 `select_fused_points()`，只有满足以下条件的体素才进入 `/drone_0_fusion/fused_cloud`：

```text
effective_probability >= current_min_probability
hits >= current_min_hits
```

其中 `effective_probability` 会考虑 `dual_bonus`。若体素同时被 depth 和 LiDAR 观测到，则在 logit 域增加一致性奖励，再映射回概率。该机制体现了一个直观假设：两种不同传感器独立观测到同一空间体素时，其作为真实障碍的可信度应高于单传感器孤立观测。

`dual_bonus` 的有效概率可写为：

```text
p_eff(v) = sigma(logit(p_occ(v)) + b * I_d(v) * I_l(v))
```

其中：

```text
logit(p) = log(p / (1 - p))
```

`b` 为 `current_dual_bonus`。如果体素只被单一传感器观测，则 `I_d(v)*I_l(v)=0`，有效概率退化为原始 `p_occ(v)`；如果两类传感器都观测到该体素，则在 logit 域增加 `b`。这种设计比直接给概率加常数更稳定，因为 logit 域加成不会轻易把概率推到非法范围，也能保持低概率和高概率区域的非线性差异。

最终输出条件可进一步写为：

```text
y(v) =
1, if p_eff(v) >= tau and h(v) >= h_min
0, otherwise
```

输出点云则由所有 `y(v)=1` 的体素中心组成。该判据把“概率置信”“观测支持次数”和“跨模态一致性”合并到一个轻量可解释的筛选规则中。

### 4.5 融合输出接入 EGO-Planner

融合节点输出仍然是标准 `sensor_msgs/PointCloud2`，因此 EGO-Planner 不需要知道上游是单传感器还是多传感器。融合版 launch 将规划器的 `grid_map/cloud` remap 到 `/drone_0_fusion/fused_cloud`，而原版 launch 则继续接 `/drone_0_pcl_render_node/cloud`。这种设计带来两个直接好处：第一，原 EGO-Planner 核心 C++ 规划逻辑可以保持稳定；第二，评测时可以用同一地图、同一目标点、同一规划器参数对比 plain 与 fusion 两条链路。

从系统接口角度看，EGO-Planner 接收到的仍然是：

```text
grid_map/cloud <- PointCloud2
```

区别只在于该 `PointCloud2` 的生成方式不同。plain 模式下：

```text
grid_map/cloud = P_d
```

fusion 模式下：

```text
grid_map/cloud = P_f = Phi(P_d, P_l, x_uav, theta)
```

其中 `Phi(·)` 表示本文的体素级融合算子，`theta` 表示融合参数集合：

```text
theta = {rho, R_max, R_near, lambda_d, lambda_l, tau, h_min, b}
```

这种接口保持策略使本项目的优化重点集中在感知输入质量，而不是侵入式修改规划器本体。对于工程维护而言，这意味着普通 EGO-Planner、融合 EGO-Planner、不同自适应参数配置之间可以通过 launch 参数切换，便于复现实验和定位问题。

## 5 Dempster-Shafer 证据指标层

### 5.1 引入 D-S 的原因

Log-odds 融合适合形成最终占据概率，但它并不直接告诉工程人员当前融合结果的不确定性来自哪里。例如，一个体素概率较低，可能是两类传感器都没有足够证据，也可能是 depth 认为占据而 LiDAR 认为空闲产生冲突。仅看最终概率难以区分这两类情况。Dempster-Shafer 证据理论的优势在于显式保留 unknown 和 conflict 信息，因此适合作为融合质量诊断层。

本项目中的 D-S 设计遵循一个重要边界：D-S 不是默认替代主链路 Log-odds 的输出层，而是作为指标层和自适应评分惩罚项使用。这样做既避免直接改变规划器输入导致不稳定，也为后续更复杂的证据融合提供可观测接口。

在本项目中，D-S 证据理论的辨识框架可以定义为：

```text
Omega = {O, F}
```

其中 `O` 表示体素被占据，`F` 表示体素为空闲。由于传感器可能无法给出充分判断，因此还需要考虑全集 `Omega` 本身，用于表示未知或不确定。对每个体素 `v` 和每个传感器 `s`，定义质量函数：

```text
m_s^v(O) + m_s^v(F) + m_s^v(Omega) = 1
```

其中 `m_s^v(O)` 是占据质量，`m_s^v(F)` 是空闲质量，`m_s^v(Omega)` 是未知质量。与概率不同，D-S 质量允许一部分信任分配给“不知道”，这正是它适合传感器融合诊断的原因。

| 符号             | 含义                                 | 工程对应                                        |
| ---------------- | ------------------------------------ | ----------------------------------------------- |
| `Omega`        | D-S 辨识框架                         | `{occupied, free}`                            |
| `O`            | 占据命题                             | 体素为障碍                                      |
| `F`            | 空闲命题                             | 体素为空闲                                      |
| `m_s^v(O)`     | 传感器 `s` 对体素 `v` 的占据质量 | `depth_occ_mass` / `lidar_occ_mass`         |
| `m_s^v(F)`     | 传感器 `s` 对体素 `v` 的空闲质量 | `depth_free_mass` / `lidar_free_mass`       |
| `m_s^v(Omega)` | 传感器 `s` 对体素 `v` 的未知质量 | `depth_unknown_mass` / `lidar_unknown_mass` |
| `K(v)`         | 两路传感器冲突度                     | `conflict`                                    |
| `Bel_v(O)`     | 组合后的占据 belief                  | `belief_occupied`                             |
| `Bel_v(F)`     | 组合后的空闲 belief                  | `belief_free`                                 |

### 5.2 体素质量分配

融合节点通过 `ds_mass_from_probability()` 将单传感器占据概率映射为三类质量：

```text
m(occupied), m(free), m(unknown)
```

其中 unknown 随概率降低而增加，并受到 `ds_unknown_floor` 约束；free 质量由 `ds_free_scale` 控制；occupied 质量由剩余质量得到。对于没有被某个传感器观测到的体素，该传感器质量被视为完全 unknown，即 `(0, 0, 1)`。这使得 D-S 指标能够区分“只有单传感器支持”和“两传感器共同支持”的体素。

代码中的质量分配可以抽象为：

```text
m_s^v(Omega) = clamp(u_0 + alpha_u * (1 - p_s(v)))
```

```text
m_s^v(F) = min(1 - m_s^v(Omega), gamma_f * (1 - p_s(v)))
```

```text
m_s^v(O) = 1 - m_s^v(F) - m_s^v(Omega)
```

其中 `u_0` 对应 `ds_unknown_floor`，`gamma_f` 对应 `ds_free_scale`，`alpha_u` 是 unknown 随低置信概率增加的比例系数。当前实现中，若某个体素没有被 depth 观测，则 depth 对该体素的质量为：

```text
m_d^v(O)=0, m_d^v(F)=0, m_d^v(Omega)=1
```

LiDAR 同理。这样，单传感器缺失不会被错误解释为“空闲证据”，而是被解释为“未知”。这对避障系统非常重要，因为未观测区域和明确空闲区域在安全意义上不同。

### 5.3 证据组合与冲突计算

对于同一体素，depth 与 LiDAR 的质量通过 `combine_ds_masses()` 组合。冲突项定义为：

```text
conflict = occ_a * free_b + free_a * occ_b
```

组合后的 occupied、free 和 unknown 会除以 `1 - conflict` 进行归一化。最终每个体素可得到：

- `belief_occupied`：占据置信度；
- `belief_free`：空闲置信度；
- `unknown`：证据不足程度；
- `conflict`：传感器冲突程度。

聚合后的在线 JSON 通过 `/drone_0_fusion/ds_metrics` 发布，字段包括 `belief_occupied_mean`、`unknown_mean`、`conflict_mean`、`fused_voxels` 和 `dual_voxels`。这些指标使工程人员能够判断融合输出是否处在高置信状态，是否存在大量未知体素，是否存在两路传感器冲突。

完整的 Dempster 组合规则可以写为：

```text
K(v) = m_d^v(O) * m_l^v(F) + m_d^v(F) * m_l^v(O)
```

```text
m^v(O) =
[m_d^v(O)m_l^v(O) + m_d^v(O)m_l^v(Omega) + m_d^v(Omega)m_l^v(O)]
/ [1 - K(v)]
```

```text
m^v(F) =
[m_d^v(F)m_l^v(F) + m_d^v(F)m_l^v(Omega) + m_d^v(Omega)m_l^v(F)]
/ [1 - K(v)]
```

```text
m^v(Omega) =
[m_d^v(Omega)m_l^v(Omega)]
/ [1 - K(v)]
```

其中 `K(v)` 越大，说明 depth 与 LiDAR 在该体素上越冲突。若 `K(v)` 接近 0，则两路传感器基本一致或至少不矛盾；若 `K(v)` 较高，则说明一方强烈支持占据而另一方强烈支持空闲，需要在调试时重点检查坐标系、时间同步、遮挡关系或传感器模型。

发布到 `/drone_0_fusion/ds_metrics` 的聚合量可以表示为：

```text
unknown_mean = (1 / |V_f|) * sum_{v in V_f} m^v(Omega)
```

```text
conflict_mean = (1 / |V_f|) * sum_{v in V_f} K(v)
```

```text
belief_occupied_mean = (1 / |V_f|) * sum_{v in V_f} m^v(O)
```

其中 `V_f` 是最终输出融合体素集合。通过这三个均值，可以快速判断当前融合输出是“高占据置信、低冲突”，还是“高未知、高冲突”的不稳定状态。

### 5.4 D-S 在自适应评分中的作用

当前项目还将 D-S 指标纳入候选参数评分惩罚项。`compute_ds_candidate_penalty()` 会对候选输出体素中的 unknown 和 conflict 求均值，并按照 `adaptive_ds_unknown_weight` 与 `adaptive_ds_conflict_weight` 形成轻量惩罚。其目的不是让 D-S 指标主导参数选择，而是在 F1/Recall 相近时倾向于选择不确定性和冲突度更低的候选组合。

候选参数 `theta_k` 生成预测体素集合 `V_k` 后，D-S 惩罚项可写为：

```text
J_DS(theta_k) =
w_u * min(1, mean_unknown(theta_k) / u_ref)
+ w_c * min(1, mean_conflict(theta_k) / c_ref)
```

其中 `w_u` 对应 `adaptive_ds_unknown_weight`，`w_c` 对应 `adaptive_ds_conflict_weight`，`u_ref` 对应 `adaptive_ds_unknown_ref`，`c_ref` 对应 `adaptive_ds_conflict_ref`。该惩罚项会从候选 utility 中扣除：

```text
utility'(theta_k) = utility(theta_k) - J_DS(theta_k)
```

因此，当两个候选在 F1/Recall 增益接近时，系统更倾向于选择 unknown 更低、conflict 更低的体素集合。

这一区分非常关键：D-S 在当前版本中是“解释性指标 + 轻量评分修正”，不是强行替代主融合概率模型。它的价值主要体现在可解释性、调试和后续闭环优化扩展上。

## 6 在线自适应参数优化

### 6.1 从经验调参到候选搜索

原始或普通融合链路中，`min_probability`、`min_hits` 等阈值通常由人工经验设置。固定阈值的问题在于，不同地图密度、不同无人机位置、不同传感器观测状态下，最佳阈值可能不同。阈值过低会保留噪声，导致局部地图膨胀和轨迹绕行；阈值过高会丢失障碍体素，导致规划器低估风险。

本项目将部分关键融合参数改造成在线候选搜索。当前默认真正在线自适应的参数有三个：

- `current_min_probability`
- `current_min_hits`
- `current_dual_bonus`

它们分别对应障碍接受概率阈值、体素命中次数门限和双传感器一致性奖励。代码中还保留了 `adaptive_lidar_growth`、`adaptive_z_max`、`adaptive_depth_decay`、`adaptive_near_field_radius` 等候选方向，但根据已有 batch 结果，这些方向不作为默认启用项。

在线自适应可以形式化为一个离散候选优化问题。设候选参数组合为：

```text
theta_k = (tau_k, h_k, b_k)
```

其中 `tau_k` 是概率阈值候选，`h_k` 是命中次数候选，`b_k` 是 dual bonus 候选。当前默认搜索空间为：

```text
Tau = {tau_min, tau_min + Delta_tau, ..., tau_max}
H = {1, 2, 3}
B = {b_min, b_min + Delta_b, ..., b_max}
```

候选集合为三者笛卡尔积：

```text
Theta = Tau x H x B
```

每一帧同步点云到达后，融合节点枚举 `theta_k in Theta`，生成候选输出体素集合 `V_k`，并计算它相对局部 GT 的指标。该过程本质上是在线离散搜索，而不是梯度优化。

| 符号        | 含义                       | 参数对应                                 |
| ----------- | -------------------------- | ---------------------------------------- |
| `theta_k` | 第 `k` 个候选参数组合    | `(threshold, min_hits, dual_bonus)`    |
| `Tau`     | 概率阈值候选集合           | `adaptive_min_probability_*`           |
| `H`       | 命中次数候选集合           | `adaptive_min_hits_*`                  |
| `B`       | 一致性奖励候选集合         | `adaptive_dual_bonus_*`                |
| `V_k`     | 候选参数生成的预测体素集合 | `predicted`                            |
| `G`       | 局部 GT 体素集合           | `/map_generator/global_cloud` 局部体素 |
| `S_k`     | 候选 EMA 得分              | `candidate_scores[candidate_key]`      |
| `alpha`   | EMA 更新系数               | `adaptive_score_alpha`                 |

### 6.2 局部 GT 体素监督

自适应参数选择需要一个评分标准。当前仿真中存在 `/map_generator/global_cloud`，因此融合节点可以在无人机附近 `adaptive_eval_range` 范围内提取局部 GT 体素集合。对于每个候选参数组合，节点生成预测障碍体素集合，并与局部 GT 体素计算 Precision、Recall 和 F1。

为了避免融合只是在绝对指标上看起来可用，项目采用“相对最佳单传感器”的收益作为核心评分依据。节点先分别计算 depth-only 和 LiDAR-only 的局部指标，再取二者中更好的 Recall 和 F1 作为 baseline。候选融合输出的收益为：

```text
gain_recall = candidate_recall - best_single_recall
gain_f1     = candidate_f1     - best_single_f1
```

随后定义局部 utility：

```text
utility = gain_f1 + 0.35 * gain_recall
```

这种评分方式体现了项目目标：融合不仅要“比某一个弱传感器好”，而要尽量优于当前局部条件下表现最好的单传感器。

具体地，设候选预测集合为 `V_k`，局部 GT 体素集合为 `G`。则：

```text
TP_k = |V_k intersection G|
FP_k = |V_k - G|
FN_k = |G - V_k|
```

Precision、Recall 和 F1 定义为：

```text
Precision_k = TP_k / (TP_k + FP_k)
```

```text
Recall_k = TP_k / (TP_k + FN_k)
```

```text
F1_k = 2 * Precision_k * Recall_k / (Precision_k + Recall_k)
```

为了衡量融合是否真正优于单传感器，先分别计算 depth-only 与 LiDAR-only 的指标：

```text
Recall_single = max(Recall_depth, Recall_lidar)
F1_single = max(F1_depth, F1_lidar)
```

候选收益为：

```text
Delta Recall_k = Recall_k - Recall_single
Delta F1_k = F1_k - F1_single
```

局部融合收益函数为：

```text
U_local(theta_k) = Delta F1_k + beta_r * Delta Recall_k - P_radius(theta_k)
```

其中 `beta_r=0.35`，对应当前代码中 Recall 增益权重；`P_radius(theta_k)` 是 near-field radius 候选偏离默认值时的轻量惩罚，在当前默认参数下 near-field radius 不参与在线启用，因此主要保留为实验扩展项。

### 6.3 EMA 平滑与切换抑制

在线参数如果每一帧都剧烈变化，会导致融合点云抖动，进而影响局部地图和轨迹规划。因此项目对每个候选组合维护 EMA 平滑得分：

```text
score_t = (1 - alpha) * score_{t-1} + alpha * utility_t
```

其中 `adaptive_score_alpha` 控制更新速度。若最优候选相对当前候选优势不足 `1e-4`，节点会保留当前参数组合，避免微弱得分差导致频繁切换。此外，候选并列时会选择距离当前参数更近的组合，进一步提高稳定性。

结合 D-S 惩罚和可选闭环奖励后，候选即时效用可以写为：

```text
U(theta_k) =
w_local * U_local(theta_k)
- J_DS(theta_k)
+ B_CL(theta_k)
+ B_retention(theta_k)
```

其中 `w_local` 对应 `closed_loop_local_score_weight`，默认强调局部融合收益；`J_DS` 是 D-S 不确定性/冲突惩罚；`B_CL` 是闭环候选历史得分奖励，当前 optimizer 默认关闭时为 0；`B_retention` 是保留率实验项，默认不作为主要收益点。

每个候选组合维护独立 EMA：

```text
S_k(t) =
U(theta_k, t), if S_k(t-1) is empty
(1 - alpha) * S_k(t-1) + alpha * U(theta_k, t), otherwise
```

最终选择：

```text
theta*(t) = argmax_{theta_k in Theta} S_k(t)
```

但为避免频繁切换，代码还设置了稳定门限。若当前候选 `theta_cur` 的得分与最优候选差距不足：

```text
S_{theta*}(t) - S_{theta_cur}(t) < epsilon
```

则保持当前参数不变。当前 `epsilon` 对应代码中的 `1e-4`。当多个候选得分几乎相同，系统选择与当前参数距离更近的候选：

```text
D(theta_k, theta_cur)
= |tau_k - tau_cur|
+ 0.25 * |h_k - h_cur|
+ 0.15 * |b_k - b_cur|
```

距离项的作用不是作为主要优化目标，而是在得分相等时抑制无意义切换。

### 6.4 `min_probability` 自适应

`min_probability` 控制体素进入融合点云所需的最低占据概率。当前默认候选区间通常由 `adaptive_min_probability_min=0.20`、`adaptive_min_probability_max=0.35` 和 `adaptive_min_probability_step=0.02` 控制。较低阈值可以提高召回率，但可能引入噪声；较高阈值可以提高保守性和精度，但可能漏掉弱观测障碍。在线自适应让系统根据局部 GT 与两路传感器状态在该区间内选择更合适阈值。

从优化角度看，`min_probability` 主要控制 Precision-Recall tradeoff：

```text
tau down -> |V_k| up -> Recall may increase, FP may increase
tau up   -> |V_k| down -> Precision may increase, FN may increase
```

因此它适合使用 `Delta F1 + beta_r * Delta Recall` 作为目标，而不是只优化 Recall。如果只优化 Recall，系统可能倾向于输出过多体素；如果只优化 Precision，则可能过度保守，漏掉弱障碍。

### 6.5 `min_hits` 自适应

`min_hits` 控制体素至少被命中多少次才保留。当前默认候选为 `1~3`。当场景中有效障碍较稀疏、单帧观测不足时，较低 `min_hits` 有利于保留障碍；当点云噪声较多或局部地图抖动明显时，较高 `min_hits` 有利于过滤偶然点。该参数与 `min_probability` 共同决定“保留更多障碍候选”和“抑制稀疏噪声”之间的折中。

`min_hits` 可以理解为对观测重复性的约束：

```text
h(v) = I_d(v) * support_d(v) + I_l(v) * support_l(v)
```

当一个体素内点数达到一定密度时，代码会增加 support hits。最终保留条件 `h(v) >= h_min` 会过滤单次偶然观测。相比 `min_probability`，`min_hits` 更偏向抑制稀疏噪声；两者联合搜索可以避免单独调阈值时出现“概率足够但支持太弱”或“支持较强但概率边界不足”的问题。

### 6.6 `dual_bonus` 自适应

`dual_bonus` 是本项目针对多模态一致性新增的重要参数。若一个体素同时被 depth 和 LiDAR 观测到，则其有效概率在 logit 域获得额外加成。该参数反映了“跨模态一致观测更可信”的思想。当前默认候选通常在 `0.0~0.2` 范围内搜索。

已有长时段评测显示，开启 `dual_bonus` 后，`fusion_recall` 从 `0.166402` 提升到 `0.169008`，`fusion_f1` 从 `0.285162` 提升到 `0.288916`，因此项目将其作为默认保留优化方向。近期 `stable_cl_dualonly_20260521` 数据中，融合相对最佳单传感器仍保持正收益：`fusion_recall_gain_vs_best_single=0.013175`，`fusion_f1_gain_vs_best_single=0.020647`。这说明双传感器一致性奖励在当前评测链路中具有可解释且可量化的积极意义。

## 7 感知-规划闭环反馈

### 7.1 为什么需要闭环指标

单纯优化融合 F1 和 Recall 并不必然带来更好的飞行轨迹。规划链路中还存在局部地图更新、膨胀半径、A* 搜索、B-spline 优化和位置指令执行等环节。一个融合参数可能提高局部 F1，但同时增加点云抖动，导致重规划频繁；也可能降低点云数量，却让轨迹更平滑、更短。因此，本项目进一步设计 closed-loop feedback，将感知结果与规划表现联系起来。

如果把融合层看作参数化感知前端，规划执行结果可以写成：

```text
T = Planner(Map(P_f(theta)), x_0, x_goal)
```

其中 `theta` 是融合参数，`P_f(theta)` 是融合点云，`Map(·)` 是局部占据地图更新，`Planner(·)` 是 EGO-Planner 的前端搜索与后端优化，`T` 是最终执行轨迹。由于 `theta` 对最终轨迹的影响经过了地图、规划和执行多个环节，单纯用当前帧 F1/Recall 优化 `theta` 只是在优化感知局部目标，并不一定优化飞行任务目标。

因此，理想目标应从局部感知指标扩展为感知-规划联合收益：

```text
J(theta) =
w_f * F1(theta)
+ w_r * Recall(theta)
- w_p * PathLength(theta)
- w_q * Replan(theta)
- w_c * CollisionRisk(theta)
- w_s * SmoothnessCost(theta)
- w_j * MapJitter(theta)
```

其中前两项鼓励融合质量，后五项约束路径、重规划、安全性、轨迹平滑度和地图稳定性。当前代码中的 `closed_loop_feedback_node.py` 已经实现该思想的工程近似，但为了稳定性，在线 optimizer 默认关闭，只把反馈作为观测和实验输入。

| 符号               | 含义              | 代码字段                          |
| ------------------ | ----------------- | --------------------------------- |
| `J(theta)`       | 感知-规划联合评分 | `closed_loop_score`             |
| `PathLength`     | 窗口路径长度      | `window_path_length_m`          |
| `Replan`         | 重规划代理计数    | `window_replan_proxy_count`     |
| `CollisionRisk`  | 碰撞风险          | `window_collision_risk_score`   |
| `SmoothnessCost` | 加速度 RMS 惩罚   | `window_accel_rms_mps2`         |
| `MapJitter`      | 占据地图抖动      | `window_occupancy_jitter_ratio` |
| `U_DS`           | D-S 未知惩罚      | `window_ds_unknown_mean`        |
| `C_DS`           | D-S 冲突惩罚      | `window_ds_conflict_mean`       |

### 7.2 闭环反馈节点

`closed_loop_feedback_node.py` 订阅 odom、occupancy、global cloud、dynamic obstacle cloud 和 D-S metrics，每秒发布一次 `/drone_0_fusion/closed_loop_feedback`。其窗口指标包括：

- `window_path_length_m`
- `window_replan_proxy_count`
- `window_collision_risk_score`
- `window_min_obstacle_distance_m`
- `window_safety_violation_ratio`
- `window_accel_rms_mps2`
- `window_occupancy_jitter_ratio`
- `window_ds_unknown_mean`
- `window_ds_conflict_mean`

节点将这些指标合成为闭环评分：

```text
score = -(w_path * path_penalty
        + w_replan * replan_penalty
        + w_collision * collision_risk
        + w_smoothness * smoothness_penalty
        + w_jitter * map_jitter
        + w_ds_unknown * ds_unknown_penalty
        + w_ds_conflict * ds_conflict_penalty)
```

得分越高表示综合代价越低。该指标不是直接控制无人机运动，而是用于评估参数是否改善了感知-规划联合表现。

闭环节点使用最近 `T_w` 秒滑动窗口统计，而不是使用从仿真开始到当前的累计值。设窗口为：

```text
W(t) = [t - T_w, t]
```

窗口路径长度为：

```text
L_W(t) = sum_{i in W(t)} ||x_i - x_{i-1}||_2
```

窗口重规划代理计数由占据地图点数变化事件近似：

```text
N_replan,W(t) = count(|N_occ(i)-N_occ(i-1)| / max(1,N_occ(i-1)) > eta)
```

窗口加速度 RMS 为：

```text
a_rms,W(t) = sqrt((1/N) * sum_{i in W(t)} a_i^2)
```

窗口地图抖动为：

```text
J_map,W(t) = std(N_occ,W) / mean(N_occ,W)
```

碰撞风险由窗口最近障碍距离 `d_min,W` 计算：

```text
R_collision,W(t) =
0, if d_min,W >= r_safe
(r_safe - d_min,W) / r_safe, otherwise
```

各项归一化后形成代价：

```text
C_W(t) =
w_path * clamp(L_W / L_ref)
+ w_replan * clamp(N_replan,W / N_ref)
+ w_collision * R_collision,W
+ w_smooth * clamp(a_rms,W / a_ref)
+ w_jitter * clamp(J_map,W / J_ref)
+ w_unknown * clamp(U_DS / U_ref)
+ w_conflict * clamp(C_DS / C_ref)
```

闭环评分定义为负代价：

```text
score_W(t) = -C_W(t)
```

并使用 EMA 平滑：

```text
score_ema(t) = (1 - alpha_cl) * score_ema(t-1) + alpha_cl * score_W(t)
```

这样，短时异常不会立即主导判断，而长期持续恶化会逐步反映到 `closed_loop_score_ema` 中。

### 7.3 延迟归因与默认关闭策略

项目曾尝试将 closed-loop score 直接加入候选参数选择，但完整 batch 结果显示直接或过强闭环归因会使长期融合指标下降。其原因在于参数选择、地图更新、规划重算和轨迹执行之间存在延迟，如果把当前飞行结果直接归因给刚刚切换的参数组合，就可能把旧轨迹或旧地图问题错误归因给新参数。

为解决该问题，当前代码引入 `closed_loop_action_delay_sec`，默认 3 秒。融合节点维护候选组合历史队列，将反馈归因到 `now - action_delay_sec` 时刻生效的参数组合。该机制已经完成 smoke 验证，但最新完整测试仍不支持将 closed-loop optimizer 默认启用。因此当前工程结论是：

- `closed_loop_feedback_node.py` 默认可作为指标观测节点；
- `closed_loop_feedback_enable=True` 可让融合节点接收反馈；
- `closed_loop_optimizer_enable=False` 是默认稳定策略；
- sliding-window optimizer 保留为实验功能，后续需要更多 batch 验证。

这种保守处理体现了项目对实验结论的边界控制：不是所有新增自适应方向都应写成正收益，只有经过指标验证的方向才进入默认链路。

延迟归因可以形式化描述如下。设融合参数组合历史为：

```text
H = {(t_1, theta_1), (t_2, theta_2), ..., (t_n, theta_n)}
```

当闭环反馈在时刻 `t` 到达时，不把该反馈归因给当前参数 `theta(t)`，而是归因给：

```text
t_attr = t - Delta_t
```

其中 `Delta_t` 对应 `closed_loop_action_delay_sec`。系统从历史队列中找到 `t_attr` 时刻生效的候选组合：

```text
theta_attr = theta(t_attr)
```

然后更新该候选组合的闭环 EMA：

```text
S_CL(theta_attr) =
(1 - alpha_candidate) * S_CL(theta_attr)
+ alpha_candidate * score_ema(t)
```

当 closed-loop optimizer 开启时，候选组合再次被枚举，会获得闭环奖励：

```text
B_CL(theta_k) = w_cl * S_CL(theta_k)
```

并进入总效用：

```text
U(theta_k) = w_local * U_local(theta_k) - J_DS(theta_k) + B_CL(theta_k)
```

当前完整 batch 表明该机制仍未稳定产生正收益，因此默认 `closed_loop_optimizer_enable=False`。这并不否定闭环反馈节点的价值，而是说明“可观测闭环指标”和“用闭环指标在线控制参数”是两个不同层级。前者已经有助于评估系统，后者还需要更严格的延迟建模、噪声抑制和长时段验证。

## 8 实验与指标分析

### 8.1 评测方法

项目提供 RViz 可视化和 headless 批量评测两种验证方式。RViz 主要用于观察无人机路径、规划线、占据地图和各类点云之间的空间关系；headless 评测则用于批量统计融合质量、路径长度、重规划次数、安全距离、轨迹平滑度、地图抖动和 D-S 指标。

一键启动脚本 `tools/start_rviz_sim.sh` 支持 `--mode plain` 与 `--mode fusion` 两种模式。plain 模式保持原 EGO-Planner 单传感器输入，fusion 模式启动 simulated LiDAR、融合节点、D-S 指标和闭环反馈节点。脚本在启动前会清理旧的 ROS/RViz 相关进程，避免多个仿真实例互相串话。

headless 评测主要由 `tools/compare_fusion.sh` 与 `tools/sim_flight_stats_report.py` 完成。前者负责无人值守启动仿真和生成融合指标，后者负责从仿真日志和 ROS topic 中统计轨迹、地图与安全相关指标。相比只看 RViz，headless 评测可以更客观地比较不同参数配置。

### 8.2 融合质量指标

融合质量主要使用以下指标：

- `fusion_recall`：融合点云覆盖局部 GT 障碍体素的比例；
- `fusion_f1`：融合输出在 precision 和 recall 之间的综合指标；
- `fusion_recall_gain_vs_best_single`：相对最佳单传感器 Recall 的增益；
- `fusion_f1_gain_vs_best_single`：相对最佳单传感器 F1 的增益。

这些指标的关键不是绝对值越高越好，而是融合结果是否稳定优于当前局部条件下表现最好的单一路径。项目中多次使用 `gain_vs_best_single` 作为核心判断依据，避免只与较弱单传感器比较造成虚假收益。

### 8.3 规划与安全指标

规划质量和安全性主要由以下指标描述：

- `path_length_m`：实际飞行路径长度；
- `replan_count`：规划器重规划次数；
- `avg_replan_interval_sec`：平均重规划间隔；
- `accel_rms_mps2`：轨迹执行加速度 RMS，近似反映平滑性；
- `min_obstacle_distance_m`：飞行过程中最近障碍距离；
- `safety_violation_ratio`：低于安全半径的采样比例；
- `occupancy_jitter_ratio`：占据地图点数抖动比例；
- `goal_reached_time_sec`：到点时间。

这些指标用于判断融合改进是否真正服务于规划任务。一个融合策略如果只提高点云数量，却导致 `occupancy_jitter_ratio` 或 `replan_count` 明显升高，就不能简单认为它有效。

### 8.4 代表性实验结果

从已有评测结果看，当前默认融合链路相对最佳单传感器具有正收益。`tune_default_20260522.summary.json` 中：

```text
fusion_recall = 0.065303
fusion_f1 = 0.118324
fusion_recall_gain_vs_best_single = 0.014841
fusion_f1_gain_vs_best_single = 0.023261
```

`tune_p24_30_db00_15_20260522.summary.json` 中：

```text
fusion_recall = 0.068125
fusion_f1 = 0.121576
fusion_recall_gain_vs_best_single = 0.016044
fusion_f1_gain_vs_best_single = 0.024680
```

`tune_p26_32_db00_15_20260522.summary.json` 中：

```text
fusion_recall = 0.058881
fusion_f1 = 0.102858
fusion_recall_gain_vs_best_single = 0.019048
fusion_f1_gain_vs_best_single = 0.029461
```

这些结果说明，在不同参数区间下，融合链路通常能够保持相对最佳单传感器的正向增益。虽然绝对 F1 和 Recall 会随地图、时长和参数变化而波动，但 `gain_vs_best_single` 的正值证明多模态融合不是简单增加点数，而是在当前评测定义下提高了局部障碍表达质量。

### 8.5 `dual_bonus` 的效果

`dual_bonus` 是最能体现多模态一致性价值的参数之一。根据已有长时段评测，开启后：

```text
fusion_recall: 0.166402 -> 0.169008
fusion_f1:     0.285162 -> 0.288916
```

该提升幅度并非数量级变化，但方向稳定且解释清楚：同时被 depth 和 LiDAR 命中的体素更可能是真实障碍，适当提高其有效概率有利于保留跨模态一致障碍。近期 `stable_cl_dualonly_20260521` 结果也显示：

```text
fusion_recall_gain_vs_best_single = 0.013175
fusion_f1_gain_vs_best_single = 0.020647
```

因此，`dual_bonus` 可以作为论文中的一个可靠优化点。它不是“万能参数”，但它把多模态融合从点云数量叠加推进到一致性证据利用。

### 8.6 D-S 指标的观测意义

`tune_default_20260522_sim_stats` 中 D-S 统计结果为：

```text
ds_unknown_mean = 0.188959
ds_conflict_mean = 0.003322
ds_belief_occupied_mean = 0.719590
```

该结果说明当前默认融合输出中，平均 occupied belief 较高，unknown 处于可观测但不极端的水平，conflict 较低。这符合当前仿真环境下 depth 与模拟 LiDAR 大多数情况下不强烈冲突的预期。D-S 指标的价值不在于单次结果本身，而在于它为后续诊断提供了入口：如果某次仿真中 `conflict_mean` 明显升高，应优先检查两路点云时序、坐标系或动态障碍注入；如果 `unknown_mean` 升高，则可能说明传感器覆盖不足或阈值过严。

### 8.7 动态障碍物仿真与安全指标扩展

在静态随机森林障碍物之外，本项目还扩展了动态障碍物仿真与评测能力。系统新增 `dynamic_obstacle_cloud.py` 节点，可生成多个周期运动的圆柱形障碍物，并同时发布动态点云 `/drone_0_dynamic_obstacles/cloud` 和 RViz 可视化 Marker `/drone_0_dynamic_obstacles/markers`。动态点云可以注入深度相机与模拟 LiDAR 感知链路，使局部占据地图随障碍物运动实时更新，从而触发 EGO-Planner 原有在线重规划机制。

在融合版仿真中，动态障碍物点云可进入多模态融合流程，并与静态环境点云共同影响 `/drone_0_fusion/fused_cloud` 和局部膨胀地图。RViz 中也提供动态障碍物显示，便于同时观察移动障碍物、融合点云、占据地图和无人机轨迹之间的空间关系。

评测层面，项目补充了动态安全指标，包括最近动态障碍距离、动态安全半径侵入次数、动态安全违规比例和动态碰撞风险分数。在代表性四动态障碍物测试 `dynamic_aggressiveness_4obs_20260516` 中，系统记录到：

```text
dynamic_obstacle_points_latest = 617
min_dynamic_obstacle_distance_m = 0.3551
safety_radius_m = 0.35
dynamic_safety_violation_count = 0
dynamic_safety_violation_ratio = 0.0
dynamic_collision_risk_score = 0.0
```

该结果说明，在该动态障碍物配置下，无人机相对动态障碍物保持了略高于安全半径的最近距离，未发生动态安全半径侵入，动态碰撞风险分数为 0。该扩展使项目不仅能评估静态障碍物避障，也能对移动障碍物场景下的感知输入、局部地图更新和在线重规划响应进行可视化观察与量化分析。

### 8.8 闭环 optimizer 的负面结果与工程取舍

实验数据也显示，并非所有更复杂的优化都会带来收益。`stable_cl_off_20260521.summary.json` 中：

```text
fusion_recall = 0.059327
fusion_f1 = 0.104665
fusion_recall_gain_vs_best_single = 0.018893
fusion_f1_gain_vs_best_single = 0.029318
```

而 `stable_cl_on_20260521.summary.json` 中：

```text
fusion_recall = 0.023169
fusion_f1 = 0.044271
fusion_recall_gain_vs_best_single = -0.009337
fusion_f1_gain_vs_best_single = -0.016385
```

这说明当前 closed-loop optimizer 开启后反而降低融合指标。原因可能包括闭环反馈延迟、局部规划结果与候选参数之间归因不充分、窗口指标噪声较大、反馈权重不适合当前仿真时长等。因此项目将其从默认评测链路中剥离，仅保留闭环反馈和延迟归因代码作为实验功能。这个结果在论文中应作为一个重要工程结论：优化系统不应盲目追求复杂度，经过指标验证后保留简单有效链路，往往比引入未验证闭环控制更可靠。

## 9 相对原 EGO-Planner 的优化总结

### 9.1 感知输入从单模态升级为多模态

原版链路主要依赖 depth cloud。本项目新增模拟 LiDAR 点云，并将 depth 与 LiDAR 统一到体素级融合框架中。该优化使规划器输入不再完全依赖单一传感器，在近场细节和中远场稳定性之间取得更好平衡。

### 9.2 从点云输入升级为证据输入

原始点云只是空间点集合，缺少置信度和来源信息。本项目通过 `VoxelEvidence` 保存 Log-odds、hits、seen_depth、seen_lidar 和 D-S 质量，使每个体素都成为带来源和置信度的证据单元。最终输出虽然仍是 PointCloud2，但其背后已经经过概率证据筛选。

### 9.3 从固定阈值升级为在线自适应

原链路中的感知阈值通常固定。项目将 `min_probability`、`min_hits` 和 `dual_bonus` 变为在线候选参数，通过局部 GT、单传感器 baseline、F1/Recall gain、EMA 平滑和 D-S 惩罚进行选择。这使参数能够随局部场景变化而调整，减少纯手工经验调参。

### 9.4 从不可解释融合升级为可诊断融合

没有 D-S 指标时，工程人员只能通过最终点云形态和规划效果猜测融合质量。本项目发布 `ds_metrics`，将 occupied belief、unknown 和 conflict 显式暴露出来。该设计让融合问题可以被拆分为“证据不足”“传感器冲突”“输出过稀/过密”等更具体的问题。

### 9.5 从主观观察升级为自动化评测

原始 RViz 观察适合调试，但不适合长期参数对比。本项目补充 `compare_fusion.sh`、`sim_flight_stats_report.py` 和多组评测脚本，能够输出融合、规划、安全和稳定性指标。通过这些指标，项目能够判断哪些自适应方向应该保留，哪些应降级为实验功能。

### 9.6 从杂乱可视化升级为干净单机 RViz 演示

项目新增 `drone0_clean.rviz` 和 `tools/start_rviz_sim.sh`，解决历史 RViz 配置中多机 marker、旧路径残留、颜色混乱和启动前旧进程未清理的问题。当前 RViz 配置将 depth、LiDAR、fused cloud、occupancy inflate、无人机模型和路径分开显示，使演示和调试更清晰。

## 10 局限性分析

### 10.1 仿真传感器与真实传感器仍有差距

当前 LiDAR 是由全局点云和无人机位姿裁剪得到的模拟点云，虽然可以验证多模态融合逻辑，但与真实 LiDAR 在扫描线结构、运动畸变、时间同步误差、外参误差和反射强度方面仍存在差距。若进入真机阶段，需要补充传感器标定、TF 树校验和真实时间戳同步。

### 10.2 局部 GT 依赖仿真环境

在线自适应参数使用 `/map_generator/global_cloud` 构建局部 GT 体素，这是仿真中可用的监督信号。真实环境下无法直接获得全局 GT，因此需要将自适应评分替换为自监督指标，例如短时地图一致性、轨迹安全距离、传感器重投影一致性或历史观测稳定性。

### 10.3 闭环优化仍需更多验证

当前 closed-loop feedback 已能发布滑动窗口指标，但 closed-loop optimizer 默认关闭。现有完整 batch 证明其开启后可能降低长期指标，因此未来如果继续推进，需要重新设计归因机制、降低反馈权重、延长评测时长，并引入真实重规划 topic，而不是只依赖 occupancy 变化近似重规划。

## 11 后续优化方向

第一，可以将 D-S 证据从指标层进一步推进到融合输出层。例如在体素筛选时引入 conflict-aware threshold，使高冲突体素需要更严格证据，而低 unknown 且双传感器一致的体素可以适当放宽阈值。

第二，可以将在线自适应从仿真 GT 监督升级为自监督闭环优化。真实系统无法使用 `/map_generator/global_cloud` 作为 GT，因此需要设计不依赖真实地图的指标，例如局部地图时序一致性、轨迹安全距离、控制输入平滑度和传感器重观测一致性。

第三，可以接入真实 LiDAR 或深度相机，并建立外参、时间同步和 TF 检查脚本。只有完成真实传感器接入，才能验证当前距离自适应权重和 D-S 指标是否具有 sim-to-real 泛化能力。

第四，可以进一步优化规划质量指标。当前 `replan_proxy_count` 部分依赖 occupancy 变化近似，后续应直接从 planner 状态、`planning/bspline` 或重规划日志中获得更准确的重规划次数。

第五，可以加入多随机种子、多地图密度和长时段 batch。当前已有多组实验支持融合正收益，但若要形成更强论文结论，应统计均值、方差、成功率、到点率和置信区间。

第六，可以针对不同场景建立参数策略库。例如稀疏障碍地图、密集森林、窄通道、远场障碍和动态障碍分别使用不同初始候选范围，再由在线自适应做微调。

## 12 结论

本文基于当前 ROS 2 EGO-Planner 工程，系统分析了本项目相对原 EGO-Planner 的优化设计。项目没有直接重写规划器核心，而是从感知输入质量、融合可解释性、在线参数自适应和自动化评测四个层面对原链路进行增强。通过模拟 LiDAR 与深度点云的体素级 Log-odds 融合，项目将单模态 depth 输入升级为多模态证据输入；通过距离自适应传感器权重，项目在近场细节和中远场稳定性之间建立了可解释折中；通过 Dempster-Shafer evidence metrics，项目将融合质量拆解为 occupied belief、unknown 和 conflict，使融合调试从纯可视化观察转向指标诊断；通过 `min_probability`、`min_hits` 和 `dual_bonus` 的在线候选搜索，项目将固定经验阈值升级为局部指标驱动的自适应参数选择。

实验结果表明，当前默认融合链路在多个 batch 中相对最佳单传感器获得正向 F1 和 Recall 增益，`dual_bonus` 在长时段评测中带来稳定小幅收益，D-S 指标能够有效反映融合输出的不确定性和冲突度。同时，closed-loop optimizer 的负面 batch 结果也说明复杂闭环优化并不必然有效，必须通过完整评测决定是否进入默认链路。因此，当前项目的最可靠贡献不是单一算法公式，而是一套面向 EGO-Planner 的多模态感知增强、证据诊断、自适应调参和自动化验证工程体系。

综合来看，本项目将 EGO-Planner 从“单传感器局部避障仿真”扩展为“多模态融合驱动的局部避障评测系统”。该系统为后续真实传感器接入、飞控桥接、动态障碍物鲁棒评测和感知-规划联合优化提供了可运行、可观察、可量化的基础。

## 参考文献

[1] Zhou, X., Wang, Z., Ye, H., et al. EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors.
[2] Zhou, X., Zhu, X., Wang, Z., et al. EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments.
[3] Hornung, A., Wurm, K. M., Bennewitz, M., et al. OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees.
[4] Dempster, A. P. Upper and Lower Probabilities Induced by a Multivalued Mapping.
[5] Shafer, G. A Mathematical Theory of Evidence.
[6] ROS 2 Humble Documentation.
[7] 本项目源码：`src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py`。
[8] 本项目源码：`src/planner/plan_manage/scripts/closed_loop_feedback_node.py`。
[9] 本项目评测脚本：`tools/compare_fusion.sh` 与 `tools/sim_flight_stats_report.py`。
