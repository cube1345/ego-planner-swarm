# 一种面向 EGO-Planner 的深度相机与 LiDAR 几何融合及闭环自适应方法

更新时间：2026-05-13

## 摘要

针对基于 EGO-Planner 的无人机局部避障系统，本文围绕当前 ROS 2 工程实现，设计并落地了一种深度相机与 LiDAR 的几何级多模态融合方法。该方法以深度点云与模拟 LiDAR 点云为输入，通过近似时间同步、统一世界坐标系下的预处理、体素级 log-odds 占据证据累积，生成可直接送入 `grid_map/cloud` 的融合障碍点云。在此基础上，系统引入 `min_probability`、`min_hits` 和 `dual_bonus` 三个面向融合层的在线自适应参数，并进一步接入 closed-loop feedback，将融合质量、路径长度、轨迹平滑度、安全半径侵入比例和地图抖动等指标合成为二级评分修正项。现有批量评测结果显示，当前默认保留链路相对最佳单传感器具有稳定融合收益，且闭环反馈在长时段 headless 评估中改善了路径长度、加速度 RMS、安全侵入比例和地图抖动。本文总结的重点不在于提出新型规划器，而在于说明如何以较小工程侵入方式增强 EGO-Planner 的局部障碍感知输入质量，并把经验调参升级为可度量的感知-规划联合收益优化。

## 关键词

无人机避障；EGO-Planner；多模态融合；深度相机；LiDAR；在线自适应参数；ROS 2

## 1. 引言

局部轨迹优化类无人机避障系统的上限，很大程度上取决于局部障碍地图的质量。若感知输入出现空洞、时间不同步、近场或远场结构缺失，则规划器即使本身稳定，也容易生成不理想甚至不可行的轨迹。单一深度点云在近场细节表达上通常较强，但在中远距离和遮挡边界处容易出现观测不完整；单一 LiDAR 在几何测距上更稳，但在近场密集结构和垂向表达方面又有局限。因此，在不重构 EGO-Planner 主体的前提下，利用多模态融合改善 `grid_map/cloud` 的输入质量，是一个务实且工程代价可控的方向。

当前工程没有直接改写 EGO-Planner 的优化器或飞控接口，而是首先把自适应集中在融合层。这样做的原因是：融合层既直接决定局部地图的稠密程度与可信度，又不会破坏原有 EGO-Planner 的优化结构，因此更适合作为第一阶段的自适应入口。

## 2. 系统架构

当前融合链路由 `single_run_in_sim_fusion.launch.py` 启动。其感知输入包括：

- 深度点云：`/drone_0_pcl_render_node/cloud`
- 模拟 LiDAR 点云：`/drone_0_lidar/points`

融合输出为：

- `/drone_0_fusion/fused_cloud`

随后通过 `advanced_param.launch.py` 将该输出接入规划器的 `grid_map/cloud`。

对应地，原版非融合链路 `single_run_in_sim.launch.py` 不启动模拟 LiDAR 节点，也不启动融合节点，而是让 `grid_map/cloud` 直接读取深度点云。这使得当前项目天然具备“原版链路 vs 融合链路”的对比基础。

## 3. 几何融合方法

### 3.1 基本流程

当前融合方法可概括为如下流程：

1. 对深度点云和 LiDAR 点云做近似时间同步；
2. 在统一坐标系下进行高度与量程裁剪；
3. 将两路点云离散到统一体素网格；
4. 为每个体素累积来自 depth 和 lidar 的 log-odds 占据证据；
5. 根据体素最终概率与命中次数筛选保留体素；
6. 将保留体素中心发布为融合点云。

因此，该方法并不是原始点级拼接，而是“体素级占据证据融合”。

### 3.2 距离自适应传感器权重

在当前实现中，深度点云与 LiDAR 的证据强度并不固定相同，而是与距离相关。设计直觉是：

- 近场结构更信任 depth；
- 中远场结构更信任 lidar。

也就是说，当前融合器并不是只做“多传感器并集”，而是把传感器物理特性直接编码进概率模型中。

### 3.3 体素证据累积

设某体素在 depth 和 lidar 下分别得到占据概率估计 $p_d$ 与 $p_l$，则可将其映射到 log-odds 形式并累加：

$$
L = \sum \log \frac{p}{1-p}
$$

再通过 sigmoid 恢复最终占据概率：

$$
P = \frac{1}{1 + e^{-L}}
$$

若该概率高于阈值 `min_probability`，且命中次数满足 `min_hits`，则该体素进入最终融合结果。

因此，`min_probability`、`min_hits` 与 `dual_bonus` 分别决定障碍体素保留阈值、最小命中证据和双传感器一致性奖励，是当前默认保留的三个在线自适应对象。

## 4. 在线自适应参数设计

### 4.1 当前在线自适应参数

当前工程中，默认被在线调节的参数有三个：

- `current_min_probability`
- `current_min_hits`
- `current_dual_bonus`

它们来自静态基准参数和候选空间：

- `min_probability`
- `min_hits`
- `adaptive_dual_bonus_min ~ adaptive_dual_bonus_max`

并在每帧内由在线策略重新选择。

### 4.2 候选空间

当前实现使用以下参数定义候选搜索空间：

- `adaptive_min_probability_min`
- `adaptive_min_probability_max`
- `adaptive_min_probability_step`
- `adaptive_min_hits_min`
- `adaptive_min_hits_max`
- `adaptive_dual_bonus_min`
- `adaptive_dual_bonus_max`
- `adaptive_dual_bonus_step`

在当前项目默认配置下，`min_probability` 候选区间为：

$$
0.20 \sim 0.35
$$

步长为：

$$
0.02
$$

`min_hits` 当前默认候选范围为 `1 ~ 3`，`dual_bonus` 当前默认候选范围为 `0.0 ~ 0.2`，步长为 `0.1`。

### 4.3 评估目标

当前在线自适应并不是直接最大化融合结果自身的绝对 `f1`，而是最大化相对最佳单传感器的收益：

$$
utility = gain\_f1 + 0.35 \cdot gain\_recall
$$

其中：

$$
gain\_f1 = fusion\_f1 - best\_single\_f1
$$

$$
gain\_recall = fusion\_recall - best\_single\_recall
$$

这样做的含义是：

- 若融合没有真正优于最佳单传感器，则不会因为点更多而被误判为更好；
- 在线阈值调节的目标被明确限制为“创造真实融合收益”。

### 4.4 平滑策略

为抑制单帧波动，系统对候选阈值的 utility 做指数滑动平均：

$$
S_t = (1-\alpha)S_{t-1} + \alpha U_t
$$

其中 $\alpha$ 对应：

- `adaptive_score_alpha`

它不是被在线调节的对象，而是自适应过程的稳定性超参数。

## 5. 目前到底有几个“自适应参数”

从严格工程定义看，当前默认在线自适应参数有 3 个：

- `min_probability`
- `min_hits`
- `dual_bonus`

其余 `adaptive_*` 参数大多属于以下两类：

1. 开关与边界控制

   - `adaptive_min_probability_enable`
   - `adaptive_min_probability_min`
   - `adaptive_min_probability_max`
   - `adaptive_min_probability_step`
   - `adaptive_min_hits_enable`
   - `adaptive_dual_bonus_enable`
2. 评估与平滑超参数

   - `adaptive_eval_range`
   - `adaptive_min_gt_voxels`
   - `adaptive_score_alpha`
   - `closed_loop_feedback_weight`

此外，代码中虽然声明或实现了：

- `adaptive_target_retention`
- `adaptive_retention_band`
- `adaptive_lidar_growth_enable`
- `adaptive_z_max_enable`
- `adaptive_depth_decay_enable`
- `adaptive_near_field_radius_enable`

但当前 batch 结果不支持把这些方向默认启用，因此不能算当前保留链路中的有效收益点。

## 6. 工程验证链路

### 6.1 RViz 演示链路

当前项目已经新增：

- `drone0_clean.rviz`
- `tools/run_adaptive_rviz_demo.sh`

这样做的原因是：旧的 `default.rviz` 混入了大量多机历史显示项，容易在单机仿真下造成误判。新的干净 RViz 配置只保留当前 `drone_0` 相关显示，从而更适合验证当前演示链路是否真的在运动与避障。

### 6.2 Headless 指标链路

当前项目的 headless 评测由以下脚本组成：

- `tools/compare_fusion.sh`
- `fusion_benefit_report.py`
- `sim_flight_stats_report.py`

当前版本已修复：

- 重复 `rclpy.shutdown()` 导致的统计脚本异常退出
- `replan_count` 基线错误和提前收尾问题

因此，当前文档中引用的融合收益与重规划统计，已经有更稳定的工程采集链路支撑。

## 7. 当前数据结论

根据当前项目中已经验证过的结果，可以得到以下结论：

1. 固定高阈值 `min_probability=0.55` 时，融合收益很差，甚至相对最佳单传感器为负增益。
2. 将阈值降低到 `0.40` 后，融合开始出现正收益，但提升有限。
3. 将在线自适应限制在 `0.20 ~ 0.35` 低阈值区间内时，融合表现更稳定，且对 Recall 与 F1 都能持续带来正收益。
4. `adaptive_dual_bonus_scored_light_long_20260513` 中，开启 `dual_bonus` 自适应后，`fusion_recall` 从 `0.166402` 提升到 `0.169008`，`fusion_f1` 从 `0.285162` 提升到 `0.288916`。
5. `closed_loop_dual_bonus_long_20260513` 中，闭环反馈使路径长度从 `23.382 m` 降到 `23.063 m`，`accel_rms` 从 `9.103` 降到 `6.280`，安全半径侵入比例从 `0.1093` 降到 `0.0498`，占据地图抖动从 `0.1205` 降到 `0.1135`。
6. `adaptive_lidar_growth`、`adaptive_z_max` 和过宽 `adaptive_near_field_radius=4.5` 的 batch 结果不稳定或负收益，因此当前不作为默认有效设计。

因此，从当前工程结果看，把自适应集中在融合层并引入轻量闭环反馈是合理的；但自适应方向必须经过 batch 指标筛选，不能把所有可调参数都默认宣传为有效优化。

## 8. 讨论

### 8.1 当前方案的优势

- 不需要改写 EGO-Planner 主体优化器
- 接口保持为标准 `PointCloud2`
- 在线自适应对象仍集中在融合层，便于排查与解释
- 既能在 RViz 演示，也能做 headless 批量评测
- 闭环反馈把感知质量与规划安全性纳入同一评估链路

### 8.2 当前方案的局限

- 在线自适应仍主要作用于融合层，尚未扩展到规划器代价权重层
- `replan_proxy_count` 在线反馈仍是占据变化近似，后续可替换为真实 planner 状态
- `adaptive_lidar_growth`、`adaptive_z_max` 等方向已有负收益案例，需要保持关闭
- 仿真中使用 `force_zero_stamp` 以适配当前数据路径，这属于工程性同步修补，不等价于真机时钟同步

## 9. 结论

基于当前项目代码可以明确得出：现阶段的多模态融合增强，本质上是“体素级几何占据证据融合 + 三参数在线自适应 + 感知-规划闭环反馈”。
当前默认在线自适应参数为 `min_probability`、`min_hits` 和 `dual_bonus`。其他 `adaptive_*` 参数要么用于定义搜索边界，要么属于试验后未默认启用的方向，不能作为当前有效收益点表述。

从工程角度看，这种设计是合理的第一步：它既能显式提升融合输入质量，又不会把问题扩散到规划层和控制层，从而保留了系统调试与结果解释的可控性。
