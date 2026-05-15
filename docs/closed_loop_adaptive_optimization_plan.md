# 感知-规划联合收益闭环自适应优化计划

更新时间：2026-05-13

## 当前指标结论

当前 headless 评估结果支持“保留当前默认优化链路”，但不支持把所有试验过的自适应方向都写成有效优化。

已保留并默认启用的设计：

- 多模态几何融合：融合结果相对最佳单传感器的 `positive_ratio_recall_gain` 和 `positive_ratio_f1_gain` 在当前 batch 中均为 `1.0`，说明融合输出稳定优于单一路径。
- 在线 `min_probability`：当前默认开启，用于在局部 GT 体素监督下选择障碍接受阈值。
- 在线 `min_hits`：当前默认开启，用于在融合证据稀疏或冗余时调整体素命中次数要求。
- 在线 `dual_bonus`：当前默认开启，用于奖励 depth 与 lidar 同时命中的体素。`adaptive_dual_bonus_scored_light_long_20260513` 中，开启后 `fusion_recall` 从 `0.166402` 提升到 `0.169008`，`fusion_f1` 从 `0.285162` 提升到 `0.288916`，判定为 `keep`。
- 闭环反馈：`closed_loop_dual_bonus_long_20260513` 中，开启后路径长度从 `23.382 m` 降到 `23.063 m`，`accel_rms` 从 `9.103` 降到 `6.280`，安全半径侵入比例从 `0.1093` 降到 `0.0498`，占据地图抖动从 `0.1205` 降到 `0.1135`。虽然融合 F1 略降，但感知-规划综合收益更好，判定为 `keep`。

试验后不建议作为默认启用的方向：

- `adaptive_lidar_growth`：`adaptive_lidar_growth_gate_20260505` 判定为 `disable`，开启后 `fusion_recall` 和 `fusion_f1` 均下降。
- `adaptive_z_max`：`adaptive_z_max_gate_20260505` 判定为 `disable`，开启后 `fusion_recall` 和 `fusion_f1` 均下降。
- 过宽 `adaptive_near_field_radius=4.5`：`adaptive_near_radius_4_5_20260513` 判定为 `disable`。较窄的 `3.5` 版本有局部收益，但相对最佳单传感器增益不稳定，因此当前只作为候选实验方向，不默认启用。

## 目标

下一阶段把当前融合层自适应参数从“基于融合指标的在线选择”升级为“面向感知-规划联合收益的闭环优化”。

当前已落地三步，但 sliding-window optimizer 目前只保留为实验开关，不作为默认启用项：

- 在 headless batch 评估链路中合成闭环目标 `J`，用于判断自适应参数是否值得保留。
- 新增 `closed_loop_feedback_node.py`，在线发布闭环反馈 JSON；融合节点订阅该反馈并把滑动窗口 `J` 引入候选参数评分。
- `closed_loop_feedback_node.py` 已从累计统计升级为最近 `N` 秒滑动窗口统计；融合节点会把延迟后的参数组合与最近窗口 `J` 绑定，形成每个候选组合自己的闭环 EMA 得分。
- 新增 `closed_loop_action_delay_sec`，默认 `3.0s`。反馈到达时不再归因给当前参数组合，而是归因给 `now - action_delay_sec` 时刻生效的组合，减少 planner / map / trajectory 链路延迟导致的错误归因。

在线节点内部的候选选择仍保留局部融合收益作为主约束。sliding-window optimizer 能按候选参数组合记录最近飞行表现。早期直接归因版本的完整 A/B 显示会拉低长期指标，因此默认关闭；当前已修正为延迟归因版本，后续需要重新做完整 batch 判断是否可启用。

最新完整测试结论：

- `sliding_window_optimizer_full_off_20260513`：`fusion_recall=0.043899`，`fusion_f1=0.080403`，`closed_loop_score=-0.432826`。
- `sliding_window_optimizer_full_on_20260513`：`fusion_recall=0.028493`，`fusion_f1=0.052071`，`closed_loop_score=-0.477907`。
- `sliding_window_optimizer_full_on_w001_20260513`：`fusion_recall=0.036899`，`fusion_f1=0.067376`，`closed_loop_score=-0.487639`。

因此，当前版本保留窗口指标发布和候选组合 EMA 代码，但默认不启用 `closed_loop_optimizer_enable`。

延迟归因 smoke：

- `delayed_attribution_smoke_20260513` 已验证 `closed_loop_optimizer=True action_delay_sec=3.00` 可正常运行。
- 该 smoke 只验证运行链路，不作为长期收益结论。

当前已验证的自适应参数主要集中在融合层：

- `min_probability`
- `min_hits`
- `dual_bonus`

其中 `min_probability`、`min_hits`、`dual_bonus` 已进入当前默认融合仿真链路；`near_field_radius`、`lidar_growth`、`z_max`、`depth_decay` 虽然有代码开关和评估脚本，但当前 batch 结果不足以支持默认启用。

后续目标不是继续盲目增加参数，而是把参数选择目标从单纯融合质量扩展到飞行任务质量、安全性和稳定性。

## 联合优化目标

计划建立统一评分函数：

```text
J = w1 * F1 + w2 * Recall - w3 * PathLength - w4 * ReplanCount - w5 * CollisionRisk
```

其中：

- `F1`：融合障碍点云与局部 GT 体素的 F1。
- `Recall`：融合障碍点云对局部 GT 体素的召回率。
- `PathLength`：无人机实际飞行路径长度。
- `ReplanCount`：规划器重规划次数。
- `CollisionRisk`：碰撞风险，可由最近障碍距离或占据体素侵入安全半径近似表示。

## 需要补齐的指标

### 融合质量

- `fusion_recall`
- `fusion_f1`
- `fusion_recall_gain_vs_best_single`
- `fusion_f1_gain_vs_best_single`

### 规划质量

- 路径长度
- 到点时间
- 重规划次数
- 轨迹平滑度

### 安全性

- 最近障碍距离
- 碰撞风险计数
- 安全半径侵入次数

### 稳定性

- 参数切换频率
- 融合点云体素数抖动
- 局部占据地图抖动

## 初步实现路线

1. 已扩展 `sim_flight_stats_report.py`，补充最近障碍距离、碰撞风险、占据地图抖动、参数切换频率和轨迹平滑度近似指标。
2. 已扩展 `compare_fusion.sh`，把融合指标和规划指标汇总到同一个 summary JSON。
3. 已新增 `tools/closed_loop_score.py`，根据可配置权重计算 `J`。
4. 当前 `evaluate_adaptive_dual_bonus.sh` 已把 `closed_loop_score` 纳入 keep / disable 判定。
5. 已将规划反馈发布为在线 topic，并支持把滑动窗口闭环评分接入融合节点候选评分。
6. 已新增候选参数组合级别的 closed-loop EMA，并加入 `action_delay_sec` 延迟归因；该优化器默认关闭，避免影响稳定链路。
7. 后续继续跑消融实验：单传感器、固定融合、自适应融合、闭环联合优化。

## 在线反馈 topic

当前 topic：

```text
/drone_0_fusion/closed_loop_feedback
```

消息类型：

```text
std_msgs/String
```

内容为 JSON，主要字段：

- `closed_loop_score`
- `closed_loop_score_ema`
- `closed_loop_window_sec`
- `window_path_length_m`
- `window_replan_proxy_count`
- `window_collision_risk_score`
- `window_min_obstacle_distance_m`
- `window_safety_violation_ratio`
- `window_accel_rms_mps2`
- `window_occupancy_jitter_ratio`
- `path_length_m`
- `replan_proxy_count`
- `collision_risk_score`
- `min_obstacle_distance_m`
- `safety_violation_ratio`
- `accel_rms_mps2`
- `occupancy_jitter_ratio`

融合节点订阅该 topic 后，会把 `closed_loop_score_ema` 归因到当前正在使用的候选参数组合：

```text
(min_probability, min_hits, lidar_growth, near_field_radius, dual_bonus)
```

每个组合维护独立的 closed-loop EMA。后续该组合再次被枚举时，其历史窗口 `J` 会进入候选评分。

为了避免把旧轨迹或旧地图状态错误归因给刚切换的候选组合，当前使用延迟归因：

```text
attribution_time = feedback_receive_time - closed_loop_action_delay_sec
```

融合节点维护候选组合历史队列，并把反馈归因给 `attribution_time` 时刻正在生效的组合。

## 当前工程边界

当前版本完成的是“在线滑动窗口闭环反馈 + 延迟归因的候选组合级闭环 EMA + headless 闭环判定”，不是完整的强化学习或 MPC 式闭环控制。候选组合级 optimizer 当前默认关闭，因为延迟归因版本还需要完整 batch 复验。反馈节点中的 `replan_proxy_count` 当前由占据地图变化近似得到，后续如果稳定接入 `planning/bspline` 或 planner 状态 topic，可以替换为真实重规划计数。

## 预期简历表述

设计面向感知-规划联合收益的在线自适应参数优化策略，将融合质量、路径长度、重规划次数、碰撞风险和地图稳定性纳入统一评分函数，实现从经验调参到闭环指标驱动优化的升级。
