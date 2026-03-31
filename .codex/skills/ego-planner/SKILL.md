---
name: ego-planner
description: 使用 EGO-Planner/EGO-Planner-v2 进行无人机局部避障，包含融合输入与参数调优策略。
---

# EGO-Planner

## 适用场景

- 未知环境中的实时局部避障与频繁重规划。
- 计算预算有限，不希望维护重型 ESDF 全局距离场。
- 需要和现有 ROS 2 感知链快速集成。

## 核心认知

- EGO-Planner 是 ESDF-free、基于梯度优化的局部规划方法。
- 输入质量（障碍观测时效与稳定性）直接决定轨迹质量。
- 若是多机协同场景，优先参考 EGO-Planner-v2 / EGO-Swarm 路线。

## 接入流程

1. 确认 odom 稳定，且 frame 对齐到 planner 的 world/map。
2. 给 `grid_map` 提供稳定障碍输入（可直接用融合点云）。
3. 设置 `manager/max_vel`、`manager/max_acc` 与飞控能力一致。
4. 验证重规划频率、最小间隙、急停行为。

## 参数调优顺序

1. 地图参数：`resolution`、`local_update_range`、`obstacles_inflation`。
2. 代价权重：`lambda_collision`、`lambda_feasibility`、`lambda_smooth`。
3. 动力学约束：`max_vel`、`max_acc`、`max_jerk`。
4. 再调融合阈值，避免“地图抖动导致轨迹抖动”。

## 多模态融合联合建议

- 近距离依赖 depth（细节更密），中远距离依赖 lidar（测距更稳）。
- 融合输出优先保持稀疏但可信，不盲目追求点数。
- 当融合节点异常（空云/超时）时，planner 应进入保守模式。

## 参考来源（联网核验）

- EGO-Planner 论文：https://arxiv.org/abs/2008.08835
- EGO-Swarm 论文：https://arxiv.org/abs/2011.04183
- EGO-Planner 仓库：https://github.com/ZJU-FAST-Lab/ego-planner
- EGO-Planner-v2 仓库：https://github.com/ZJU-FAST-Lab/EGO-Planner-v2
