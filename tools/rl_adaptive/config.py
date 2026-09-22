"""自适应融合参数的深度评分函数（Q-learning 离线版）配置。

目标：用一个神经网络 Q(s, a) 替代 fusion 节点里的手工评分函数
      utility = gain_f1 + 0.35 * gain_recall，
      让参数选择直接优化「闭环避障质量」（collision / path / replan / recall）。

训练范式：监督学习（离线 Q 拟合）。从仿真收集 (s, a, r) 样本，
          用 MSE 拟合 Q(s, a) ≈ r，推理时选 argmax_a Q(s, a)。
          数据稳定后可选 IQL/CQL 等离线 RL 进一步优化。

状态 s 全部来自仿真中可实时获取的闭环信号（不含 global_cloud GT），
这样推理阶段在真实场景同样可用。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# 动作空间：融合参数候选（与 fusion 节点 auto_tune_parameters 对齐）
# ---------------------------------------------------------------------------

# min_probability 候选（0.20 ~ 0.35, step 0.02）
MIN_PROBABILITY_CANDIDATES: List[float] = [
    round(0.20 + 0.02 * i, 4) for i in range(8)
]  # 0.20 .. 0.34

# min_hits 候选
MIN_HITS_CANDIDATES: List[int] = [1, 2, 3]

# dual_bonus 候选
DUAL_BONUS_CANDIDATES: List[float] = [0.0, 0.1, 0.2]

# 展平成动作索引 0..N_ACTIONS-1
ACTION_SPACE: List[Tuple[float, int, float]] = [
    (mp, mh, db)
    for mp in MIN_PROBABILITY_CANDIDATES
    for mh in MIN_HITS_CANDIDATES
    for db in DUAL_BONUS_CANDIDATES
]
N_ACTIONS: int = len(ACTION_SPACE)  # 8 * 3 * 3 = 72


def action_to_params(action: int) -> Tuple[float, int, float]:
    """动作索引 -> (min_probability, min_hits, dual_bonus)。"""
    return ACTION_SPACE[action]


def params_to_action(min_probability: float, min_hits: int, dual_bonus: float) -> int:
    """(min_probability, min_hits, dual_bonus) -> 最近的动作索引。"""
    best = 0
    best_dist = float("inf")
    for idx, (mp, mh, db) in enumerate(ACTION_SPACE):
        d = abs(mp - min_probability) + 0.25 * abs(mh - min_hits) + 0.15 * abs(db - dual_bonus)
        if d < best_dist:
            best_dist = d
            best = idx
    return best


# ---------------------------------------------------------------------------
# 状态空间：闭环可观测特征（推理阶段无需 GT）
# ---------------------------------------------------------------------------

@dataclass
class StateSpec:
    """状态特征定义。每个特征给出 name / 归一化方式。"""

    name: str
    norm: str = "none"  # none | clip01 | clip_abs 等


# 状态特征（维度 = len(STATE_FEATURES)）
# 全部归一化到 [0, 1] 或 [-1, 1] 附近，避免量纲差异。
STATE_FEATURES: List[StateSpec] = [
    # 局部障碍密度：无人机周围 eval_range 内的 GT/occupancy voxel 数
    # 直接用 occupancy map 点数（推理阶段可得，不依赖 global_cloud GT）
    StateSpec("occupancy_voxels", "clip01"),       # / max_occupancy
    # 融合点云规模
    StateSpec("fused_voxels", "clip01"),           # / max_fused
    # D-S 指标（融合质量）
    StateSpec("ds_unknown", "clip01"),             # 0..1
    StateSpec("ds_conflict", "clip01"),            # 0..1
    # 最近障碍距离（safety_radius 附近最重要）
    StateSpec("min_obstacle_distance", "clip01"),  # / 3.0
    # 无人机速度
    StateSpec("speed", "clip01"),                  # / max_vel
    # 到目标剩余距离
    StateSpec("distance_to_goal", "clip01"),       # / 30.0
    # 当前参数（反馈给策略，让它知道自己在哪）
    StateSpec("current_min_probability", "clip01"),  # 0.20..0.35
    StateSpec("current_min_hits", "clip01"),          # 1..3
    StateSpec("current_dual_bonus", "clip01"),        # 0..0.2
]

N_STATE: int = len(STATE_FEATURES)


# 各特征归一化参考值
NORM_REF = {
    "occupancy_voxels": 100000.0,
    "fused_voxels": 20000.0,
    "ds_unknown": 1.0,
    "ds_conflict": 1.0,
    "min_obstacle_distance": 3.0,
    "speed": 3.0,
    "distance_to_goal": 30.0,
    "current_min_probability": 0.35,
    "current_min_hits": 3.0,
    "current_dual_bonus": 0.2,
}


def normalize_state(raw: dict) -> np.ndarray:
    """把仿真里采集的原始特征字典归一化成状态向量。

    raw 字段（见 collect_data.py 输出的 CSV 列名）：
      occupancy_voxels, fused_voxels, ds_unknown, ds_conflict,
      min_obstacle_distance, speed, distance_to_goal,
      current_min_probability, current_min_hits, current_dual_bonus
    """
    vec = np.zeros(N_STATE, dtype=np.float32)
    for i, feat in enumerate(STATE_FEATURES):
        v = float(raw.get(feat.name, 0.0))
        ref = NORM_REF.get(feat.name, 1.0)
        v = v / max(ref, 1e-6)
        if feat.norm == "clip01":
            v = min(1.0, max(0.0, v))
        vec[i] = v
    return vec


# ---------------------------------------------------------------------------
# 奖励函数：闭环避障质量（推理/训练共用，无需 GT）
# ---------------------------------------------------------------------------

@dataclass
class RewardWeights:
    """奖励权重。可调，训练时也能学习这些权重（作为辅助输出）。"""

    w_collision: float = 1.0     # 碰撞风险（最重）
    w_path: float = 0.10         # 路径长度惩罚
    w_replan: float = 0.01       # 重规划惩罚
    w_belief: float = 0.30       # D-S 占据置信度（感知质量代理，不依赖 GT）
    w_ds_conflict: float = 0.05  # 传感器冲突惩罚


DEFAULT_WEIGHTS = RewardWeights()


def compute_reward(metrics: dict, weights: RewardWeights = DEFAULT_WEIGHTS) -> float:
    """从一帧的闭环指标计算奖励。

    metrics 字段（与 sim_flight_stats / closed_loop 对齐）：
      collision_risk_score, path_length_m, replan_count,
      fusion_recall, ds_conflict_mean
    """
    collision = float(metrics.get("collision_risk_score", 0.0))
    path = float(metrics.get("path_length_m", 0.0))
    replan = float(metrics.get("replan_count", 0.0))
    belief = float(metrics.get("ds_belief_occupied_mean", 0.0))
    ds_conflict = float(metrics.get("ds_conflict_mean", 0.0))

    reward = (
        -weights.w_collision * min(1.0, collision)
        - weights.w_path * min(1.0, path / 30.0)
        - weights.w_replan * min(1.0, replan / 100.0)
        + weights.w_belief * min(1.0, belief)
        - weights.w_ds_conflict * min(1.0, ds_conflict)
    )
    return float(reward)


# ---------------------------------------------------------------------------
# 数据格式（collect_data.py 输出的 CSV 列名）
# ---------------------------------------------------------------------------

CSV_COLUMNS: List[str] = [
    "episode",
    "step",
    "action",
    "min_probability",
    "min_hits",
    "dual_bonus",
    # 状态特征（raw）
    "occupancy_voxels",
    "fused_voxels",
    "ds_unknown",
    "ds_conflict",
    "min_obstacle_distance",
    "speed",
    "distance_to_goal",
    "current_min_probability",
    "current_min_hits",
    "current_dual_bonus",
    # 奖励信号
    "collision_risk_score",
    "path_length_m",
    "replan_count",
    "ds_belief_occupied_mean",
    "ds_conflict_mean",
    # 终局标志（episode 结束 = 到达目标 / 碰撞 / 超时）
    "done",
    "goal_reached",
]
