# 自适应融合参数的深度学习评分函数

用神经网络 `Q(s, a)` 替代 fusion 节点里手工的评分函数
`utility = gain_f1 + 0.35 * gain_recall`，让 `min_probability / min_hits / dual_bonus`
的选择直接优化「闭环避障质量」（collision / path / replan / recall），
而不是手工调权重。

## 为什么是「监督学习拟合 Q 函数」而不是 RL

- 当前 `auto_tune_parameters` 本质是「候选枚举 + 手工打分」，瓶颈在**打分函数**
  （之前实验发现 adaptive 收敛到 0.28~0.30，但固定 0.20 的 recall 更高——打分口径不对）。
- 监督学习拟合 `Q(s, a) ≈ r` 训练稳定、数据需求明确，且保留候选枚举框架（工程改动小）。
- 数据稳定后可平滑升级到 IQL / CQL 等离线 RL。

## 文件

| 文件 | 作用 | 运行位置 |
|---|---|---|
| `config.py` | 状态/动作/奖励定义 + CSV 列名 | 两端共用 |
| `collect_data.py` | 仿真数据采集节点，输出 CSV | 本地 ROS2 |
| `train_q.py` | 离线 Q 网络训练（PyTorch） | 训练平台 |
| `infer.py` | 推理：状态 → 最优参数 | 本地/任意 |

## 使用流程

### 1. 让 ds_metrics 带上当前参数（一次性的 fusion 节点小改动）

在 `ros2_lidar_depth_fusion_node.py` 的 `publish_ds_metrics()` 的 payload 里加：

```python
"min_probability": float(self.current_min_probability),
"min_hits": int(self.current_min_hits),
"dual_bonus": float(self.current_dual_bonus),
```

否则数据采集节点拿不到「当前动作 a」，训练失去监督。

### 2. 本地采数据

```bash
# 正常启动融合仿真（可选开动态障碍、多场景）
ros2 launch ego_planner single_run_in_sim_fusion.launch.py use_fusion:=True

# 另开终端跑采集节点
python3 tools/rl_adaptive/collect_data.py \
    --out artifacts/adaptive_data.csv --episode-limit 50
```

建议覆盖多种场景（静态森林 / 动态障碍横穿 / 稀疏开阔地），
数据多样性决定 Q 网络的泛化能力。

### 3. 训练平台训练

```bash
python3 tools/rl_adaptive/train_q.py \
    --data artifacts/adaptive_data.csv \
    --out q_model.pt \
    --epochs 50 --batch-size 256 --lr 3e-4
```

### 4. 推理接入 fusion 节点

把 `auto_tune_parameters` 里选候选的 `utility` 换成 Q 网络输出：

```python
q = q_net(state_tensor)          # (N_ACTIONS,)
best_action = q.argmax()
min_probability, min_hits, dual_bonus = action_to_params(best_action)
```

或先离线验证：`python3 tools/rl_adaptive/infer.py --model q_model.pt --state-file s.json`

## 关键设计决策（训练前先想清楚）

1. **状态 s 不含 global_cloud GT**：全部是 occupancy / fused / D-S / odom 等闭环信号，
   这样推理阶段在真实场景（无 GT）也能用。
2. **奖励 r 用闭环信号**：`-collision - path - replan + recall - ds_conflict`，
   不依赖 GT。权重在 `config.RewardWeights` 里可调。
3. **动作空间 = 72 个离散候选**：与现有 `auto_tune_parameters` 对齐，便于回退对比。
4. **`min_obstacle_distance` 已实现**：`collect_data.py` 从 fused_cloud 实时计算无人机到
   最近障碍点的欧氏距离（`min_obstacle_distance()` 方法），无需额外订阅。

## 预期收益与风险

- 收益：跨场景自适应（无需人工调参）、打分口径与避障质量对齐。
- 风险：当前指标已较高（fusion_f1 0.34、动态避障零碰撞），Q 网络的边际提升
  未必显著；更可能体现在「无监督下自动适应不同场景」的工程价值。
