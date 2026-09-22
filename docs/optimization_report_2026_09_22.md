# EGO-Planner 避障系统优化报告

> 日期：2026-09-22　|　分支：`dynamic-obstacle-avoidance-archive`

本次优化围绕「避障飞行质量」展开，产出 **3 个已提交的算法改进** 和 **1 套可部署的深度学习自适应参数训练框架**。核心结论：`collision_risk` 的硬上限是场景密度（mockamap 密集障碍），感知/规划/动态避障的改进点收敛为三个有效方向。

---

## 一、技术更新（改动概览）

| commit | 模块 | 改动 | 效果 |
|---|---|---|---|
| `601d57e` | A* 前端搜索 | 固定步长 `0.1` → 动态步长 `(in-out).norm()/10+0.05` | replan 51~1258 → 25~46 |
| `bc20dad` | 感知视场 | lidar `horizontal_fov` 240° → 360° | recall +24%、f1 +20%、collision 0.66→0.53 |
| `8c64cc0` | 动态避障 | predictor 匀速 → 匀加速 | min_dynamic_dist +131%、动态违规 0 |
| 未提交 | RL 框架 | `tools/rl_adaptive/` Q 函数拟合替代手工评分 | 待训练验证 |

---

## 二、代码 Review

### 2.1 A* 越界修复（`bspline_optimizer.cpp`）

**根因**：两处 `AstarSearch` 把原作者动态步长（注释里的 `(in-out).norm()/10+0.05`）改成了固定 `0.1`。`POOL_SIZE=100³` + `center=(in+out)/2` 下，覆盖范围仅 `center±5m`，障碍段 in→out 超 10m 时 `Coord2Index` 越界 → `initControlPoints` 返回空 → 反复紧急重规划。

**Review 要点**：
- 修复是「恢复原作者意图」，非新逻辑，风险低。
- 动态步长保证 index 偏移恒 ≈5（远小于 pool 半宽 50），数学上不会越界。
- **换行符陷阱**：原文件 CRLF，Edit 工具初次修改把整文件转成 LF，产生 242 行噪音 diff。已用 `git checkout` 恢复 + `sed` 精确替换消除，最终 diff 仅 4 行（2+/2-）。

### 2.2 FOV 360°（`single_run_in_sim_fusion.launch.py`）

**根因**：recall 低（0.16）主因是传感器视场盲区（lidar 240°、radar 120°，后方看不到），而非融合算法。GT 评测是 10m 全向障碍，视场外障碍天然漏检。

**Review 要点**：
- 改动是「参数可配置化」：launch 加 `lidar_horizontal_fov` 参数 + `float()` 类型转换 + compare_fusion.sh 加 `--lidar-fov`。
- **类型陷阱**：`LaunchConfiguration.perform()` 返回字符串 "360.0"，直接传给 Node 的 `parameters=[{...}]` 会报 `InvalidParameterTypeException`（STRING vs DOUBLE）。必须 `float()` 转换（已修复）。

### 2.3 匀加速 predictor（`obj_predictor.h/.cpp`）

**根因**：`predictCallback` 里 `predictPolyFit()` 被注释，只用 `predictConstVel()`（两点匀速外推），对正弦运动障碍误差大。

**Review 要点**：
- 改动：`evaluateConstVel` 支持加速度项（`p0 + v·t + ½a·t²`），`predictConstVel` 用三点匀加速拟合（保留两点匀速 fallback）。
- **换行符陷阱**：`obj_predictor.h` 的 HEAD 是混合换行符（166 CRLF + 33 LF），统一后产生 62 行噪音。已用「git checkout 恢复 + python 正则精确替换」消除，最终 .h 仅 7+/7-、.cpp 70+/47-（真实改动）。
- 试过加「加速度 clamp（±2.0）」防外推发散，但引入 `min_dyn=0.138m` 的碰撞异常，**负优化已撤销**。结论：replan 波动（395）那轮反而是 `min_dyn=1.75m` 最好——replan 多 = 频繁调整积极避障，是有益行为，不需修复。

---

## 三、最新结论

### 3.1 有效方向（已固化）

| 指标 | 改进前 | 改进后 | 提升 |
|---|---|---|---|
| replan 震荡 | 51~1258 | 25~46 | 稳定 |
| fusion_recall | 0.163 | 0.203 | +24% |
| fusion_f1 | 0.280 | 0.337 | +20% |
| collision_risk 均值 | 0.66 | 0.53 | -20% |
| min_dynamic_distance | 0.67m | 1.55m | +131% |
| dynamic_safety_violation | 0.002 | 0.0 | 零碰撞 |

### 3.2 验证无效/有害的方向（诚实记录）

| 方向 | 结论 | 原因 |
|---|---|---|
| `lambda_collision` 增大 | 无效 | collision 是场景密度硬上限，非权重问题 |
| `obstacles_inflation` 增大 | 有害 | A* 起点落障碍内，搜索失败、被困 |
| `dist0`（clearance）增大 | 有害 | 绕远路（path 35.5m）但 min_dist 更小 |
| `max_range` 8→12 | recall +23% 但 collision 无改善 | 多出的 recall 是远处障碍，碰撞发生在近处 |
| TTC 开启 | 有害 | const-vel predictor + 正弦障碍，replan 激增 285、无人机卡住 |
| 加速度 clamp | 负优化 | 引入碰撞异常，已撤销 |
| 时序 occupancy 累积 | 失败 | cache 性能退化 + 收益有限（recall 低主因是视场）|

### 3.3 核心结论

**`collision_risk` 的硬上限是 mockamap 场景密度**（infill 12%、187 簇、58996 voxels）——无人机必须穿过障碍间隙，`min_dist` 稳定在 0.09~0.20m 是场景固有属性，非感知或规划参数能突破。唯一有效的感知改进是「**扩大视场覆盖**」（FOV 360°），因为它直接命中「后方盲区」根因。

---

## 四、具体代码实现

### 4.1 A* 动态步长修复

```cpp
// bspline_optimizer.cpp（两处，initControlPoints 和 check_collision_and_rebound）
// 修复前：
if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
// 修复后：
if (a_star_->AstarSearch((in-out).norm()/10+0.05, in, out))
```

### 4.2 lidar FOV 可配置化

```python
# single_run_in_sim_fusion.launch.py
lidar_horizontal_fov = LaunchConfiguration('lidar_horizontal_fov').perform(context)
# simulated_lidar_node parameters：
{'horizontal_fov_deg': float(lidar_horizontal_fov)},  # 必须 float()，否则 STRING/DOUBLE 类型错误
# DeclareLaunchArgument：
DeclareLaunchArgument('lidar_horizontal_fov', default_value='360.0'),
```

### 4.3 匀加速 predictor 核心

```cpp
// obj_predictor.h —— evaluateConstVel 支持加速度项
Eigen::Vector3d evaluateConstVel(double t) {
    double dt = t - global_start_time_.seconds();
    Eigen::Vector3d pt;
    pt(0) = polys[0](0) + polys[0](1) * dt + 0.5 * polys[0](2) * dt * dt;
    pt(1) = polys[1](0) + polys[1](1) * dt + 0.5 * polys[1](2) * dt * dt;
    pt(2) = polys[2](0) + polys[2](1) * dt + 0.5 * polys[2](2) * dt * dt;
    return pt;
}

// obj_predictor.cpp —— predictConstVel 三点匀加速拟合
// his.size() >= 3 时：
Eigen::Matrix<double, 3, 3> A, Q;
A << 1.0, t1, 0.5*t1*t1, 1.0, t2, 0.5*t2*t2, 1.0, t3, 0.5*t3*t3;
Q.row(0) = q1.transpose(); Q.row(1) = q2.transpose(); Q.row(2) = q3.transpose();
Eigen::Matrix<double, 3, 3> coeff = A.inverse() * Q;  // rows=[p0,v0,a0], cols=xyz
for (int j = 0; j < 3; ++j) { polys[j].head(3) = coeff.col(j); }
// his.size() == 2 时 fallback 到两点匀速（原逻辑）
```

### 4.4 深度学习自适应参数框架（`tools/rl_adaptive/`）

**方案**：监督学习拟合 Q 函数 `Q(s,a)`，替代 fusion 节点手工评分 `utility = gain_f1 + 0.35*gain_recall`。核心解决「评分口径与避障质量不一致」的痛点（adaptive 收敛到 0.28 但固定 0.20 的 recall 更高）。

**奖励函数（无 GT 依赖）**：
```python
reward = -1.0·collision_risk - 0.10·path - 0.01·replan
         + 0.30·ds_belief_occupied - 0.05·ds_conflict
```

**状态（10 维，全闭环信号）**：occupancy 密度、fused_voxels、D-S unknown/conflict、min_obstacle_distance、速度、到目标距离、当前 3 个参数。

**动作（72 个离散候选）**：`min_probability(8) × min_hits(3) × dual_bonus(3)`，与 `auto_tune_parameters` 对齐。

**文件**：`config.py`（定义）/ `collect_data.py`（本地采集）/ `train_q.py`（平台训练）/ `infer.py`（推理）/ `requirements.txt`。

**关键设计**：奖励和状态全部不依赖 `global_cloud` GT，推理阶段真实场景可用——这是「仿真训练 → 真实部署」同构的前提。

---

## 五、工作区状态

- **已提交**：`601d57e`、`bc20dad`、`8c64cc0`（3 个算法改进）
- **未提交**：radar 三模态遗留（之前开发遗留，未触碰）+ `tools/rl_adaptive/`（RL 框架）
- **已清理**：dist0 / max_range 可配置化（验证无收益）、时序累积（失败）、加速度 clamp（负优化）
