# 算法与技术栈

## 技术栈

### ROS 2 Humble

项目运行在 ROS 2 Humble 上，使用 launch 文件组织节点，通过 topic 连接地图、传感器、融合、规划、控制和仿真。

关键消息类型：

- `sensor_msgs/msg/PointCloud2`：深度点云、LiDAR 点云、radar 点云、融合点云。
- `nav_msgs/msg/Odometry`：无人机位姿和速度反馈。
- `std_msgs/msg/String`：D-S metrics 和闭环反馈等 JSON 字符串指标。

### C++ planner stack

EGO-Planner 主体、轨迹优化、地图环境和部分仿真节点主要是 C++：

- `plan_env`
- `path_searching`
- `bspline_opt`
- `traj_utils`
- `plan_manage`
- `so3_control`
- `so3_quadrotor_simulator`

### Python fusion and evaluation stack

当前多模态融合、动态障碍点云、闭环反馈和报告工具主要是 Python：

- `ros2_lidar_depth_fusion_node.py`
- `simulated_lidar_cloud.py`
- `simulated_mmwave_radar_cloud.py`
- `dynamic_obstacle_cloud.py`
- `closed_loop_feedback_node.py`
- `fusion_benefit_report.py`

## EGO-Planner 核心思想

EGO-Planner 是 ESDF-Free 的局部轨迹规划方法。传统局部规划常先构建 ESDF，即每个栅格到最近障碍物的距离场，然后用距离场求碰撞代价和梯度。EGO-Planner 的思路是避免维护全局或局部 ESDF，直接围绕轨迹控制点和障碍物约束构造优化问题。

简化理解：

```text
输入：odom、目标点、局部障碍点云
输出：满足动力学约束且尽量远离障碍物的局部 B-spline 轨迹
```

常见优化目标可以理解为：

```text
J = lambda_smooth * J_smooth
  + lambda_collision * J_collision
  + lambda_feasibility * J_feasibility
  + lambda_endpoint * J_endpoint
```

其中：

- `J_smooth`：让轨迹更平滑，避免频繁急转。
- `J_collision`：让轨迹远离障碍物。
- `J_feasibility`：限制速度、加速度、jerk，避免无人机无法执行。
- `J_endpoint`：约束轨迹朝目标点推进。

## ESDF-Free 的意义

ESDF-Free 不代表不需要地图，而是不显式维护完整距离场。优点是：

1. 减少构建和更新 ESDF 的计算成本。
2. 更适合快速重规划。
3. 在局部避障中，可以更直接地根据障碍点和轨迹控制点构造梯度约束。

代价是：

1. 输入点云质量更关键。
2. 障碍点云抖动会更直接地影响轨迹。
3. 多模态融合必须控制噪声和虚警，否则 planner 会被错误障碍牵引。

## 体素化

点云体素化是把连续三维空间按固定分辨率切成小立方体 voxel，然后把落在同一个 voxel 内的多个点合并为一个空间单元。

代码中核心形式是：

```python
keys = np.floor(points / self.resolution).astype(np.int32)
```

数学上，对于点 `p = (x, y, z)` 和体素分辨率 `r`：

```text
key(p) = floor(p / r)
       = (floor(x / r), floor(y / r), floor(z / r))
```

voxel 中心位置为：

```text
center(key) = (key + 0.5) * r
```

体素化的意义：

- 降低点云数量。
- 抑制传感器噪声。
- 把不同模态投到同一空间索引，便于融合。
- 让 planner 输入更稳定。

## Log-Odds 占据证据

fusion 节点用概率表示某个 voxel 被障碍占据的可能性，再转换为 log-odds 累计证据。

概率 `p` 的 log-odds 为：

```text
L(p) = log(p / (1 - p))
```

多个传感器证据可以相加：

```text
L_total = L_depth + L_lidar + L_radar
```

再转回概率：

```text
p_total = 1 / (1 + exp(-L_total))
```

代码中的对应关系：

```python
logits = np.log(probs / (1.0 - probs)) * hit_boost
item.logit_sum += float(logits[idx])
probability = sigmoid(np.array([evidence.logit_sum], dtype=np.float32))[0]
```

意义：

- 同一个 voxel 被多个模态支持时，占据概率会上升。
- 稠密点数通过 `hit_boost` 增强证据。
- 单个弱证据不会无限放大，概率经过 clamp 和阈值筛选。

## Dempster-Shafer Evidence

Dempster-Shafer theory 用三类质量描述传感器证据：

```text
m(occupied), m(free), m(unknown)
```

和单一概率不同，D-S 可以显式表达 unknown 和 conflict。对多模态融合来说，这很有用：

- `unknown` 高：说明信息不足，不应过度自信。
- `conflict` 高：说明不同模态互相矛盾，需要降低信任。
- `belief_occupied` 高：说明多个模态都支持占据。

当前代码中：

```python
ds_mass_from_probability()
compute_ds_metrics()
combine_ds_masses()
```

负责把每个模态的占据概率转换为 D-S mass，并融合 depth、LiDAR、radar 的 evidence。

## 自适应参数

fusion 节点内包含若干自适应参数，用于在不同场景中自动选择更合适的融合阈值：

- `adaptive_min_probability_enable`
- `adaptive_min_hits_enable`
- `adaptive_near_field_radius_enable`
- `adaptive_lidar_growth_enable`
- `adaptive_dual_bonus_enable`
- `adaptive_depth_decay_enable`
- `adaptive_z_max_enable`
- `adaptive_ds_score_enable`

核心思想是生成候选参数集合，对每组候选参数计算 fusion 输出，再根据全局地图 ground truth 或闭环反馈打分。

常见评价目标：

```text
precision = TP / (TP + FP)
recall    = TP / (TP + FN)
F1        = 2 * precision * recall / (precision + recall)
```

调参目标不是让点云越多越好，而是在 recall、precision、轨迹稳定性之间取得平衡。

## Dynamic Obstacle Avoidance

动态避障不是只靠 planner，也依赖感知和评价链路：

- `dynamic_obstacle_cloud.py` 生成移动障碍物点云。
- LiDAR/radar 模拟节点可将动态障碍物纳入传感器点云。
- fusion 节点将动态障碍和静态地图统一体素化。
- planner 根据更新后的 grid_map 进行局部重规划。
- closed loop feedback 评估动态障碍距离、collision risk 和轨迹平滑性。

动态避障优化方向：

1. 提升动态障碍物观测时效。
2. 降低融合点云抖动。
3. 对接近无人机的动态障碍提高权重。
4. 限制过激重规划，保持控制平滑。
5. 对 moving obstacle 加入时间预测或 TTC 风险项。
