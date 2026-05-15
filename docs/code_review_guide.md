# EGO-Planner 项目 Review 入口指南

## 项目概览

ROS2 Humble 无人机局部避障，核心规划器为 EGO-Planner。当前主要工作线：多模态（深度+LiDAR）融合与在线自适应调参。

源码量约 8.9k 行 C++ + 1.4k 行 Python。

## 目录结构

```
src/
├── planner/
│   ├── plan_manage/    ← ego_planner 主包：FSM、Manager、多传感器融合节点
│   ├── bspline_opt/    ← B样条轨迹优化（核心规划引擎）
│   ├── path_searching/ ← A* 路径搜索
│   ├── plan_env/       ← 栅格地图、Raycast、障碍物预测
│   ├── traj_utils/     ← 轨迹工具、可视化、自定义 msg
│   ├── drone_detect/   ← 无人机检测
│   └── rosmsg_tcp_bridge/
├── uav_simulator/
│   ├── so3_quadrotor_simulator/ ← 四旋翼动力学仿真
│   ├── so3_control/    ← SO(3) 控制器
│   ├── local_sensing/  ← 深度相机/PCL 渲染
│   ├── map_generator/ / mockamap/ ← 地图生成
│   └── Utils/          ← pose_utils, uav_utils, quadrotor_msgs 等
```

## Review 建议路线

### 第1层：数据流（先看懂"谁连谁"）

起点：启动文件 → 理解节点间 topic 连接。

| 文件 | 说明 |
|------|------|
| `single_run_in_sim_fusion.launch.py` | 融合链路完整启动编排 |
| `single_run_in_sim.launch.py` | 原始（非融合）启动 |
| `advanced_param.launch.py` | 规划器参数注入 |

融合链路数据流：

```
/map_generator/global_cloud (GT 地图)
    ├─→ simulated_lidar_cloud.py → /drone_0_lidar/points
    └─→ pcl_render_node → /drone_0_pcl_render_node/cloud
                        ↓
         ros2_lidar_depth_fusion_node.py (自适应融合)
                        ↓
              /drone_0_fusion/fused_cloud
                        ↓
              grid_map/cloud (remap) → EGO-Planner
```

### 第2层：核心规划器（C++）

建议按此顺序读，从上游到下游：

**① `plan_env/grid_map.cpp`** (1118 行)
- 避障的环境基础
- `GridMap` 类：点云 → 栅格地图构建、ESDF、最近邻查询
- Review 重点：数据结构设计、性能热点（尤其是 `getDistance()` 和 `getOccupancy()` 的调用路径）

**② `path_searching/dyn_a_star.cpp`** (261 行)
- `AStar`：动力学约束 A* 路径搜索
- 在 `GridMap` 上做路径粗搜索

**③ `bspline_opt/bspline_optimizer.cpp`** (1861 行，最大单文件)
- `BsplineOptimizer`：B样条轨迹优化（核心算法）
- 优化变量：控制点位置
- 代价项：平滑度、碰撞、可行/动力学约束
- Review 重点：数值稳定性、代价函数权重设计、收敛条件

**④ `bspline_opt/uniform_bspline.cpp`** (377 行)
- `UniformBspline`：B样条基础计算（求值、求导、时间参数化）

**⑤ `plan_manage/ego_replan_fsm.cpp`** (985 行)
- `EGOReplanFSM`：有限状态机 + 重规划决策
- 状态：INIT → WAIT_TRIGGER → PLAN → REPLAN → EXEC → EMERGENCY
- Review 重点：状态转移逻辑、重规划触发时机、异常处理

**⑥ `plan_manage/planner_manager.cpp`** (584 行)
- `EGOPlannerManager`：统筹搜索 + 优化 + 轨迹执行
- 调用 A* → BsplineOptimizer 的串联逻辑

### 第3层：多模态融合（核心创新点）

**⑦ `ros2_lidar_depth_fusion_node.py`** (885 行)
- Voxel-level log-odds 融合
- `VoxelEvidence`：证据累积，每个 voxel 独立跟踪 logit_sum / hits
- 传感器模型：深度相机近场置信度高、LiDAR 中远场置信度高
- 在线自适应：默认搜索 `min_probability`、`min_hits`、`dual_bonus`，使融合 F1 / Recall 相对于最佳单传感器增益最大
- 闭环反馈：订阅 `/drone_0_fusion/closed_loop_feedback`，把路径长度、安全侵入比例、轨迹平滑度和地图抖动作为候选评分的二级修正

**⑧ `multi_sensor_fusion_node.cpp`** (514 行)
- C++ 版融合（可能是早期版本或备选实现）
- 对比 Python 版：性能、功能差异

**⑨ `simulated_lidar_cloud.py`** (172 行)
- 从 /map_generator/global_cloud 裁剪出 LiDAR 模拟点云
- Review 重点：模拟真实性（FOV、遮挡、降采样策略）

### 第4层：评测与工具链

| 文件 | 说明 |
|------|------|
| `fusion_benefit_report.py` | 融合收益指标（F1、Recall、增益比） |
| `sim_flight_stats_report.py` | 避障飞行统计（replan 次数、路径长度、安全距离、地图抖动） |
| `closed_loop_score.py` | 感知-规划联合收益评分 J |
| `compare_fusion.sh` | A/B 批量评估入口 |
| `run_adaptive_rviz_demo.sh` | 一键 RViz 演示 |
| `plot_fusion_benefit.py` | 指标可视化 |

## Review 检查清单

### 正确性
- [ ] 状态机是否有路径走到未定义状态？
- [ ] 融合节点 ApproximateTime 同步策略在丢帧时是否安全？
- [ ] GridMap 点云转栅格时坐标系变换是否正确？
- [ ] B-spline 优化是否有退化解（奇异控制点）？

### 性能
- [ ] GridMap `getDistance()` O(1) 还是 O(n)？有无缓存策略？
- [ ] 融合节点每帧遍历 voxel 数是否可控？
- [ ] 自适应调参的评估范围（adaptive_eval_range）对 CPU 开销的影响

### 质量
- [ ] Magic number 是否集中在开头或 header？
- [ ] ROS2 参数是否全部 declare_parameter（而非硬编码）？
- [ ] Python 节点是否有内存泄漏（list 无限增长）？
- [ ] Launch 文件是否有参数默认值/类型文档？

### 融合专项
- [ ] 自适应评分函数 `gain_f1 + 0.35 * gain_recall` 的 0.35 是否可调？
- [ ] closed-loop feedback 权重是否足够小，避免规划噪声主导融合候选选择？
- [ ] `adaptive_lidar_growth`、`adaptive_z_max` 等负收益方向是否保持默认关闭？
- [ ] 自适应线程与融合发布线程是否有竞态？
- [ ] 融合结果对 depth_decay / lidar_growth / near_field_radius 的敏感性

## 关键类关系图

```
GridMap ─────────── AStar ──────┐
  │                              │
  │  getDistance()/Occupancy()   │
  │                              ↓
  └───────────────── BsplineOptimizer
                            │
                      UniformBspline
                            │
                     EGOPlannerManager
                            │
                      EGOReplanFSM (状态机)
                            │
                     EGOPlannerNode (ROS 节点)
```

## 快速入口

```bash
# 1. 编译
colcon build --packages-select ego_planner

# 2. 启动融合仿真
bash tools/run_adaptive_rviz_demo.sh

# 3. 批量 headless 评测
bash tools/compare_fusion.sh --label my_review_run

# 4. 查看结果
cat artifacts/headless_eval/my_review_run.summary.json
```
