---
name: multimodal-fusion
description: 设计并实现深度相机+雷达/激光+IMU/里程计融合，输出给局部避障规划器。
---

# Multimodal Fusion

## 适用场景

- 深度相机近场好、雷达/激光远场稳，需要融合提升鲁棒性。
- 单一传感器在弱纹理、逆光、雨雾、反光场景下失效。
- 你要把融合结果喂给 `EGO-Planner` 的 `grid_map/cloud`。

## 融合层次（先分层再融合）

1. State fusion：IMU + 视觉/激光里程计，输出统一状态估计。
2. Geometry fusion：Depth cloud + LiDAR cloud，输出占据证据。
3. Planner fusion：把置信度映射到障碍风险，控制膨胀与阈值。

## 算法选型建议

- 经典滤波：`robot_localization`（EKF/UKF）做状态融合。
- 优化图方法：`VINS-Fusion`、`LIO-SAM`、`FAST-LIO` 等做高精度里程计。
- 学习方法：`BEVFusion` 类模型适合算力足够且有数据集训练场景。

## 工程规则

- 先完成外参与时间同步，再调融合权重。
- 近场提高 depth 权重，远场提高 lidar 权重。
- 任何融合节点都要输出：延迟、匹配成功率、空帧率。
- 不能把所有逻辑塞进一个节点，保持 `state` 与 `geometry` 解耦。

## 在本仓库的落地建议

- 使用 `src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py` 作为几何融合入口。
- 输出 topic 统一 remap 到 `grid_map/cloud`，避免修改 `plan_env` 核心逻辑。
- 先用 `single_run_in_sim_fusion.launch.py` 跑通，再接真机雷达。

## 参考来源（联网核验）

- `robot_localization`：https://github.com/cra-ros-pkg/robot_localization
- `VINS-Fusion`：https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- `LIO-SAM`：https://github.com/TixiaoShan/LIO-SAM
- `FAST-LIO`：https://github.com/hku-mars/FAST_LIO
- `BEVFusion`：https://github.com/mit-han-lab/bevfusion
- 标定工具 `Kalibr`：https://github.com/ethz-asl/kalibr
