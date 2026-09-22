# 术语表

## EGO-Planner

一种无人机局部轨迹规划方法，强调 ESDF-Free 和快速重规划。它根据 odom、目标点和局部障碍信息，优化出满足动力学约束的 B-spline 轨迹。

## ESDF

Euclidean Signed Distance Field，欧式有符号距离场。每个栅格保存到最近障碍物的距离。传统避障常用 ESDF 计算碰撞代价和梯度。

## ESDF-Free

不显式维护完整 ESDF 距离场。EGO-Planner 通过局部障碍点和轨迹控制点直接构造避障约束，减少 ESDF 更新成本。

## PointCloud2

ROS 中表达点云的标准消息类型。当前 depth、LiDAR、radar 和 fused cloud 都使用 `sensor_msgs/msg/PointCloud2`。

## 点云体素化

把连续三维点云按固定分辨率划分到 voxel 中。同一个 voxel 内多个点合并为同一个空间单元，便于融合、降噪和降低计算量。

## Voxel

三维栅格单元。可以理解为 3D 版本的 pixel。fusion 节点用 voxel 作为多模态证据对齐的基本单位。

## Log-Odds

概率的一种可加表达：

```text
L = log(p / (1 - p))
```

多模态证据可在 log-odds 空间相加，再通过 sigmoid 转回概率。

## Dempster-Shafer Evidence

一种证据融合方法，用 `occupied/free/unknown` 三类质量表达传感器判断。相比单一概率，它可以显式表示 unknown 和 conflict。

## D-S unknown

Dempster-Shafer 中的不确定性质量。unknown 高说明传感器证据不足，系统不应过度自信。

## D-S conflict

Dempster-Shafer 中的冲突量。conflict 高说明不同模态对同一 voxel 的判断矛盾。

## Recall

召回率：

```text
recall = TP / (TP + FN)
```

表示真实障碍中有多少被融合结果找到了。动态避障中 recall 太低可能漏障。

## Precision

精确率：

```text
precision = TP / (TP + FP)
```

表示融合结果中有多少是真的障碍。precision 太低会导致虚警和轨迹绕行。

## F1

precision 和 recall 的调和平均：

```text
F1 = 2 * precision * recall / (precision + recall)
```

用于综合评价融合结果。

## BMS

本项目语境中常用于描述轨迹平滑性、加速度或运动稳定性相关指标。具体含义应结合生成指标的脚本或报告确认，后续应在指标定义文件中固定解释，避免不同实验中含义漂移。

## dual_bonus

当同一个 voxel 同时被 depth 和 LiDAR 支持时，额外提高其有效占据概率的参数。它表达“多模态共同支持比单模态更可信”的思想。

当前引入 radar 后，后续可以考虑从 `dual_bonus` 扩展为更通用的 `multi_modal_bonus`，分别处理 depth+LiDAR、depth+radar、LiDAR+radar 和三模态共同支持。

## radar_requires_geometry_support

fusion 节点参数。为 True 时，radar-only voxel 不直接发布为障碍，必须有 depth 或 LiDAR 的几何支持。该参数用于减少 radar 噪声对 planner 的影响。

## ApproximateTimeSynchronizer

ROS message_filters 中的近似时间同步器。当前 depth 和 LiDAR 使用它同步；radar 不加入同步器，而是使用最新帧缓存，避免三路同步导致 callback 卡住。

## Grid Map

EGO-Planner 内部使用的局部占据地图。fusion 节点输出的 fused cloud 会进入 grid_map，影响后续碰撞检测和轨迹优化。

## B-spline

一种平滑曲线表达方式。EGO-Planner 输出 B-spline 轨迹，`traj_server` 再按时间采样生成位置控制命令。

## Replan

重规划。无人机飞行过程中根据新的 odom、目标和障碍物信息持续更新局部轨迹。

## Dynamic Obstacle

动态障碍物。当前由 `dynamic_obstacle_cloud.py` 模拟，并可进入 LiDAR/radar 点云链路，用于测试动态避障能力。
