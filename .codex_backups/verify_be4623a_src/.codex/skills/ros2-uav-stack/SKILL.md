---
name: ros2-uav-stack
description: 在 ROS 2 环境下搭建无人机避障栈（感知-地图-规划-控制）并定义稳定的数据契约。
---

# ROS2 UAV Stack

## 适用场景

- 你要把新传感器或新规划器接入现有 ROS 2 无人机项目。
- 你需要定义 topic、frame、QoS、时序与 watchdog 约束。
- 你正在做从仿真到真机的一致性迁移。

## 工作流

1. 锁定 ROS 2 distro 与中间件版本（避免跨版本接口漂移）。
2. 画清数据链：`state_estimation -> mapping/fusion -> planner -> controller_bridge`。
3. 明确每条 topic 的 `msg type / frame_id / rate / QoS / timeout`。
4. 在 launch 层做 remap，不在算法源码中硬编码 topic。
5. 做延迟预算：传感器、融合、规划、控制各自耗时和端到端总耗时。

## 必做约束

- 所有传感器消息必须时间戳有效，且可与 odom 对齐。
- `tf2` 树中 `base_link/camera/lidar/world` 的方向约定必须文档化。
- 规划器输入丢失时，必须有 fallback（悬停/减速/返航之一）。
- 先在仿真回放验证，再飞真机。

## 在本仓库的落地建议

- 用 launch remap 统一接入：优先改 `src/planner/plan_manage/launch/*.launch.py`。
- 保持 EGO-Planner 输入稳定：`grid_map/cloud`（融合点云）+ `grid_map/odom`。
- 新增节点时默认输出调试统计（帧率、丢帧率、延迟、空云占比）。

## 参考来源（联网核验）

- ROS 2 官方文档（Jazzy）：https://docs.ros.org/en/jazzy/
- ROS 2 QoS 概念：https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- ROS 2 tf2：https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html
