---
name: flight-control
description: 将局部规划轨迹稳定接入 PX4/ArduPilot 飞控链路，并建立控制安全约束。
---

# Flight Control Integration

## 适用场景

- 你需要把 ROS 2 规划输出接到飞控（PX4/ArduPilot）。
- 你在选 `px4_ros_com`、`MAVROS`、`MAVSDK` 的接入方式。
- 你要定义 offboard 控制的频率、超时、降级策略。

## 推荐接入策略

1. PX4 优先使用 ROS 2 原生接口（`px4_msgs` + `px4_ros_com`）。
2. 需要 MAVLink 生态兼容时使用 `MAVROS`。
3. 高层任务编排（mission API）优先考虑 `MAVSDK`。

## 控制链路最小契约

- 输入：轨迹或 setpoint（位置/速度/加速度/yaw）。
- 中间：桥接节点做坐标系与频率整形。
- 输出：飞控 offboard 接口。
- 安全：当规划或感知超时，桥接节点必须触发保守模式。

## 实施要点

- 把“规划器坐标系”和“飞控坐标系”转换逻辑集中在一个节点。
- 桥接节点维护心跳与模式切换，不让 planner 直接切 mode。
- 所有控制命令附带时间戳与有效期，超时即丢弃。

## 在本仓库的落地建议

- 保持 `traj_server` 只负责轨迹下发，不混入飞控模式管理。
- 新增 `controller_bridge` 节点接收 `drone_X_planning/pos_cmd` 后再发飞控。
- 先做 SITL/HITL，再上真机。

## 参考来源（联网核验）

- PX4 ROS 2 User Guide：https://docs.px4.io/main/en/ros2/
- PX4 `px4_ros_com`：https://github.com/PX4/px4_ros_com
- PX4 `px4_msgs`：https://github.com/PX4/px4_msgs
- MAVROS（ROS2 支持见主仓库）：https://github.com/mavlink/mavros
- MAVSDK：https://mavsdk.mavlink.io/main/en/
