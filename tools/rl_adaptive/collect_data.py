"""自适应参数训练数据采集节点（本地 ROS2 仿真运行）。

在本地跑仿真时启动本节点，它会订阅融合/规划/状态 topic，
把每帧的状态特征 + 当前融合参数 + 闭环指标写成一行 CSV，
供训练平台上的 train_q.py 离线训练。

运行方式（本地）：
    # 先正常启动融合仿真（single_run_in_sim_fusion.launch.py）
    python3 tools/rl_adaptive/collect_data.py \
        --out artifacts/adaptive_data.csv \
        --episode-limit 50

依赖：需要在 fusion 节点 ds_metrics 里额外发布当前参数
      min_probability / min_hits / dual_bonus（见下方 FUSION_PATCH 说明）。
      否则本节点无法知道「当前动作 a」是什么，训练将失去监督。

输出的 CSV 列名与 config.CSV_COLUMNS 对齐。
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import rclpy
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# 已加入 fusion 节点：publish_ds_metrics() 现在额外发布 min_probability /
# min_hits / dual_bonus，本节点直接从 ds_metrics 读当前动作，无需额外改动。
# ---------------------------------------------------------------------------


class DataCollector(Node):
    def __init__(self, out_path: Path, episode_limit: int):
        super().__init__("adaptive_data_collector")

        self.out_path = out_path
        self.episode_limit = episode_limit
        self.episode = 0
        self.step = 0
        self.goal = np.array([15.0, 0.0, 1.4], dtype=np.float64)  # 与仿真目标对齐

        # 最新状态缓存
        self.latest_ds = {}
        self.latest_odom = None
        self.occupancy_points = 0
        self.fused_points = 0
        self.fused_xyz = np.empty((0, 3), dtype=np.float32)  # 缓存 fused_cloud 点坐标

        # 闭环奖励信号缓存
        self.path_length = 0.0
        self.prev_pos = None
        self.replan_count = 0
        self.prev_occupancy = None
        self.safety_radius = 0.35

        self.csv_file = out_path.open("w", newline="", encoding="utf-8")
        self.writer = None  # 延迟到第一次写时初始化（需要列名）

        self.create_subscription(String, "/drone_0_fusion/ds_metrics", self.ds_cb, 10)
        self.create_subscription(Odometry, "/drone_0_visual_slam/odom", self.odom_cb, 10)
        self.create_subscription(
            PointCloud2, "/drone_0_grid/grid_map/occupancy_inflate", self.occ_cb, 10
        )
        self.create_subscription(
            PointCloud2, "/drone_0_fusion/fused_cloud", self.fused_cb, 10
        )

        # 每 0.2s 记录一行（与 sim_flight_stats 的采样节奏对齐）
        self.create_timer(0.2, self.record)

        self.get_logger().info("adaptive data collector ready")

    def ds_cb(self, msg: String):
        try:
            self.latest_ds = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def odom_cb(self, msg: Odometry):
        pos = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=np.float64,
        )
        if self.prev_pos is not None:
            self.path_length += float(np.linalg.norm(pos - self.prev_pos))
        self.prev_pos = pos
        self.latest_odom = msg

    def occ_cb(self, msg: PointCloud2):
        self.occupancy_points = sum(1 for _ in point_cloud2.read_points(msg, skip_nans=True))
        # 重规划代理：occupancy 点数突变（>8%）记为一次重规划
        if self.prev_occupancy is not None:
            denom = max(1, self.prev_occupancy)
            if abs(self.occupancy_points - self.prev_occupancy) / denom > 0.08:
                self.replan_count += 1
        self.prev_occupancy = self.occupancy_points

    def fused_cb(self, msg: PointCloud2):
        pts = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        self.fused_points = len(pts)
        if pts:
            self.fused_xyz = np.asarray(pts, dtype=np.float32)
        else:
            self.fused_xyz = np.empty((0, 3), dtype=np.float32)

    def min_obstacle_distance(self) -> float:
        """无人机到最近融合障碍点的欧氏距离（基于 fused_cloud）。

        无点云或 odom 未就绪时返回 3.0（视作远处，配合 config 里 /3.0 归一化后 ≈1）。
        """
        if self.latest_odom is None or self.fused_xyz.shape[0] == 0:
            return 3.0
        pos = np.array(
            [
                self.latest_odom.pose.pose.position.x,
                self.latest_odom.pose.pose.position.y,
                self.latest_odom.pose.pose.position.z,
            ],
            dtype=np.float32,
        )
        dists = np.linalg.norm(self.fused_xyz - pos[None, :], axis=1)
        return float(np.min(dists))

    def collision_risk(self) -> float:
        """碰撞风险：min_dist < safety_radius 时按比例线性惩罚，否则 0。"""
        d = self.min_obstacle_distance()
        if d >= self.safety_radius:
            return 0.0
        return float((self.safety_radius - d) / max(1e-6, self.safety_radius))

    def record(self):
        if self.latest_odom is None or not self.latest_ds:
            return

        # 从 ds_metrics 取当前动作（依赖 FUSION_PATCH）
        min_prob = float(self.latest_ds.get("min_probability", 0.30))
        min_hits = int(self.latest_ds.get("min_hits", 1))
        dual_bonus = float(self.latest_ds.get("dual_bonus", 0.0))

        # 状态特征（raw）
        pos = self.latest_odom.pose.pose.position
        vel = self.latest_odom.twist.twist.linear
        speed = float(np.linalg.norm([vel.x, vel.y, vel.z]))
        dist_to_goal = float(np.linalg.norm([pos.x - self.goal[0], pos.y - self.goal[1], pos.z - self.goal[2]]))

        row = {
            "episode": self.episode,
            "step": self.step,
            "action": _params_to_action_index(min_prob, min_hits, dual_bonus),
            "min_probability": min_prob,
            "min_hits": min_hits,
            "dual_bonus": dual_bonus,
            # 状态
            "occupancy_voxels": self.occupancy_points,
            "fused_voxels": self.latest_ds.get("fused_voxels", 0),
            "ds_unknown": self.latest_ds.get("unknown_mean", 0.0),
            "ds_conflict": self.latest_ds.get("conflict_mean", 0.0),
            "min_obstacle_distance": self.min_obstacle_distance(),
            "speed": speed,
            "distance_to_goal": dist_to_goal,
            "current_min_probability": min_prob,
            "current_min_hits": min_hits,
            "current_dual_bonus": dual_bonus,
            # 奖励信号（实时计算，无 GT 依赖）
            "collision_risk_score": self.collision_risk(),
            "path_length_m": self.path_length,
            "replan_count": self.replan_count,
            "ds_belief_occupied_mean": self.latest_ds.get("belief_occupied_mean", 0.0),
            "ds_conflict_mean": self.latest_ds.get("conflict_mean", 0.0),
            "done": 0,
            "goal_reached": int(dist_to_goal < 0.6),
        }

        if self.writer is None:
            from config import CSV_COLUMNS  # 依赖 config.py 在同目录

            self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_COLUMNS)
            self.writer.writeheader()

        self.writer.writerow(row)
        self.csv_file.flush()
        self.step += 1

        if dist_to_goal < 0.6:
            self.get_logger().info(f"episode {self.episode} goal reached, steps={self.step}")
            self.episode += 1
            self.step = 0
            if self.episode >= self.episode_limit:
                self.get_logger().info("episode limit reached, shutting down")
                self.csv_file.close()
                raise SystemExit


def _params_to_action_index(min_prob: float, min_hits: int, dual_bonus: float) -> int:
    """与 config.params_to_action 对齐（简单内联，避免 import 依赖）。"""
    from config import ACTION_SPACE

    best, best_d = 0, float("inf")
    for i, (mp, mh, db) in enumerate(ACTION_SPACE):
        d = abs(mp - min_prob) + 0.25 * abs(mh - min_hits) + 0.15 * abs(db - dual_bonus)
        if d < best_d:
            best_d, best = d, i
    return best


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="artifacts/adaptive_data.csv")
    p.add_argument("--episode-limit", type=int, default=50)
    args = p.parse_args()

    rclpy.init()
    node = DataCollector(Path(args.out), args.episode_limit)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
