#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Deque
from typing import List, Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


class ClosedLoopFeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__("closed_loop_feedback_node")

        self.declare_parameter("global_cloud_topic", "/map_generator/global_cloud")
        self.declare_parameter("occupancy_topic", "/drone_0_grid/grid_map/occupancy_inflate")
        self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom")
        self.declare_parameter("feedback_topic", "/drone_0_fusion/closed_loop_feedback")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("obstacle_voxel_size", 0.20)
        self.declare_parameter("z_min", -0.10)
        self.declare_parameter("z_max", 3.50)
        self.declare_parameter("safety_radius", 0.35)
        self.declare_parameter("distance_sample_stride", 10)
        self.declare_parameter("path_ref_m", 30.0)
        self.declare_parameter("replan_ref", 50.0)
        self.declare_parameter("accel_ref", 10.0)
        self.declare_parameter("jitter_ref", 0.20)
        self.declare_parameter("w_path", 0.10)
        self.declare_parameter("w_replan", 0.08)
        self.declare_parameter("w_collision", 0.35)
        self.declare_parameter("w_smoothness", 0.05)
        self.declare_parameter("w_jitter", 0.04)
        self.declare_parameter("score_alpha", 0.25)
        self.declare_parameter("window_sec", 8.0)

        self.global_cloud_topic = str(self.get_parameter("global_cloud_topic").value)
        self.occupancy_topic = str(self.get_parameter("occupancy_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.obstacle_voxel_size = float(self.get_parameter("obstacle_voxel_size").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.safety_radius = float(self.get_parameter("safety_radius").value)
        self.distance_sample_stride = max(1, int(self.get_parameter("distance_sample_stride").value))
        self.path_ref_m = float(self.get_parameter("path_ref_m").value)
        self.replan_ref = float(self.get_parameter("replan_ref").value)
        self.accel_ref = float(self.get_parameter("accel_ref").value)
        self.jitter_ref = float(self.get_parameter("jitter_ref").value)
        self.w_path = float(self.get_parameter("w_path").value)
        self.w_replan = float(self.get_parameter("w_replan").value)
        self.w_collision = float(self.get_parameter("w_collision").value)
        self.w_smoothness = float(self.get_parameter("w_smoothness").value)
        self.w_jitter = float(self.get_parameter("w_jitter").value)
        self.score_alpha = float(self.get_parameter("score_alpha").value)
        self.window_sec = max(1.0, float(self.get_parameter("window_sec").value))

        self.start_wall = time.time()
        self.obstacle_centers = np.empty((0, 3), dtype=np.float64)
        self.occ_point_counts: List[int] = []
        self.occ_latest_points = 0
        self.prev_occ_points: Optional[int] = None
        self.occupancy_change_events = 0

        self.odom_count = 0
        self.path_length = 0.0
        self.prev_pos: Optional[np.ndarray] = None
        self.prev_odom_time: Optional[float] = None
        self.prev_speed: Optional[float] = None
        self.accel_samples: List[float] = []
        self.min_obstacle_distance = float("inf")
        self.safety_violation_count = 0
        self.distance_sample_count = 0
        self.closed_loop_score_ema: Optional[float] = None
        self.path_segments_window: Deque[tuple[float, float]] = deque()
        self.accel_samples_window: Deque[tuple[float, float]] = deque()
        self.distance_samples_window: Deque[tuple[float, float]] = deque()
        self.occ_counts_window: Deque[tuple[float, int]] = deque()
        self.occ_event_times_window: Deque[float] = deque()

        self.create_subscription(PointCloud2, self.global_cloud_topic, self.global_cloud_callback, 10)
        self.create_subscription(PointCloud2, self.occupancy_topic, self.occupancy_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 100)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        period = 1.0 / max(1e-3, float(self.get_parameter("publish_rate").value))
        self.create_timer(period, self.publish_feedback)

        self.get_logger().info(
            f"closed-loop feedback ready. odom={self.odom_topic} "
            f"occupancy={self.occupancy_topic} global={self.global_cloud_topic} "
            f"out={self.feedback_topic} window_sec={self.window_sec:.1f}"
        )

    def global_cloud_callback(self, msg: PointCloud2) -> None:
        if self.obstacle_centers.size > 0:
            return
        points = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        if not points:
            return
        cloud = np.asarray(points, dtype=np.float64)
        mask = np.isfinite(cloud).all(axis=1)
        cloud = cloud[mask]
        mask = (cloud[:, 2] >= self.z_min) & (cloud[:, 2] <= self.z_max)
        cloud = cloud[mask]
        if cloud.size == 0:
            return
        keys = np.unique(np.floor(cloud / self.obstacle_voxel_size).astype(np.int32), axis=0)
        self.obstacle_centers = (keys.astype(np.float64) + 0.5) * self.obstacle_voxel_size

    def occupancy_callback(self, msg: PointCloud2) -> None:
        now = time.time()
        count = 0
        for _ in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            count += 1
        self.occ_latest_points = count
        self.occ_point_counts.append(count)
        self.occ_counts_window.append((now, count))
        if self.prev_occ_points is not None:
            denominator = max(1, self.prev_occ_points)
            if abs(count - self.prev_occ_points) / denominator > 0.08:
                self.occupancy_change_events += 1
                self.occ_event_times_window.append(now)
        self.prev_occ_points = count
        self.prune_window(now)

    def odom_callback(self, msg: Odometry) -> None:
        now = time.time()
        odom_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        pos = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=np.float64,
        )
        self.odom_count += 1
        if self.prev_pos is not None:
            dist = float(np.linalg.norm(pos - self.prev_pos))
            dt = max(1e-6, odom_time - (self.prev_odom_time or odom_time))
            self.path_length += dist
            self.path_segments_window.append((now, dist))
            speed = dist / dt
            if self.prev_speed is not None:
                accel = abs(speed - self.prev_speed) / dt
                self.accel_samples.append(accel)
                self.accel_samples_window.append((now, accel))
            self.prev_speed = speed
        self.prev_pos = pos
        self.prev_odom_time = odom_time
        self.update_obstacle_distance(pos, now)
        self.prune_window(now)

    def update_obstacle_distance(self, pos: np.ndarray, now: float) -> None:
        if self.obstacle_centers.size == 0:
            return
        if self.odom_count % self.distance_sample_stride != 0:
            return
        distances = np.linalg.norm(self.obstacle_centers - pos[None, :], axis=1)
        if distances.size == 0:
            return
        nearest = float(np.min(distances))
        self.distance_sample_count += 1
        self.min_obstacle_distance = min(self.min_obstacle_distance, nearest)
        self.distance_samples_window.append((now, nearest))
        if nearest < self.safety_radius:
            self.safety_violation_count += 1

    def publish_feedback(self) -> None:
        now = time.time()
        self.prune_window(now)
        elapsed = max(1e-6, time.time() - self.start_wall)
        window_path_length = sum(dist for _, dist in self.path_segments_window)
        window_replan_events = len(self.occ_event_times_window)
        window_accel_values = [value for _, value in self.accel_samples_window]
        window_distance_values = [value for _, value in self.distance_samples_window]
        window_occ_counts = [value for _, value in self.occ_counts_window]
        window_min_distance = min(window_distance_values) if window_distance_values else float("inf")
        window_violation_ratio = (
            sum(1 for value in window_distance_values if value < self.safety_radius)
            / max(1, len(window_distance_values))
        )

        path_penalty = self.clamp(window_path_length / max(1e-6, self.path_ref_m))
        replan_proxy = window_replan_events / max(1e-6, self.replan_ref)
        replan_penalty = self.clamp(replan_proxy)
        collision_risk = self.compute_collision_risk(window_min_distance)
        smoothness_penalty = self.clamp(self.compute_rms(window_accel_values) / max(1e-6, self.accel_ref))
        map_jitter = self.clamp(self.compute_jitter_ratio(window_occ_counts) / max(1e-6, self.jitter_ref))
        score = -(
            self.w_path * path_penalty
            + self.w_replan * replan_penalty
            + self.w_collision * collision_risk
            + self.w_smoothness * smoothness_penalty
            + self.w_jitter * map_jitter
        )
        if self.closed_loop_score_ema is None:
            self.closed_loop_score_ema = score
        else:
            self.closed_loop_score_ema = (
                (1.0 - self.score_alpha) * self.closed_loop_score_ema
                + self.score_alpha * score
            )

        payload = {
            "closed_loop_score": float(score),
            "closed_loop_score_ema": float(self.closed_loop_score_ema),
            "closed_loop_window_sec": float(self.window_sec),
            "window_path_length_m": float(window_path_length),
            "window_replan_proxy_count": int(window_replan_events),
            "window_collision_risk_score": float(collision_risk),
            "window_min_obstacle_distance_m": None
            if not math.isfinite(window_min_distance)
            else float(window_min_distance),
            "window_safety_violation_ratio": float(window_violation_ratio),
            "window_accel_rms_mps2": float(self.compute_rms(window_accel_values)),
            "window_occupancy_jitter_ratio": float(self.compute_jitter_ratio(window_occ_counts)),
            "path_length_m": float(self.path_length),
            "replan_proxy_count": int(self.occupancy_change_events),
            "collision_risk_score": float(collision_risk),
            "min_obstacle_distance_m": None
            if not math.isfinite(self.min_obstacle_distance)
            else float(self.min_obstacle_distance),
            "safety_violation_ratio": 0.0
            if self.distance_sample_count <= 0
            else float(self.safety_violation_count / max(1, self.distance_sample_count)),
            "accel_rms_mps2": float(self.compute_rms(self.accel_samples)),
            "occupancy_jitter_ratio": float(self.compute_jitter_ratio(self.occ_point_counts)),
            "elapsed_sec": float(elapsed),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.feedback_pub.publish(msg)

    def compute_collision_risk(self, min_distance: float) -> float:
        if not math.isfinite(min_distance):
            return 0.0
        if min_distance >= self.safety_radius:
            return 0.0
        return float((self.safety_radius - min_distance) / max(1e-6, self.safety_radius))

    def prune_window(self, now: float) -> None:
        cutoff = now - self.window_sec
        for values in (
            self.path_segments_window,
            self.accel_samples_window,
            self.distance_samples_window,
            self.occ_counts_window,
        ):
            while values and values[0][0] < cutoff:
                values.popleft()
        while self.occ_event_times_window and self.occ_event_times_window[0] < cutoff:
            self.occ_event_times_window.popleft()

    def compute_jitter_ratio(self, values: List[int]) -> float:
        if len(values) < 2:
            return 0.0
        arr = np.asarray(values, dtype=np.float64)
        mean = float(np.mean(arr))
        if abs(mean) < 1e-9:
            return 0.0
        return float(np.std(arr) / mean)

    def compute_rms(self, values: List[float]) -> float:
        if not values:
            return 0.0
        arr = np.asarray(values, dtype=np.float64)
        return float(math.sqrt(float(np.mean(arr * arr))))

    def clamp(self, value: float) -> float:
        return max(0.0, min(1.0, value))


def main() -> None:
    rclpy.init()
    node = ClosedLoopFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
