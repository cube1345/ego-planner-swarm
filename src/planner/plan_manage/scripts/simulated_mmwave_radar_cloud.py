#!/usr/bin/python3
from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class SimulatedMmwaveRadarCloud(Node):
    def __init__(self) -> None:
        super().__init__("simulated_mmwave_radar_cloud")

        self.declare_parameter("global_cloud_topic", "/map_generator/global_cloud")
        self.declare_parameter("dynamic_cloud_topic", "")
        self.declare_parameter("dynamic_cloud_timeout_sec", 0.5)
        self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom")
        self.declare_parameter("radar_points_topic", "/drone_0_radar/points")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("publish_rate", 12.0)
        self.declare_parameter("max_range", 10.0)
        self.declare_parameter("min_range", 0.45)
        self.declare_parameter("horizontal_fov_deg", 130.0)
        self.declare_parameter("vertical_min_deg", -12.0)
        self.declare_parameter("vertical_max_deg", 16.0)
        self.declare_parameter("voxel_size", 0.24)
        self.declare_parameter("static_keep_ratio", 0.16)
        self.declare_parameter("dynamic_keep_ratio", 0.85)
        self.declare_parameter("range_noise_std", 0.035)
        self.declare_parameter("lateral_noise_std", 0.018)
        self.declare_parameter("min_height", -0.1)
        self.declare_parameter("max_height", 3.5)
        self.declare_parameter("force_zero_stamp", False)

        self.global_cloud_topic = str(self.get_parameter("global_cloud_topic").value)
        self.dynamic_cloud_topic = str(self.get_parameter("dynamic_cloud_topic").value).strip()
        self.dynamic_cloud_timeout_sec = float(
            self.get_parameter("dynamic_cloud_timeout_sec").value
        )
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.radar_points_topic = str(self.get_parameter("radar_points_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.horizontal_fov_deg = float(self.get_parameter("horizontal_fov_deg").value)
        self.vertical_min_deg = float(self.get_parameter("vertical_min_deg").value)
        self.vertical_max_deg = float(self.get_parameter("vertical_max_deg").value)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.static_keep_ratio = min(1.0, max(0.0, float(self.get_parameter("static_keep_ratio").value)))
        self.dynamic_keep_ratio = min(1.0, max(0.0, float(self.get_parameter("dynamic_keep_ratio").value)))
        self.range_noise_std = max(0.0, float(self.get_parameter("range_noise_std").value))
        self.lateral_noise_std = max(0.0, float(self.get_parameter("lateral_noise_std").value))
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.force_zero_stamp = bool(self.get_parameter("force_zero_stamp").value)

        self.have_odom = False
        self.have_global = False
        self.position = np.zeros(3, dtype=np.float32)
        self.yaw = 0.0
        self.last_odom_stamp = None
        self.global_points = np.empty((0, 3), dtype=np.float32)
        self.dynamic_points = np.empty((0, 3), dtype=np.float32)
        self.last_dynamic_wall = 0.0
        self.rng = np.random.default_rng(19)

        self.create_subscription(PointCloud2, self.global_cloud_topic, self.global_cloud_callback, 10)
        if self.dynamic_cloud_topic:
            self.create_subscription(PointCloud2, self.dynamic_cloud_topic, self.dynamic_cloud_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.pub = self.create_publisher(PointCloud2, self.radar_points_topic, 10)
        self.timer = self.create_timer(1.0 / max(1e-3, self.publish_rate), self.publish_cloud)

        self.get_logger().info(
            f"simulated mmwave radar ready. global={self.global_cloud_topic} "
            f"dynamic={self.dynamic_cloud_topic or '<none>'} odom={self.odom_topic} "
            f"out={self.radar_points_topic}"
        )

    def global_cloud_callback(self, msg: PointCloud2) -> None:
        self.global_points = self.cloud_to_xyz(msg)
        self.have_global = True

    def dynamic_cloud_callback(self, msg: PointCloud2) -> None:
        self.dynamic_points = self.cloud_to_xyz(msg)
        self.last_dynamic_wall = self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, msg: Odometry) -> None:
        self.position[:] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.last_odom_stamp = msg.header.stamp
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.have_odom = True

    def publish_cloud(self) -> None:
        if not self.have_global or not self.have_odom:
            return
        static_pts = self.filter_by_radar_frustum(self.global_points)
        dynamic_pts = self.filter_by_radar_frustum(self.fresh_dynamic_points())

        static_pts = self.sample_points(static_pts, self.static_keep_ratio)
        dynamic_pts = self.sample_points(dynamic_pts, self.dynamic_keep_ratio)

        if static_pts.size and dynamic_pts.size:
            pts = np.vstack((static_pts, dynamic_pts))
        elif static_pts.size:
            pts = static_pts
        else:
            pts = dynamic_pts

        pts = self.add_measurement_noise(pts)
        pts = self.voxel_downsample(pts)
        self.publish_points(pts)

    def fresh_dynamic_points(self) -> np.ndarray:
        if self.dynamic_points.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_dynamic_wall > self.dynamic_cloud_timeout_sec:
            return np.empty((0, 3), dtype=np.float32)
        return self.dynamic_points

    def filter_by_radar_frustum(self, pts: np.ndarray) -> np.ndarray:
        if pts.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        rel = pts - self.position[None, :]
        ranges = np.linalg.norm(rel, axis=1)
        mask = (ranges >= self.min_range) & (ranges <= self.max_range)
        mask &= (pts[:, 2] >= self.min_height) & (pts[:, 2] <= self.max_height)
        if not np.any(mask):
            return np.empty((0, 3), dtype=np.float32)

        rel = rel[mask]
        pts = pts[mask]
        horiz = np.degrees(np.arctan2(rel[:, 1], rel[:, 0]) - self.yaw)
        horiz = (horiz + 180.0) % 360.0 - 180.0
        mask = np.abs(horiz) <= self.horizontal_fov_deg / 2.0
        if not np.any(mask):
            return np.empty((0, 3), dtype=np.float32)

        rel = rel[mask]
        pts = pts[mask]
        vertical = np.degrees(
            np.arctan2(rel[:, 2], np.linalg.norm(rel[:, :2], axis=1) + 1e-6)
        )
        mask = (vertical >= self.vertical_min_deg) & (vertical <= self.vertical_max_deg)
        return pts[mask] if np.any(mask) else np.empty((0, 3), dtype=np.float32)

    def sample_points(self, pts: np.ndarray, keep_ratio: float) -> np.ndarray:
        if pts.size == 0 or keep_ratio <= 0.0:
            return np.empty((0, 3), dtype=np.float32)
        keep = self.rng.random(pts.shape[0]) <= keep_ratio
        return pts[keep]

    def add_measurement_noise(self, pts: np.ndarray) -> np.ndarray:
        if pts.size == 0 or (self.range_noise_std <= 0.0 and self.lateral_noise_std <= 0.0):
            return pts
        rel = pts - self.position[None, :]
        ranges = np.linalg.norm(rel, axis=1)
        unit = rel / np.maximum(ranges[:, None], 1e-6)
        radial = unit * self.rng.normal(0.0, self.range_noise_std, size=(pts.shape[0], 1))
        lateral = self.rng.normal(0.0, self.lateral_noise_std, size=pts.shape).astype(np.float32)
        return (pts + radial.astype(np.float32) + lateral).astype(np.float32)

    def voxel_downsample(self, pts: np.ndarray) -> np.ndarray:
        if pts.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        keys = np.floor(pts / self.voxel_size).astype(np.int32)
        _, unique_indices = np.unique(keys, axis=0, return_index=True)
        return pts[np.sort(unique_indices)]

    def publish_points(self, pts: np.ndarray) -> None:
        header = Header()
        if self.force_zero_stamp:
            header.stamp.sec = 0
            header.stamp.nanosec = 0
        elif self.last_odom_stamp is not None:
            header.stamp = self.last_odom_stamp
        else:
            header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, pts.tolist()))

    @staticmethod
    def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
        pts = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        if not pts:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32).reshape((-1, 3))


def main() -> None:
    rclpy.init()
    node = SimulatedMmwaveRadarCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
