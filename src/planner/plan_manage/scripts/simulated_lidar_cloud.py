#!/usr/bin/python3
from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2


class SimulatedLidarCloud(Node):
    def __init__(self) -> None:
        super().__init__('simulated_lidar_cloud')

        self.declare_parameter('global_cloud_topic', '/map_generator/global_cloud')
        self.declare_parameter('dynamic_cloud_topic', '')
        self.declare_parameter('dynamic_cloud_timeout_sec', 0.5)
        self.declare_parameter('odom_topic', '/drone_0_visual_slam/odom')
        self.declare_parameter('lidar_points_topic', '/drone_0_lidar/points')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('horizontal_fov_deg', 270.0)
        self.declare_parameter('vertical_min_deg', -20.0)
        self.declare_parameter('vertical_max_deg', 20.0)
        self.declare_parameter('voxel_size', 0.15)
        self.declare_parameter('keep_ratio', 0.55)
        self.declare_parameter('min_height', -0.1)
        self.declare_parameter('max_height', 3.5)
        self.declare_parameter('force_zero_stamp', False)

        self.global_cloud_topic = str(self.get_parameter('global_cloud_topic').value)
        self.dynamic_cloud_topic = str(self.get_parameter('dynamic_cloud_topic').value).strip()
        self.dynamic_cloud_timeout_sec = float(self.get_parameter('dynamic_cloud_timeout_sec').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.lidar_points_topic = str(self.get_parameter('lidar_points_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.horizontal_fov_deg = float(self.get_parameter('horizontal_fov_deg').value)
        self.vertical_min_deg = float(self.get_parameter('vertical_min_deg').value)
        self.vertical_max_deg = float(self.get_parameter('vertical_max_deg').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.keep_ratio = float(self.get_parameter('keep_ratio').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.force_zero_stamp = bool(self.get_parameter('force_zero_stamp').value)

        self.have_odom = False
        self.have_global = False
        self.position = np.zeros(3, dtype=np.float32)
        self.yaw = 0.0
        self.last_odom_stamp = None
        self.global_points = np.empty((0, 3), dtype=np.float32)
        self.dynamic_points = np.empty((0, 3), dtype=np.float32)
        self.last_dynamic_wall = 0.0
        self.rng = np.random.default_rng(7)

        self.create_subscription(PointCloud2, self.global_cloud_topic, self.global_cloud_callback, 10)
        if self.dynamic_cloud_topic:
            self.create_subscription(PointCloud2, self.dynamic_cloud_topic, self.dynamic_cloud_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.pub = self.create_publisher(PointCloud2, self.lidar_points_topic, 10)
        self.timer = self.create_timer(1.0 / max(1e-3, self.publish_rate), self.publish_cloud)

        self.get_logger().info(
            f'simulated lidar ready. global={self.global_cloud_topic} dynamic={self.dynamic_cloud_topic or "<none>"} '
            f'odom={self.odom_topic} out={self.lidar_points_topic}'
        )

    def global_cloud_callback(self, msg: PointCloud2) -> None:
        pts = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ]
        if not pts:
            self.global_points = np.empty((0, 3), dtype=np.float32)
            self.have_global = True
            return
        self.global_points = np.asarray(pts, dtype=np.float32)
        self.have_global = True

    def dynamic_cloud_callback(self, msg: PointCloud2) -> None:
        pts = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ]
        if not pts:
            self.dynamic_points = np.empty((0, 3), dtype=np.float32)
        else:
            self.dynamic_points = np.asarray(pts, dtype=np.float32)
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
        if not self.have_global or not self.have_odom or self.global_points.size == 0:
            return

        pts = self.combined_points()
        rel = pts - self.position[None, :]
        ranges = np.linalg.norm(rel, axis=1)
        mask = (ranges > 0.3) & (ranges <= self.max_range)
        mask &= (pts[:, 2] >= self.min_height) & (pts[:, 2] <= self.max_height)
        if not np.any(mask):
            self.publish_points(np.empty((0, 3), dtype=np.float32))
            return

        rel = rel[mask]
        pts = pts[mask]
        ranges = ranges[mask]

        horiz = np.degrees(np.arctan2(rel[:, 1], rel[:, 0]) - self.yaw)
        horiz = (horiz + 180.0) % 360.0 - 180.0
        horiz_half = self.horizontal_fov_deg / 2.0
        mask = np.abs(horiz) <= horiz_half
        if not np.any(mask):
            self.publish_points(np.empty((0, 3), dtype=np.float32))
            return

        rel = rel[mask]
        pts = pts[mask]
        ranges = ranges[mask]
        vertical = np.degrees(np.arctan2(rel[:, 2], np.linalg.norm(rel[:, :2], axis=1) + 1e-6))
        mask = (vertical >= self.vertical_min_deg) & (vertical <= self.vertical_max_deg)
        if not np.any(mask):
            self.publish_points(np.empty((0, 3), dtype=np.float32))
            return

        pts = pts[mask]
        if pts.shape[0] == 0:
            self.publish_points(np.empty((0, 3), dtype=np.float32))
            return

        keep = self.rng.random(pts.shape[0]) <= self.keep_ratio
        pts = pts[keep]
        if pts.shape[0] == 0:
            self.publish_points(np.empty((0, 3), dtype=np.float32))
            return

        keys = np.floor(pts / self.voxel_size).astype(np.int32)
        _, unique_indices = np.unique(keys, axis=0, return_index=True)
        pts = pts[np.sort(unique_indices)]
        self.publish_points(pts)

    def combined_points(self) -> np.ndarray:
        if self.dynamic_points.size == 0:
            return self.global_points
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_dynamic_wall > self.dynamic_cloud_timeout_sec:
            return self.global_points
        if self.global_points.size == 0:
            return self.dynamic_points
        return np.vstack((self.global_points, self.dynamic_points))

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
        msg = point_cloud2.create_cloud_xyz32(header, pts.tolist())
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = SimulatedLidarCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
