#!/usr/bin/python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_OBSTACLE_SPECS = (
    "0.0,1.7,0.75,0.24,1.10,0.0,0.8,9.0,0.0"
)


@dataclass
class ObstacleSpec:
    base_x: float
    base_y: float
    base_z: float
    radius: float
    height: float
    amp_x: float
    amp_y: float
    period: float
    phase: float


class DynamicObstacleCloud(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_cloud")

        self.declare_parameter("cloud_topic", "/drone_0_dynamic_obstacles/cloud")
        self.declare_parameter("marker_topic", "/drone_0_dynamic_obstacles/markers")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("publish_rate", 15.0)
        self.declare_parameter("point_spacing", 0.12)
        self.declare_parameter("force_zero_stamp", True)
        self.declare_parameter("obstacle_specs", DEFAULT_OBSTACLE_SPECS)
        self.declare_parameter("pose_topic_prefix", "/dynamic/pose_")
        self.declare_parameter("legacy_marker_topic", "/dynamic/obj")
        self.declare_parameter("publish_legacy_prediction_topics", True)

        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.point_spacing = max(0.03, float(self.get_parameter("point_spacing").value))
        self.force_zero_stamp = bool(self.get_parameter("force_zero_stamp").value)
        self.specs = self.parse_specs(str(self.get_parameter("obstacle_specs").value))
        self.pose_topic_prefix = str(self.get_parameter("pose_topic_prefix").value)
        self.legacy_marker_topic = str(self.get_parameter("legacy_marker_topic").value)
        self.publish_legacy_prediction_topics = bool(
            self.get_parameter("publish_legacy_prediction_topics").value
        )

        self.cloud_pub = self.create_publisher(PointCloud2, self.cloud_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.pose_pubs = []
        self.legacy_marker_pub = None
        if self.publish_legacy_prediction_topics:
            self.pose_pubs = [
                self.create_publisher(PoseStamped, f"{self.pose_topic_prefix}{idx}", 10)
                for idx in range(len(self.specs))
            ]
            self.legacy_marker_pub = self.create_publisher(Marker, self.legacy_marker_topic, 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(1e-3, self.publish_rate), self.publish)

        self.get_logger().info(
            f"dynamic obstacles ready. count={len(self.specs)} cloud={self.cloud_topic} "
            f"markers={self.marker_topic} spacing={self.point_spacing:.2f}"
        )

    def parse_specs(self, raw: str) -> List[ObstacleSpec]:
        specs: List[ObstacleSpec] = []
        for item in raw.split(";"):
            item = item.strip()
            if not item:
                continue
            values = [float(part.strip()) for part in item.split(",")]
            if len(values) != 9:
                raise ValueError(
                    "Each obstacle spec must have 9 comma-separated values: "
                    "base_x,base_y,base_z,radius,height,amp_x,amp_y,period,phase"
                )
            specs.append(ObstacleSpec(*values))
        return specs

    def now_seconds(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def obstacle_center(self, spec: ObstacleSpec, t: float) -> np.ndarray:
        omega = 2.0 * math.pi / max(1e-3, spec.period)
        s = math.sin(omega * t + spec.phase)
        return np.array(
            [spec.base_x + spec.amp_x * s, spec.base_y + spec.amp_y * s, spec.base_z],
            dtype=np.float32,
        )

    def cylinder_points(self, center: np.ndarray, spec: ObstacleSpec) -> np.ndarray:
        radius = max(self.point_spacing, spec.radius)
        height = max(self.point_spacing, spec.height)
        xs = np.arange(-radius, radius + 0.5 * self.point_spacing, self.point_spacing)
        ys = np.arange(-radius, radius + 0.5 * self.point_spacing, self.point_spacing)
        zs = np.arange(-height * 0.5, height * 0.5 + 0.5 * self.point_spacing, self.point_spacing)
        pts = []
        for x in xs:
            for y in ys:
                if x * x + y * y > radius * radius:
                    continue
                for z in zs:
                    pts.append((center[0] + x, center[1] + y, center[2] + z))
        if not pts:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)

    def make_header(self) -> Header:
        header = Header()
        if self.force_zero_stamp:
            header.stamp.sec = 0
            header.stamp.nanosec = 0
        else:
            header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def make_live_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def make_marker(self, idx: int, spec: ObstacleSpec, center: np.ndarray, header: Header) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = "dynamic_obstacles"
        marker.id = idx
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 2.0 * spec.radius
        marker.scale.y = 2.0 * spec.radius
        marker.scale.z = spec.height
        r, g, b = self.marker_color(idx)
        marker.color.a = 0.65
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        return marker

    def publish_prediction_topics(self, idx: int, marker: Marker, live_header: Header) -> None:
        if not self.publish_legacy_prediction_topics:
            return
        if idx < len(self.pose_pubs):
            pose = PoseStamped()
            pose.header = live_header
            pose.pose = marker.pose
            self.pose_pubs[idx].publish(pose)
        if self.legacy_marker_pub is not None:
            legacy_marker = Marker()
            legacy_marker.header = live_header
            legacy_marker.ns = marker.ns
            legacy_marker.id = marker.id
            legacy_marker.type = marker.type
            legacy_marker.action = marker.action
            legacy_marker.pose = marker.pose
            legacy_marker.scale = marker.scale
            legacy_marker.color = marker.color
            self.legacy_marker_pub.publish(legacy_marker)

    def publish(self) -> None:
        t = self.now_seconds()
        clouds = []
        markers = MarkerArray()
        header = self.make_header()
        live_header = self.make_live_header()

        for idx, spec in enumerate(self.specs):
            center = self.obstacle_center(spec, t)
            clouds.append(self.cylinder_points(center, spec))

            marker = self.make_marker(idx, spec, center, header)
            markers.markers.append(marker)
            self.publish_prediction_topics(idx, marker, live_header)

        if clouds:
            pts = np.vstack(clouds)
        else:
            pts = np.empty((0, 3), dtype=np.float32)
        msg = point_cloud2.create_cloud_xyz32(header, pts.tolist())
        self.cloud_pub.publish(msg)
        self.marker_pub.publish(markers)

    def marker_color(self, idx: int) -> tuple[float, float, float]:
        palette = (
            (1.0, 0.25, 0.05),
            (1.0, 0.75, 0.10),
            (0.20, 0.85, 1.0),
            (0.55, 0.35, 1.0),
            (0.20, 1.0, 0.45),
        )
        return palette[idx % len(palette)]


def main() -> None:
    rclpy.init()
    node = DynamicObstacleCloud()
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
