#!/usr/bin/env python3
"""Depth-adaptive LiDAR + depth-cloud fusion for the current EGO-Planner stack.

This node is designed for ROS 2 Humble and should be run from a Python 3.10
Conda environment after sourcing the system ROS environment.

Algorithm choice:
- Practical occupancy-evidence fusion adapted from probabilistic occupancy
  mapping (OctoMap-style log-odds evidence fusion) and the recent insight that
  sensor importance varies with depth (depth-adaptive multimodal fusion).
- Near field: depth camera gets higher confidence.
- Mid/far field: LiDAR gets higher confidence.
- Output: a fused occupied PointCloud2 in the planner/world frame, ready to be
  remapped into the existing `grid_map/cloud` input of this repository.
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Iterable, Tuple

import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def clamp_prob(p: np.ndarray, eps: float = 1e-3) -> np.ndarray:
    return np.clip(p, eps, 1.0 - eps)


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass
class VoxelEvidence:
    logit_sum: float = 0.0
    hits: int = 0
    seen_depth: bool = False
    seen_lidar: bool = False


class LidarDepthFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_depth_fusion_node")

        self.declare_parameter("depth_cloud_topic", "/drone_0_pcl_render_node/cloud")
        self.declare_parameter("lidar_cloud_topic", "/lidar/points")
        self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom")
        self.declare_parameter("output_topic", "/drone_0_fusion/fused_cloud")
        self.declare_parameter("output_frame", "world")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("z_min", -0.10)
        self.declare_parameter("z_max", 3.50)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("near_field_radius", 4.0)
        self.declare_parameter("depth_decay", 4.5)
        self.declare_parameter("lidar_growth", 5.0)
        self.declare_parameter("min_probability", 0.55)
        self.declare_parameter("min_hits", 1)
        self.declare_parameter("sync_queue", 10)
        self.declare_parameter("sync_slop", 0.08)
        self.declare_parameter("publish_debug_stats_every", 20)

        self.depth_cloud_topic = str(self.get_parameter("depth_cloud_topic").value)
        self.lidar_cloud_topic = str(self.get_parameter("lidar_cloud_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.near_field_radius = float(self.get_parameter("near_field_radius").value)
        self.depth_decay = float(self.get_parameter("depth_decay").value)
        self.lidar_growth = float(self.get_parameter("lidar_growth").value)
        self.min_probability = float(self.get_parameter("min_probability").value)
        self.min_hits = int(self.get_parameter("min_hits").value)
        sync_queue = int(self.get_parameter("sync_queue").value)
        sync_slop = float(self.get_parameter("sync_slop").value)
        self.debug_period = int(self.get_parameter("publish_debug_stats_every").value)

        self.vehicle_position = np.zeros(3, dtype=np.float64)
        self.have_odom = False
        self.frame_count = 0

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.fused_pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.depth_sub = message_filters.Subscriber(self, PointCloud2, self.depth_cloud_topic)
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, self.lidar_cloud_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.lidar_sub],
            queue_size=sync_queue,
            slop=sync_slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info(
            f"Fusion node ready. depth={self.depth_cloud_topic} "
            f"lidar={self.lidar_cloud_topic} out={self.output_topic} "
            f"frame={self.output_frame}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.vehicle_position[:] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.have_odom = True

    def sync_callback(self, depth_msg: PointCloud2, lidar_msg: PointCloud2) -> None:
        if not self.have_odom:
            self.get_logger().warn("Skipping fusion frame: odom not available yet.")
            return

        depth_pts = self.cloud_to_xyz(depth_msg)
        lidar_pts = self.cloud_to_xyz(lidar_msg)

        depth_pts = self.filter_points(depth_pts)
        lidar_pts = self.filter_points(lidar_pts)

        fused_points, stats = self.fuse_clouds(depth_pts, lidar_pts)
        template_msg = depth_msg if stamp_to_ns(depth_msg.header.stamp) >= stamp_to_ns(lidar_msg.header.stamp) else lidar_msg
        cloud_msg = self.xyz_to_cloud(fused_points, template_msg)
        self.fused_pub.publish(cloud_msg)

        self.frame_count += 1
        if self.frame_count % max(1, self.debug_period) == 0:
            self.get_logger().info(
                f"fusion frame={self.frame_count} depth_pts={stats['depth_points']} "
                f"lidar_pts={stats['lidar_points']} fused_voxels={stats['fused_voxels']} "
                f"depth_only={stats['depth_only_voxels']} lidar_only={stats['lidar_only_voxels']} "
                f"dual={stats['dual_voxels']}"
            )

    def cloud_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        pts = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        if not pts:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32).reshape((-1, 3))

    def filter_points(self, pts: np.ndarray) -> np.ndarray:
        if pts.size == 0:
            return pts
        mask = np.isfinite(pts).all(axis=1)
        pts = pts[mask]
        if pts.size == 0:
            return pts.reshape((-1, 3))
        mask = (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max)
        pts = pts[mask]
        if pts.size == 0:
            return pts.reshape((-1, 3))
        ranges = np.linalg.norm(pts - self.vehicle_position[None, :], axis=1)
        return pts[ranges <= self.max_range]

    def depth_probability(self, ranges: np.ndarray) -> np.ndarray:
        # Stronger in the near field, decays with distance.
        base = 0.25 + 0.55 * np.exp(-ranges / max(self.depth_decay, 1e-3))
        near_bonus = np.where(ranges <= self.near_field_radius, 0.10, 0.0)
        return clamp_prob(base + near_bonus)

    def lidar_probability(self, ranges: np.ndarray) -> np.ndarray:
        # More reliable with distance and for sparse geometry.
        base = 0.35 + 0.45 * (1.0 - np.exp(-ranges / max(self.lidar_growth, 1e-3)))
        far_bonus = np.where(ranges > self.near_field_radius, 0.10, 0.0)
        return clamp_prob(base + far_bonus)

    def accumulate_voxels(
        self,
        voxels: Dict[Tuple[int, int, int], VoxelEvidence],
        points: np.ndarray,
        modality: str,
    ) -> None:
        if points.size == 0:
            return
        keys = np.floor(points / self.resolution).astype(np.int32)
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        centers = (unique_keys.astype(np.float32) + 0.5) * self.resolution
        ranges = np.linalg.norm(centers - self.vehicle_position[None, :], axis=1)

        if modality == "depth":
            probs = self.depth_probability(ranges)
        else:
            probs = self.lidar_probability(ranges)

        voxel_counts = np.bincount(inverse, minlength=unique_keys.shape[0])
        hit_boost = np.minimum(np.log1p(voxel_counts), 1.6)
        logits = np.log(probs / (1.0 - probs)) * hit_boost

        for idx, key_arr in enumerate(unique_keys):
            key = (int(key_arr[0]), int(key_arr[1]), int(key_arr[2]))
            item = voxels[key]
            item.logit_sum += float(logits[idx])
            item.hits += int(voxel_counts[idx])
            if modality == "depth":
                item.seen_depth = True
            else:
                item.seen_lidar = True

    def fuse_clouds(self, depth_pts: np.ndarray, lidar_pts: np.ndarray) -> Tuple[np.ndarray, dict]:
        voxels: Dict[Tuple[int, int, int], VoxelEvidence] = defaultdict(VoxelEvidence)
        self.accumulate_voxels(voxels, depth_pts, "depth")
        self.accumulate_voxels(voxels, lidar_pts, "lidar")

        fused = []
        depth_only = 0
        lidar_only = 0
        dual = 0

        for key, evidence in voxels.items():
            probability = float(sigmoid(np.array([evidence.logit_sum], dtype=np.float32))[0])
            if probability < self.min_probability or evidence.hits < self.min_hits:
                continue
            if evidence.seen_depth and evidence.seen_lidar:
                dual += 1
            elif evidence.seen_depth:
                depth_only += 1
            else:
                lidar_only += 1
            fused.append(((np.array(key, dtype=np.float32) + 0.5) * self.resolution).tolist())

        if fused:
            fused_np = np.asarray(fused, dtype=np.float32)
        else:
            fused_np = np.empty((0, 3), dtype=np.float32)

        stats = {
            "depth_points": int(depth_pts.shape[0]),
            "lidar_points": int(lidar_pts.shape[0]),
            "fused_voxels": int(fused_np.shape[0]),
            "depth_only_voxels": depth_only,
            "lidar_only_voxels": lidar_only,
            "dual_voxels": dual,
        }
        return fused_np, stats

    def xyz_to_cloud(self, pts: np.ndarray, template_msg: PointCloud2) -> PointCloud2:
        header = template_msg.header
        if self.output_frame:
            header.frame_id = self.output_frame
        return point_cloud2.create_cloud_xyz32(header, pts.tolist())


def main() -> None:
    rclpy.init()
    node = LidarDepthFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
