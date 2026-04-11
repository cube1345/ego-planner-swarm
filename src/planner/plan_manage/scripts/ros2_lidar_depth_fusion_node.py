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
from typing import Dict, Iterable, Set, Tuple

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


@dataclass
class Scores:
    precision: float
    recall: float
    f1: float


class LidarDepthFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_depth_fusion_node")

        self.declare_parameter("depth_cloud_topic", "/drone_0_pcl_render_node/cloud")
        self.declare_parameter("lidar_cloud_topic", "/lidar/points")
        self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom")
        self.declare_parameter("output_topic", "/drone_0_fusion/fused_cloud")
        self.declare_parameter("output_frame", "world")
        self.declare_parameter("global_cloud_topic", "/map_generator/global_cloud")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("z_min", -0.10)
        self.declare_parameter("z_max", 3.50)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("near_field_radius", 4.0)
        self.declare_parameter("depth_decay", 4.5)
        self.declare_parameter("lidar_growth", 5.0)
        self.declare_parameter("min_probability", 0.30)
        self.declare_parameter("min_hits", 1)
        self.declare_parameter("adaptive_min_probability_enable", False)
        self.declare_parameter("adaptive_min_probability_min", 0.20)
        self.declare_parameter("adaptive_min_probability_max", 0.35)
        self.declare_parameter("adaptive_min_probability_step", 0.02)
        self.declare_parameter("adaptive_target_retention", 0.30)
        self.declare_parameter("adaptive_retention_band", 0.05)
        self.declare_parameter("adaptive_eval_range", 10.0)
        self.declare_parameter("adaptive_min_gt_voxels", 40)
        self.declare_parameter("adaptive_score_alpha", 0.35)
        self.declare_parameter("sync_queue", 10)
        self.declare_parameter("sync_slop", 0.08)
        self.declare_parameter("publish_debug_stats_every", 20)

        self.depth_cloud_topic = str(self.get_parameter("depth_cloud_topic").value)
        self.lidar_cloud_topic = str(self.get_parameter("lidar_cloud_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.global_cloud_topic = str(self.get_parameter("global_cloud_topic").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.near_field_radius = float(self.get_parameter("near_field_radius").value)
        self.depth_decay = float(self.get_parameter("depth_decay").value)
        self.lidar_growth = float(self.get_parameter("lidar_growth").value)
        self.min_probability = float(self.get_parameter("min_probability").value)
        self.min_hits = int(self.get_parameter("min_hits").value)
        self.adaptive_min_probability_enable = bool(
            self.get_parameter("adaptive_min_probability_enable").value
        )
        self.adaptive_min_probability_min = float(
            self.get_parameter("adaptive_min_probability_min").value
        )
        self.adaptive_min_probability_max = float(
            self.get_parameter("adaptive_min_probability_max").value
        )
        self.adaptive_min_probability_step = float(
            self.get_parameter("adaptive_min_probability_step").value
        )
        self.adaptive_target_retention = float(
            self.get_parameter("adaptive_target_retention").value
        )
        self.adaptive_retention_band = float(
            self.get_parameter("adaptive_retention_band").value
        )
        self.adaptive_eval_range = float(self.get_parameter("adaptive_eval_range").value)
        self.adaptive_min_gt_voxels = int(self.get_parameter("adaptive_min_gt_voxels").value)
        self.adaptive_score_alpha = float(self.get_parameter("adaptive_score_alpha").value)
        sync_queue = int(self.get_parameter("sync_queue").value)
        sync_slop = float(self.get_parameter("sync_slop").value)
        self.debug_period = int(self.get_parameter("publish_debug_stats_every").value)
        self.candidate_probabilities = self.build_probability_candidates()
        self.candidate_scores = {
            threshold: None for threshold in self.candidate_probabilities
        }
        if self.adaptive_min_probability_enable and self.candidate_probabilities:
            center_index = len(self.candidate_probabilities) // 2
            self.current_min_probability = float(self.candidate_probabilities[center_index])
        else:
            self.current_min_probability = float(self.min_probability)

        self.vehicle_position = np.zeros(3, dtype=np.float64)
        self.have_odom = False
        self.have_global = False
        self.frame_count = 0
        self.global_keys = np.empty((0, 3), dtype=np.int32)
        self.global_centers = np.empty((0, 3), dtype=np.float32)

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(PointCloud2, self.global_cloud_topic, self.global_cloud_callback, 10)
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

    def global_cloud_callback(self, msg: PointCloud2) -> None:
        points = self.cloud_to_xyz(msg)
        points = self.filter_points(points, range_limit=self.max_range)
        keys = self.voxel_keys(points)
        if keys.size == 0:
            self.global_keys = np.empty((0, 3), dtype=np.int32)
            self.global_centers = np.empty((0, 3), dtype=np.float32)
        else:
            self.global_keys = keys
            self.global_centers = (keys.astype(np.float32) + 0.5) * self.resolution
        self.have_global = True

    def sync_callback(self, depth_msg: PointCloud2, lidar_msg: PointCloud2) -> None:
        if not self.have_odom:
            self.get_logger().warn("Skipping fusion frame: odom not available yet.")
            return

        depth_pts = self.cloud_to_xyz(depth_msg)
        lidar_pts = self.cloud_to_xyz(lidar_msg)

        depth_pts = self.filter_points(depth_pts)
        lidar_pts = self.filter_points(lidar_pts)

        fusion_state = self.build_fusion_state(depth_pts, lidar_pts)
        self.auto_tune_min_probability(fusion_state, depth_pts, lidar_pts)
        fused_points, stats = self.select_fused_points(
            fusion_state, self.current_min_probability
        )
        template_msg = depth_msg if stamp_to_ns(depth_msg.header.stamp) >= stamp_to_ns(lidar_msg.header.stamp) else lidar_msg
        cloud_msg = self.xyz_to_cloud(fused_points, template_msg)
        self.fused_pub.publish(cloud_msg)

        self.frame_count += 1
        if self.frame_count % max(1, self.debug_period) == 0:
            self.get_logger().info(
                f"fusion frame={self.frame_count} depth_pts={stats['depth_points']} "
                f"lidar_pts={stats['lidar_points']} fused_voxels={stats['fused_voxels']} "
                f"depth_only={stats['depth_only_voxels']} lidar_only={stats['lidar_only_voxels']} "
                f"dual={stats['dual_voxels']} retention={stats['retention_ratio']:.3f} "
                f"min_prob={self.current_min_probability:.3f}"
            )

    def cloud_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        pts = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        if not pts:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32).reshape((-1, 3))

    def filter_points(self, pts: np.ndarray, range_limit: float | None = None) -> np.ndarray:
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
        max_allowed_range = self.max_range if range_limit is None else min(self.max_range, range_limit)
        return pts[ranges <= max_allowed_range]

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

    def build_fusion_state(self, depth_pts: np.ndarray, lidar_pts: np.ndarray) -> dict:
        voxels: Dict[Tuple[int, int, int], VoxelEvidence] = defaultdict(VoxelEvidence)
        self.accumulate_voxels(voxels, depth_pts, "depth")
        self.accumulate_voxels(voxels, lidar_pts, "lidar")

        candidate_voxels = len(voxels)
        voxel_probabilities: Dict[Tuple[int, int, int], float] = {}

        for key, evidence in voxels.items():
            probability = float(sigmoid(np.array([evidence.logit_sum], dtype=np.float32))[0])
            voxel_probabilities[key] = probability

        return {
            "voxels": voxels,
            "depth_points": int(depth_pts.shape[0]),
            "lidar_points": int(lidar_pts.shape[0]),
            "candidate_voxels": int(candidate_voxels),
            "voxel_probabilities": voxel_probabilities,
            "voxel_hits": {key: evidence.hits for key, evidence in voxels.items()},
        }

    def select_fused_points(self, fusion_state: dict, threshold: float) -> Tuple[np.ndarray, dict]:
        voxels = fusion_state["voxels"]
        voxel_probabilities = fusion_state["voxel_probabilities"]
        fused = []
        depth_only = 0
        lidar_only = 0
        dual = 0

        for key, evidence in voxels.items():
            probability = float(voxel_probabilities[key])
            if probability < threshold or evidence.hits < self.min_hits:
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
            "depth_points": fusion_state["depth_points"],
            "lidar_points": fusion_state["lidar_points"],
            "candidate_voxels": fusion_state["candidate_voxels"],
            "fused_voxels": int(fused_np.shape[0]),
            "depth_only_voxels": depth_only,
            "lidar_only_voxels": lidar_only,
            "dual_voxels": dual,
            "retention_ratio": float(
                fused_np.shape[0] / max(1, fusion_state["candidate_voxels"])
            ),
        }
        return fused_np, stats

    def auto_tune_min_probability(
        self, fusion_state: dict, depth_pts: np.ndarray, lidar_pts: np.ndarray
    ) -> None:
        if not self.adaptive_min_probability_enable:
            return
        if not self.have_global or self.global_keys.size == 0:
            return

        local_gt_mask = (
            np.linalg.norm(self.global_centers - self.vehicle_position[None, :], axis=1)
            <= self.adaptive_eval_range
        )
        if not np.any(local_gt_mask):
            return
        gt_keys = self.keys_to_set(self.global_keys[local_gt_mask])
        if len(gt_keys) < self.adaptive_min_gt_voxels:
            return

        depth_keys = self.points_to_local_eval_keys(depth_pts)
        lidar_keys = self.points_to_local_eval_keys(lidar_pts)
        depth_scores = self.compute_scores(depth_keys, gt_keys)
        lidar_scores = self.compute_scores(lidar_keys, gt_keys)
        best_single_recall = max(depth_scores.recall, lidar_scores.recall)
        best_single_f1 = max(depth_scores.f1, lidar_scores.f1)

        best_probability = self.current_min_probability
        best_score = None
        voxel_probabilities = fusion_state.get("voxel_probabilities", {})
        voxel_hits = fusion_state.get("voxel_hits", {})

        for threshold in self.candidate_probabilities:
            predicted = {
                key
                for key, probability in voxel_probabilities.items()
                if probability >= threshold and int(voxel_hits.get(key, 0)) >= self.min_hits
            }
            predicted = self.limit_keys_to_eval_range(predicted)
            scores = self.compute_scores(predicted, gt_keys)
            gain_recall = scores.recall - best_single_recall
            gain_f1 = scores.f1 - best_single_f1
            utility = gain_f1 + 0.35 * gain_recall
            previous = self.candidate_scores[threshold]
            ema_score = (
                utility
                if previous is None
                else (1.0 - self.adaptive_score_alpha) * previous
                + self.adaptive_score_alpha * utility
            )
            self.candidate_scores[threshold] = ema_score
            if best_score is None or ema_score > best_score + 1e-9:
                best_score = ema_score
                best_probability = threshold
            elif best_score is not None and abs(ema_score - best_score) <= 1e-9:
                best_probability = min(best_probability, threshold)

        self.current_min_probability = min(
            self.adaptive_min_probability_max,
            max(self.adaptive_min_probability_min, best_probability),
        )

    def build_probability_candidates(self) -> list[float]:
        lower = min(self.adaptive_min_probability_min, self.adaptive_min_probability_max)
        upper = max(self.adaptive_min_probability_min, self.adaptive_min_probability_max)
        step = max(self.adaptive_min_probability_step, 1e-3)
        candidate_values = []
        current = lower
        while current <= upper + 1e-9:
            candidate_values.append(round(current, 4))
            current += step
        if not candidate_values:
            candidate_values = [round(self.min_probability, 4)]
        return candidate_values

    def voxel_keys(self, pts: np.ndarray) -> np.ndarray:
        if pts.size == 0:
            return np.empty((0, 3), dtype=np.int32)
        keys = np.floor(pts / self.resolution).astype(np.int32)
        return np.unique(keys, axis=0)

    def keys_to_set(self, keys: np.ndarray) -> Set[Tuple[int, int, int]]:
        return {(int(k[0]), int(k[1]), int(k[2])) for k in keys}

    def points_to_local_eval_keys(self, pts: np.ndarray) -> Set[Tuple[int, int, int]]:
        keys = self.voxel_keys(pts)
        if keys.size == 0:
            return set()
        centers = (keys.astype(np.float32) + 0.5) * self.resolution
        mask = (
            np.linalg.norm(centers - self.vehicle_position[None, :], axis=1)
            <= self.adaptive_eval_range
        )
        if not np.any(mask):
            return set()
        return self.keys_to_set(keys[mask])

    def limit_keys_to_eval_range(self, keys: Set[Tuple[int, int, int]]) -> Set[Tuple[int, int, int]]:
        limited = set()
        for key in keys:
            center = (np.array(key, dtype=np.float32) + 0.5) * self.resolution
            if np.linalg.norm(center - self.vehicle_position) <= self.adaptive_eval_range:
                limited.add(key)
        return limited

    def compute_scores(
        self, pred: Set[Tuple[int, int, int]], gt: Set[Tuple[int, int, int]]
    ) -> Scores:
        pred_size = len(pred)
        gt_size = len(gt)
        if gt_size == 0:
            return Scores(precision=1.0, recall=1.0, f1=1.0)
        if pred_size == 0:
            return Scores(precision=0.0, recall=0.0, f1=0.0)
        true_positive = len(pred & gt)
        precision = true_positive / max(1, pred_size)
        recall = true_positive / max(1, gt_size)
        if precision + recall < 1e-9:
            f1 = 0.0
        else:
            f1 = 2.0 * precision * recall / (precision + recall)
        return Scores(precision=precision, recall=recall, f1=f1)

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
