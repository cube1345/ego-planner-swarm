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

import json
import math
import time
from collections import deque
from collections import defaultdict
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, Set, Tuple

import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


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
    depth_occ_mass: float = 0.0
    depth_free_mass: float = 0.0
    depth_unknown_mass: float = 1.0
    lidar_occ_mass: float = 0.0
    lidar_free_mass: float = 0.0
    lidar_unknown_mass: float = 1.0


@dataclass
class Scores:
    precision: float
    recall: float
    f1: float


@dataclass
class ModalityVoxelState:
    keys: np.ndarray
    key_tuples: list[Tuple[int, int, int]]
    ranges: np.ndarray
    counts: np.ndarray


class LidarDepthFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_depth_fusion_node")

        self.declare_parameter("depth_cloud_topic", "/drone_0_pcl_render_node/cloud")
        self.declare_parameter("lidar_cloud_topic", "/lidar/points")
        self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom")
        self.declare_parameter("output_topic", "/drone_0_fusion/fused_cloud")
        self.declare_parameter("output_frame", "world")
        self.declare_parameter("global_cloud_topic", "/map_generator/global_cloud")
        self.declare_parameter("ds_metrics_topic", "/drone_0_fusion/ds_metrics")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("z_min", -0.10)
        self.declare_parameter("z_max", 3.50)
        self.declare_parameter("adaptive_z_max_enable", False)
        self.declare_parameter("adaptive_z_max_min", 3.0)
        self.declare_parameter("adaptive_z_max_max", 3.5)
        self.declare_parameter("adaptive_z_max_step", 0.25)
        self.declare_parameter("adaptive_depth_decay_enable", False)
        self.declare_parameter("adaptive_depth_decay_min", 3.5)
        self.declare_parameter("adaptive_depth_decay_max", 5.5)
        self.declare_parameter("adaptive_depth_decay_step", 1.0)
        self.declare_parameter("max_range", 12.0)
        self.declare_parameter("near_field_radius", 4.0)
        self.declare_parameter("adaptive_near_field_radius_enable", False)
        self.declare_parameter("adaptive_near_field_radius_min", 3.0)
        self.declare_parameter("adaptive_near_field_radius_max", 5.0)
        self.declare_parameter("adaptive_near_field_radius_step", 1.0)
        self.declare_parameter("depth_decay", 4.5)
        self.declare_parameter("lidar_growth", 5.0)
        self.declare_parameter("min_probability", 0.30)
        self.declare_parameter("min_hits", 1)
        self.declare_parameter("adaptive_min_probability_enable", False)
        self.declare_parameter("adaptive_min_probability_min", 0.20)
        self.declare_parameter("adaptive_min_probability_max", 0.35)
        self.declare_parameter("adaptive_min_probability_step", 0.02)
        self.declare_parameter("adaptive_min_hits_enable", False)
        self.declare_parameter("adaptive_min_hits_min", 1)
        self.declare_parameter("adaptive_min_hits_max", 3)
        self.declare_parameter("adaptive_target_retention", 0.30)
        self.declare_parameter("adaptive_retention_band", 0.05)
        self.declare_parameter("adaptive_retention_enable", False)
        self.declare_parameter("adaptive_lidar_growth_enable", False)
        self.declare_parameter("adaptive_lidar_growth_min", 3.8)
        self.declare_parameter("adaptive_lidar_growth_max", 4.2)
        self.declare_parameter("adaptive_lidar_growth_step", 0.2)
        self.declare_parameter("adaptive_dual_bonus_enable", False)
        self.declare_parameter("adaptive_dual_bonus_min", 0.0)
        self.declare_parameter("adaptive_dual_bonus_max", 0.2)
        self.declare_parameter("adaptive_dual_bonus_step", 0.1)
        self.declare_parameter("adaptive_eval_range", 10.0)
        self.declare_parameter("adaptive_min_gt_voxels", 40)
        self.declare_parameter("adaptive_score_alpha", 0.35)
        self.declare_parameter("closed_loop_feedback_enable", False)
        self.declare_parameter("closed_loop_feedback_topic", "/drone_0_fusion/closed_loop_feedback")
        self.declare_parameter("closed_loop_feedback_weight", 0.05)
        self.declare_parameter("closed_loop_feedback_alpha", 0.25)
        self.declare_parameter("closed_loop_optimizer_enable", True)
        self.declare_parameter("closed_loop_local_score_weight", 1.0)
        self.declare_parameter("closed_loop_candidate_score_alpha", 0.30)
        self.declare_parameter("closed_loop_action_delay_sec", 3.0)
        self.declare_parameter("closed_loop_action_history_sec", 20.0)
        self.declare_parameter("ds_evidence_enable", True)
        self.declare_parameter("ds_unknown_floor", 0.10)
        self.declare_parameter("ds_free_scale", 0.35)
        self.declare_parameter("sync_queue", 10)
        self.declare_parameter("sync_slop", 0.08)
        self.declare_parameter("publish_debug_stats_every", 20)

        self.depth_cloud_topic = str(self.get_parameter("depth_cloud_topic").value)
        self.lidar_cloud_topic = str(self.get_parameter("lidar_cloud_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.global_cloud_topic = str(self.get_parameter("global_cloud_topic").value)
        self.ds_metrics_topic = str(self.get_parameter("ds_metrics_topic").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.adaptive_z_max_enable = bool(
            self.get_parameter("adaptive_z_max_enable").value
        )
        self.adaptive_z_max_min = float(self.get_parameter("adaptive_z_max_min").value)
        self.adaptive_z_max_max = float(self.get_parameter("adaptive_z_max_max").value)
        self.adaptive_z_max_step = float(self.get_parameter("adaptive_z_max_step").value)
        self.adaptive_depth_decay_enable = bool(
            self.get_parameter("adaptive_depth_decay_enable").value
        )
        self.adaptive_depth_decay_min = float(
            self.get_parameter("adaptive_depth_decay_min").value
        )
        self.adaptive_depth_decay_max = float(
            self.get_parameter("adaptive_depth_decay_max").value
        )
        self.adaptive_depth_decay_step = float(
            self.get_parameter("adaptive_depth_decay_step").value
        )
        self.max_range = float(self.get_parameter("max_range").value)
        self.near_field_radius = float(self.get_parameter("near_field_radius").value)
        self.adaptive_near_field_radius_enable = bool(
            self.get_parameter("adaptive_near_field_radius_enable").value
        )
        self.adaptive_near_field_radius_min = float(
            self.get_parameter("adaptive_near_field_radius_min").value
        )
        self.adaptive_near_field_radius_max = float(
            self.get_parameter("adaptive_near_field_radius_max").value
        )
        self.adaptive_near_field_radius_step = float(
            self.get_parameter("adaptive_near_field_radius_step").value
        )
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
        self.adaptive_min_hits_enable = bool(
            self.get_parameter("adaptive_min_hits_enable").value
        )
        self.adaptive_min_hits_min = int(
            self.get_parameter("adaptive_min_hits_min").value
        )
        self.adaptive_min_hits_max = int(
            self.get_parameter("adaptive_min_hits_max").value
        )
        self.adaptive_target_retention = float(
            self.get_parameter("adaptive_target_retention").value
        )
        self.adaptive_retention_band = float(
            self.get_parameter("adaptive_retention_band").value
        )
        self.adaptive_retention_enable = bool(
            self.get_parameter("adaptive_retention_enable").value
        )
        self.adaptive_lidar_growth_enable = bool(
            self.get_parameter("adaptive_lidar_growth_enable").value
        )
        self.adaptive_lidar_growth_min = float(
            self.get_parameter("adaptive_lidar_growth_min").value
        )
        self.adaptive_lidar_growth_max = float(
            self.get_parameter("adaptive_lidar_growth_max").value
        )
        self.adaptive_lidar_growth_step = float(
            self.get_parameter("adaptive_lidar_growth_step").value
        )
        self.adaptive_dual_bonus_enable = bool(
            self.get_parameter("adaptive_dual_bonus_enable").value
        )
        self.adaptive_dual_bonus_min = float(
            self.get_parameter("adaptive_dual_bonus_min").value
        )
        self.adaptive_dual_bonus_max = float(
            self.get_parameter("adaptive_dual_bonus_max").value
        )
        self.adaptive_dual_bonus_step = float(
            self.get_parameter("adaptive_dual_bonus_step").value
        )
        self.adaptive_eval_range = float(self.get_parameter("adaptive_eval_range").value)
        self.adaptive_min_gt_voxels = int(self.get_parameter("adaptive_min_gt_voxels").value)
        self.adaptive_score_alpha = float(self.get_parameter("adaptive_score_alpha").value)
        self.closed_loop_feedback_enable = bool(
            self.get_parameter("closed_loop_feedback_enable").value
        )
        self.closed_loop_feedback_topic = str(
            self.get_parameter("closed_loop_feedback_topic").value
        )
        self.closed_loop_feedback_weight = float(
            self.get_parameter("closed_loop_feedback_weight").value
        )
        self.closed_loop_feedback_alpha = float(
            self.get_parameter("closed_loop_feedback_alpha").value
        )
        self.closed_loop_optimizer_enable = bool(
            self.get_parameter("closed_loop_optimizer_enable").value
        )
        self.closed_loop_local_score_weight = float(
            self.get_parameter("closed_loop_local_score_weight").value
        )
        self.closed_loop_candidate_score_alpha = float(
            self.get_parameter("closed_loop_candidate_score_alpha").value
        )
        self.closed_loop_action_delay_sec = max(
            0.0, float(self.get_parameter("closed_loop_action_delay_sec").value)
        )
        self.closed_loop_action_history_sec = max(
            self.closed_loop_action_delay_sec + 1.0,
            float(self.get_parameter("closed_loop_action_history_sec").value),
        )
        self.ds_evidence_enable = bool(self.get_parameter("ds_evidence_enable").value)
        self.ds_unknown_floor = min(
            0.95, max(0.0, float(self.get_parameter("ds_unknown_floor").value))
        )
        self.ds_free_scale = min(
            0.95, max(0.0, float(self.get_parameter("ds_free_scale").value))
        )
        sync_queue = int(self.get_parameter("sync_queue").value)
        sync_slop = float(self.get_parameter("sync_slop").value)
        self.debug_period = int(self.get_parameter("publish_debug_stats_every").value)
        self.support_dense_count_threshold = 2
        self.candidate_z_max_values = self.build_z_max_candidates()
        self.current_z_max = float(self.z_max)
        self.candidate_depth_decay_values = self.build_depth_decay_candidates()
        self.current_depth_decay = float(self.depth_decay)
        self.candidate_near_field_radii = self.build_near_field_radius_candidates()
        self.candidate_probabilities = self.build_probability_candidates()
        self.candidate_min_hits = self.build_min_hits_candidates()
        self.candidate_lidar_growths = self.build_lidar_growth_candidates()
        self.candidate_dual_bonuses = self.build_dual_bonus_candidates()
        self.candidate_scores = {
            (threshold, min_hits, lidar_growth, near_field_radius, dual_bonus): None
            for threshold in self.candidate_probabilities
            for min_hits in self.candidate_min_hits
            for lidar_growth in self.candidate_lidar_growths
            for near_field_radius in self.candidate_near_field_radii
            for dual_bonus in self.candidate_dual_bonuses
        }
        if self.adaptive_min_probability_enable and self.candidate_probabilities:
            center_index = len(self.candidate_probabilities) // 2
            self.current_min_probability = float(self.candidate_probabilities[center_index])
        else:
            self.current_min_probability = float(self.min_probability)
        if self.adaptive_near_field_radius_enable and self.candidate_near_field_radii:
            center_index = len(self.candidate_near_field_radii) // 2
            self.current_near_field_radius = float(
                self.candidate_near_field_radii[center_index]
            )
        else:
            self.current_near_field_radius = float(self.near_field_radius)
        if self.adaptive_min_hits_enable and self.candidate_min_hits:
            center_index = len(self.candidate_min_hits) // 2
            self.current_min_hits = int(self.candidate_min_hits[center_index])
        else:
            self.current_min_hits = int(self.min_hits)
        if self.adaptive_lidar_growth_enable and self.candidate_lidar_growths:
            center_index = len(self.candidate_lidar_growths) // 2
            self.current_lidar_growth = float(self.candidate_lidar_growths[center_index])
        else:
            self.current_lidar_growth = float(self.lidar_growth)
        if self.adaptive_dual_bonus_enable and self.candidate_dual_bonuses:
            center_index = len(self.candidate_dual_bonuses) // 2
            self.current_dual_bonus = float(self.candidate_dual_bonuses[center_index])
        else:
            self.current_dual_bonus = 0.0

        self.vehicle_position = np.zeros(3, dtype=np.float64)
        self.have_odom = False
        self.have_global = False
        self.frame_count = 0
        self.global_keys = np.empty((0, 3), dtype=np.int32)
        self.global_centers = np.empty((0, 3), dtype=np.float32)
        self.closed_loop_score_ema = 0.0
        self.closed_loop_feedback_ready = False
        self.closed_loop_candidate_scores: dict[Tuple[float, int, float, float, float], float] = {}
        self.closed_loop_action_history: Deque[
            tuple[float, Tuple[float, int, float, float, float]]
        ] = deque()
        self.record_current_candidate_action()

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(PointCloud2, self.global_cloud_topic, self.global_cloud_callback, 10)
        if self.closed_loop_feedback_enable:
            self.create_subscription(
                String, self.closed_loop_feedback_topic, self.closed_loop_feedback_callback, 10
            )
        self.fused_pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.ds_metrics_pub = self.create_publisher(String, self.ds_metrics_topic, 10)

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
            f"frame={self.output_frame} "
            f"near_radius_candidates={self.candidate_near_field_radii} "
            f"closed_loop_feedback={self.closed_loop_feedback_enable} "
            f"closed_loop_optimizer={self.closed_loop_optimizer_enable} "
            f"action_delay_sec={self.closed_loop_action_delay_sec:.2f} "
            f"ds_evidence={self.ds_evidence_enable}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.vehicle_position[:] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.have_odom = True

    def closed_loop_feedback_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        score = payload.get("closed_loop_score_ema", payload.get("closed_loop_score"))
        try:
            score_value = float(score)
        except (TypeError, ValueError):
            return
        if not math.isfinite(score_value):
            return
        if not self.closed_loop_feedback_ready:
            self.closed_loop_score_ema = score_value
            self.closed_loop_feedback_ready = True
            self.update_closed_loop_candidate_score(score_value, time.time())
            return
        alpha = min(1.0, max(0.0, self.closed_loop_feedback_alpha))
        self.closed_loop_score_ema = (
            (1.0 - alpha) * self.closed_loop_score_ema + alpha * score_value
        )
        self.update_closed_loop_candidate_score(score_value, time.time())

    def update_closed_loop_candidate_score(self, score_value: float, now: float) -> None:
        if not self.closed_loop_optimizer_enable:
            return
        candidate_key = self.lookup_delayed_candidate_key(now - self.closed_loop_action_delay_sec)
        if candidate_key is None:
            return
        previous = self.closed_loop_candidate_scores.get(candidate_key)
        alpha = min(1.0, max(0.0, self.closed_loop_candidate_score_alpha))
        if previous is None:
            self.closed_loop_candidate_scores[candidate_key] = score_value
        else:
            self.closed_loop_candidate_scores[candidate_key] = (
                (1.0 - alpha) * previous + alpha * score_value
            )

    def record_current_candidate_action(self) -> None:
        if not self.closed_loop_optimizer_enable:
            return
        now = time.time()
        candidate_key = self.current_candidate_key()
        if self.closed_loop_action_history and self.closed_loop_action_history[-1][1] == candidate_key:
            return
        self.closed_loop_action_history.append((now, candidate_key))
        cutoff = now - self.closed_loop_action_history_sec
        while self.closed_loop_action_history and self.closed_loop_action_history[0][0] < cutoff:
            self.closed_loop_action_history.popleft()

    def lookup_delayed_candidate_key(
        self, target_time: float
    ) -> Tuple[float, int, float, float, float] | None:
        if not self.closed_loop_action_history:
            return None
        delayed_key = self.closed_loop_action_history[0][1]
        for action_time, candidate_key in self.closed_loop_action_history:
            if action_time > target_time:
                break
            delayed_key = candidate_key
        return delayed_key

    def current_candidate_key(self) -> Tuple[float, int, float, float, float]:
        return (
            round(float(self.current_min_probability), 4),
            int(self.current_min_hits),
            round(float(self.current_lidar_growth), 4),
            round(float(self.current_near_field_radius), 4),
            round(float(self.current_dual_bonus), 4),
        )

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

        self.current_z_max = self.choose_z_max()
        self.current_depth_decay = self.choose_depth_decay()
        depth_pts = self.cloud_to_xyz(depth_msg)
        lidar_pts = self.cloud_to_xyz(lidar_msg)

        depth_pts = self.filter_points(depth_pts)
        lidar_pts = self.filter_points(lidar_pts)

        fusion_state = self.build_fusion_state(
            depth_pts,
            lidar_pts,
            self.current_near_field_radius,
            self.current_lidar_growth,
        )
        fusion_state = self.auto_tune_parameters(fusion_state, depth_pts, lidar_pts)
        fused_points, stats = self.select_fused_points(
            fusion_state, self.current_min_probability, self.current_min_hits
        )
        template_msg = depth_msg if stamp_to_ns(depth_msg.header.stamp) >= stamp_to_ns(lidar_msg.header.stamp) else lidar_msg
        cloud_msg = self.xyz_to_cloud(fused_points, template_msg)
        self.fused_pub.publish(cloud_msg)
        self.publish_ds_metrics(stats)

        self.frame_count += 1
        if self.frame_count % max(1, self.debug_period) == 0:
            self.get_logger().info(
                f"fusion frame={self.frame_count} depth_pts={stats['depth_points']} "
                f"lidar_pts={stats['lidar_points']} fused_voxels={stats['fused_voxels']} "
                f"depth_only={stats['depth_only_voxels']} lidar_only={stats['lidar_only_voxels']} "
                f"dual={stats['dual_voxels']} retention={stats['retention_ratio']:.3f} "
                f"min_prob={self.current_min_probability:.3f} "
                f"near_radius={self.current_near_field_radius:.2f} "
                f"lidar_growth={self.current_lidar_growth:.2f} "
                f"min_hits={self.current_min_hits} "
                f"dual_bonus={self.current_dual_bonus:.2f} "
                f"z_max={self.current_z_max:.2f} "
                f"depth_decay={self.current_depth_decay:.2f} "
                f"ds_occ={stats['ds_belief_occupied_mean']:.3f} "
                f"ds_unknown={stats['ds_unknown_mean']:.3f} "
                f"ds_conflict={stats['ds_conflict_mean']:.3f}"
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
        mask = (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.current_z_max)
        pts = pts[mask]
        if pts.size == 0:
            return pts.reshape((-1, 3))
        ranges = np.linalg.norm(pts - self.vehicle_position[None, :], axis=1)
        max_allowed_range = self.max_range if range_limit is None else min(self.max_range, range_limit)
        return pts[ranges <= max_allowed_range]

    def depth_probability(self, ranges: np.ndarray, near_field_radius: float) -> np.ndarray:
        # Stronger in the near field, decays with distance.
        base = 0.25 + 0.55 * np.exp(-ranges / max(self.current_depth_decay, 1e-3))
        near_bonus = np.where(ranges <= near_field_radius, 0.10, 0.0)
        return clamp_prob(base + near_bonus)

    def lidar_probability(
        self, ranges: np.ndarray, near_field_radius: float, lidar_growth: float
    ) -> np.ndarray:
        # More reliable with distance and for sparse geometry.
        base = 0.35 + 0.45 * (1.0 - np.exp(-ranges / max(lidar_growth, 1e-3)))
        far_bonus = np.where(ranges > near_field_radius, 0.10, 0.0)
        return clamp_prob(base + far_bonus)

    def build_modality_state(self, points: np.ndarray) -> ModalityVoxelState:
        if points.size == 0:
            return ModalityVoxelState(
                keys=np.empty((0, 3), dtype=np.int32),
                key_tuples=[],
                ranges=np.empty((0,), dtype=np.float32),
                counts=np.empty((0,), dtype=np.int32),
            )
        keys = np.floor(points / self.resolution).astype(np.int32)
        unique_keys, inverse = np.unique(keys, axis=0, return_inverse=True)
        centers = (unique_keys.astype(np.float32) + 0.5) * self.resolution
        ranges = np.linalg.norm(centers - self.vehicle_position[None, :], axis=1)
        voxel_counts = np.bincount(inverse, minlength=unique_keys.shape[0]).astype(np.int32)
        key_tuples = [
            (int(key_arr[0]), int(key_arr[1]), int(key_arr[2])) for key_arr in unique_keys
        ]
        return ModalityVoxelState(
            keys=unique_keys,
            key_tuples=key_tuples,
            ranges=ranges.astype(np.float32),
            counts=voxel_counts,
        )

    def accumulate_voxel_state(
        self,
        voxels: Dict[Tuple[int, int, int], VoxelEvidence],
        state: ModalityVoxelState,
        modality: str,
        near_field_radius: float,
        lidar_growth: float,
    ) -> None:
        if state.keys.size == 0:
            return

        if modality == "depth":
            probs = self.depth_probability(state.ranges, near_field_radius)
        else:
            probs = self.lidar_probability(state.ranges, near_field_radius, lidar_growth)

        hit_boost = np.minimum(np.log1p(state.counts), 1.6)
        logits = np.log(probs / (1.0 - probs)) * hit_boost

        for idx, key in enumerate(state.key_tuples):
            item = voxels[key]
            item.logit_sum += float(logits[idx])
            occ_mass, free_mass, unknown_mass = self.ds_mass_from_probability(float(probs[idx]))
            support_hits = 1
            if int(state.counts[idx]) >= self.support_dense_count_threshold:
                support_hits += 1
            item.hits += support_hits
            if modality == "depth":
                item.seen_depth = True
                item.depth_occ_mass = max(item.depth_occ_mass, occ_mass)
                item.depth_free_mass = max(item.depth_free_mass, free_mass)
                item.depth_unknown_mass = min(item.depth_unknown_mass, unknown_mass)
            else:
                item.seen_lidar = True
                item.lidar_occ_mass = max(item.lidar_occ_mass, occ_mass)
                item.lidar_free_mass = max(item.lidar_free_mass, free_mass)
                item.lidar_unknown_mass = min(item.lidar_unknown_mass, unknown_mass)

    def compute_fusion_output(
        self,
        depth_state: ModalityVoxelState,
        lidar_state: ModalityVoxelState,
        near_field_radius: float,
        lidar_growth: float,
    ) -> dict:
        voxels: Dict[Tuple[int, int, int], VoxelEvidence] = defaultdict(VoxelEvidence)
        self.accumulate_voxel_state(
            voxels, depth_state, "depth", near_field_radius, lidar_growth
        )
        self.accumulate_voxel_state(
            voxels, lidar_state, "lidar", near_field_radius, lidar_growth
        )

        candidate_voxels = len(voxels)
        voxel_probabilities: Dict[Tuple[int, int, int], float] = {}
        ds_metrics_by_key: Dict[Tuple[int, int, int], dict] = {}

        for key, evidence in voxels.items():
            probability = float(sigmoid(np.array([evidence.logit_sum], dtype=np.float32))[0])
            voxel_probabilities[key] = probability
            ds_metrics_by_key[key] = self.compute_ds_metrics(evidence)

        return {
            "voxels": voxels,
            "candidate_voxels": int(candidate_voxels),
            "voxel_probabilities": voxel_probabilities,
            "voxel_hits": {key: evidence.hits for key, evidence in voxels.items()},
            "ds_metrics": ds_metrics_by_key,
        }

    def build_fusion_state(
        self,
        depth_pts: np.ndarray,
        lidar_pts: np.ndarray,
        near_field_radius: float,
        lidar_growth: float,
    ) -> dict:
        depth_state = self.build_modality_state(depth_pts)
        lidar_state = self.build_modality_state(lidar_pts)
        fusion_state = self.compute_fusion_output(
            depth_state, lidar_state, near_field_radius, lidar_growth
        )
        fusion_state["depth_state"] = depth_state
        fusion_state["lidar_state"] = lidar_state
        fusion_state["depth_points"] = int(depth_pts.shape[0])
        fusion_state["lidar_points"] = int(lidar_pts.shape[0])
        fusion_state["near_field_radius"] = float(near_field_radius)
        fusion_state["lidar_growth"] = float(lidar_growth)
        return fusion_state

    def select_fused_points(
        self, fusion_state: dict, threshold: float, min_hits: int
    ) -> Tuple[np.ndarray, dict]:
        voxels = fusion_state["voxels"]
        voxel_probabilities = fusion_state["voxel_probabilities"]
        ds_metrics_by_key = fusion_state.get("ds_metrics", {})
        fused = []
        depth_only = 0
        lidar_only = 0
        dual = 0
        ds_occ_values = []
        ds_unknown_values = []
        ds_conflict_values = []

        for key, evidence in voxels.items():
            probability = self.effective_probability(
                float(voxel_probabilities[key]), evidence, self.current_dual_bonus
            )
            if probability < threshold or evidence.hits < min_hits:
                continue
            ds_metrics = ds_metrics_by_key.get(key)
            if ds_metrics is not None:
                ds_occ_values.append(float(ds_metrics["belief_occupied"]))
                ds_unknown_values.append(float(ds_metrics["unknown"]))
                ds_conflict_values.append(float(ds_metrics["conflict"]))
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
            "ds_belief_occupied_mean": self.safe_mean(ds_occ_values),
            "ds_unknown_mean": self.safe_mean(ds_unknown_values),
            "ds_conflict_mean": self.safe_mean(ds_conflict_values),
        }
        return fused_np, stats

    def publish_ds_metrics(self, stats: dict) -> None:
        if not self.ds_evidence_enable:
            return
        payload = {
            "frame": int(self.frame_count),
            "belief_occupied_mean": float(stats["ds_belief_occupied_mean"]),
            "unknown_mean": float(stats["ds_unknown_mean"]),
            "conflict_mean": float(stats["ds_conflict_mean"]),
            "fused_voxels": int(stats["fused_voxels"]),
            "dual_voxels": int(stats["dual_voxels"]),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.ds_metrics_pub.publish(msg)

    def ds_mass_from_probability(self, probability: float) -> tuple[float, float, float]:
        if not self.ds_evidence_enable:
            return 0.0, 0.0, 1.0
        unknown = self.ds_unknown_floor + (1.0 - probability) * 0.35
        unknown = min(0.95, max(self.ds_unknown_floor, unknown))
        remaining = max(0.0, 1.0 - unknown)
        free_mass = min(remaining, self.ds_free_scale * (1.0 - probability))
        occ_mass = max(0.0, remaining - free_mass)
        total = occ_mass + free_mass + unknown
        if total <= 1e-9:
            return 0.0, 0.0, 1.0
        return occ_mass / total, free_mass / total, unknown / total

    def compute_ds_metrics(self, evidence: VoxelEvidence) -> dict:
        if not self.ds_evidence_enable:
            return {"belief_occupied": 0.0, "belief_free": 0.0, "unknown": 1.0, "conflict": 0.0}
        depth_mass = (
            evidence.depth_occ_mass,
            evidence.depth_free_mass,
            evidence.depth_unknown_mass,
        )
        lidar_mass = (
            evidence.lidar_occ_mass,
            evidence.lidar_free_mass,
            evidence.lidar_unknown_mass,
        )
        if not evidence.seen_depth:
            depth_mass = (0.0, 0.0, 1.0)
        if not evidence.seen_lidar:
            lidar_mass = (0.0, 0.0, 1.0)
        return self.combine_ds_masses(depth_mass, lidar_mass)

    def combine_ds_masses(
        self, first: tuple[float, float, float], second: tuple[float, float, float]
    ) -> dict:
        occ_a, free_a, unknown_a = first
        occ_b, free_b, unknown_b = second
        conflict = occ_a * free_b + free_a * occ_b
        denominator = max(1e-6, 1.0 - conflict)
        occ = (occ_a * occ_b + occ_a * unknown_b + unknown_a * occ_b) / denominator
        free = (free_a * free_b + free_a * unknown_b + unknown_a * free_b) / denominator
        unknown = (unknown_a * unknown_b) / denominator
        return {
            "belief_occupied": float(max(0.0, min(1.0, occ))),
            "belief_free": float(max(0.0, min(1.0, free))),
            "unknown": float(max(0.0, min(1.0, unknown))),
            "conflict": float(max(0.0, min(1.0, conflict))),
        }

    def safe_mean(self, values: list[float]) -> float:
        if not values:
            return 0.0
        return float(sum(values) / len(values))

    def auto_tune_parameters(
        self, fusion_state: dict, depth_pts: np.ndarray, lidar_pts: np.ndarray
    ) -> dict:
        if (
            not self.adaptive_min_probability_enable
            and not self.adaptive_min_hits_enable
            and not self.adaptive_lidar_growth_enable
            and not self.adaptive_dual_bonus_enable
            and not self.adaptive_near_field_radius_enable
        ):
            return fusion_state
        if not self.have_global or self.global_keys.size == 0:
            return fusion_state

        local_gt_mask = (
            np.linalg.norm(self.global_centers - self.vehicle_position[None, :], axis=1)
            <= self.adaptive_eval_range
        )
        if not np.any(local_gt_mask):
            return fusion_state
        gt_keys = self.keys_to_set(self.global_keys[local_gt_mask])
        if len(gt_keys) < self.adaptive_min_gt_voxels:
            return fusion_state

        depth_keys = self.points_to_local_eval_keys(depth_pts)
        lidar_keys = self.points_to_local_eval_keys(lidar_pts)
        depth_scores = self.compute_scores(depth_keys, gt_keys)
        lidar_scores = self.compute_scores(lidar_keys, gt_keys)
        best_single_recall = max(depth_scores.recall, lidar_scores.recall)
        best_single_f1 = max(depth_scores.f1, lidar_scores.f1)

        best_probability = self.current_min_probability
        best_min_hits = self.current_min_hits
        best_lidar_growth = self.current_lidar_growth
        best_near_field_radius = self.current_near_field_radius
        best_dual_bonus = self.current_dual_bonus
        best_fusion_state = fusion_state
        best_score = None
        current_candidate = (
            round(self.current_min_probability, 4),
            int(self.current_min_hits),
            round(self.current_lidar_growth, 4),
            round(self.current_near_field_radius, 4),
            round(self.current_dual_bonus, 4),
        )

        def candidate_distance(
            probability: float,
            min_hits: int,
            lidar_growth: float,
            near_field_radius: float,
            dual_bonus: float,
        ) -> float:
            return abs(probability - self.current_min_probability) + 0.25 * abs(
                min_hits - self.current_min_hits
            ) + 0.10 * abs(
                lidar_growth - self.current_lidar_growth
            ) + 0.05 * abs(
                near_field_radius - self.current_near_field_radius
            ) + 0.15 * abs(
                dual_bonus - self.current_dual_bonus
            )

        fusion_states_by_shape: dict[Tuple[float, float], dict] = {
            (
                round(float(fusion_state["lidar_growth"]), 4),
                round(float(fusion_state["near_field_radius"]), 4),
            ): fusion_state
        }

        for near_field_radius in self.candidate_near_field_radii:
            radius_key = round(float(near_field_radius), 4)
            radius_penalty = 0.0015 * abs(float(near_field_radius) - self.near_field_radius)
            for lidar_growth in self.candidate_lidar_growths:
                growth_key = round(float(lidar_growth), 4)
                state_key = (growth_key, radius_key)
                candidate_state = fusion_states_by_shape.get(state_key)
                if candidate_state is None:
                    candidate_state = self.compute_fusion_output(
                        fusion_state["depth_state"],
                        fusion_state["lidar_state"],
                        float(near_field_radius),
                        float(lidar_growth),
                    )
                    candidate_state["depth_state"] = fusion_state["depth_state"]
                    candidate_state["lidar_state"] = fusion_state["lidar_state"]
                    candidate_state["depth_points"] = fusion_state["depth_points"]
                    candidate_state["lidar_points"] = fusion_state["lidar_points"]
                    candidate_state["near_field_radius"] = float(near_field_radius)
                    candidate_state["lidar_growth"] = float(lidar_growth)
                    fusion_states_by_shape[state_key] = candidate_state

                voxel_probabilities = candidate_state.get("voxel_probabilities", {})
                voxel_hits = candidate_state.get("voxel_hits", {})
                voxels = candidate_state["voxels"]

                for dual_bonus in self.candidate_dual_bonuses:
                    dual_key = round(float(dual_bonus), 4)
                    for min_hits in self.candidate_min_hits:
                        for threshold in self.candidate_probabilities:
                            predicted = {
                                key
                                for key, probability in voxel_probabilities.items()
                                if self.effective_probability(
                                    float(probability), voxels[key], float(dual_bonus)
                                )
                                >= threshold
                                and int(voxel_hits.get(key, 0)) >= min_hits
                            }
                            predicted = self.limit_keys_to_eval_range(predicted)
                            scores = self.compute_scores(predicted, gt_keys)
                            gain_recall = scores.recall - best_single_recall
                            gain_f1 = scores.f1 - best_single_f1
                            local_utility = gain_f1 + 0.35 * gain_recall - radius_penalty
                            candidate_key = (
                                round(threshold, 4),
                                int(min_hits),
                                growth_key,
                                radius_key,
                                dual_key,
                            )
                            utility = self.closed_loop_local_score_weight * local_utility
                            utility += self.closed_loop_candidate_bonus(candidate_key)
                            if self.adaptive_retention_enable:
                                retention_ratio = len(predicted) / max(1, candidate_state["candidate_voxels"])
                                deviation = abs(retention_ratio - self.adaptive_target_retention)
                                if deviation <= self.adaptive_retention_band:
                                    band_center = max(1e-3, self.adaptive_retention_band)
                                    retention_bonus = 1.0 - deviation / band_center
                                    utility += 0.03 * retention_bonus
                                else:
                                    retention_penalty = (
                                        deviation - self.adaptive_retention_band
                                    ) / max(1e-3, 1.0 - self.adaptive_retention_band)
                                    utility -= 0.12 * retention_penalty
                            previous = self.candidate_scores[candidate_key]
                            ema_score = (
                                utility
                                if previous is None
                                else (1.0 - self.adaptive_score_alpha) * previous
                                + self.adaptive_score_alpha * utility
                            )
                            self.candidate_scores[candidate_key] = ema_score
                            if best_score is None or ema_score > best_score + 1e-9:
                                best_score = ema_score
                                best_probability = threshold
                                best_min_hits = int(min_hits)
                                best_lidar_growth = float(lidar_growth)
                                best_near_field_radius = float(near_field_radius)
                                best_dual_bonus = float(dual_bonus)
                                best_fusion_state = candidate_state
                            elif best_score is not None and abs(ema_score - best_score) <= 1e-9:
                                best_distance = candidate_distance(
                                    best_probability,
                                    best_min_hits,
                                    best_lidar_growth,
                                    best_near_field_radius,
                                    best_dual_bonus,
                                )
                                if (
                                    candidate_distance(
                                        threshold,
                                        min_hits,
                                        float(lidar_growth),
                                        float(near_field_radius),
                                        float(dual_bonus),
                                    )
                                    < best_distance
                                ):
                                    best_probability = threshold
                                    best_min_hits = int(min_hits)
                                    best_lidar_growth = float(lidar_growth)
                                    best_near_field_radius = float(near_field_radius)
                                    best_dual_bonus = float(dual_bonus)
                                    best_fusion_state = candidate_state

        current_score = self.candidate_scores.get(current_candidate)
        if (
            current_score is not None
            and best_score is not None
            and best_score - current_score < 1e-4
        ):
            best_probability = self.current_min_probability
            best_min_hits = self.current_min_hits
            best_lidar_growth = self.current_lidar_growth
            best_near_field_radius = self.current_near_field_radius
            best_dual_bonus = self.current_dual_bonus
            best_fusion_state = fusion_states_by_shape.get(
                (
                    round(self.current_lidar_growth, 4),
                    round(self.current_near_field_radius, 4),
                ),
                fusion_state,
            )

        self.current_min_probability = float(best_probability)
        self.current_min_hits = int(best_min_hits)
        self.current_lidar_growth = float(best_lidar_growth)
        self.current_near_field_radius = float(best_near_field_radius)
        self.current_dual_bonus = float(best_dual_bonus)
        self.record_current_candidate_action()
        return best_fusion_state

    def closed_loop_candidate_bonus(
        self, candidate_key: Tuple[float, int, float, float, float]
    ) -> float:
        if (
            not self.closed_loop_feedback_enable
            or not self.closed_loop_optimizer_enable
            or not self.closed_loop_feedback_ready
            or self.closed_loop_feedback_weight <= 0.0
        ):
            return 0.0
        candidate_score = self.closed_loop_candidate_scores.get(candidate_key)
        if candidate_score is None:
            return 0.0
        return self.closed_loop_feedback_weight * candidate_score

    def build_probability_candidates(self) -> list[float]:
        if not self.adaptive_min_probability_enable:
            return [round(self.min_probability, 4)]
        return self.build_candidate_values(
            self.adaptive_min_probability_min,
            self.adaptive_min_probability_max,
            self.adaptive_min_probability_step,
            self.min_probability,
        )

    def build_min_hits_candidates(self) -> list[int]:
        if not self.adaptive_min_hits_enable:
            return [int(self.min_hits)]
        lower = min(self.adaptive_min_hits_min, self.adaptive_min_hits_max)
        upper = max(self.adaptive_min_hits_min, self.adaptive_min_hits_max)
        return list(range(max(1, lower), max(1, upper) + 1))

    def build_lidar_growth_candidates(self) -> list[float]:
        if not self.adaptive_lidar_growth_enable:
            return [round(self.lidar_growth, 4)]
        return self.build_candidate_values(
            self.adaptive_lidar_growth_min,
            self.adaptive_lidar_growth_max,
            self.adaptive_lidar_growth_step,
            self.lidar_growth,
        )

    def build_near_field_radius_candidates(self) -> list[float]:
        if not self.adaptive_near_field_radius_enable:
            return [round(self.near_field_radius, 4)]
        return self.build_candidate_values(
            self.adaptive_near_field_radius_min,
            self.adaptive_near_field_radius_max,
            self.adaptive_near_field_radius_step,
            self.near_field_radius,
        )

    def build_dual_bonus_candidates(self) -> list[float]:
        if not self.adaptive_dual_bonus_enable:
            return [0.0]
        return self.build_candidate_values(
            self.adaptive_dual_bonus_min,
            self.adaptive_dual_bonus_max,
            self.adaptive_dual_bonus_step,
            0.0,
        )

    def build_z_max_candidates(self) -> list[float]:
        if not self.adaptive_z_max_enable:
            return [round(self.z_max, 4)]
        return self.build_candidate_values(
            self.adaptive_z_max_min,
            self.adaptive_z_max_max,
            self.adaptive_z_max_step,
            self.z_max,
        )

    def build_depth_decay_candidates(self) -> list[float]:
        if not self.adaptive_depth_decay_enable:
            return [round(self.depth_decay, 4)]
        return self.build_candidate_values(
            self.adaptive_depth_decay_min,
            self.adaptive_depth_decay_max,
            self.adaptive_depth_decay_step,
            self.depth_decay,
        )

    def effective_probability(
        self, probability: float, evidence: VoxelEvidence, dual_bonus: float
    ) -> float:
        if dual_bonus <= 0.0 or not (evidence.seen_depth and evidence.seen_lidar):
            return probability
        logit = math.log(max(probability, 1e-3) / max(1.0 - probability, 1e-3))
        return float(sigmoid(np.array([logit + dual_bonus], dtype=np.float32))[0])

    def choose_dual_bonus(self, fusion_state: dict) -> float:
        if not self.candidate_dual_bonuses:
            return 0.0
        candidate_voxels = max(1, int(fusion_state["candidate_voxels"]))
        dual_voxels = sum(
            1
            for evidence in fusion_state["voxels"].values()
            if evidence.seen_depth and evidence.seen_lidar
        )
        dual_ratio = dual_voxels / candidate_voxels
        ordered = sorted(float(value) for value in self.candidate_dual_bonuses)
        if dual_ratio < 0.04:
            return ordered[0]
        if dual_ratio < 0.10:
            return ordered[len(ordered) // 2]
        return ordered[-1]

    def choose_z_max(self) -> float:
        if (
            not self.adaptive_z_max_enable
            or not self.have_global
            or self.global_centers.size == 0
        ):
            return float(self.z_max)
        local_mask = (
            np.linalg.norm(self.global_centers - self.vehicle_position[None, :], axis=1)
            <= self.adaptive_eval_range
        )
        if not np.any(local_mask):
            return float(self.z_max)
        local_heights = self.global_centers[local_mask, 2]
        if local_heights.size < self.adaptive_min_gt_voxels:
            return float(self.z_max)
        target_height = float(np.percentile(local_heights, 95.0)) + 0.20
        candidates = sorted(float(value) for value in self.candidate_z_max_values)
        for candidate in candidates:
            if candidate >= target_height:
                return candidate
        return candidates[-1]

    def choose_depth_decay(self) -> float:
        if (
            not self.adaptive_depth_decay_enable
            or not self.have_global
            or self.global_centers.size == 0
        ):
            return float(self.depth_decay)
        local_mask = (
            np.linalg.norm(self.global_centers - self.vehicle_position[None, :], axis=1)
            <= self.adaptive_eval_range
        )
        if not np.any(local_mask):
            return float(self.depth_decay)
        local_heights = self.global_centers[local_mask, 2]
        if local_heights.size < self.adaptive_min_gt_voxels:
            return float(self.depth_decay)
        vertical_spread = float(
            np.percentile(local_heights, 90.0) - np.percentile(local_heights, 10.0)
        )
        target_decay = 5.5 if vertical_spread >= 1.0 else 4.5
        candidates = sorted(float(value) for value in self.candidate_depth_decay_values)
        for candidate in candidates:
            if candidate >= target_decay:
                return candidate
        return candidates[-1]

    def build_candidate_values(
        self, lower_bound: float, upper_bound: float, step_size: float, fallback: float
    ) -> list[float]:
        lower = min(lower_bound, upper_bound)
        upper = max(lower_bound, upper_bound)
        step = max(step_size, 1e-3)
        candidate_values = []
        current = lower
        while current <= upper + 1e-9:
            candidate_values.append(round(current, 4))
            current += step
        if not candidate_values:
            candidate_values = [round(fallback, 4)]
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
