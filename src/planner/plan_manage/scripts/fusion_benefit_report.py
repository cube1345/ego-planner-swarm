#!/usr/bin/env python3
from __future__ import annotations

import csv
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, Optional, Set, Tuple

import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

VoxelSet = Set[Tuple[int, int, int]]


@dataclass
class Scores:
    precision: float
    recall: float
    f1: float
    pred_size: int
    gt_size: int
    true_positive: int


class FusionBenefitReport(Node):
    def __init__(self) -> None:
        super().__init__("fusion_benefit_report")

        self.declare_parameter("depth_topic", "/drone_0_pcl_render_node/cloud")
        self.declare_parameter("lidar_topic", "/drone_0_lidar/points")
        self.declare_parameter("fusion_topic", "/drone_0_fusion/fused_cloud")
        self.declare_parameter("global_cloud_topic", "/map_generator/global_cloud")
        self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("z_min", -0.10)
        self.declare_parameter("z_max", 3.50)
        self.declare_parameter("eval_range", 10.0)
        self.declare_parameter("sync_queue", 20)
        self.declare_parameter("sync_slop", 0.10)
        self.declare_parameter("report_every", 20)
        self.declare_parameter("window_size", 80)
        self.declare_parameter("min_gt_voxels", 40)
        self.declare_parameter("csv_path", "")

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.fusion_topic = str(self.get_parameter("fusion_topic").value)
        self.global_cloud_topic = str(self.get_parameter("global_cloud_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.eval_range = float(self.get_parameter("eval_range").value)
        self.report_every = int(self.get_parameter("report_every").value)
        self.window_size = int(self.get_parameter("window_size").value)
        self.min_gt_voxels = int(self.get_parameter("min_gt_voxels").value)
        sync_queue = int(self.get_parameter("sync_queue").value)
        sync_slop = float(self.get_parameter("sync_slop").value)
        csv_path = str(self.get_parameter("csv_path").value).strip()

        self.have_odom = False
        self.have_global = False
        self.frame_count = 0
        self.vehicle_position = np.zeros(3, dtype=np.float32)
        self.global_keys = np.empty((0, 3), dtype=np.int32)
        self.global_centers = np.empty((0, 3), dtype=np.float32)

        self.history: Deque[Dict[str, float]] = deque(maxlen=max(1, self.window_size))

        self.csv_file = None
        self.csv_writer: Optional[csv.DictWriter] = None
        if csv_path:
            csv_file_path = Path(csv_path).expanduser()
            csv_file_path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = csv_file_path.open("w", newline="", encoding="utf-8")
            fieldnames = [
                "frame",
                "depth_precision",
                "depth_recall",
                "depth_f1",
                "lidar_precision",
                "lidar_recall",
                "lidar_f1",
                "fusion_precision",
                "fusion_recall",
                "fusion_f1",
                "fusion_recall_gain_vs_best_single",
                "fusion_f1_gain_vs_best_single",
                "union_recall",
                "gt_voxels",
                "depth_voxels",
                "lidar_voxels",
                "fusion_voxels",
            ]
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
            self.csv_writer.writeheader()

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(PointCloud2, self.global_cloud_topic, self.global_cloud_callback, 10)

        self.depth_sub = message_filters.Subscriber(self, PointCloud2, self.depth_topic)
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, self.lidar_topic)
        self.fusion_sub = message_filters.Subscriber(self, PointCloud2, self.fusion_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.lidar_sub, self.fusion_sub],
            queue_size=sync_queue,
            slop=sync_slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info(
            "fusion benefit report ready. "
            f"depth={self.depth_topic} lidar={self.lidar_topic} fusion={self.fusion_topic} "
            f"global={self.global_cloud_topic} odom={self.odom_topic}"
        )

    def destroy_node(self) -> bool:
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        return super().destroy_node()

    def odom_callback(self, msg: Odometry) -> None:
        self.vehicle_position[:] = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.have_odom = True

    def global_cloud_callback(self, msg: PointCloud2) -> None:
        points = self.cloud_to_xyz(msg)
        points = self.filter_points(points)
        keys = self.voxel_keys(points)
        if keys.size == 0:
            self.global_keys = np.empty((0, 3), dtype=np.int32)
            self.global_centers = np.empty((0, 3), dtype=np.float32)
        else:
            self.global_keys = keys
            self.global_centers = (keys.astype(np.float32) + 0.5) * self.resolution
        self.have_global = True

    def sync_callback(self, depth_msg: PointCloud2, lidar_msg: PointCloud2, fusion_msg: PointCloud2) -> None:
        if not self.have_odom or not self.have_global:
            return
        if self.global_keys.size == 0:
            return

        local_gt_mask = np.linalg.norm(self.global_centers - self.vehicle_position[None, :], axis=1) <= self.eval_range
        if not np.any(local_gt_mask):
            return
        gt_keys = self.keys_to_set(self.global_keys[local_gt_mask])
        if len(gt_keys) < self.min_gt_voxels:
            return

        depth_keys = self.cloud_msg_to_local_voxels(depth_msg)
        lidar_keys = self.cloud_msg_to_local_voxels(lidar_msg)
        fusion_keys = self.cloud_msg_to_local_voxels(fusion_msg)

        depth_scores = self.compute_scores(depth_keys, gt_keys)
        lidar_scores = self.compute_scores(lidar_keys, gt_keys)
        fusion_scores = self.compute_scores(fusion_keys, gt_keys)

        union_scores = self.compute_scores(depth_keys | lidar_keys, gt_keys)
        best_single_recall = max(depth_scores.recall, lidar_scores.recall)
        best_single_f1 = max(depth_scores.f1, lidar_scores.f1)

        row = {
            "depth_precision": depth_scores.precision,
            "depth_recall": depth_scores.recall,
            "depth_f1": depth_scores.f1,
            "lidar_precision": lidar_scores.precision,
            "lidar_recall": lidar_scores.recall,
            "lidar_f1": lidar_scores.f1,
            "fusion_precision": fusion_scores.precision,
            "fusion_recall": fusion_scores.recall,
            "fusion_f1": fusion_scores.f1,
            "fusion_recall_gain_vs_best_single": fusion_scores.recall - best_single_recall,
            "fusion_f1_gain_vs_best_single": fusion_scores.f1 - best_single_f1,
            "union_recall": union_scores.recall,
            "gt_voxels": float(len(gt_keys)),
            "depth_voxels": float(depth_scores.pred_size),
            "lidar_voxels": float(lidar_scores.pred_size),
            "fusion_voxels": float(fusion_scores.pred_size),
        }
        self.history.append(row)
        self.frame_count += 1

        if self.csv_writer is not None:
            csv_row = {"frame": self.frame_count}
            for key, value in row.items():
                csv_row[key] = value
            self.csv_writer.writerow(csv_row)
            self.csv_file.flush()

        if self.frame_count % max(1, self.report_every) == 0:
            self.report_rolling_metrics()

    def cloud_msg_to_local_voxels(self, msg: PointCloud2) -> VoxelSet:
        points = self.cloud_to_xyz(msg)
        points = self.filter_points(points)
        keys = self.voxel_keys(points)
        if keys.size == 0:
            return set()
        centers = (keys.astype(np.float32) + 0.5) * self.resolution
        mask = np.linalg.norm(centers - self.vehicle_position[None, :], axis=1) <= self.eval_range
        if not np.any(mask):
            return set()
        return self.keys_to_set(keys[mask])

    def cloud_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        points = [
            (float(p[0]), float(p[1]), float(p[2]))
            for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ]
        if not points:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32).reshape((-1, 3))

    def filter_points(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        mask = np.isfinite(points).all(axis=1)
        points = points[mask]
        if points.size == 0:
            return points.reshape((-1, 3))
        mask = (points[:, 2] >= self.z_min) & (points[:, 2] <= self.z_max)
        points = points[mask]
        if points.size == 0:
            return points.reshape((-1, 3))
        ranges = np.linalg.norm(points - self.vehicle_position[None, :], axis=1)
        return points[ranges <= self.eval_range]

    def voxel_keys(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return np.empty((0, 3), dtype=np.int32)
        keys = np.floor(points / self.resolution).astype(np.int32)
        return np.unique(keys, axis=0)

    def keys_to_set(self, keys: np.ndarray) -> VoxelSet:
        return {(int(k[0]), int(k[1]), int(k[2])) for k in keys}

    def compute_scores(self, pred: VoxelSet, gt: VoxelSet) -> Scores:
        pred_size = len(pred)
        gt_size = len(gt)
        if gt_size == 0:
            return Scores(precision=1.0, recall=1.0, f1=1.0, pred_size=pred_size, gt_size=0, true_positive=0)
        if pred_size == 0:
            return Scores(precision=0.0, recall=0.0, f1=0.0, pred_size=0, gt_size=gt_size, true_positive=0)
        tp = len(pred & gt)
        precision = tp / max(1, pred_size)
        recall = tp / max(1, gt_size)
        if precision + recall < 1e-9:
            f1 = 0.0
        else:
            f1 = 2.0 * precision * recall / (precision + recall)
        return Scores(
            precision=precision,
            recall=recall,
            f1=f1,
            pred_size=pred_size,
            gt_size=gt_size,
            true_positive=tp,
        )

    def rolling_mean(self, key: str) -> float:
        if not self.history:
            return 0.0
        return float(sum(item[key] for item in self.history) / len(self.history))

    def report_rolling_metrics(self) -> None:
        frames = len(self.history)
        if frames == 0:
            return
        d_r = self.rolling_mean("depth_recall")
        l_r = self.rolling_mean("lidar_recall")
        f_r = self.rolling_mean("fusion_recall")
        d_f1 = self.rolling_mean("depth_f1")
        l_f1 = self.rolling_mean("lidar_f1")
        f_f1 = self.rolling_mean("fusion_f1")
        gain_r = self.rolling_mean("fusion_recall_gain_vs_best_single")
        gain_f1 = self.rolling_mean("fusion_f1_gain_vs_best_single")
        union_r = self.rolling_mean("union_recall")

        self.get_logger().info(
            "[fusion-benefit] "
            f"window={frames} "
            f"recall(d/l/f)={d_r:.3f}/{l_r:.3f}/{f_r:.3f} "
            f"f1(d/l/f)={d_f1:.3f}/{l_f1:.3f}/{f_f1:.3f} "
            f"gain_recall={gain_r:+.3f} "
            f"gain_f1={gain_f1:+.3f} "
            f"union_recall={union_r:.3f}"
        )


def main() -> None:
    rclpy.init()
    node = FusionBenefitReport()
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
