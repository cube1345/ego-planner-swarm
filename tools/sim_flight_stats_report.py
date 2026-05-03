#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import re
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def wrap_angle_rad(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_from_points(p0: np.ndarray, p1: np.ndarray) -> float:
    delta = p1 - p0
    return math.atan2(float(delta[1]), float(delta[0]))


def estimate_clusters(keys: Set[Tuple[int, int, int]], min_size: int) -> int:
    if not keys:
        return 0
    remaining = set(keys)
    clusters = 0
    neighbors = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]

    while remaining:
        start = remaining.pop()
        queue = [start]
        size = 0
        while queue:
            current = queue.pop()
            size += 1
            cx, cy, cz = current
            for dx, dy, dz in neighbors:
                nxt = (cx + dx, cy + dy, cz + dz)
                if nxt in remaining:
                    remaining.remove(nxt)
                    queue.append(nxt)
        if size >= min_size:
            clusters += 1
    return clusters


class SimFlightStatsReport(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("sim_flight_stats_report")
        self.args = args

        self.start_wall = time.time()
        self.last_wall = self.start_wall
        self.duration_sec = float(args.duration_sec)

        self.global_cloud_received = False
        self.obstacle_point_count = 0
        self.obstacle_voxel_count = 0
        self.obstacle_cluster_count = 0
        self.occ_latest_points = 0
        self.occ_peak_points = 0

        if args.launch_log_path:
            # The redirected batch launch log starts recording before this node starts.
            # Use zero baseline so early replans are still counted for this run.
            self.replan_base_total = 0
            self.replan_final_total = 0
        else:
            self.replan_base_total = self.scan_replan_total()
            self.replan_final_total = self.replan_base_total

        self.odom_count = 0
        self.path_length = 0.0
        self.max_speed = 0.0
        self.turn_count = 0
        self.last_turn_time: Optional[float] = None
        self.last_heading: Optional[float] = None
        self.prev_pos: Optional[np.ndarray] = None
        self.prev_odom_time: Optional[float] = None
        self.goal_reached_time: Optional[float] = None
        self.finished = False

        self.turn_threshold_rad = math.radians(float(args.turn_angle_deg))
        self.min_turn_separation_sec = float(args.min_turn_separation_sec)
        self.min_segment_distance = float(args.min_segment_distance)
        self.goal = np.array([float(args.goal_x), float(args.goal_y), float(args.goal_z)], dtype=np.float64)
        self.goal_tol = float(args.goal_tolerance)
        self.goal_exit_margin = float(args.goal_exit_margin)
        self.left_goal_region = False

        self.global_sub = self.create_subscription(
            PointCloud2, args.global_cloud_topic, self.global_cloud_callback, 10
        )
        self.occ_sub = self.create_subscription(
            PointCloud2, args.occupancy_topic, self.occupancy_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, args.odom_topic, self.odom_callback, 100
        )
        self.timer = self.create_timer(0.2, self.on_timer)

        self.get_logger().info(
            f"stats monitor started. duration={self.duration_sec:.1f}s "
            f"global={args.global_cloud_topic} occupancy={args.occupancy_topic} odom={args.odom_topic} "
            f"replan_log_root={args.replan_log_root}"
        )

    def global_cloud_callback(self, msg: PointCloud2) -> None:
        if self.global_cloud_received:
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
        mask = (cloud[:, 2] >= self.args.z_min) & (cloud[:, 2] <= self.args.z_max)
        cloud = cloud[mask]

        self.obstacle_point_count = int(cloud.shape[0])
        if cloud.shape[0] == 0:
            return

        keys_np = np.floor(cloud / self.args.obstacle_voxel_size).astype(np.int32)
        unique_np = np.unique(keys_np, axis=0)
        keys: Set[Tuple[int, int, int]] = {
            (int(k[0]), int(k[1]), int(k[2])) for k in unique_np
        }
        self.obstacle_voxel_count = len(keys)
        self.obstacle_cluster_count = estimate_clusters(keys, self.args.min_cluster_voxels)
        self.global_cloud_received = True

        self.get_logger().info(
            f"global obstacle snapshot captured: points={self.obstacle_point_count} "
            f"voxels={self.obstacle_voxel_count} clusters={self.obstacle_cluster_count}"
        )

    def occupancy_callback(self, msg: PointCloud2) -> None:
        count = 0
        for _ in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            count += 1
        self.occ_latest_points = count
        if count > self.occ_peak_points:
            self.occ_peak_points = count

    def scan_replan_total(self) -> int:
        if self.args.launch_log_path:
            launch_log = Path(self.args.launch_log_path)
            if launch_log.is_file():
                return self.scan_replan_from_files([launch_log])
        root = Path(self.args.replan_log_root)
        if not root.exists():
            return 0
        return self.scan_replan_from_files(self.collect_replan_files(root))

    def collect_replan_files(self, root: Path) -> List[Path]:
        candidates = [p for p in root.rglob("ego_planner_node*.log") if p.is_file()]
        launch_logs = [p for p in root.rglob("launch.log") if p.is_file()]
        candidate_map = {str(p.resolve()): p for p in candidates + launch_logs}
        if not candidate_map:
            candidate_map = {str(p.resolve()): p for p in root.rglob("*") if p.is_file()}
        return sorted(
            candidate_map.values(),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )[: int(self.args.replan_log_file_limit)]

    def scan_replan_from_files(self, files: List[Path]) -> int:
        indexed_pattern = re.compile(r"\[drone\s+\d+\s+replan\s+(\d+)\]")
        warn_pattern = re.compile(r"current traj in collision, replan\.", re.IGNORECASE)
        max_index = -1
        warn_count = 0

        for file in files:
            try:
                text = file.read_text(encoding="utf-8", errors="ignore")
            except Exception:
                continue
            for match in indexed_pattern.finditer(text):
                idx = int(match.group(1))
                if idx > max_index:
                    max_index = idx
            warn_count += len(warn_pattern.findall(text))

        if max_index >= 0:
            return max_index + 1
        return warn_count

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
            speed = dist / dt
            if speed > self.max_speed:
                self.max_speed = speed

            if dist >= self.min_segment_distance:
                heading = heading_from_points(self.prev_pos, pos)
                if self.last_heading is not None:
                    delta = abs(wrap_angle_rad(heading - self.last_heading))
                    enough_gap = (
                        self.last_turn_time is None
                        or (now - self.last_turn_time) >= self.min_turn_separation_sec
                    )
                    if delta >= self.turn_threshold_rad and enough_gap:
                        self.turn_count += 1
                        self.last_turn_time = now
                self.last_heading = heading

        self.prev_pos = pos
        self.prev_odom_time = odom_time

        dist_to_goal = float(np.linalg.norm(pos - self.goal))
        if dist_to_goal > (self.goal_tol + self.goal_exit_margin):
            self.left_goal_region = True

        if self.goal_reached_time is None:
            if self.left_goal_region and dist_to_goal <= self.goal_tol:
                self.goal_reached_time = now - self.start_wall

    def on_timer(self) -> None:
        elapsed = time.time() - self.start_wall
        if elapsed - self.last_wall < 5.0:
            return
        self.last_wall = elapsed
        self.replan_final_total = self.scan_replan_total()
        replan_count = max(0, self.replan_final_total - self.replan_base_total)
        self.get_logger().info(
            f"progress {elapsed:.1f}/{self.duration_sec:.1f}s "
            f"replan={replan_count} turns={self.turn_count}"
        )

    def finish_and_shutdown(self) -> None:
        if self.finished:
            return
        self.finished = True
        elapsed = time.time() - self.start_wall
        self.replan_final_total = self.scan_replan_total()
        replan_count = max(0, self.replan_final_total - self.replan_base_total)
        avg_replan_interval = 0.0 if replan_count <= 0 else float(elapsed / max(1, replan_count))

        result = {
            "duration_sec": float(elapsed),
            "obstacle_points": int(self.obstacle_point_count),
            "obstacle_voxels": int(self.obstacle_voxel_count),
            "obstacle_clusters_est": int(self.obstacle_cluster_count),
            "occupancy_points_latest": int(self.occ_latest_points),
            "occupancy_points_peak": int(self.occ_peak_points),
            "replan_count": int(replan_count),
            "replan_base_total": int(self.replan_base_total),
            "replan_final_total": int(self.replan_final_total),
            "avg_replan_interval_sec": float(avg_replan_interval),
            "turn_count": int(self.turn_count),
            "path_length_m": float(self.path_length),
            "max_speed_mps": float(self.max_speed),
            "goal_reached_time_sec": None
            if self.goal_reached_time is None
            else float(self.goal_reached_time),
            "goal_xyz": [float(self.goal[0]), float(self.goal[1]), float(self.goal[2])],
            "goal_tolerance_m": float(self.goal_tol),
            "odom_samples": int(self.odom_count),
        }

        out_dir = Path(self.args.output_dir).resolve()
        out_dir.mkdir(parents=True, exist_ok=True)
        json_path = out_dir / "sim_stats_summary.json"
        md_path = out_dir / "sim_stats_summary.md"
        svg_path = out_dir / "sim_stats_dashboard.svg"

        json_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
        md_lines = [
            "# Simulation Stats Summary",
            "",
            "| metric | value |",
            "|---|---:|",
        ]
        for key, value in result.items():
            md_lines.append(f"| {key} | {value} |")
        md_path.write_text("\n".join(md_lines) + "\n", encoding="utf-8")
        self.write_svg_dashboard(svg_path, result)

        if rclpy.ok():
            self.get_logger().info(f"done. json={json_path}")
            self.get_logger().info(f"done. md={md_path}")
            self.get_logger().info(f"done. svg={svg_path}")

    def write_svg_dashboard(self, path: Path, result: Dict[str, object]) -> None:
        width = 1180
        height = 720
        cards = [
            ("Obstacle Clusters(est)", f"{result['obstacle_clusters_est']}"),
            ("Replan Count", f"{result['replan_count']}"),
            ("Turn Count", f"{result['turn_count']}"),
            ("Duration(s)", f"{float(result['duration_sec']):.1f}"),
            ("Obstacle Voxels(global)", f"{result['obstacle_voxels']}"),
            ("Occ Points(peak)", f"{result['occupancy_points_peak']}"),
            ("Path Length(m)", f"{float(result['path_length_m']):.2f}"),
            ("Avg Replan Intv(s)", f"{float(result['avg_replan_interval_sec']):.2f}"),
            ("Max Speed(m/s)", f"{float(result['max_speed_mps']):.2f}")
        ]

        lines: List[str] = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<defs><linearGradient id="g" x1="0" y1="0" x2="1" y2="1">'
            '<stop offset="0%" stop-color="#0b1f33"/><stop offset="100%" stop-color="#1f4b6f"/>'
            "</linearGradient></defs>",
            f'<rect x="0" y="0" width="{width}" height="{height}" fill="url(#g)"/>',
            '<text x="36" y="54" fill="white" font-size="30" font-family="Segoe UI, Arial" font-weight="700">'
            "EGO Planner Simulation Stats"
            "</text>",
        ]

        x0, y0 = 34, 90
        card_w, card_h = 265, 132
        gap_x, gap_y = 20, 18
        for idx, (label, value) in enumerate(cards):
            row = idx // 4
            col = idx % 4
            x = x0 + col * (card_w + gap_x)
            y = y0 + row * (card_h + gap_y)
            lines.append(
                f'<rect x="{x}" y="{y}" width="{card_w}" height="{card_h}" rx="14" fill="#f8fbff" opacity="0.98"/>'
            )
            lines.append(
                f'<text x="{x + 14}" y="{y + 34}" fill="#425466" font-size="18" font-family="Segoe UI, Arial">{label}</text>'
            )
            lines.append(
                f'<text x="{x + 14}" y="{y + 92}" fill="#0b1f33" font-size="42" font-family="Segoe UI, Arial" font-weight="700">{value}</text>'
            )

        note_y = y0 + 2 * (card_h + gap_y) + 26
        goal_time = result["goal_reached_time_sec"]
        goal_text = "not reached" if goal_time is None else f"{float(goal_time):.2f}s"
        lines.extend(
            [
                f'<text x="36" y="{note_y}" fill="#e7f0f8" font-size="18" font-family="Segoe UI, Arial">'
                f'Goal reached time: {goal_text}'
                "</text>",
                f'<text x="36" y="{note_y + 30}" fill="#e7f0f8" font-size="16" font-family="Segoe UI, Arial">'
                "Obstacle clusters are estimated from global cloud voxel connected-components."
                "</text>",
                f'<text x="36" y="{note_y + 56}" fill="#e7f0f8" font-size="16" font-family="Segoe UI, Arial">'
                "Turn count is measured from odom heading changes above threshold."
                "</text>",
                "</svg>",
            ]
        )
        path.write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect simulation stats for obstacle avoidance run.")
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--output-dir", type=str, default="artifacts/sim_stats")
    parser.add_argument("--global-cloud-topic", type=str, default="/map_generator/global_cloud")
    parser.add_argument("--occupancy-topic", type=str, default="/drone_0_grid/grid_map/occupancy_inflate")
    parser.add_argument("--odom-topic", type=str, default="/drone_0_visual_slam/odom")
    parser.add_argument("--launch-log-path", type=str, default="")
    parser.add_argument("--replan-log-root", type=str, default=os.environ.get("ROS_LOG_DIR", "/tmp/ros_logs"))
    parser.add_argument("--replan-log-file-limit", type=int, default=80)
    parser.add_argument("--obstacle-voxel-size", type=float, default=0.20)
    parser.add_argument("--min-cluster-voxels", type=int, default=24)
    parser.add_argument("--z-min", type=float, default=-0.10)
    parser.add_argument("--z-max", type=float, default=3.50)
    parser.add_argument("--turn-angle-deg", type=float, default=28.0)
    parser.add_argument("--min-turn-separation-sec", type=float, default=1.0)
    parser.add_argument("--min-segment-distance", type=float, default=0.10)
    parser.add_argument("--goal-x", type=float, default=15.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-z", type=float, default=1.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.6)
    parser.add_argument("--goal-exit-margin", type=float, default=1.0)
    return parser.parse_args()


def assert_output_in_repo(path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    resolved = path.resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"output_dir must be inside repo: {resolved}")


def main() -> None:
    args = parse_args()
    out_dir = Path(args.output_dir)
    assert_output_in_repo(out_dir)

    rclpy.init()
    node = SimFlightStatsReport(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        node.finish_and_shutdown()
    finally:
        if not node.finished:
            node.finish_and_shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
