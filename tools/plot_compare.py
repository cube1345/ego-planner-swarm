#!/usr/bin/env python3
import argparse
import csv
import os
import re
import sys
from typing import Dict, List, Tuple

try:
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib is required. Install with: pip install matplotlib")
    sys.exit(1)

SUMMARY_PATTERN = {
    "depth_success_rate": re.compile(r"depth success_rate=([0-9.]+|N/A)"),
    "fusion_success_rate": re.compile(r"fusion success_rate=([0-9.]+|N/A)"),
    "depth_traj_failed": re.compile(r"depth traj failed count=([0-9]+)"),
    "fusion_traj_failed": re.compile(r"fusion traj failed count=([0-9]+)"),
    "depth_collided": re.compile(r"depth collided count=([0-9]+)"),
    "fusion_collided": re.compile(r"fusion collided count=([0-9]+)"),
    "depth_rebound": re.compile(r"depth rebound count=([0-9]+)"),
    "fusion_rebound": re.compile(r"fusion rebound count=([0-9]+)"),
}


def parse_summary(path: str) -> Dict[str, float]:
    metrics: Dict[str, float] = {}
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    for key, pattern in SUMMARY_PATTERN.items():
        match = pattern.search(text)
        if not match:
            continue
        value = match.group(1)
        if value == "N/A":
            continue
        metrics[key] = float(value)
    return metrics


def read_conflict_csv(path: str) -> Tuple[List[float], List[float]]:
    times: List[float] = []
    ratios: List[float] = []
    if not os.path.exists(path):
        return times, ratios
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                times.append(float(row["time"]))
                ratios.append(float(row["conflict_ratio"]))
            except (ValueError, KeyError):
                continue
    return times, ratios


def plot_metrics(metrics: Dict[str, float], out_prefix: str, depth_csv: str, fusion_csv: str) -> None:
    metric_items = [
        ("success_rate", "depth_success_rate", "fusion_success_rate"),
        ("traj_failed", "depth_traj_failed", "fusion_traj_failed"),
        ("collided", "depth_collided", "fusion_collided"),
        ("rebound", "depth_rebound", "fusion_rebound"),
    ]

    for label, depth_key, fusion_key in metric_items:
        depth_val = metrics.get(depth_key, 0.0)
        fusion_val = metrics.get(fusion_key, 0.0)
        fig, ax = plt.subplots(figsize=(4, 3))
        ax.bar(["depth", "fusion"], [depth_val, fusion_val])
        ax.set_title(label)
        fig.tight_layout()
        out_path = f"{out_prefix}_{label}.png"
        fig.savefig(out_path, dpi=150)
        print(f"saved plot to {out_path}")
        plt.close(fig)

    # Conflict ratio over time
    depth_t, depth_r = read_conflict_csv(depth_csv)
    fusion_t, fusion_r = read_conflict_csv(fusion_csv)
    fig, ax = plt.subplots(figsize=(5, 3))
    ax.plot(depth_t, depth_r, label="depth")
    ax.plot(fusion_t, fusion_r, label="fusion")
    max_ratio = 0.0
    if depth_r:
        max_ratio = max(max_ratio, max(depth_r))
    if fusion_r:
        max_ratio = max(max_ratio, max(fusion_r))
    if max_ratio == 0.0:
        ax.set_ylim(-0.01, 0.01)
        ax.text(0.5, 0.5, "all zero", transform=ax.transAxes, ha="center", va="center")
    ax.set_title("conflict_ratio")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("conflict_ratio")
    ax.legend()
    fig.tight_layout()
    out_path = f"{out_prefix}_conflict_ratio.png"
    fig.savefig(out_path, dpi=150)
    print(f"saved plot to {out_path}")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot fusion comparison metrics.")
    workspace_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(workspace_dir, "../output")
    os.makedirs(output_dir, exist_ok=True)
    parser.add_argument("--summary", default=os.path.join(output_dir, "compare_fusion_summary.txt"), help="summary text file")
    parser.add_argument("--depth-csv", default=os.path.join(output_dir, "grid_map_fusion_stats_depth.csv"), help="depth stats CSV")
    parser.add_argument("--fusion-csv", default=os.path.join(output_dir, "grid_map_fusion_stats_fusion.csv"), help="fusion stats CSV")
    parser.add_argument("--out", default=os.path.join(output_dir, "fusion_compare"), help="output file prefix")
    args = parser.parse_args()

    if not os.path.exists(args.summary):
        print(f"summary not found: {args.summary}")
        return 1

    metrics = parse_summary(args.summary)
    if not metrics:
        print("no metrics found in summary")
        return 1

    plot_metrics(metrics, args.out, args.depth_csv, args.fusion_csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
