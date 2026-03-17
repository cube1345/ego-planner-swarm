#!/usr/bin/env python3
import argparse
import os
import re
from typing import Dict

import matplotlib.pyplot as plt
from matplotlib import patches


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

METRICS = [
    ("fusion_success_rate", "Fusion Success Rate", True),
    ("fusion_traj_failed", "Fusion Trajectory Failures", False),
    ("fusion_collided", "Fusion Collision Recoveries", False),
    ("fusion_rebound", "Fusion Rebound Events", False),
]

BASELINE_COLOR = "#b45309"
OPTIMIZED_COLOR = "#0f766e"
BG_COLOR = "#f6f5f1"
CARD_COLOR = "#fffdfa"
GRID_COLOR = "#d6d3d1"
TEXT_COLOR = "#1f2937"
MUTED_TEXT = "#6b7280"


def parse_summary(path: str) -> Dict[str, float]:
    with open(path, "r", encoding="utf-8") as file_obj:
        text = file_obj.read()

    metrics: Dict[str, float] = {}
    for key, pattern in SUMMARY_PATTERN.items():
        match = pattern.search(text)
        if not match:
            continue
        value = match.group(1)
        if value == "N/A":
            continue
        metrics[key] = float(value)
    return metrics


def improvement(before_value: float, after_value: float, higher_is_better: bool) -> float:
    if higher_is_better:
        return after_value - before_value
    return before_value - after_value


def apply_style() -> None:
    plt.rcParams.update(
        {
            "figure.facecolor": BG_COLOR,
            "axes.facecolor": CARD_COLOR,
            "savefig.facecolor": BG_COLOR,
            "font.family": "DejaVu Sans",
            "axes.edgecolor": "#e7e5e4",
            "axes.labelcolor": TEXT_COLOR,
            "xtick.color": MUTED_TEXT,
            "ytick.color": MUTED_TEXT,
            "text.color": TEXT_COLOR,
        }
    )


def make_dashboard(before_metrics: Dict[str, float], after_metrics: Dict[str, float], out_path: str) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.subplots_adjust(top=0.82, left=0.06, right=0.97, bottom=0.08, hspace=0.32, wspace=0.20)

    fig.text(0.06, 0.94, "Timing Alignment And Fallback Optimization", fontsize=24, fontweight="bold", ha="left")
    fig.text(
        0.06,
        0.905,
        "Baseline vs optimized fusion behavior under the same 30s paired simulation protocol.",
        fontsize=12,
        color=MUTED_TEXT,
        ha="left",
    )

    for ax, (metric_key, title, higher_is_better) in zip(axes.flat, METRICS):
        before_value = before_metrics[metric_key]
        after_value = after_metrics[metric_key]
        gain = improvement(before_value, after_value, higher_is_better)
        ymax = max(before_value, after_value) * 1.25
        if ymax <= 0:
            ymax = 1.0

        bars = ax.bar(
            ["Before", "After"],
            [before_value, after_value],
            color=[BASELINE_COLOR, OPTIMIZED_COLOR],
            width=0.56,
            edgecolor="none",
            zorder=3,
        )
        ax.set_ylim(0, ymax)
        ax.grid(axis="y", color=GRID_COLOR, linestyle="--", linewidth=0.8, alpha=0.7, zorder=0)
        ax.set_axisbelow(True)
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.set_title(title, loc="left", fontsize=14, fontweight="bold", pad=12)

        value_fmt = "{:.3f}" if "rate" in metric_key else "{:.0f}"
        for bar, value in zip(bars, [before_value, after_value]):
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                bar.get_height() + ymax * 0.03,
                value_fmt.format(value),
                ha="center",
                va="bottom",
                fontsize=11,
                fontweight="bold",
            )

        better = gain >= 0
        badge_color = OPTIMIZED_COLOR if better else "#b91c1c"
        badge = patches.FancyBboxPatch(
            (0.03, 0.84),
            0.46,
            0.11,
            transform=ax.transAxes,
            boxstyle="round,pad=0.015,rounding_size=0.03",
            linewidth=0.0,
            facecolor=badge_color,
        )
        ax.add_patch(badge)
        prefix = "+" if gain >= 0 else "-"
        gain_text = f"{prefix}{abs(gain):.3f}" if "rate" in metric_key else f"{prefix}{abs(gain):.0f}"
        ax.text(0.05, 0.895, f"{gain_text} vs baseline", transform=ax.transAxes, fontsize=10, color="white", va="center")

    fig.savefig(out_path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot before/after timing optimization results.")
    parser.add_argument("--before", required=True, help="Summary file before optimization")
    parser.add_argument("--after", required=True, help="Summary file after optimization")
    parser.add_argument("--out", required=True, help="Output image path")
    args = parser.parse_args()

    before_metrics = parse_summary(args.before)
    after_metrics = parse_summary(args.after)
    if not before_metrics or not after_metrics:
      print("missing metrics in before or after summary")
      return 1

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    apply_style()
    make_dashboard(before_metrics, after_metrics, args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
