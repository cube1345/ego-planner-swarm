#!/usr/bin/env python3
import argparse
import os
import re
from typing import Dict, List, Tuple

METRIC_PATTERNS = {
    "success_rate": re.compile(r"fusion success_rate=([0-9.]+|N/A)"),
    "traj_failed": re.compile(r"fusion traj failed count=([0-9]+)"),
    "collided": re.compile(r"fusion collided count=([0-9]+)"),
    "rebound": re.compile(r"fusion rebound count=([0-9]+)"),
}


def parse_summary(path: str) -> Dict[str, float]:
    with open(path, "r", encoding="utf-8") as file_obj:
        text = file_obj.read()

    metrics: Dict[str, float] = {}
    for metric_name, pattern in METRIC_PATTERNS.items():
        match_obj = pattern.search(text)
        if not match_obj:
            continue
        value_text = match_obj.group(1)
        if value_text == "N/A":
            continue
        metrics[metric_name] = float(value_text)
    return metrics


def _svg_header(width: int, height: int, title_text: str) -> List[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f7f9fc"/>',
        f'<text x="{width // 2}" y="30" text-anchor="middle" font-size="20" font-family="Arial" fill="#1f2937">{title_text}</text>',
    ]


def _write_svg(path: str, lines: List[str]) -> None:
    lines.append("</svg>")
    with open(path, "w", encoding="utf-8") as file_obj:
        file_obj.write("\n".join(lines))


def plot_two_bar_svg(metric_name: str, baseline_value: float, optimized_value: float, output_path: str) -> None:
    width = 640
    height = 420
    margin_left = 90
    margin_bottom = 70
    plot_width = 500
    plot_height = 250

    max_value = max(baseline_value, optimized_value)
    if max_value <= 0:
        max_value = 1.0

    lines = _svg_header(width, height, f"Fusion Comparison: {metric_name}")
    axis_x = margin_left
    axis_y = height - margin_bottom
    lines.append(f'<line x1="{axis_x}" y1="80" x2="{axis_x}" y2="{axis_y}" stroke="#111827" stroke-width="2"/>')
    lines.append(f'<line x1="{axis_x}" y1="{axis_y}" x2="{axis_x + plot_width}" y2="{axis_y}" stroke="#111827" stroke-width="2"/>')

    bar_width = 130
    bar_gap = 120
    first_x = axis_x + 80
    second_x = first_x + bar_width + bar_gap

    def bar_height(value: float) -> float:
        return (value / max_value) * plot_height

    baseline_height = bar_height(baseline_value)
    optimized_height = bar_height(optimized_value)

    lines.append(
        f'<rect x="{first_x}" y="{axis_y - baseline_height:.2f}" width="{bar_width}" height="{baseline_height:.2f}" fill="#2563eb" rx="4"/>'
    )
    lines.append(
        f'<rect x="{second_x}" y="{axis_y - optimized_height:.2f}" width="{bar_width}" height="{optimized_height:.2f}" fill="#16a34a" rx="4"/>'
    )

    lines.append(f'<text x="{first_x + bar_width / 2}" y="{axis_y + 28}" text-anchor="middle" font-size="14" font-family="Arial">baseline</text>')
    lines.append(f'<text x="{second_x + bar_width / 2}" y="{axis_y + 28}" text-anchor="middle" font-size="14" font-family="Arial">optimized</text>')

    lines.append(
        f'<text x="{first_x + bar_width / 2}" y="{axis_y - baseline_height - 10:.2f}" text-anchor="middle" font-size="14" font-family="Arial" fill="#1f2937">{baseline_value:.3f}</text>'
    )
    lines.append(
        f'<text x="{second_x + bar_width / 2}" y="{axis_y - optimized_height - 10:.2f}" text-anchor="middle" font-size="14" font-family="Arial" fill="#1f2937">{optimized_value:.3f}</text>'
    )

    _write_svg(output_path, lines)


def build_report(baseline_metrics: Dict[str, float], optimized_metrics: Dict[str, float], output_path: str) -> None:
    lines = ["fusion comparison report", ""]
    for metric_name in ["success_rate", "traj_failed", "collided", "rebound"]:
        baseline_value = baseline_metrics.get(metric_name)
        optimized_value = optimized_metrics.get(metric_name)
        if baseline_value is None or optimized_value is None:
            lines.append(f"{metric_name}: missing")
            continue

        delta = optimized_value - baseline_value
        if metric_name == "success_rate":
            better = "optimized" if delta > 0 else "baseline"
        else:
            better = "optimized" if delta < 0 else "baseline"

        lines.append(
            f"{metric_name}: baseline={baseline_value:.3f}, optimized={optimized_value:.3f}, delta={delta:.3f}, better={better}"
        )

    with open(output_path, "w", encoding="utf-8") as file_obj:
        file_obj.write("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot baseline vs optimized fusion metrics to SVG.")
    parser.add_argument("--baseline", required=True, help="baseline summary txt")
    parser.add_argument("--optimized", required=True, help="optimized summary txt")
    parser.add_argument("--outdir", default="output", help="output directory")
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    baseline_metrics = parse_summary(args.baseline)
    optimized_metrics = parse_summary(args.optimized)

    if not baseline_metrics or not optimized_metrics:
        print("failed: missing metrics in baseline or optimized summary")
        return 1

    for metric_name in ["success_rate", "traj_failed", "collided", "rebound"]:
        if metric_name not in baseline_metrics or metric_name not in optimized_metrics:
            continue
        out_path = os.path.join(args.outdir, f"fusion_compare_runs_{metric_name}.svg")
        plot_two_bar_svg(metric_name, baseline_metrics[metric_name], optimized_metrics[metric_name], out_path)
        print(f"saved {out_path}")

    report_path = os.path.join(args.outdir, "fusion_compare_runs_report.txt")
    build_report(baseline_metrics, optimized_metrics, report_path)
    print(f"saved {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
