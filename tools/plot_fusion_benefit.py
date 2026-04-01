#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Dict, List, Optional, Sequence, Tuple


REQUIRED_COLUMNS = [
    "frame",
    "depth_recall",
    "lidar_recall",
    "fusion_recall",
    "depth_f1",
    "lidar_f1",
    "fusion_f1",
    "fusion_recall_gain_vs_best_single",
    "fusion_f1_gain_vs_best_single",
]

COLORS = {
    "depth": "#1f77b4",
    "lidar": "#ff7f0e",
    "fusion": "#2ca02c",
    "gain_recall": "#17becf",
    "gain_f1": "#d62728",
    "bg_dark": "#0e1b2a",
    "bg_light": "#17314a",
    "panel": "#f8fbff",
    "grid": "#d9e3ef",
    "axis": "#3b4b5f",
    "text": "#1a2733",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize fusion benefit metrics from fusion_benefit_report.py CSV (SVG output)."
    )
    parser.add_argument(
        "--csv",
        required=True,
        help="Input CSV path from fusion_benefit_report.py. Must be inside this repo.",
    )
    parser.add_argument(
        "--out-dir",
        default="artifacts/fusion_metrics/plots",
        help="Output directory for SVG and summary files. Must be inside this repo.",
    )
    parser.add_argument(
        "--rolling-window",
        type=int,
        default=25,
        help="Rolling mean window for trend smoothing.",
    )
    parser.add_argument(
        "--title",
        default="Fusion Benefit Evaluation",
        help="Title prefix for figures.",
    )
    return parser.parse_args()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def assert_inside_repo(path: Path, root: Path, label: str) -> Path:
    resolved = path.resolve()
    if not resolved.is_relative_to(root):
        raise ValueError(f"{label} must stay inside repo: {resolved}")
    return resolved


def float_or_nan(text: str) -> float:
    try:
        return float(text)
    except Exception:
        return float("nan")


def is_finite(value: float) -> bool:
    return not math.isnan(value) and not math.isinf(value)


def load_csv(csv_path: Path) -> Dict[str, List[float]]:
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {csv_path}")
        missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV missing required columns: {missing}")

        data: Dict[str, List[float]] = {name: [] for name in reader.fieldnames}
        for row in reader:
            for key in data.keys():
                data[key].append(float_or_nan(row.get(key, "")))

    if not data["frame"]:
        raise ValueError(f"CSV is empty: {csv_path}")
    return data


def rolling_mean(values: Sequence[float], window: int) -> List[Optional[float]]:
    if window <= 1:
        return [value if is_finite(value) else None for value in values]
    result: List[Optional[float]] = [None] * len(values)
    for index in range(len(values)):
        left = max(0, index - window + 1)
        segment = [value for value in values[left : index + 1] if is_finite(value)]
        if len(segment) < window:
            result[index] = None
        else:
            result[index] = mean(segment)
    return result


def safe_mean(values: Sequence[float]) -> float:
    valid = [value for value in values if is_finite(value)]
    return mean(valid) if valid else 0.0


def percentile(values: Sequence[float], percent: float) -> float:
    valid = sorted(value for value in values if is_finite(value))
    if not valid:
        return 0.0
    if len(valid) == 1:
        return valid[0]
    rank = (len(valid) - 1) * (percent / 100.0)
    left = int(math.floor(rank))
    right = int(math.ceil(rank))
    if left == right:
        return valid[left]
    weight = rank - left
    return valid[left] * (1.0 - weight) + valid[right] * weight


def compute_summary(data: Dict[str, List[float]]) -> Dict[str, float]:
    gain_recall = data["fusion_recall_gain_vs_best_single"]
    gain_f1 = data["fusion_f1_gain_vs_best_single"]
    valid_recall_gain = [value for value in gain_recall if is_finite(value)]
    valid_f1_gain = [value for value in gain_f1 if is_finite(value)]

    return {
        "frames": float(len(data["frame"])),
        "depth_recall_mean": safe_mean(data["depth_recall"]),
        "lidar_recall_mean": safe_mean(data["lidar_recall"]),
        "fusion_recall_mean": safe_mean(data["fusion_recall"]),
        "depth_f1_mean": safe_mean(data["depth_f1"]),
        "lidar_f1_mean": safe_mean(data["lidar_f1"]),
        "fusion_f1_mean": safe_mean(data["fusion_f1"]),
        "gain_recall_mean": safe_mean(gain_recall),
        "gain_f1_mean": safe_mean(gain_f1),
        "gain_recall_positive_ratio": (sum(value > 0.0 for value in valid_recall_gain) / len(valid_recall_gain))
        if valid_recall_gain
        else 0.0,
        "gain_f1_positive_ratio": (sum(value > 0.0 for value in valid_f1_gain) / len(valid_f1_gain))
        if valid_f1_gain
        else 0.0,
        "gain_recall_p50": percentile(gain_recall, 50.0),
        "gain_recall_p90": percentile(gain_recall, 90.0),
        "gain_f1_p50": percentile(gain_f1, 50.0),
        "gain_f1_p90": percentile(gain_f1, 90.0),
    }


def svg_text(parts: List[str], x: float, y: float, text: str, size: int = 14, color: str = COLORS["text"], anchor: str = "start", weight: int = 400) -> None:
    parts.append(
        f'<text x="{x:.1f}" y="{y:.1f}" fill="{color}" font-family="Segoe UI, Arial, sans-serif" '
        f'font-size="{size}" text-anchor="{anchor}" font-weight="{weight}">{text}</text>'
    )


def draw_panel(parts: List[str], x: float, y: float, w: float, h: float, title: str) -> None:
    parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="16" fill="{COLORS["panel"]}" />')
    svg_text(parts, x + 18, y + 34, title, size=20, weight=700)


def map_xy(x: float, y: float, x_min: float, x_max: float, y_min: float, y_max: float, area: Tuple[float, float, float, float]) -> Tuple[float, float]:
    ax, ay, aw, ah = area
    x_ratio = 0.0 if x_max <= x_min else (x - x_min) / (x_max - x_min)
    y_ratio = 0.0 if y_max <= y_min else (y - y_min) / (y_max - y_min)
    px = ax + x_ratio * aw
    py = ay + ah - y_ratio * ah
    return px, py


def draw_axes(parts: List[str], area: Tuple[float, float, float, float], y_min: float, y_max: float, y_ticks: int = 5) -> None:
    ax, ay, aw, ah = area
    parts.append(f'<rect x="{ax:.1f}" y="{ay:.1f}" width="{aw:.1f}" height="{ah:.1f}" fill="white" stroke="#d4deea" />')
    for tick in range(y_ticks + 1):
        y_value = y_min + (y_max - y_min) * tick / y_ticks
        y_line = ay + ah - (ah * tick / y_ticks)
        parts.append(f'<line x1="{ax:.1f}" y1="{y_line:.1f}" x2="{ax + aw:.1f}" y2="{y_line:.1f}" stroke="{COLORS["grid"]}" />')
        svg_text(parts, ax - 6, y_line + 5, f"{y_value:.2f}", size=11, anchor="end", color=COLORS["axis"])
    parts.append(f'<line x1="{ax:.1f}" y1="{ay + ah:.1f}" x2="{ax + aw:.1f}" y2="{ay + ah:.1f}" stroke="{COLORS["axis"]}" />')
    parts.append(f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{ax:.1f}" y2="{ay + ah:.1f}" stroke="{COLORS["axis"]}" />')


def draw_series(parts: List[str], x_values: Sequence[float], y_values: Sequence[Optional[float]], area: Tuple[float, float, float, float], y_min: float, y_max: float, color: str, width: float, alpha: float = 1.0) -> None:
    x_min = min(x_values)
    x_max = max(x_values)
    commands: List[str] = []
    segment_open = False
    for x_value, y_value in zip(x_values, y_values):
        if y_value is None or not is_finite(y_value):
            segment_open = False
            continue
        px, py = map_xy(x_value, float(y_value), x_min, x_max, y_min, y_max, area)
        if not segment_open:
            commands.append(f"M {px:.2f} {py:.2f}")
            segment_open = True
        else:
            commands.append(f"L {px:.2f} {py:.2f}")
    if commands:
        parts.append(
            f'<path d="{" ".join(commands)}" fill="none" stroke="{color}" stroke-width="{width}" opacity="{alpha:.3f}" />'
        )


def draw_legend(parts: List[str], x: float, y: float, labels: List[Tuple[str, str]]) -> None:
    cursor = x
    for label, color in labels:
        parts.append(f'<rect x="{cursor:.1f}" y="{y - 10:.1f}" width="14" height="14" rx="3" fill="{color}" />')
        svg_text(parts, cursor + 20, y + 2, label, size=12)
        cursor += 118


def plot_dashboard_svg(data: Dict[str, List[float]], summary: Dict[str, float], out_svg: Path, rolling_window: int, title: str) -> None:
    width = 1780
    height = 1120
    parts: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">'
    ]
    parts.append("<defs>")
    parts.append(
        f'<linearGradient id="bg" x1="0" y1="0" x2="1" y2="1"><stop offset="0%" stop-color="{COLORS["bg_dark"]}" /><stop offset="100%" stop-color="{COLORS["bg_light"]}" /></linearGradient>'
    )
    parts.append("</defs>")
    parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="url(#bg)" />')
    svg_text(parts, 50, 52, title, size=30, color="white", weight=800)

    draw_panel(parts, 40, 80, 840, 490, "Recall Trend")
    draw_panel(parts, 900, 80, 840, 490, "F1 Trend")
    draw_panel(parts, 40, 590, 840, 490, "Fusion Gain vs Best Single")
    draw_panel(parts, 900, 590, 840, 490, "Mean Metrics by Modality")

    frame = data["frame"]
    if len(frame) < 2:
        frame = [float(index + 1) for index in range(len(frame))]
    x_values = frame

    recall_area = (90, 150, 760, 370)
    draw_axes(parts, recall_area, 0.0, 1.0)
    draw_series(parts, x_values, data["depth_recall"], recall_area, 0.0, 1.0, COLORS["depth"], 1.0, alpha=0.30)
    draw_series(parts, x_values, data["lidar_recall"], recall_area, 0.0, 1.0, COLORS["lidar"], 1.0, alpha=0.30)
    draw_series(parts, x_values, data["fusion_recall"], recall_area, 0.0, 1.0, COLORS["fusion"], 1.0, alpha=0.30)
    draw_series(parts, x_values, rolling_mean(data["depth_recall"], rolling_window), recall_area, 0.0, 1.0, COLORS["depth"], 2.5, alpha=1.0)
    draw_series(parts, x_values, rolling_mean(data["lidar_recall"], rolling_window), recall_area, 0.0, 1.0, COLORS["lidar"], 2.5, alpha=1.0)
    draw_series(parts, x_values, rolling_mean(data["fusion_recall"], rolling_window), recall_area, 0.0, 1.0, COLORS["fusion"], 3.0, alpha=1.0)
    draw_legend(parts, 120, 130, [("depth", COLORS["depth"]), ("lidar", COLORS["lidar"]), ("fusion", COLORS["fusion"])])

    f1_area = (950, 150, 760, 370)
    draw_axes(parts, f1_area, 0.0, 1.0)
    draw_series(parts, x_values, data["depth_f1"], f1_area, 0.0, 1.0, COLORS["depth"], 1.0, alpha=0.30)
    draw_series(parts, x_values, data["lidar_f1"], f1_area, 0.0, 1.0, COLORS["lidar"], 1.0, alpha=0.30)
    draw_series(parts, x_values, data["fusion_f1"], f1_area, 0.0, 1.0, COLORS["fusion"], 1.0, alpha=0.30)
    draw_series(parts, x_values, rolling_mean(data["depth_f1"], rolling_window), f1_area, 0.0, 1.0, COLORS["depth"], 2.5, alpha=1.0)
    draw_series(parts, x_values, rolling_mean(data["lidar_f1"], rolling_window), f1_area, 0.0, 1.0, COLORS["lidar"], 2.5, alpha=1.0)
    draw_series(parts, x_values, rolling_mean(data["fusion_f1"], rolling_window), f1_area, 0.0, 1.0, COLORS["fusion"], 3.0, alpha=1.0)
    draw_legend(parts, 980, 130, [("depth", COLORS["depth"]), ("lidar", COLORS["lidar"]), ("fusion", COLORS["fusion"])])

    gain_recall = data["fusion_recall_gain_vs_best_single"]
    gain_f1 = data["fusion_f1_gain_vs_best_single"]
    valid_gain = [abs(value) for value in gain_recall + gain_f1 if is_finite(value)]
    gain_limit = max(0.02, max(valid_gain) * 1.2 if valid_gain else 0.2)
    gain_area = (90, 660, 760, 370)
    draw_axes(parts, gain_area, -gain_limit, gain_limit)
    draw_series(parts, x_values, gain_recall, gain_area, -gain_limit, gain_limit, COLORS["gain_recall"], 2.2, alpha=1.0)
    draw_series(parts, x_values, gain_f1, gain_area, -gain_limit, gain_limit, COLORS["gain_f1"], 2.2, alpha=1.0)
    zero_y = map_xy(x_values[0], 0.0, min(x_values), max(x_values), -gain_limit, gain_limit, gain_area)[1]
    parts.append(f'<line x1="{gain_area[0]:.1f}" y1="{zero_y:.1f}" x2="{gain_area[0] + gain_area[2]:.1f}" y2="{zero_y:.1f}" stroke="#111" stroke-dasharray="6 6" />')
    draw_legend(parts, 120, 640, [("gain_recall", COLORS["gain_recall"]), ("gain_f1", COLORS["gain_f1"])])

    bar_area = (950, 660, 760, 370)
    draw_axes(parts, bar_area, 0.0, 1.0)
    labels = ["depth", "lidar", "fusion"]
    recall_means = [summary["depth_recall_mean"], summary["lidar_recall_mean"], summary["fusion_recall_mean"]]
    f1_means = [summary["depth_f1_mean"], summary["lidar_f1_mean"], summary["fusion_f1_mean"]]
    x_base = bar_area[0] + 120
    step = 230
    bar_width = 58
    for index, label in enumerate(labels):
        center = x_base + index * step
        recall_height = recall_means[index] * bar_area[3]
        f1_height = f1_means[index] * bar_area[3]
        rx = center - bar_width - 8
        fx = center + 8
        ry = bar_area[1] + bar_area[3] - recall_height
        fy = bar_area[1] + bar_area[3] - f1_height
        parts.append(f'<rect x="{rx:.1f}" y="{ry:.1f}" width="{bar_width:.1f}" height="{recall_height:.1f}" fill="#4e79a7" />')
        parts.append(f'<rect x="{fx:.1f}" y="{fy:.1f}" width="{bar_width:.1f}" height="{f1_height:.1f}" fill="#f28e2b" />')
        svg_text(parts, center, bar_area[1] + bar_area[3] + 28, label, size=13, anchor="middle")
        svg_text(parts, rx + bar_width / 2.0, ry - 8, f"{recall_means[index]:.3f}", size=11, anchor="middle", color="#3b4b5f")
        svg_text(parts, fx + bar_width / 2.0, fy - 8, f"{f1_means[index]:.3f}", size=11, anchor="middle", color="#3b4b5f")
    draw_legend(parts, 980, 640, [("Recall", "#4e79a7"), ("F1", "#f28e2b")])

    parts.append("</svg>")
    out_svg.write_text("\n".join(parts), encoding="utf-8")


def histogram(values: Sequence[float], bins: int, left: float, right: float) -> List[int]:
    counts = [0] * bins
    if right <= left:
        return counts
    width = (right - left) / bins
    for value in values:
        if not is_finite(value):
            continue
        position = int((value - left) / width)
        if position < 0:
            position = 0
        if position >= bins:
            position = bins - 1
        counts[position] += 1
    return counts


def plot_gain_distribution_svg(data: Dict[str, List[float]], out_svg: Path, title: str) -> None:
    width = 1500
    height = 560
    parts: List[str] = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">']
    parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="#f4f8fc" />')
    svg_text(parts, 40, 44, f"{title} - Gain Distribution", size=28, weight=800)

    gain_recall = data["fusion_recall_gain_vs_best_single"]
    gain_f1 = data["fusion_f1_gain_vs_best_single"]
    all_gain = [value for value in gain_recall + gain_f1 if is_finite(value)]
    limit = max(0.02, max(abs(value) for value in all_gain) * 1.1 if all_gain else 0.2)
    bins = 28
    recall_hist = histogram(gain_recall, bins, -limit, limit)
    f1_hist = histogram(gain_f1, bins, -limit, limit)
    peak = max(max(recall_hist), max(f1_hist), 1)

    area = (80, 90, 1360, 420)
    draw_axes(parts, area, 0.0, float(peak))
    bin_width = area[2] / bins
    for index in range(bins):
        rx = area[0] + index * bin_width + 2
        rw = max(1.0, bin_width * 0.42)
        fh = (f1_hist[index] / peak) * area[3]
        rh = (recall_hist[index] / peak) * area[3]
        parts.append(
            f'<rect x="{rx:.1f}" y="{area[1] + area[3] - rh:.1f}" width="{rw:.1f}" height="{rh:.1f}" fill="{COLORS["gain_recall"]}" opacity="0.75" />'
        )
        parts.append(
            f'<rect x="{rx + rw + 3:.1f}" y="{area[1] + area[3] - fh:.1f}" width="{rw:.1f}" height="{fh:.1f}" fill="{COLORS["gain_f1"]}" opacity="0.75" />'
        )
    draw_legend(parts, 100, 80, [("gain_recall", COLORS["gain_recall"]), ("gain_f1", COLORS["gain_f1"])])
    svg_text(parts, area[0] + area[2] / 2.0, area[1] + area[3] + 36, f"Gain range: [{-limit:.3f}, {limit:.3f}]", size=13, anchor="middle")

    parts.append("</svg>")
    out_svg.write_text("\n".join(parts), encoding="utf-8")


def plot_positive_ratio_svg(summary: Dict[str, float], out_svg: Path, title: str) -> None:
    width = 760
    height = 520
    parts: List[str] = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">']
    parts.append(f'<rect x="0" y="0" width="{width}" height="{height}" fill="#f4f8fc" />')
    svg_text(parts, 34, 44, f"{title} - Positive Gain Ratio", size=26, weight=800)

    labels = ["gain_recall > 0", "gain_f1 > 0"]
    values = [
        summary["gain_recall_positive_ratio"] * 100.0,
        summary["gain_f1_positive_ratio"] * 100.0,
    ]
    colors = [COLORS["gain_recall"], COLORS["gain_f1"]]
    area = (90, 90, 620, 360)
    draw_axes(parts, area, 0.0, 100.0)

    step = area[2] / len(labels)
    bar_width = step * 0.42
    for index, (label, value, color) in enumerate(zip(labels, values, colors)):
        center = area[0] + step * (index + 0.5)
        bar_x = center - bar_width / 2.0
        bar_h = (value / 100.0) * area[3]
        bar_y = area[1] + area[3] - bar_h
        parts.append(f'<rect x="{bar_x:.1f}" y="{bar_y:.1f}" width="{bar_width:.1f}" height="{bar_h:.1f}" fill="{color}" opacity="0.85" />')
        svg_text(parts, center, area[1] + area[3] + 28, label, size=12, anchor="middle")
        svg_text(parts, center, bar_y - 10, f"{value:.1f}%", size=14, anchor="middle", weight=700)

    parts.append("</svg>")
    out_svg.write_text("\n".join(parts), encoding="utf-8")


def write_summary(summary: Dict[str, float], out_json: Path, out_md: Path) -> None:
    out_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    lines = ["# Fusion Benefit Summary", "", "| metric | value |", "|---|---:|"]
    for key, value in summary.items():
        lines.append(f"| {key} | {value:.6f} |")
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    root = repo_root()
    csv_path = assert_inside_repo(Path(args.csv), root, "CSV path")
    out_dir = assert_inside_repo(Path(args.out_dir), root, "Output directory")
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    out_dir.mkdir(parents=True, exist_ok=True)
    data = load_csv(csv_path)
    summary = compute_summary(data)

    dashboard_svg = out_dir / "fusion_benefit_dashboard.svg"
    gain_svg = out_dir / "fusion_benefit_gain_distribution.svg"
    ratio_svg = out_dir / "fusion_benefit_positive_ratio.svg"
    summary_json = out_dir / "fusion_benefit_summary.json"
    summary_md = out_dir / "fusion_benefit_summary.md"

    plot_dashboard_svg(data, summary, dashboard_svg, max(1, args.rolling_window), args.title)
    plot_gain_distribution_svg(data, gain_svg, args.title)
    plot_positive_ratio_svg(summary, ratio_svg, args.title)
    write_summary(summary, summary_json, summary_md)

    print(f"[ok] csv: {csv_path}")
    print(f"[ok] out: {out_dir}")
    print(f"[ok] dashboard: {dashboard_svg}")
    print(f"[ok] gain distribution: {gain_svg}")
    print(f"[ok] positive ratio: {ratio_svg}")
    print(f"[ok] summary json: {summary_json}")
    print(f"[ok] summary md: {summary_md}")


if __name__ == "__main__":
    main()
