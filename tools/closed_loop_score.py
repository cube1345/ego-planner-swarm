#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Optional


def finite_float(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except Exception:
        return default
    return number if math.isfinite(number) else default


def clamp(value: float, lower: float = 0.0, upper: float = 1.0) -> float:
    return max(lower, min(upper, value))


def load_json(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def compute_closed_loop_score(
    fusion: Dict[str, Any], sim: Dict[str, Any], args: argparse.Namespace
) -> Dict[str, Any]:
    fusion_f1 = finite_float(fusion.get("fusion_f1"))
    fusion_recall = finite_float(fusion.get("fusion_recall"))
    path_length = finite_float(sim.get("path_length_m"))
    replan_count = finite_float(sim.get("replan_count"))
    collision_risk = finite_float(sim.get("collision_risk_score"), default=1.0)
    accel_rms = finite_float(sim.get("accel_rms_mps2"))
    occupancy_jitter = finite_float(sim.get("occupancy_jitter_ratio"))
    switch_rate = finite_float(sim.get("adaptive_param_switch_rate"))
    ds_unknown = finite_float(sim.get("ds_unknown_mean"))
    ds_conflict = finite_float(sim.get("ds_conflict_mean"))

    path_penalty = clamp(path_length / max(1e-6, args.path_ref_m))
    replan_penalty = clamp(replan_count / max(1e-6, args.replan_ref))
    smoothness_penalty = clamp(accel_rms / max(1e-6, args.accel_ref))
    map_jitter_penalty = clamp(occupancy_jitter / max(1e-6, args.jitter_ref))
    switch_penalty = clamp(switch_rate)
    ds_unknown_penalty = clamp(ds_unknown / max(1e-6, args.ds_unknown_ref))
    ds_conflict_penalty = clamp(ds_conflict / max(1e-6, args.ds_conflict_ref))

    closed_loop_score = (
        args.w_f1 * fusion_f1
        + args.w_recall * fusion_recall
        - args.w_path * path_penalty
        - args.w_replan * replan_penalty
        - args.w_collision * collision_risk
        - args.w_smoothness * smoothness_penalty
        - args.w_jitter * map_jitter_penalty
        - args.w_switch * switch_penalty
        - args.w_ds_unknown * ds_unknown_penalty
        - args.w_ds_conflict * ds_conflict_penalty
    )

    return {
        "closed_loop_score": float(closed_loop_score),
        "objective": (
            "J = w_f1*F1 + w_recall*Recall - w_path*PathLengthNorm "
            "- w_replan*ReplanNorm - w_collision*CollisionRisk "
            "- w_smoothness*SmoothnessNorm - w_jitter*MapJitterNorm "
            "- w_switch*ParamSwitchRate - w_ds_unknown*DSUnknownNorm "
            "- w_ds_conflict*DSConflictNorm"
        ),
        "weights": {
            "w_f1": args.w_f1,
            "w_recall": args.w_recall,
            "w_path": args.w_path,
            "w_replan": args.w_replan,
            "w_collision": args.w_collision,
            "w_smoothness": args.w_smoothness,
            "w_jitter": args.w_jitter,
            "w_switch": args.w_switch,
            "w_ds_unknown": args.w_ds_unknown,
            "w_ds_conflict": args.w_ds_conflict,
        },
        "raw_metrics": {
            "fusion_f1": fusion_f1,
            "fusion_recall": fusion_recall,
            "path_length_m": path_length,
            "replan_count": replan_count,
            "collision_risk_score": collision_risk,
            "accel_rms_mps2": accel_rms,
            "occupancy_jitter_ratio": occupancy_jitter,
            "adaptive_param_switch_rate": switch_rate,
            "ds_unknown_mean": ds_unknown,
            "ds_conflict_mean": ds_conflict,
        },
        "normalized_penalties": {
            "path_length": path_penalty,
            "replan_count": replan_penalty,
            "collision_risk": collision_risk,
            "smoothness": smoothness_penalty,
            "map_jitter": map_jitter_penalty,
            "param_switch": switch_penalty,
            "ds_unknown": ds_unknown_penalty,
            "ds_conflict": ds_conflict_penalty,
        },
        "references": {
            "path_ref_m": args.path_ref_m,
            "replan_ref": args.replan_ref,
            "accel_ref": args.accel_ref,
            "jitter_ref": args.jitter_ref,
            "ds_unknown_ref": args.ds_unknown_ref,
            "ds_conflict_ref": args.ds_conflict_ref,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Merge fusion and flight stats into a closed-loop objective.")
    parser.add_argument("--fusion-summary", required=True)
    parser.add_argument("--sim-summary", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--update-fusion-summary", action="store_true")
    parser.add_argument("--w-f1", type=float, default=1.0)
    parser.add_argument("--w-recall", type=float, default=0.6)
    parser.add_argument("--w-path", type=float, default=0.10)
    parser.add_argument("--w-replan", type=float, default=0.08)
    parser.add_argument("--w-collision", type=float, default=0.35)
    parser.add_argument("--w-smoothness", type=float, default=0.05)
    parser.add_argument("--w-jitter", type=float, default=0.04)
    parser.add_argument("--w-switch", type=float, default=0.03)
    parser.add_argument("--w-ds-unknown", type=float, default=0.02)
    parser.add_argument("--w-ds-conflict", type=float, default=0.04)
    parser.add_argument("--path-ref-m", type=float, default=30.0)
    parser.add_argument("--replan-ref", type=float, default=50.0)
    parser.add_argument("--accel-ref", type=float, default=10.0)
    parser.add_argument("--jitter-ref", type=float, default=0.20)
    parser.add_argument("--ds-unknown-ref", type=float, default=0.50)
    parser.add_argument("--ds-conflict-ref", type=float, default=0.08)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    fusion_path = Path(args.fusion_summary)
    sim_path = Path(args.sim_summary)
    output_path = Path(args.output)

    fusion = load_json(fusion_path)
    sim = load_json(sim_path)
    closed_loop = compute_closed_loop_score(fusion, sim, args)
    merged = {
        "fusion": fusion,
        "simulation": sim,
        "closed_loop": closed_loop,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(merged, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    if args.update_fusion_summary and fusion_path.is_file():
        fusion["simulation"] = sim
        fusion["closed_loop"] = closed_loop
        fusion_path.write_text(json.dumps(fusion, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(json.dumps(closed_loop, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
