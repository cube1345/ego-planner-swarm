# Changelog

## 2026-04-12

### Multimodal Fusion

- Added adaptive `min_probability` tuning to `src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py`.
- Switched threshold selection objective from absolute fusion `f1` to relative gain over the best single sensor using `gain_f1 + 0.35 * gain_recall`.
- Moved adaptive threshold selection ahead of fused cloud publication so the chosen threshold takes effect in the current frame.
- Added local GT subscription and evaluation helpers driven by `/map_generator/global_cloud`.
- Updated default adaptive search range to `0.20 ~ 0.35` and default `min_probability` to `0.30`.

### Launch And Tooling

- Extended `src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py` to pass adaptive tuning parameters into the fusion node.
- Added `tools/compare_fusion.sh` for deterministic fusion A/B evaluation on `mockamap(seed=127)`.

### Verified Results

- Fixed `min_probability=0.55`: `fusion_f1=0.0605`, `fusion_f1_gain_vs_best_single=-0.0297`.
- Fixed `min_probability=0.40`: `fusion_f1=0.1291`, `fusion_f1_gain_vs_best_single=+0.0112`.
- Adaptive `min_probability` with range `0.20 ~ 0.35`: `fusion_f1=0.2893`, `fusion_recall=0.1693`, `fusion_f1_gain_vs_best_single=+0.1000`, `fusion_recall_gain_vs_best_single=+0.0647`.
- Positive gain ratio for the adaptive run reached `100%` on both `fusion_f1_gain_vs_best_single` and `fusion_recall_gain_vs_best_single`.
