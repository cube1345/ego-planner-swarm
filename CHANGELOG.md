# Changelog

## 2026-05-25

### README And Usage Docs

- Added `README.md` as the English usage entry for the current ROS 2 UAV avoidance stack.
- Added `README.zh-CN.md` as the Chinese usage entry.
- Replaced legacy `Readme.md` content with a short pointer to the standard English and Chinese README files.
- Kept README content focused on project purpose, build, RViz simulation startup, headless evaluation, and stopping simulation.

## 2026-05-24(bash scripts)

### One-Command Simulation Launcher

- Added `tools/start_rviz_sim.sh` for one-command simulation startup.
- The launcher clears stale ROS / RViz processes before starting a new run.
- Supported launch modes:
  - `--mode fusion`
  - `--mode plain`
  - `--no-rviz`
  - `--kill-only`
- Default RViz config remains `src/planner/plan_manage/launch/drone0_clean.rviz`.

## 2026-05-10

### RViz And UAV Visualization

- Updated `drone0_clean.rviz` so the default UAV display uses the original `/drone_0_vis/robot` hummingbird mesh.
- Added a disabled-by-default `/drone_0_vis/robot_body` fallback marker display for debugging mesh visibility issues.
- Updated `odom_visualization` to publish a lightweight fallback body marker while keeping the original mesh marker path.
- Adjusted simulator visualization parameters so the UAV mesh is easier to see in RViz.

## 2026-04-20(in Dynamic)

### Multimodal Fusion And Dynamic Simulation Support

- Kept the fusion simulation path based on `single_run_in_sim_fusion.launch.py`.
- Confirmed the current fusion chain uses depth cloud, simulated LiDAR cloud, fused cloud, D-S metrics, and closed-loop feedback observation.
- Added / retained dynamic obstacle cloud support through `dynamic_obstacle_cloud.py`.
- Dynamic obstacle outputs include:
  - `/drone_0_dynamic_obstacles/cloud`
  - `/drone_0_dynamic_obstacles/markers`
- Dynamic obstacle metrics are available through the headless stats pipeline, including minimum dynamic obstacle distance, dynamic safety violation ratio, and dynamic collision risk score.

## 2026-04-10（docs）

### Paper And Documentation

- Added `docs/ego_planner_multimodal_fusion_paper.md` as the main paper-style project document.
- Expanded the paper with detailed mathematical descriptions for:
  - multimodal voxel fusion
  - distance-adaptive confidence
  - Log-odds evidence accumulation
  - Dempster-Shafer evidence metrics
  - online adaptive parameter selection
  - closed-loop feedback and delayed attribution
- Added a short dynamic obstacle simulation and safety-metric section to the paper.


### Current Engineering Position

- The stable default demo path remains static-obstacle RViz simulation with the fusion chain available through `tools/start_rviz_sim.sh --mode fusion`.
- Dynamic obstacle support is treated as simulation and evaluation extension rather than a fully validated standalone dynamic-avoidance algorithm.
- Closed-loop feedback remains useful for observation and metrics; the online closed-loop optimizer remains disabled by default.

### ds？...


### Simulation And RViz

- Added `src/planner/plan_manage/launch/drone0_clean.rviz` as the clean single-drone RViz view.
- Changed `src/planner/plan_manage/launch/rviz.launch.py` default config from `default.rviz` to `drone0_clean.rviz`.
- Added `tools/run_adaptive_rviz_demo.sh` to clear stale ROS / RViz processes before launching one clean adaptive fusion demo.
- Documented the operational difference between the original non-fusion chain and the adaptive fusion chain.



### Headless Metrics

- Fixed duplicate shutdown handling in `src/planner/plan_manage/scripts/fusion_benefit_report.py`.
- Fixed `tools/sim_flight_stats_report.py` so `replan_count` is counted from the final batch `launch.log`.
- Changed `tools/sim_flight_stats_report.py` to finish on external stop rather than timing out early, preventing tail-end replans from being dropped.
- Changed `tools/compare_fusion.sh` stop order to stop launch first, then stats, then report, so final counts match the completed run.

## ？？

### Documentation Refresh

- Rewrote `Readme.md` to match the current project structure and launch recommendations.
- Updated `docs/latest_operation_guide.md` to use the clean RViz path and the adaptive fusion demo script.
- Updated `docs/multimodal_fusion_code_walkthrough.md` to reflect the current code path and the actual active adaptive-parameter logic.
- Updated `docs/multimodal_fusion_paper_style.md` to align the paper-style description with the current engineering implementation.


### Adaptive Parameter Status

- Clarified that the current project has only one truly online adaptive parameter: fusion-layer `min_probability`.
- Clarified that `adaptive_min_probability_min/max/step`, `adaptive_eval_range`, `adaptive_min_gt_voxels`, and `adaptive_score_alpha` are controller hyperparameters for the adaptation process rather than independently adapted runtime parameters.
- Clarified that `adaptive_target_retention` and `adaptive_retention_band` are declared but are not part of the current online decision logic.


### Multimodal Fusion

- Added adaptive `min_probability` tuning to `src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py`.
- Switched threshold selection objective from absolute fusion `f1` to relative gain over the best single sensor using `gain_f1 + 0.35 * gain_recall`.
- Moved adaptive threshold selection ahead of fused cloud publication so the chosen threshold takes effect in the current frame.
- Added local GT subscription and evaluation helpers driven by `/map_generator/global_cloud`.
- Updated default adaptive search range to `0.20 ~ 0.35` and default `min_probability` to `0.30`.

### Launch And Tooling

- Extended `src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py` to pass adaptive tuning parameters into the fusion node.
- Aligned the fusion simulation launch with the normal single-sensor scene by using the `random_forest` layout, restoring the straight-line waypoint pattern, and setting the default `map_size_y` back to `25.0` for easier RViz comparison.
- Added `tools/compare_fusion.sh` for deterministic fusion A/B evaluation on `mockamap(seed=127)`.

### Adaptive Parameter Principle

- The adaptive parameter currently focuses on the fusion-layer obstacle acceptance threshold `min_probability` rather than planner weights or state-estimation gains.
- For every synchronized depth and lidar frame, the node first builds a shared voxel evidence table. Each voxel accumulates log-odds style support from depth and lidar observations.
- The node then evaluates multiple candidate `min_probability` values inside the configured range `adaptive_min_probability_min ~ adaptive_min_probability_max`.
- For each candidate threshold, the node converts the accepted voxels into a local obstacle set and compares it against the local ground-truth obstacle voxels from `/map_generator/global_cloud`.
- The scoring target is not raw fusion `f1`. Instead, it optimizes relative benefit over the stronger single sensor:
  `utility = gain_f1 + 0.35 * gain_recall`
- Here, `gain_f1` means `fusion_f1 - best_single_f1`, and `gain_recall` means `fusion_recall - best_single_recall`. This forces adaptation to prefer thresholds that create real multimodal gain rather than only increasing fused point count.
- Candidate utilities are smoothed with an exponential moving average controlled by `adaptive_score_alpha`, which suppresses frame-level noise and reduces threshold oscillation.
- The selected threshold is applied in the current frame before the fused cloud is published, so the planner immediately consumes the newly selected obstacle map.
- In the current validated configuration, the profitable search region is concentrated in the low-threshold band `0.20 ~ 0.35`, with the adaptive process frequently converging toward the lower boundary when denser obstacle retention improves net gain.

### Verified Results

- Fixed `min_probability=0.55`: `fusion_f1=0.0605`, `fusion_f1_gain_vs_best_single=-0.0297`.
- Fixed `min_probability=0.40`: `fusion_f1=0.1291`, `fusion_f1_gain_vs_best_single=+0.0112`.
- Adaptive `min_probability` with range `0.20 ~ 0.35`: `fusion_f1=0.2893`, `fusion_recall=0.1693`, `fusion_f1_gain_vs_best_single=+0.1000`, `fusion_recall_gain_vs_best_single=+0.0647`.
- Positive gain ratio for the adaptive run reached `100%` on both `fusion_f1_gain_vs_best_single` and `fusion_recall_gain_vs_best_single`.
