# Repository Guidelines

## Project Structure & Module Organization
This repository is a ROS 2 (`ament_cmake`) workspace. Core code lives under `src/`:
- `src/planner/`: planning stack (`plan_env`, `path_searching`, `bspline_opt`, `plan_manage`, `traj_utils`, `drone_detect`, `rosmsg_tcp_bridge`).
- `src/uav_simulator/`: simulator, sensing, maps, and utility message packages.
- `tools/`: experiment and analysis scripts (for example `compare_fusion.sh`, `plot_compare.py`).
- `output/`: generated experiment logs/plots.

Build artifacts are workspace-level: `build/`, `install/`, `log/`.

## Project Focus
- This project targets **algorithmic optimization of EGO-Planner on ROS 2**, with emphasis on the **perception-layer map modeling pipeline**.
- Optimization work should prioritize **multi-modal environment sensing fusion** (for example depth + LiDAR + odometry-consistent updates).
- Any optimization is incomplete without an objective comparison against a baseline.

## Build, Test, and Development Commands
Run from repository root:
- `colcon build --symlink-install`  
  Builds all packages and keeps install symlinks convenient for iteration.
- `source install/setup.bash`  
  Loads workspace overlays before running nodes.
- `ros2 launch ego_planner rviz.launch.py`  
  Starts RViz visualization.
- `ros2 launch ego_planner single_run_in_sim.launch.py`  
  Runs a single-drone simulation.
- `colcon test` and `colcon test-result --verbose`  
  Executes package tests/lint checks and prints results.

## Coding Style & Naming Conventions
- C++ standard is C++17 across packages (`CMAKE_CXX_STANDARD 17`).
- Follow existing C++ style: braces on new lines, 2-space indentation, and `snake_case` for variables/functions.
- Keep package and file naming ROS-friendly and lowercase (for example `plan_env`, `drone_detect_node.cpp`).
- Python launch/config helpers should follow PEP 8 where practical.

## Testing Guidelines
- Prefer package-scoped test runs while iterating: `colcon test --packages-select <pkg_name>`.
- Existing tests include gtest-based coverage (for example `src/uav_simulator/Utils/uav_utils/src/uav_utils_test.cpp`).
- Name new C++ tests as `*_test.cpp` and register with `ament_add_gtest`.
- Add regression tests for planner/sensing changes that alter map fusion, trajectory validity, or collision behavior.

## Optimization Comparison Workflow
- For each map-modeling optimization, run **paired experiments**: `baseline` vs `optimized`.
- Comparison dimensions may vary by change, e.g.:
  - trajectory success/failure
  - collision/rebound counts
  - map conflict ratio over time
  - planning stability/latency
- Store logs and metric summaries under `output/` with clear run labels.
- Use the existing **rospy virtual environment** for plotting and result visualization.
- Preferred plotting flow:
  - `source <rospy_venv>/bin/activate`
  - `python tools/plot_compare.py`
- Every optimization PR should include at least one generated figure and a short interpretation of the result.

## Commit & Pull Request Guidelines
Recent history mixes brief updates (`Update Readme.md`) and scoped commits (`feat(grid_map): ...`). For consistency, use:
- `<type>(<scope>): <imperative summary>` (example: `fix(plan_env): guard lidar-depth sync timeout`).

For PRs, include:
- What changed and why.
- Affected packages/launch files.
- Reproduction and verification steps (commands used).
- Screenshots/plots for visualization or metrics changes (from `output/` when relevant).

## Cube Work Protocol (Project-Integrated)
This section applies when collaborating with cube on this repository.

### Identity and Communication
- Role: copilot executor; cube sets direction, I implement.
- Language: follow cube language preference; keep technical terms in English.
- Environment convention: use POSIX paths in Bash (`/c/Users/...`) and Windows paths for non-Bash tools (`C:\\Users\\...`) when operating on Win11 Git Bash.
- Response prefix: `⚡ 模式：<mode>` based on task type.

### Core Execution Rules
- Verify before claiming: do not state unverified build/test/API facts.
- Read before answer: any file/function reference must be read first.
- Default flow: implement first, self-fix errors, then report.
- Ask first only when:
  - There are 2+ fundamentally different implementation paths.
  - Root cause cannot be inferred from code or reproducible evidence.
- Architecture-impacting changes (public interface/schema/auth/new module) require plan-first and call-site impact scan.

### Repo-Specific Delivery Standard
- Use minimal-change debugging loop: Reproduce -> Isolate -> Root Cause -> Fix -> Verify.
- For planner/simulator changes, always verify with at least one launch path, e.g.:
  - `colcon build --symlink-install`
  - `source install/setup.bash`
  - `ros2 launch ego_planner single_run_in_sim.launch.py`
- For mapping fusion or metrics updates, run and check `tools/compare_fusion.sh` and `tools/plot_compare.py` outputs under `output/`.
- Plot generation must run inside the prepared `rospy` virtual environment for consistency across comparisons.
- Multi-step tasks must be tracked to closure; if direction drifts (large unexpected diff or repeated edits), stop and reassess.

### Git and Safety
- Prefer atomic commits with clear scope.
- Use `git revert` for rollback; avoid destructive history edits unless explicitly requested.
- Never overwrite unrelated local changes.
- If uncertain, state uncertainty explicitly and mark pending verification (`TODO: verify`).
