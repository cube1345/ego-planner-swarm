#!/bin/bash
# 优化前后性能/数值稳定性对比脚本
# 用于统计融合主循环耗时和冲突率

set -e

LOG_BEFORE="/tmp/fusion_before.log"
LOG_AFTER="/tmp/fusion_after.log"

# 1. 还原为优化前版本（假设有git分支或commit）
# git checkout HEAD~1 src/planner/plan_env/src/grid_map.cpp
# 2. 编译并运行一次，统计耗时和冲突率
# (此处仅伪代码，实际需根据你的launch和统计方式调整)
# ros2 launch ego_planner advanced_param.launch.py > "$LOG_BEFORE" 2>&1
# grep "fusion耗时" "$LOG_BEFORE"
# grep "冲突率" "$LOG_BEFORE"

# 3. 应用优化后版本（已完成）
# git checkout main src/planner/plan_env/src/grid_map.cpp
# 4. 编译并运行一次，统计耗时和冲突率
# ros2 launch ego_planner advanced_param.launch.py > "$LOG_AFTER" 2>&1
# grep "fusion耗时" "$LOG_AFTER"
# grep "冲突率" "$LOG_AFTER"

# 5. 对比结果
# echo "==== 优化前 ===="
# cat "$LOG_BEFORE"
# echo "==== 优化后 ===="
# cat "$LOG_AFTER"

# 实际用法：
# - 可在融合主循环前后加RCLCPP_INFO统计时间
# - 冲突率已在md_.last_conflict_ratio_输出
# - 可用plot_compare.py可视化冲突分布

# 优化好处说明：
# 1. 数值稳定性提升，避免除零和溢出。
# 2. 性能提升，结构更易并行和批量处理。
# 3. 可读性提升，便于维护和调试。
# 4. 冲突判定更清晰，统计更准确。

# 如需自动化对比，可补充具体launch和统计命令。
