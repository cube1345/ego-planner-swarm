# Ego-Planner 多模态融合项目公共知识库

本目录用于持续记录本项目的目标、架构、算法、实验进展、运行方法和已知问题。后续每次完成代码改动、参数调整、仿真验证或指标复盘，都应同步更新这里，避免项目知识只留在聊天记录或临时笔记里。

## 快速入口

- [项目目标与当前状态](./00_project_goal_and_status.md)
- [系统架构与节点关系](./01_system_architecture.md)
- [算法与技术栈](./02_algorithms_and_tech_stack.md)
- [多模态融合链路](./03_multimodal_fusion_pipeline.md)
- [运行、验证与排错手册](./04_runbook.md)
- [进度日志](./05_progress_log.md)
- [术语表](./06_glossary.md)

## 更新规则

1. 改 launch、topic、节点关系时，更新 `01_system_architecture.md` 和 `04_runbook.md`。
2. 改 fusion、planner、动态避障算法时，更新 `02_algorithms_and_tech_stack.md` 和 `03_multimodal_fusion_pipeline.md`。
3. 跑完实验或拿到指标时，更新 `05_progress_log.md`，写清日期、配置、命令、结果、结论。
4. 遇到环境问题、构建问题、source 问题时，更新 `04_runbook.md` 的已知问题。
5. 新增概念解释时，补到 `06_glossary.md`。

## 当前主线

当前项目主线是把 EGO-Planner 的局部避障系统扩展为三模态感知融合：

```text
Depth cloud + LiDAR cloud + mmWave radar cloud
    -> occupancy evidence fusion
    -> /drone_0_fusion/fused_cloud
    -> EGO-Planner grid_map
    -> local trajectory
    -> pos_cmd
    -> simulator odom feedback
```

当前已经验证 `/drone_0_radar/points` 和 `/drone_0_fusion/fused_cloud` 都有非空输出。下一步重点是补充 radar 对 fused voxel 的贡献统计，例如 `radar_only_voxels`、`depth_radar_voxels`、`lidar_radar_voxels`、`tri_modal_voxels`，从而判断第三模态是否真的对融合结果产生贡献。
