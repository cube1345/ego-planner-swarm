# EGO-Planner 多模态融合代码说明（ROS 2）

更新时间：2026-05-13

## 1. 当前仓库里的融合链路是什么

当前仿真中的多模态融合是一个几何层融合链路，不是状态估计融合，也不是规划权重融合。

输入：

- 深度点云：`/drone_0_pcl_render_node/cloud`
- 模拟 LiDAR 点云：`/drone_0_lidar/points`

输出：

- 融合点云：`/drone_0_fusion/fused_cloud`

规划器最终订阅：

- `grid_map/cloud`

在融合启动文件里，`grid_map/cloud` 被重映射到 `/drone_0_fusion/fused_cloud`。

关键文件：

- `src/planner/plan_manage/launch/single_run_in_sim_fusion.launch.py`
- `src/planner/plan_manage/scripts/ros2_lidar_depth_fusion_node.py`
- `src/planner/plan_manage/scripts/simulated_lidar_cloud.py`
- `src/planner/plan_manage/launch/advanced_param.launch.py`

## 2. 数据流

```text
/map_generator/global_cloud
        |
        v
simulated_lidar_cloud.py ----------> /drone_0_lidar/points
                                     (simulated lidar)

/drone_0_pcl_render_node/cloud ----> /drone_0_pcl_render_node/cloud
                                     (depth cloud)

                  +-----------------------------------+
                  | ros2_lidar_depth_fusion_node.py   |
                  | - ApproximateTime sync            |
                  | - z/range filter                  |
                  | - voxel evidence fusion           |
                  | - adaptive min_probability        |
                  | - adaptive min_hits / dual_bonus  |
                  | - closed-loop score feedback      |
                  +----------------+------------------+
                                   |
                                   v
                     /drone_0_fusion/fused_cloud
                                   |
                                   v
                  advanced_param.launch.py -> grid_map/cloud
                                   |
                                   v
                            EGO-Planner grid_map
```

## 3. 启动层怎么把融合接进去

### 3.1 融合启动文件

`single_run_in_sim_fusion.launch.py` 做了四件事：

1. 启动地图和原始仿真传感器
2. 启动模拟 LiDAR 节点
3. 启动融合节点
4. 把 planner 的 `grid_map/cloud` 改接融合点云

当前这个启动文件还负责把自适应阈值参数传给融合节点，包括：

- `min_probability`
- `min_hits`
- `adaptive_min_probability_enable`
- `adaptive_min_probability_min`
- `adaptive_min_probability_max`
- `adaptive_min_probability_step`
- `adaptive_min_hits_enable`
- `adaptive_min_hits_min`
- `adaptive_min_hits_max`
- `adaptive_dual_bonus_enable`
- `adaptive_dual_bonus_min`
- `adaptive_dual_bonus_max`
- `adaptive_dual_bonus_step`
- `adaptive_eval_range`
- `adaptive_min_gt_voxels`
- `adaptive_score_alpha`
- `closed_loop_feedback_enable`
- `closed_loop_feedback_weight`

### 3.2 非融合原版链路

`single_run_in_sim.launch.py` 不启动模拟 LiDAR，也不启动融合节点。  
它让 planner 直接读取 `pcl_render_node/cloud`。

这也是为什么原版链路更简单、更容易排查。

## 4. 融合节点内部逻辑

### 4.1 预处理

融合节点先对两路点云做：

- 非有限值剔除
- 高度裁剪：`z_min ~ z_max`
- 距离裁剪：`max_range`

对应函数：

- `cloud_to_xyz()`
- `filter_points()`

### 4.2 距离自适应证据模型

当前算法不是简单拼接点云，而是把两类点映射成“占据证据”。

设计原则：

- 近场更信任 depth
- 中远场更信任 lidar

对应函数：

- `depth_probability()`
- `lidar_probability()`

### 4.3 体素级证据融合

融合核心是：

- 把两路点投影到统一体素网格
- 以体素为单位积累 depth 与 lidar 的 log-odds 证据
- 计算每个体素的最终占据概率
- 用 `min_probability` 和 `min_hits` 决定体素是否保留

对应函数：

- `accumulate_voxels()`
- `build_fusion_state()`
- `select_fused_points()`

所以当前方法本质上是：

“体素级、概率证据式、距离自适应的几何融合”

不是：

- 原始点云并集
- 图像层融合
- 学习式语义融合

## 5. 当前在线自适应参数到底有几个

当前默认真正在线自适应的参数有三个：

- `current_min_probability`
- `current_min_hits`
- `current_dual_bonus`

它们的静态基准值来自：

- `min_probability`
- `min_hits`
- `adaptive_dual_bonus_min ~ adaptive_dual_bonus_max`

它们的候选搜索空间由以下参数控制：

- `adaptive_min_probability_min`
- `adaptive_min_probability_max`
- `adaptive_min_probability_step`
- `adaptive_min_hits_min`
- `adaptive_min_hits_max`
- `adaptive_dual_bonus_min`
- `adaptive_dual_bonus_max`
- `adaptive_dual_bonus_step`

### 5.1 自适应流程

每次同步回调内都会调用：

- `auto_tune_parameters()`

流程如下：

1. 从 `/map_generator/global_cloud` 里取无人机附近的局部 GT 体素
2. 计算当前 depth-only 和 lidar-only 的局部指标
3. 枚举 `min_probability`、`min_hits`、`dual_bonus` 的候选组合
4. 对每个候选组合生成一份预测障碍集合
5. 计算候选组合相对最佳单传感器的收益：
   - `gain_recall`
   - `gain_f1`
6. 定义目标函数：
   - `utility = gain_f1 + 0.35 * gain_recall`
7. 对每个候选 utility 做 EMA 平滑
8. 若 closed-loop feedback 可用，则对候选评分加入较小权重的闭环修正
9. 取最高分候选作为当前帧的 `current_min_probability`、`current_min_hits` 和 `current_dual_bonus`

### 5.2 哪些参数只是“控制自适应过程”

以下参数不是被在线调整的对象，而是控制在线调节过程的超参数：

- `adaptive_min_probability_enable`
- `adaptive_eval_range`
- `adaptive_min_gt_voxels`
- `adaptive_score_alpha`
- `closed_loop_feedback_weight`

### 5.3 当前试验后不默认启用的参数

代码里还声明或实现了：

- `adaptive_target_retention`
- `adaptive_retention_band`
- `adaptive_lidar_growth_enable`
- `adaptive_z_max_enable`
- `adaptive_depth_decay_enable`
- `adaptive_near_field_radius_enable`

其中 `adaptive_lidar_growth`、`adaptive_z_max` 和过宽 `adaptive_near_field_radius` 已经做过 batch 评估，但当前指标不支持默认启用。也就是说，当前默认保留的是：

- 融合层障碍接受阈值 `min_probability`
- 体素命中门限 `min_hits`
- 双传感器一致性奖励 `dual_bonus`

## 6. 为什么要有 `force_zero_stamp`

当前仿真链路中，`pcl_render_node/cloud` 的时间戳是零。  
如果模拟 LiDAR 正常使用当前时钟，`ApproximateTimeSynchronizer` 很难稳定把两路数据配上。

所以 `simulated_lidar_cloud.py` 里引入了：

- `force_zero_stamp`

并且在融合仿真启动时默认设成：

- `True`

这样做的目的不是“更真实”，而是保证当前仿真路径里同步回调稳定触发。

## 7. RViz 链路为什么后来要改

之前的 `default.rviz` 混了很多 `drone_1 ~ drone_20` 的历史显示项，  
即使当前只跑一架 `drone_0`，也容易出现：

- 视觉混乱
- 看错机器人
- 误判无人机没动

所以当前项目新增并推荐：

- `src/planner/plan_manage/launch/drone0_clean.rviz`

同时 `rviz.launch.py` 的默认配置也已经切换到这份干净单机视图。

## 8. 当前工程层面的可靠启动方式

为了避免旧进程残留导致两套 `/drone_0` 仿真互相串话，当前新增了：

- `tools/run_adaptive_rviz_demo.sh`

它会先清掉旧的：

- `single_run_in_sim_fusion.launch.py`
- `single_run_in_sim.launch.py`
- `ego_planner_node`
- `traj_server`
- `rviz2`

然后只启动一套干净的自适应融合仿真，再打开 `drone0_clean.rviz`。

## 9. Headless 评测链路

当前 headless 评测由：

- `tools/compare_fusion.sh`
- `src/planner/plan_manage/scripts/fusion_benefit_report.py`
- `tools/sim_flight_stats_report.py`

共同构成。

已修复的问题：

- `fusion_benefit_report.py` 重复 `rclpy.shutdown()` 报错
- `sim_flight_stats_report.py` 的 `replan_count` 错误基线和提前收尾问题

当前 `replan_count` 的统计依据是最终的：

- `launch.log`

而不是不稳定的 ROS 内部日志文件。

## 10. 结论

从代码结构看，当前仓库里的“多模态融合”是：

- 几何层融合
- 体素证据融合
- 三参数在线自适应
- 感知-规划闭环反馈修正

它增强的是 planner 的局部地图输入质量，而不是直接改 planner 优化器本身。

当前默认真正在线自适应的参数有三个：

- `min_probability`
- `min_hits`
- `dual_bonus`

这也是当前文档、仿真链路和 headless 评测里都应统一遵守的工程事实。
