# 多模态融合链路

## 当前三模态

当前项目中的三类几何感知输入是：

1. Depth camera cloud
   - topic: `/drone_0_pcl_render_node/cloud`
   - 特点：近场密集，几何细节好，远距离可靠性下降。
2. LiDAR cloud
   - topic: `/drone_0_lidar/points`
   - 特点：中远距离测距稳定，点云比 depth 稀疏。
3. mmWave radar cloud
   - topic: `/drone_0_radar/points`
   - 特点：动态目标和部分恶劣条件下更鲁棒，但角分辨率和几何细节弱，噪声更明显。

融合输出：

```text
/drone_0_fusion/fused_cloud
```

指标输出：

```text
/drone_0_fusion/ds_metrics
```

## 为什么 radar 不加入 ApproximateTimeSynchronizer

Depth 和 LiDAR 当前通过 ApproximateTimeSynchronizer 同步。Radar 使用最新帧缓存：

```python
self.latest_radar_points
self.last_radar_cloud_wall
```

原因是三路同步更容易卡住。仿真里 depth/LiDAR 时间戳可能存在特殊处理，例如 `force_zero_stamp`；如果 radar 也进入同步器，一旦三者时间戳不满足 slop，就会导致 fusion callback 不触发。

因此 radar 的工程策略是：

```text
depth + lidar 同步触发 fusion
radar 取最近一帧
如果 radar 超时，则本帧不使用 radar
```

## Radar 输入处理

当前 fusion 节点中 radar 参数包括：

- `radar_cloud_topic`
- `radar_cloud_timeout_sec`
- `radar_max_range`
- `radar_growth`
- `radar_dynamic_bonus`
- `radar_requires_geometry_support`

关键逻辑：

```python
def radar_cloud_callback(self, msg):
    points = self.cloud_to_xyz(msg)
    self.latest_radar_points = points
    self.last_radar_cloud_wall = time.time()
```

```python
def fresh_radar_points(self):
    if radar 超时:
        return empty
    return filter_points(latest_radar_points, range_limit=radar_max_range)
```

## 每个模态的概率模型

Depth：

```text
p_depth = 0.25 + 0.55 * exp(-range / depth_decay) + near_bonus
```

含义：近距离更可信，随距离衰减。

LiDAR：

```text
p_lidar = 0.35 + 0.45 * (1 - exp(-range / lidar_growth)) + far_bonus
```

含义：中远距离更可信，远场加权更高。

Radar：

```text
p_radar = 0.38 + 0.34 * (1 - exp(-range / radar_growth)) + far_bonus + radar_dynamic_bonus
```

含义：给 radar 一个偏保守的中等占据概率，在中远距离略增强。`radar_dynamic_bonus` 用于表达 radar 对动态目标的潜在优势。

## Voxel Evidence

每个 voxel 保存：

```text
logit_sum
hits
seen_depth
seen_lidar
seen_radar
depth_occ/free/unknown_mass
lidar_occ/free/unknown_mass
radar_occ/free/unknown_mass
```

核心逻辑：

```text
点云 -> voxel key -> modality state -> voxel evidence -> probability -> fused points
```

## Radar Geometry Support Gate

参数：

```text
radar_requires_geometry_support=True
```

含义：默认不允许 radar-only voxel 直接发布为融合障碍。只有 depth 或 LiDAR 也看到该 voxel 时，radar 证据才帮助增强占据概率。

这样做的原因：

- mmWave radar 几何角分辨率较弱。
- radar-only 点可能包含虚警和噪声。
- EGO-Planner 对障碍输入敏感，虚警会导致轨迹绕行或抖动。

如果未来要测试 radar-only 动态障碍召回能力，可以临时设为 False，但必须同时观察 false positive、轨迹抖动和 collision risk。

## 当前已验证

用户在仿真中验证：

```text
ros2 topic list | grep radar
-> /drone_0_radar/points
```

```text
ros2 topic echo --once /drone_0_radar/points --field width
-> 2354
```

```text
ros2 topic echo --once /drone_0_fusion/fused_cloud --field width
-> 12199
```

结论：

```text
radar 发布端已接通
fusion 输出非空
三模态链路具备继续做贡献分析的条件
```

## Radar 贡献统计

fusion debug log 中已经加入：

- `radar_points`
- `radar_only_voxels`
- `depth_radar_voxels`
- `lidar_radar_voxels`
- `tri_modal_voxels`
- `multi_modal_voxels`

这些指标用于回答：

```text
radar 只是发布了，还是确实增强了融合结果？
```

最近一次验证日志：

```text
fusion frame=40 depth_pts=5135 lidar_pts=6832 radar_pts=2276
fused_voxels=10527
depth_only=3524 lidar_only=5266 radar_only=0
depth_lidar=1244 depth_radar=264 lidar_radar=126 tri_modal=103
multi=1737 dual=1634 retention=0.856
```

结论：

```text
radar 不只是发布了点云，而是实际参与了融合 voxel 统计。
depth_radar、lidar_radar、tri_modal 均非 0，说明 radar 与 depth/LiDAR 在空间上存在有效重合。
radar_only=0 符合当前 radar_requires_geometry_support=True 的预期。
```

如果 `depth_radar_voxels`、`lidar_radar_voxels` 或 `tri_modal_voxels` 长期接近 0，说明 radar 与其他模态空间重合较低，需要检查 frame、FOV、range、voxel_size 或时间同步。

下一步需要把同样字段写入 `/drone_0_fusion/ds_metrics`，方便实验脚本和闭环反馈节点记录 radar 贡献。
