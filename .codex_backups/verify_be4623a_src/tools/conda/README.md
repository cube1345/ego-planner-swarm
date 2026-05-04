# Conda Fusion Runtime

This repository is ROS 2 Humble based, so the Conda environment should use Python 3.10.
ROS 2 Python packages (`rclpy`, `message_filters`, `sensor_msgs_py`) still come from the system ROS installation after you source `/opt/ros/humble/setup.bash`.

## 1. Create the environment

```bash
conda env create -f tools/conda/environment.yml
conda activate ego-fusion-py310  # or reuse your existing `rospy` env if it is Python 3.10
```

## 2. Source ROS 2 and the workspace

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 3. Run the fusion node

```bash
python tools/conda/ros2_lidar_depth_fusion_node.py --ros-args \
  -p depth_cloud_topic:=/drone_0_pcl_render_node/cloud \
  -p lidar_cloud_topic:=/lidar/points \
  -p odom_topic:=/drone_0_visual_slam/odom \
  -p output_topic:=/drone_0_fusion/fused_cloud \
  -p output_frame:=world
```

## 4. Feed the fused cloud into the planner

Because the existing launch files already parameterize `cloud_topic`, launch the planner with the fused output topic:

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
  use_dynamic:=False \
  cloud_topic:=fusion/fused_cloud
```

If your LiDAR cloud is not already in `world`, transform it before fusion or publish it in the same frame as the depth cloud.
