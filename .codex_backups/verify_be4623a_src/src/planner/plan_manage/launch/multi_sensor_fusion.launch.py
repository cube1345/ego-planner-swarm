from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_topic = LaunchConfiguration('depth_topic')
    lidar_topic = LaunchConfiguration('lidar_topic')
    output_topic = LaunchConfiguration('output_topic')
    target_frame = LaunchConfiguration('target_frame')
    depth_input_is_image = LaunchConfiguration('depth_input_is_image')

    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/drone_0_pcl_render_node/cloud'),
        DeclareLaunchArgument('lidar_topic', default_value='/drone_0_lidar/points'),
        DeclareLaunchArgument('output_topic', default_value='/fusion_cloud'),
        DeclareLaunchArgument('target_frame', default_value='world'),
        DeclareLaunchArgument('depth_input_is_image', default_value='false'),

        # TODO: If depth_input_is_image=true, update camera intrinsics to calibrated values.
        Node(
            package='ego_planner',
            executable='multi_sensor_fusion_node',
            name='multi_sensor_fusion_node',
            output='screen',
            parameters=[{
                'depth_input_is_image': depth_input_is_image,
                'depth_topic': depth_topic,
                'lidar_topic': lidar_topic,
                'output_topic': output_topic,
                'target_frame': target_frame,
                'sync_queue_size': 20,
                'sync_slop_sec': 0.08,
                'tf_lookup_timeout_sec': 0.02,
                'voxel_leaf_size': 0.10,
                'enable_sor': True,
                'sor_mean_k': 12,
                'sor_stddev_mul': 1.0,
                'warn_min_hz': 10.0,
            }]
        )
    ])
