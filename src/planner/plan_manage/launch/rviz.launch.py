from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value='default.rviz',
            description='RViz config file located in ego_planner/launch',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', PathJoinSubstitution([
                FindPackageShare('ego_planner'), 'launch', rviz_config,
            ])],
        ),
    ])
