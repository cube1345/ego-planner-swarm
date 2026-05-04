import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def launch_setup(context, *args, **kwargs):
    obj_num = LaunchConfiguration('obj_num').perform(context)
    drone_id = LaunchConfiguration('drone_id').perform(context)
    map_size_x = LaunchConfiguration('map_size_x').perform(context)
    map_size_y = LaunchConfiguration('map_size_y').perform(context)
    map_size_z = LaunchConfiguration('map_size_z').perform(context)
    init_x = LaunchConfiguration('init_x').perform(context)
    init_y = LaunchConfiguration('init_y').perform(context)
    init_z = LaunchConfiguration('init_z').perform(context)
    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    point_num = LaunchConfiguration('point_num').perform(context)
    point0_x = LaunchConfiguration('point0_x').perform(context)
    point0_y = LaunchConfiguration('point0_y').perform(context)
    point0_z = LaunchConfiguration('point0_z').perform(context)
    use_mockamap = LaunchConfiguration('use_mockamap')
    use_dynamic = LaunchConfiguration('use_dynamic')
    fusion_python_executable = LaunchConfiguration('fusion_python_executable').perform(context)
    use_fusion = LaunchConfiguration('use_fusion')
    use_fusion_value = LaunchConfiguration('use_fusion').perform(context).lower() in ('1', 'true', 'yes')
    adaptive_min_probability_enable = LaunchConfiguration('adaptive_min_probability_enable').perform(context)
    min_probability = LaunchConfiguration('min_probability').perform(context)
    min_hits = LaunchConfiguration('min_hits').perform(context)
    adaptive_min_probability_min = LaunchConfiguration('adaptive_min_probability_min').perform(context)
    adaptive_min_probability_max = LaunchConfiguration('adaptive_min_probability_max').perform(context)
    adaptive_min_probability_step = LaunchConfiguration('adaptive_min_probability_step').perform(context)
    adaptive_min_hits_enable = LaunchConfiguration('adaptive_min_hits_enable').perform(context)
    near_field_radius = LaunchConfiguration('near_field_radius').perform(context)
    lidar_growth = LaunchConfiguration('lidar_growth').perform(context)
    adaptive_min_hits_min = LaunchConfiguration('adaptive_min_hits_min').perform(context)
    adaptive_min_hits_max = LaunchConfiguration('adaptive_min_hits_max').perform(context)
    adaptive_target_retention = LaunchConfiguration('adaptive_target_retention').perform(context)
    adaptive_retention_band = LaunchConfiguration('adaptive_retention_band').perform(context)
    adaptive_eval_range = LaunchConfiguration('adaptive_eval_range').perform(context)
    adaptive_min_gt_voxels = LaunchConfiguration('adaptive_min_gt_voxels').perform(context)
    adaptive_score_alpha = LaunchConfiguration('adaptive_score_alpha').perform(context)

    pkg_share = get_package_share_directory('ego_planner')
    pkg_prefix = get_package_prefix('ego_planner')
    fusion_script = os.path.join(pkg_prefix, 'lib', 'ego_planner', 'ros2_lidar_depth_fusion_node.py')

    odom_topic_full = f'/drone_{drone_id}_{odom_topic}'
    lidar_topic_full = f'/drone_{drone_id}_lidar/points'
    fused_topic_full = f'/drone_{drone_id}_fusion/fused_cloud'
    cloud_topic = 'fusion/fused_cloud' if use_fusion_value else 'pcl_render_node/cloud'

    map_generator_node = Node(
        package='map_generator',
        executable='random_forest',
        name='random_forest',
        output='screen',
        parameters=[
            {'map/x_size': 26.0},
            {'map/y_size': 20.0},
            {'map/z_size': 3.0},
            {'map/resolution': 0.1},
            {'ObstacleShape/seed': 1.0},
            {'map/obs_num': 250},
            {'ObstacleShape/lower_rad': 0.5},
            {'ObstacleShape/upper_rad': 0.7},
            {'ObstacleShape/lower_hei': 0.0},
            {'ObstacleShape/upper_hei': 3.0},
            {'map/circle_num': 250},
            {'ObstacleShape/radius_l': 0.7},
            {'ObstacleShape/radius_h': 0.5},
            {'ObstacleShape/z_l': 0.7},
            {'ObstacleShape/z_h': 0.8},
            {'ObstacleShape/theta': 0.5},
            {'pub_rate': 1.0},
            {'min_distance': 0.8},
        ],
        condition=UnlessCondition(use_mockamap),
    )

    mockamap_node = Node(
        package='mockamap',
        executable='mockamap_node',
        name='mockamap_node',
        output='screen',
        remappings=[('/mock_map', '/map_generator/global_cloud')],
        parameters=[
            {'seed': 127},
            {'update_freq': 0.5},
            {'resolution': 0.1},
            {'x_length': int(float(map_size_x))},
            {'y_length': int(float(map_size_y))},
            {'z_length': int(float(map_size_z))},
            {'type': 1},
            {'complexity': 0.05},
            {'fill': 0.12},
            {'fractal': 1},
            {'attenuation': 0.1},
        ],
        condition=IfCondition(use_mockamap),
    )

    advanced_param_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'advanced_param.launch.py')),
        launch_arguments={
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'odometry_topic': odom_topic,
            'obj_num_set': obj_num,
            'camera_pose_topic': 'pcl_render_node/camera_pose',
            'depth_topic': 'pcl_render_node/depth',
            'cloud_topic': cloud_topic,
            'cx': str(321.04638671875),
            'cy': str(243.44969177246094),
            'fx': str(387.229248046875),
            'fy': str(387.229248046875),
            'max_vel': str(2.0),
            'max_acc': str(6.0),
            'planning_horizon': str(7.5),
            'use_distinctive_trajs': 'True',
            'flight_type': str(2),
            'point_num': point_num,
            'point0_x': point0_x,
            'point0_y': point0_y,
            'point0_z': point0_z,
            'point1_x': str(-15.0),
            'point1_y': str(0.0),
            'point1_z': str(1.0),
            'point2_x': str(15.0),
            'point2_y': str(0.0),
            'point2_z': str(1.0),
            'point3_x': str(-15.0),
            'point3_y': str(0.0),
            'point3_z': str(1.0),
            'point4_x': str(15.0),
            'point4_y': str(0.0),
            'point4_z': str(1.0),
        }.items(),
    )

    traj_server_node = Node(
        package='ego_planner',
        executable='traj_server',
        name=f'drone_{drone_id}_traj_server',
        output='screen',
        remappings=[
            ('position_cmd', f'drone_{drone_id}_planning/pos_cmd'),
            ('planning/bspline', f'drone_{drone_id}_planning/bspline'),
        ],
        parameters=[{'traj_server/time_forward': 1.0}],
    )

    simulator_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'simulator.launch.py')),
        launch_arguments={
            'use_dynamic': use_dynamic,
            'drone_id': drone_id,
            'map_size_x_': map_size_x,
            'map_size_y_': map_size_y,
            'map_size_z_': map_size_z,
            'init_x_': init_x,
            'init_y_': init_y,
            'init_z_': init_z,
            'odometry_topic': odom_topic,
        }.items(),
    )

    simulated_lidar_node = Node(
        package='ego_planner',
        executable='simulated_lidar_cloud.py',
        name=f'drone_{drone_id}_simulated_lidar',
        output='screen',
        parameters=[
            {'global_cloud_topic': '/map_generator/global_cloud'},
            {'odom_topic': odom_topic_full},
            {'lidar_points_topic': lidar_topic_full},
            {'frame_id': 'world'},
            {'publish_rate': 10.0},
            {'max_range': 8.0},
            {'horizontal_fov_deg': 240.0},
            {'vertical_min_deg': -18.0},
            {'vertical_max_deg': 18.0},
            {'voxel_size': 0.18},
            {'keep_ratio': 0.45},
            # pcl_render_node/cloud currently carries zero stamp in this sim path,
            # keep lidar stamp aligned so ApproximateTime synchronizer can trigger.
            {'force_zero_stamp': True},
        ],
        condition=IfCondition(use_fusion),
    )

    fusion_process = ExecuteProcess(
        cmd=[
            fusion_python_executable, fusion_script,
            '--ros-args',
            '-p', f'depth_cloud_topic:=/drone_{drone_id}_pcl_render_node/cloud',
            '-p', f'lidar_cloud_topic:={lidar_topic_full}',
            '-p', f'odom_topic:={odom_topic_full}',
            '-p', f'output_topic:={fused_topic_full}',
            '-p', 'output_frame:=world',
            '-p', 'publish_debug_stats_every:=20',
            '-p', f'near_field_radius:={near_field_radius}',
            '-p', f'lidar_growth:={lidar_growth}',
            '-p', 'max_range:=10.0',
            '-p', f'min_probability:={min_probability}',
            '-p', f'min_hits:={min_hits}',
            '-p', f'adaptive_min_probability_enable:={adaptive_min_probability_enable}',
            '-p', f'adaptive_min_probability_min:={adaptive_min_probability_min}',
            '-p', f'adaptive_min_probability_max:={adaptive_min_probability_max}',
            '-p', f'adaptive_min_probability_step:={adaptive_min_probability_step}',
            '-p', f'adaptive_min_hits_enable:={adaptive_min_hits_enable}',
            '-p', f'adaptive_min_hits_min:={adaptive_min_hits_min}',
            '-p', f'adaptive_min_hits_max:={adaptive_min_hits_max}',
            '-p', f'adaptive_target_retention:={adaptive_target_retention}',
            '-p', f'adaptive_retention_band:={adaptive_retention_band}',
            '-p', f'adaptive_eval_range:={adaptive_eval_range}',
            '-p', f'adaptive_min_gt_voxels:={adaptive_min_gt_voxels}',
            '-p', f'adaptive_score_alpha:={adaptive_score_alpha}',
        ],
        output='screen',
        condition=IfCondition(use_fusion),
    )

    return [
        map_generator_node,
        mockamap_node,
        advanced_param_include,
        traj_server_node,
        simulated_lidar_node,
        fusion_process,
        simulator_include,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('obj_num', default_value='10'),
        DeclareLaunchArgument('drone_id', default_value='0'),
        DeclareLaunchArgument('map_size_x', default_value='50.0'),
        DeclareLaunchArgument('map_size_y', default_value='25.0'),
        DeclareLaunchArgument('map_size_z', default_value='2.0'),
        DeclareLaunchArgument('init_x', default_value='-15.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='0.1'),
        DeclareLaunchArgument('odom_topic', default_value='visual_slam/odom'),
        DeclareLaunchArgument('point_num', default_value='4'),
        DeclareLaunchArgument('point0_x', default_value='15.0'),
        DeclareLaunchArgument('point0_y', default_value='0.0'),
        DeclareLaunchArgument('point0_z', default_value='1.0'),
        DeclareLaunchArgument('use_mockamap', default_value='False'),
        DeclareLaunchArgument('use_dynamic', default_value='False'),
        DeclareLaunchArgument('fusion_python_executable', default_value='/usr/bin/python3'),
        DeclareLaunchArgument('use_fusion', default_value='True'),
        DeclareLaunchArgument('near_field_radius', default_value='4.0'),
        DeclareLaunchArgument('lidar_growth', default_value='5.0'),
        DeclareLaunchArgument('min_probability', default_value='0.30'),
        DeclareLaunchArgument('min_hits', default_value='1'),
        DeclareLaunchArgument('adaptive_min_probability_enable', default_value='True'),
        DeclareLaunchArgument('adaptive_min_probability_min', default_value='0.20'),
        DeclareLaunchArgument('adaptive_min_probability_max', default_value='0.35'),
        DeclareLaunchArgument('adaptive_min_probability_step', default_value='0.02'),
        DeclareLaunchArgument('adaptive_min_hits_enable', default_value='True'),
        DeclareLaunchArgument('adaptive_min_hits_min', default_value='1'),
        DeclareLaunchArgument('adaptive_min_hits_max', default_value='3'),
        DeclareLaunchArgument('adaptive_target_retention', default_value='0.30'),
        DeclareLaunchArgument('adaptive_retention_band', default_value='0.05'),
        DeclareLaunchArgument('adaptive_eval_range', default_value='10.0'),
        DeclareLaunchArgument('adaptive_min_gt_voxels', default_value='40'),
        DeclareLaunchArgument('adaptive_score_alpha', default_value='0.35'),
        OpaqueFunction(function=launch_setup),
    ])
