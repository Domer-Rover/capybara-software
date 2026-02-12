#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware for simulation'
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket port'
    )

    gerbil_bringup_share = FindPackageShare('gerbil_bringup')

    # Base robot launch (controllers, ZED, robot_state_publisher)
    gerbil_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                gerbil_bringup_share,
                'launch',
                'gerbil.launch.xml'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_rviz': 'false',
            'launch_imu': 'false',
            'launch_zed': 'true',
        }.items()
    )

    # Foxglove bridge for remote visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'address': '0.0.0.0',
            'num_threads': 4,
            'max_qos_depth': 10,
            'send_buffer_limit': 10000000,
        }],
        output='screen'
    )

    # --- CHANGED: Pointcloud to LaserScan (The "Thick Line" Fix) ---
    # This replaces depthimage_to_laserscan. 
    # It takes the full 3D cloud and flattens it, preventing "invisible table" crashes.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',   # Stabilizes the scan even if robot tilts
            'transform_tolerance': 0.01,
            'min_height': 0.1,             # Ignore floor (0.0)
            'max_height': 1.5,             # See tall obstacles
            'angle_min': -1.5708,          # -90 degrees
            'angle_max': 1.5708,           # +90 degrees
            'angle_increment': 0.0087,     # 0.5 degrees density
            'scan_time': 0.1,              # 10Hz
            'range_min': 0.45,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        # Remap ZED's 3D Cloud -> /scan
        remappings=[
            ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
            ('scan', '/scan')
        ],
        output='screen'
    )

    # SLAM Toolbox (Consumes the improved /scan from above)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            PathJoinSubstitution([
                gerbil_bringup_share,
                'config',
                'slam_toolbox.yaml'
            ])
        ],
        output='screen'
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        foxglove_port_arg,
        gerbil_launch,
        foxglove_bridge,
        pointcloud_to_laserscan, # The new node
        slam_toolbox,
    ])