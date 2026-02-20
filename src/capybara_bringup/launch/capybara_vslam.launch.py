#!/usr/bin/env python3
"""VSLAM launch — no physical LIDAR required.

Uses ZED depth point cloud converted to a virtual laser scan via
pointcloud_to_laserscan, fed into SLAM Toolbox for mapping/localization.

Usage:
  ros2 launch capybara_bringup capybara_vslam.launch.py use_mock_hardware:=false
"""

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

    capybara_bringup_share = FindPackageShare('capybara_bringup')

    # Base robot launch — no LIDAR, ZED provides all sensing
    capybara_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                capybara_bringup_share,
                'launch',
                'capybara.launch.xml'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_rviz': 'false',
            'launch_zed': 'true',
            'launch_lidar': 'false',
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
            'send_buffer_limit': 41943040,
        }],
        output='screen'
    )

    # Convert ZED point cloud → virtual laser scan for SLAM Toolbox
    # ZED 2i has ~110° horizontal FOV, so scan covers -55° to +55°
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.1,
            'min_height': 0.1,       # ignore floor
            'max_height': 1.5,       # ignore ceiling/sky
            'angle_min': -1.0,       # ~57° left — within ZED FOV
            'angle_max': 1.0,        # ~57° right — within ZED FOV
            'angle_increment': 0.00872665,  # 0.5° resolution
            'scan_time': 0.0667,     # matches ZED 15 Hz pub rate
            'range_min': 0.3,        # ZED minimum reliable depth
            'range_max': 10.0,       # ZED max reliable depth
            'use_inf': True,         # report out-of-range as inf (not NaN)
        }],
        output='screen'
    )

    # SLAM Toolbox — uses virtual /scan from pointcloud_to_laserscan
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            PathJoinSubstitution([
                capybara_bringup_share,
                'config',
                'slam_toolbox_vslam.yaml'
            ])
        ],
        output='screen'
    )

    # ArUco marker detection (OpenCV, DICT_6X6_250)
    aruco_detector = Node(
        package='capybara_bringup',
        executable='aruco_detector.py',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'marker_size': 0.15,
            'dictionary_id': 10,
            'camera_frame': 'zed_left_camera_optical_frame',
        }],
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        foxglove_port_arg,
        capybara_launch,
        foxglove_bridge,
        pointcloud_to_laserscan,
        slam_toolbox,
        aruco_detector,
    ])
