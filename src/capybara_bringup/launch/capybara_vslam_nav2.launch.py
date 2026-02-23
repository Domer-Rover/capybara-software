#!/usr/bin/env python3
"""VSLAM + Nav2 using ZED2i RGB-D input into RTAB-Map.

Architecture:
    ZED SDK  →  VIO odometry (odom→base_footprint TF)
                RGB image + depth map
    RTAB-Map →  visual loop closure + 2D occupancy grid (/map) + map→odom TF
    Nav2     →  global path planning on /map, local obstacle avoidance on /scan
    pointcloud_to_laserscan → /scan for Nav2 costmap obstacle detection

This replaces both capybara_slam_zed.launch.py and capybara_nav2_zed.launch.py.

Prerequisites:
    sudo apt install ros-humble-rtabmap-ros

Usage:
    ros2 launch capybara_bringup capybara_vslam_nav2.launch.py
    ros2 launch capybara_bringup capybara_vslam_nav2.launch.py use_mock_hardware:=true
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
        description='Use mock hardware for testing without physical robot'
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket port'
    )

    capybara_bringup_share = FindPackageShare('capybara_bringup')
    nav2_bringup_share = FindPackageShare('nav2_bringup')

    # --- Base robot: controllers, ZED camera, robot_state_publisher ---
    capybara_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([capybara_bringup_share, 'launch', 'capybara.launch.xml'])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_rviz':  'false',
            'launch_zed':   'true',
            'launch_lidar': 'false',
        }.items()
    )

    # --- RTAB-Map VSLAM ---
    # Subscribes to ZED RGB + depth + VIO odom.
    # Publishes: /map (2D occupancy grid for Nav2), map→odom TF (loop closure corrections).
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            PathJoinSubstitution([capybara_bringup_share, 'config', 'rtabmap.yaml']),
        ],
        remappings=[
            # RGB-D inputs from ZED SDK
            ('rgb/image',       '/zed/zed_node/rgb/image_rect_color'),
            ('depth/image',     '/zed/zed_node/depth/depth_registered'),
            ('rgb/camera_info', '/zed/zed_node/rgb/camera_info'),
            # Odometry: use ZED VIO directly — smooth, IMU-fused, never jumps
            ('odom',            '/zed/zed_node/odom'),
        ],
        arguments=['--delete_db_on_start'],  # fresh map on every launch (remove for persistent maps)
    )

    # --- ZED point cloud → 2D LaserScan for Nav2 costmap obstacle detection ---
    # RTAB-Map handles map building; this LaserScan is purely for Nav2 local+global costmaps.
    # Height filter: 5–60cm captures rover-height obstacles, ignores ground plane.
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
            ('scan',     '/scan'),
        ],
        parameters=[{
            'target_frame':        'base_footprint',
            'transform_tolerance': 0.5,  # wide tolerance for ZED bursty point cloud output
            'min_height':          0.05,
            'max_height':          0.60,
            'angle_min':          -3.14159,
            'angle_max':           3.14159,
            'angle_increment':     0.00873,
            'scan_time':           0.1,
            'range_min':           0.3,
            'range_max':           5.0,
            'use_inf':             True,
            'inf_epsilon':         1.0,
        }],
    )

    # --- Nav2 (map-based) ---
    # Uses /map from RTAB-Map for global planning.
    # Uses /scan from pointcloud_to_laserscan for costmap obstacle detection.
    # Does NOT launch map_server or AMCL — RTAB-Map provides map→odom TF directly.
    nav2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time':  'false',
            'params_file':   PathJoinSubstitution([
                capybara_bringup_share, 'config', 'nav2_vslam_params.yaml'
            ]),
        }.items()
    )

    # --- Foxglove bridge for visualization ---
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[
            PathJoinSubstitution([capybara_bringup_share, 'config', 'foxglove_bridge.yaml']),
            {'port': LaunchConfiguration('foxglove_port')},
        ],
        output='screen'
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        foxglove_port_arg,
        capybara_launch,
        rtabmap,
        pointcloud_to_laserscan,
        nav2,
        foxglove_bridge,
    ])
