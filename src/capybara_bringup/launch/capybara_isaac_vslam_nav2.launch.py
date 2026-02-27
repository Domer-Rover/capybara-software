#!/usr/bin/env python3
"""VSLAM + Nav2 using Isaac ROS cuVSLAM (NVIDIA cuVSLAM visual odometry).

Architecture:
    ZED SDK  →  stereo images (left + right rectified) + IMU
    cuVSLAM  →  GPU-accelerated visual odometry (odom→base_footprint TF)
    Nav2     →  global path planning on /map (from slam_toolbox), local obstacle avoidance on /scan
    pointcloud_to_laserscan → /scan for Nav2 costmap obstacle detection
    slam_toolbox → builds a 2D occupancy grid from the ZED point cloud (via laserscan)

NOTE: Isaac ROS cuVSLAM is pure odometry — it does NOT build a persistent map or perform
loop closure. For map building, slam_toolbox is launched alongside to consume the cuVSLAM
odometry and produce a /map occupancy grid for Nav2.

Prerequisites:
    # Add Isaac ROS apt repo (Isaac ROS 3.x for Humble):
    wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
    grep -qxF 'deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0' \
        /etc/apt/sources.list || echo 'deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0' \
        | sudo tee -a /etc/apt/sources.list
    sudo apt update && sudo apt install ros-humble-isaac-ros-visual-slam

    # Verify installation:
    ros2 pkg list | grep isaac_ros_visual_slam

Known issues on Jetson Orin Nano:
    - "Failed to initialize CUVSLAM tracker: 4" with CUDA 12.2 — if this occurs,
      try setting CUDA_VISIBLE_DEVICES=0 or fall back to capybara_vslam_nav2.launch.py
    - Tracking loss at high speeds — keep velocity under ~0.3 m/s for stable tracking

Usage:
    ros2 launch capybara_bringup capybara_isaac_vslam_nav2.launch.py
    ros2 launch capybara_bringup capybara_isaac_vslam_nav2.launch.py use_mock_hardware:=true
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

    # --- Isaac ROS cuVSLAM ---
    # Consumes ZED stereo rectified images + IMU.
    # Publishes: odom→zed_camera_link TF (GPU-accelerated visual odometry).
    # cuVSLAM does NOT publish a map — slam_toolbox handles map building below.
    #
    # Frame notes:
    #   - input_base_frame: the frame cuVSLAM anchors odometry to (zed_camera_link,
    #     because that's the ZED SDK's positional tracking root after SDK 5.2)
    #   - Nav2 reaches base_footprint via the static URDF joint: zed_camera_link→base_footprint
    isaac_vslam = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam_node',
        output='screen',
        parameters=[{
            'enable_image_denoising':       False,   # ZED SDK already rectifies images
            'rectified_images':             True,
            'enable_slam_visualization':    True,
            'enable_observations_view':     True,
            'enable_landmarks_view':        True,
            'input_base_frame':             'zed_camera_link',
            'input_left_camera_frame':      'zed_left_camera_frame_optical',
            'input_right_camera_frame':     'zed_right_camera_frame_optical',
            'input_imu_frame':              'zed_imu_link',
            'enable_imu_fusion':            True,
            'gravitational_force':          [0.0, 0.0, -9.81],
            # Publish odom→zed_camera_link TF (matches what ZED SDK 5.2 uses as its TF root)
            'map_frame':                    'map',
            'odom_frame':                   'odom',
            'base_frame':                   'zed_camera_link',
        }],
        remappings=[
            ('stereo_camera/left/image',        '/zed/zed_node/left/image_rect_color'),
            ('stereo_camera/right/image',       '/zed/zed_node/right/image_rect_color'),
            ('stereo_camera/left/camera_info',  '/zed/zed_node/left/camera_info'),
            ('stereo_camera/right/camera_info', '/zed/zed_node/right/camera_info'),
            ('visual_slam/imu',                 '/zed/zed_node/imu/data'),
        ],
    )

    # --- SLAM Toolbox (map building from ZED point cloud) ---
    # cuVSLAM provides odometry; slam_toolbox converts that + point cloud scan into a /map.
    # Uses the ZED-tuned slam_toolbox config (5m range, 110° FOV).
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([capybara_bringup_share, 'config', 'slam_toolbox_zed.yaml']),
            {'use_sim_time': False},
        ],
        remappings=[
            ('/odom', '/visual_slam/tracking/odometry'),  # cuVSLAM odometry feed to slam_toolbox
        ],
    )

    # --- ZED point cloud → 2D LaserScan for Nav2 costmap + slam_toolbox ---
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
            'transform_tolerance': 0.5,
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
    # Uses /map from slam_toolbox for global planning.
    # Uses /scan from pointcloud_to_laserscan for costmap obstacle detection.
    # cuVSLAM publishes odom→zed_camera_link TF (Nav2 uses base_footprint via static URDF joint).
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
        isaac_vslam,
        slam_toolbox,
        pointcloud_to_laserscan,
        nav2,
        foxglove_bridge,
    ])
