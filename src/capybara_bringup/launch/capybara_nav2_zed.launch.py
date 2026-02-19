#!/usr/bin/env python3
"""Nav2 odom-only navigation using ZED point cloud for obstacle detection.

Identical to capybara_nav2_simple.launch.py but replaces the STL-19P LIDAR
with pointcloud_to_laserscan — converts the ZED2i depth point cloud to a
2D /scan LaserScan that nav2 costmaps can consume directly.

Use this when the LIDAR is not available or not connected.
For LIDAR-based obstacle detection use:
  capybara_nav2_simple.launch.py

Requires:
  sudo apt install ros-humble-pointcloud-to-laserscan

Usage:
  ros2 launch capybara_bringup capybara_nav2_zed.launch.py use_mock_hardware:=false

Send a goal in Foxglove on /goal_pose (frame: odom), or:
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'odom'}, pose: {position: {x: 1.5}, orientation: {w: 1.0}}}"
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

    nav2_params = PathJoinSubstitution([
        capybara_bringup_share, 'config', 'nav2_odom_only_params.yaml'
    ])

    # Base robot launch (controllers, ZED, robot_state_publisher)
    capybara_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                capybara_bringup_share, 'launch', 'capybara.launch.xml'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_rviz': 'false',
            'launch_zed': 'true',
        }.items()
    )

    # Foxglove bridge
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

    # ZED point cloud → /scan conversion for nav2 obstacle detection.
    # Slices the 3D point cloud into a horizontal band (5–60 cm above ground)
    # and projects it to a 2D LaserScan on /scan.
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
            'transform_tolerance': 0.01,
            'min_height':          0.05,    # ignore ground plane (< 5 cm)
            'max_height':          0.60,    # capture obstacles up to 60 cm (low objects in grass)
            'angle_min':          -3.14159, # full 180° horizontal sweep from ZED
            'angle_max':           3.14159,
            'angle_increment':     0.00873, # ~0.5° resolution
            'scan_time':           0.033,   # matches 30fps grab rate
            'range_min':           0.3,     # ZED2i min reliable depth
            'range_max':           5.0,     # practical outdoor obstacle range
            'use_inf':             True,
            'inf_epsilon':         1.0,
        }],
        output='screen'
    )

    # --- Nav2 (odom-only, no map/AMCL) ---

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
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
        # Robot base (controllers + ZED)
        capybara_launch,
        foxglove_bridge,
        # ZED depth → /scan for obstacle detection (no LIDAR needed)
        pointcloud_to_laserscan,
        # Navigation (odom-only, no map/AMCL)
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        # ArUco detection
        aruco_detector,
    ])
