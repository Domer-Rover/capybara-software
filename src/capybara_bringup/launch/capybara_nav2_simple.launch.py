#!/usr/bin/env python3
"""Nav2 odom-only navigation with STL-19P LIDAR obstacle detection.

Navigates purely in the odom frame using ZED visual odometry.
No SLAM map, no AMCL localization. Send goals relative to the odom origin.
Requires the STL-19P LIDAR to be physically connected (/dev/ttyUSB0).

For LIDAR-free obstacle detection using ZED depth, use:
  capybara_nav2_zed.launch.py  (uses pointcloud_to_laserscan instead)

Usage:
  ros2 launch capybara_bringup capybara_nav2_simple.launch.py use_mock_hardware:=false

Send a goal (e.g. 1.5m / ~5ft forward) in Foxglove on /goal_pose (frame: odom), or:
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

    # STL-19P LIDAR — provides /scan for nav2 costmap obstacle detection
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 38400,
            'frame_id': 'laser_frame',
            'scan_mode': 'Standard',
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
            'camera_frame': 'zed_left_camera_frame_optical',
        }],
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        foxglove_port_arg,
        # Robot base (controllers + ZED)
        capybara_launch,
        foxglove_bridge,
        # LIDAR for obstacle detection
        lidar_node,
        # Navigation (odom-only, no map/AMCL)
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        # ArUco detection
        aruco_detector,
    ])
