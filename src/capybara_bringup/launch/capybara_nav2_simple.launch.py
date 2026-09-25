#!/usr/bin/env python3
"""Outdoor Nav2 test: odom-only navigation with LD19 obstacle avoidance.

Navigates in the odom frame using ZED visual odometry. No map, no AMCL, so
goals are relative to wherever the rover booted. The joystick overrides Nav2
at any time (hold the deadman); Nav2 resumes 0.5s after release.

  ros2 launch capybara_bringup capybara_nav2_simple.launch.py

Before driving, in Foxglove: check /scan shows the world in front (not behind)
and that the antennas are gone, and that /zed/zed_node/odom is publishing.

Send a 10 m straight-line goal:
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'odom'}, pose: {position: {x: 10.0}, orientation: {w: 1.0}}}"

Cancel:
  ros2 action send_goal /navigate_to_pose ... is not needed — release is via the
  joystick deadman, or Ctrl-C the launch and take over with the controller.
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

    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Joystick override (recommended outdoors)'
    )

    launch_gps_arg = DeclareLaunchArgument(
        'launch_gps',
        default_value='false',
        description='Launch the u-blox GPS node (publishes /fix)'
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
            'launch_lidar': 'true',
            'use_joystick': LaunchConfiguration('use_joystick'),
            'launch_gps': LaunchConfiguration('launch_gps'),
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

    # --- Nav2 (odom-only, no map/AMCL) ---

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', '/nav_vel')],
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
        remappings=[('cmd_vel', '/nav_vel')],
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
    

    return LaunchDescription([
        use_mock_hardware_arg,
        use_joystick_arg,
        launch_gps_arg,
        foxglove_port_arg,
        # Robot base (controllers + ZED + LIDAR)
        capybara_launch,
        foxglove_bridge,
        # Navigation (odom-only, no map/AMCL)
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        # Future object-detection feature:  
    ])
