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

    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='false',
        description='Enable teleop_twist_joy joystick control'
    )

    capybara_bringup_share = FindPackageShare('capybara_bringup')

    # Base robot launch (controllers, ZED, LIDAR, robot_state_publisher)
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
            'launch_lidar': 'true',
            'use_joystick': LaunchConfiguration('use_joystick'),
        }.items()
    )

    # Foxglove bridge for remote visualization
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

    # SLAM Toolbox for mapping and localization
    # Now uses real LIDAR /scan directly instead of depth-to-laser conversion
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            PathJoinSubstitution([
                capybara_bringup_share,
                'config',
                'slam_toolbox.yaml'
            ])
        ],
        output='screen'
    )

    return LaunchDescription([
        use_mock_hardware_arg,
        foxglove_port_arg,
        use_joystick_arg,
        capybara_launch,
        foxglove_bridge,
        slam_toolbox,
    ])
