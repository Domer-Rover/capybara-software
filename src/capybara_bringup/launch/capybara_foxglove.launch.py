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

    launch_zed_arg = DeclareLaunchArgument(
        'launch_zed',
        default_value='true',
        description='Launch ZED2i camera'
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket port'
    )

    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='false',
        description='Use teleop_twist_joy (true) or teleop_twist_keyboard (false)'
    )

    capybara_bringup_share = FindPackageShare('capybara_bringup')

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
            'launch_zed': LaunchConfiguration('launch_zed'),
            'use_joystick': LaunchConfiguration('use_joystick'),
        }.items()
    )

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
        launch_zed_arg,
        foxglove_port_arg,
        use_joystick_arg,
        capybara_launch,
        foxglove_bridge,
        aruco_detector,
    ])
