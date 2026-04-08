#!/usr/bin/env python3
"""SLAM mapping using ZED2i depth instead of the STL-19P LIDAR.

Converts the ZED point cloud to a 2D /scan LaserScan via pointcloud_to_laserscan,
then feeds it directly into slam_toolbox (same pipeline as the LIDAR SLAM launch).
Use this when the LIDAR mount is not yet built or the sensor is unavailable.

VSLAM note:
    The ZED SDK already runs visual odometry internally and publishes to
    /zed/zed_node/odom.  If you want full graph-based VSLAM with loop closure
    (no scan feed at all), install rtabmap_ros and use its stereo_odometry +
    rtabmap nodes instead of slam_toolbox.  That requires:
        sudo apt install ros-humble-rtabmap-ros
    For now this launch reuses slam_toolbox with the ZED depth scan so you
    stay on the same map format and tooling as the LIDAR SLAM launch.

Requires:
    sudo apt install ros-humble-pointcloud-to-laserscan

Usage:
    ros2 launch capybara_bringup capybara_slam_zed.launch.py
    ros2 launch capybara_bringup capybara_slam_zed.launch.py use_mock_hardware:=true
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

    # Base robot (controllers, ZED, robot_state_publisher — no LIDAR)
    capybara_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                capybara_bringup_share, 'launch', 'capybara.launch.xml'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_rviz':   'false',
            'launch_zed':    'true',
            'launch_lidar':  'false',   # no physical LIDAR needed
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

    # ZED point cloud → /scan (same settings as capybara_nav2_zed.launch.py).
    # Slices the 3D depth cloud into a horizontal band so slam_toolbox gets a
    # 2D laser scan it can use for map building.  The band (5–60 cm) captures
    # objects at rover-wheel height while ignoring the ground plane.
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
            'transform_tolerance': 0.1,
            'min_height':          0.05,    # ignore ground plane (< 5 cm)
            'max_height':          0.60,    # capture obstacles up to 60 cm
            'angle_min':          -1.5708,  # ZED FOV is ~110° (not full 360°)
            'angle_max':           1.5708,
            'angle_increment':     0.008720, # span/increment = 360.33 → ceil=361 (pointcloud) and round+1=361 (slam_toolbox)
            'scan_time':           0.033,   # 30 fps
            'range_min':           0.3,     # ZED2i min reliable depth
            'range_max':           5.0,     # practical outdoor obstacle range
            'use_inf':             True,
            'inf_epsilon':         1.0,
        }],
        output='screen'
    )

    # SLAM Toolbox — uses slam_toolbox_zed.yaml (ZED-optimised: 5 m range,
    # 0.2 m travel threshold).  Loop closure is less reliable than with LIDAR
    # because the ZED FOV is ~110° (vs 360° for the STL-19P), so map quality
    # degrades in long corridors or large open areas.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            PathJoinSubstitution([
                capybara_bringup_share, 'config', 'slam_toolbox_zed.yaml'
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
            'marker_size':    0.15,
            'dictionary_id':  10,
            'camera_frame':   'zed_left_camera_frame_optical',
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
