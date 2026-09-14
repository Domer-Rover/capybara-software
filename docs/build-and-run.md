# Build and Run

## Build

```bash
cd ~/domerrover/capybara-software
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch capybara_bringup capybara_foxglove.launch.py      # manual driving + Foxglove
ros2 launch capybara_bringup capybara_nav2_simple.launch.py   # Nav2, odom frame, no map
ros2 launch capybara_bringup capybara_slam.launch.py          # build a map
ros2 launch capybara_bringup capybara_nav2_slam.launch.py     # Nav2 on saved map
```

Common args: `use_mock_hardware:=true`, `launch_zed:=false`, `use_joystick:=true`

## Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

## Checks

```bash
ros2 control list_controllers
ros2 topic hz /zed/zed_node/odom
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Troubleshooting

- Permission denied on a serial port or ZED: `id -nG` must include `dialout`, `video`, `zed`. Log out and back in after groups change.
- Port busy: another launch is running. `sudo fuser -v /dev/ttyUSB0` (admin).
- Motors don't move: check you didn't pass `use_mock_hardware:=true`, and `ros2 control list_controllers` shows both controllers active.
