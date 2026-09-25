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
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped -p speed:=0.2 -p turn:=0.5
```

## Field test: GPS + joystick + recording

```bash
ros2 launch capybara_bringup capybara_foxglove.launch.py launch_gps:=true use_joystick:=true
```

Record a bag so drift can be measured afterwards (run from a writable directory):

```bash
ros2 bag record -o ~/bags/$(date +%F_%H%M) \
  /zed/zed_node/odom /zed/zed_node/pose /zed/zed_node/imu/data \
  /fix /tf /tf_static /joint_states /diff_drive_controller/cmd_vel_unstamped
```

Quick sanity checks before driving off:

```bash
ros2 topic echo /fix --once          # GPS has a fix (status.status >= 0)
ros2 topic echo /joy --once          # controller is connected
ros2 topic hz /zed/zed_node/odom     # VIO is publishing
```

VIO drift test: drive a closed loop back to the exact start point, then compare the
first and last `/zed/zed_node/pose` in the bag.

## Checks

```bash
ros2 control list_controllers
ros2 topic hz /zed/zed_node/odom
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Troubleshooting

- Permission denied on a serial port or ZED: `id -nG` must include `dialout`, `video`, `zed`. Log out and back in after groups change.
- Port busy: another launch is running. `sudo fuser -v /dev/rover_roboclaw` (admin).
- Motors don't move: check you didn't pass `use_mock_hardware:=true`, and `ros2 control list_controllers` shows both controllers active.
