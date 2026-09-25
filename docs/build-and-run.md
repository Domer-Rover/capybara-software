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

## Outdoor Nav2 test

```bash
ros2 launch capybara_bringup capybara_nav2_simple.launch.py
```

Joystick override is on by default: hold the deadman and you drive, release and
Nav2 takes back over 0.5 s later (`twist_mux`, joystick priority 100 vs Nav2 10).

Check in Foxglove first: `/scan` shows what is *in front* and the antennas are
gone, and `/zed/zed_node/odom` is publishing. Then send a 10 m goal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 10.0}, orientation: {w: 1.0}}}"
```

Record it:

```bash
ros2 bag record -o ~/bags/nav2_$(date +%F_%H%M) \
  /scan /zed/zed_node/odom /zed/zed_node/pose /tf /tf_static \
  /nav_vel /joy_vel /diff_drive_controller/cmd_vel_unstamped /plan /goal_pose
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
- Port busy: another launch is running. `sudo fuser -v /dev/rover_roboclaw` (admin).
- Motors don't move: check you didn't pass `use_mock_hardware:=true`, and `ros2 control list_controllers` shows both controllers active.
