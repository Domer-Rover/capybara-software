# Build and Run

## Build

```bash
cd ~/domerrover/capybara-software
colcon build --symlink-install
source install/setup.bash
```

First time on a machine (admin, installs the packages the launch files run):

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Launch

```bash
ros2 launch capybara_bringup capybara_foxglove.launch.py      # manual driving + Foxglove
ros2 launch capybara_bringup capybara_nav2_simple.launch.py   # Nav2, odom frame, no map
ros2 launch capybara_bringup capybara_slam.launch.py          # build a map
ros2 launch capybara_bringup capybara_nav2_slam.launch.py     # Nav2 on saved map
```

Common args: `use_mock_hardware:=true`, `launch_zed:=false`, `use_joystick:=true`,
`launch_gps:=true`

Speeds: the drive controller is capped at 0.7 m/s (rover can do ~1.0); Nav2
cruises at 0.4 m/s.

## Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/joy_vel -p speed:=0.2 -p turn:=0.5
```

`/joy_vel` goes through `twist_mux`, so keyboard and joystick both outrank Nav2.

## Field test: GPS + joystick + recording

```bash
ros2 launch capybara_bringup capybara_foxglove.launch.py launch_gps:=true use_joystick:=true
```

Record a bag so drift can be measured afterwards:

```bash
mkdir -p ~/bags
ros2 bag record -o ~/bags/$(date +%F_%H%M) \
  /zed/zed_node/odom /zed/zed_node/pose /zed/zed_node/imu/data \
  /fix /tf /tf_static /joint_states /diff_drive_controller/cmd_vel_unstamped
```

Quick sanity checks before driving off:

```bash
ros2 topic echo /fix --once          # GPS has a fix (status.status >= 0)
ros2 topic echo /scan --once         # LIDAR alive; check the arc in Foxglove too
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

Nav2 setup: `SmacPlanner2D` planner (no turning-radius constraint, unlike the
car-style Hybrid-A* it replaced), `RegulatedPurePursuitController`, obstacle +
inflation costmap layers off `/scan` only, and no spin recovery (the rover
stalls pivoting in place). Global costmap 40x40 m at 10 cm, local 6x6 m at 5 cm,
footprint 1.18 x 1.02 m, inflation 0.85 m.

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

## Running detached (survives losing WiFi)

A launch started over SSH dies when the SSH connection drops, and with the
RoboClaw serial timeout at 0 the rover keeps driving on its last command. Start
it inside `tmux` so it keeps running when SSH, Foxglove, or both go away — the
joystick still works because it runs on the rover.

```bash
tmux new -s rover          # start a named session
# ... source the workspace and run the launch as usual ...
# detach with: Ctrl-b then d
```

Reattach after reconnecting, from any SSH session:

```bash
tmux attach -t rover       # Ctrl-c inside stops the launch
tmux ls                    # list sessions
```

If tmux is missing: `sudo apt install tmux` (admin).

## Field checklist

1. `ls -l /dev/rover_*` — udev names exist
2. `python3 scripts/serial_timeout.py` — not `DISABLED`
3. Measure the rover footprint and compare with `nav2_odom_only_params.yaml`
4. Start the launch inside `tmux`
5. In Foxglove: `/scan` front arc correct and antennas gone, `/zed/zed_node/odom` publishing, `/fix` has a fix
6. Joystick override works before sending any Nav2 goal
7. Start the bag, then drive

## Motor safety

The RoboClaws stop on their own only if their serial timeout is set. With it at
0, killing a launch leaves the last command latched and the rover keeps driving.
Stop all launches first (the port opens once), then:

```bash
python3 scripts/serial_timeout.py            # read all three boards
python3 scripts/serial_timeout.py --set 0.2  # 200 ms, resolution is 0.1 s
```

## Troubleshooting

- Permission denied on a serial port or ZED: `id -nG` must include `dialout`, `video`, `zed`. Log out and back in after groups change.
- Port busy: another launch is running. `sudo fuser -v /dev/rover_roboclaw` (admin).
- Motors don't move: check you didn't pass `use_mock_hardware:=true`, and `ros2 control list_controllers` shows both controllers active.
