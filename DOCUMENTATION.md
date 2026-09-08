# Capybara Robot Documentation

## Developer Accounts on the Jetson

Every developer gets their own Linux user on the Jetson. Each user has their own
home directory, their own clone of the team repos, their own branches, and their
own git author. Everyone can be SSHed in at the same time. ROS 2 and the ZED SDK
are installed once system-wide and shared.

The only hard limit is hardware: the RoboClaw, LIDAR, GPS, and ZED can each be
opened by one process at a time, so only one person can drive the rover at once.
Coordinate in the team chat before launching hardware.

### Onboard a new developer (done by an admin, once per person)

```bash
cd ~/domerrover/capybara-software
sudo scripts/add_dev_user.sh <username> "<Full Name>" <git-email>
# e.g.
sudo scripts/add_dev_user.sh alice "Alice Example" alice@nd.edu
```

You will be prompted for the new user's login password. The script then:

1. Creates the Linux user and home directory (`adduser`).
2. Adds them to the `dialout` (serial ports) and `video` (cameras) groups (`usermod -aG`).
3. Adds `source /opt/ros/humble/setup.bash` to their `.bashrc`.
4. Generates an SSH key (`~/.ssh/id_ed25519`) for GitHub.
5. Clones `capybara-software`, `gerbil-software`, and `phoenix-software` into `~/domerrover/`.
6. Sets their global git `user.name` and `user.email`.
7. Prints the public key.

Send the printed public key to the new developer. They add it at
<https://github.com/settings/keys> (New SSH key). GitHub does not accept account
passwords for git anymore, so this key is what lets them push.

### First login (done by the developer)

```bash
ssh <username>@<jetson-ip>
```

Nothing else to configure. Check that git and GitHub work:

```bash
ssh -T git@github.com          # should greet you by GitHub username
cd ~/domerrover/capybara-software
git status
```

Optional: run `ssh-copy-id <username>@<jetson-ip>` once from your laptop to
skip the password on future logins.

### Daily workflow

```bash
ssh <username>@<jetson-ip>
cd ~/domerrover/capybara-software
git checkout main && git pull
git checkout -b feat/my-change
colcon build --symlink-install
source install/setup.bash
ros2 launch capybara_bringup capybara_foxglove.launch.py use_mock_hardware:=false
# ... commit, push, open a PR
```

Your `build/` and `install/` directories live inside your own clone, so they
never collide with anyone else's.

### The deployed copy

The shared runtime account holds the checkout that runs during missions. It
only ever tracks `main`. Do not develop in it; merge a PR and pull there instead.

---

## Jetson Serial Port Setup

Configure UART for RoboClaw, Modems, Science Sensors 

```bash
# Check if port exists
ls -l /dev/ttyTHS1

# Stop serial console service
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Reboot to apply group changes
sudo reboot

# Or set permissions temporarily for current session
sudo chmod 666 /dev/ttyTHS1
```

---

## Docker Container Usage

### Build Container Images

```bash
# Development image (Mac/Windows with VNC)
docker-compose build ros-dev

# Jetson image (headless)
docker-compose build ros-jetson
```

### Run Containers

```bash
# Development container (foreground, aka your not on the Jetson, but need ros2)
docker-compose up ros-dev # Access via browser: http://localhost:6080


# Jetson container (detached/background mode) (use this when sshed into Jetson)
docker-compose up -d ros-jetson # -d flag runs container in background without blocking terminal

# Enter running Jetson container
docker exec -it capybara-jetson bash
```

### Stop Containers

```bash
# Stop specific service
docker-compose stop ros-dev

# Stop and remove containers
docker-compose down

# Force rebuild (no cache)
docker-compose build --no-cache ros-dev
```

---

## ROS2 Workspace Build

### Inside Container

```bash
# Navigate to workspace
cd ~/ros2_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --symlink-install

# Build specific packages
colcon build --packages-select capybara_hw capybara_description capybara_bringup

# Build with release optimizations
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

---

## ZED2i Camera Setup

### Prerequisites

ZED SDK is automatically installed during Docker image build:
- Development image (amd64 only): x86_64 SDK with CUDA
- Jetson image: L4T SDK for Jetson boards
- Mac ARM64: Skipped (not supported by Stereolabs)

### Verify ZED Installation

```bash
# Inside container, check SDK version
/usr/local/zed/tools/ZED_Explorer

# Check if camera is detected (camera must be plugged in)
ls /dev/video*

# Test camera access
/usr/local/zed/tools/ZED_Diagnostic
```

### Build ZED ROS2 Wrapper

The wrapper is included as a git submodule and builds with the workspace:

```bash
cd ~/ros2_ws

# Build ZED packages
colcon build --packages-select zed_wrapper zed_components zed_ros2

# Source workspace
source install/setup.bash

# Verify packages installed
ros2 pkg list | grep zed
```

### Launch ZED Camera

```bash
# ZED2i camera node
ros2 launch zed_wrapper zed2i.launch.py

# Check published topics
ros2 topic list | grep zed

# View camera feed in RViz
rviz2
```

### ZED Topics

Common topics published by ZED wrapper:
- `/zed/zed_node/rgb/image_rect_color` - RGB image
- `/zed/zed_node/depth/depth_registered` - Depth map
- `/zed/zed_node/point_cloud/cloud_registered` - Point cloud
- `/zed/zed_node/odom` - Visual odometry
- `/zed/zed_node/pose` - Camera pose

---

## Launch Arguments

### Main Launch File

```bash
ros2 launch capybara_bringup capybara_foxglove.launch.py
```

### Available Arguments

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `use_mock_hardware` | `false` | `true`, `false` | Enable mock hardware for testing without physical robot |
| `launch_rviz` | `false` | `true`, `false` | Launch RViz visualization |
| `launch_zed` | `true` | `true`, `false` | Launch ZED camera node |

### Common Launch Commands

```bash
# Development: Mock hardware with RViz
ros2 launch capybara_bringup capybara_foxglove.launch.py use_mock_hardware:=true launch_rviz:=true

# Development: Mock hardware without RViz (headless)
ros2 launch capybara_bringup capybara_foxglove.launch.py use_mock_hardware:=true launch_rviz:=false

# Jetson: Real hardware without RViz
ros2 launch capybara_bringup capybara_foxglove.launch.py use_mock_hardware:=false launch_rviz:=false

# Jetson: Real hardware with ZED camera
ros2 launch capybara_bringup capybara_foxglove.launch.py use_mock_hardware:=false launch_zed:=true
```

---

## Teleoperation

```bash
# Standard teleop

# This will be SLOW but controllable
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped -p speed:=0.02 -p turn:=0.2

```

---

## Verification Commands

### Check Topics

```bash
# List all topics
ros2 topic list

# Check controller command topic
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped

# Check odometry
ros2 topic echo /diff_drive_controller/odom

# Check joint states
ros2 topic echo /joint_states
```

### Check Controllers

```bash
# List loaded controllers
ros2 control list_controllers

# Check controller status
ros2 control list_hardware_interfaces
```

### Check TF Tree

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo odom base_link
```

---

## Troubleshooting

### Controller Manager Not Ready

```bash
# Check if controller_manager is running
ros2 service list | grep controller_manager

# Manually spawn controllers
ros2 run controller_manager spawner diff_drive_controller
ros2 run controller_manager spawner joint_state_broadcaster
```

### Serial Port Permission Denied

```bash
# Check current permissions
ls -l /dev/ttyTHS1

# Fix temporarily
sudo chmod 666 /dev/ttyTHS1

# Fix permanently
sudo usermod -a -G dialout $USER
sudo reboot
```

### RoboClaw Communication Test

```bash
# Test script location
cd ~/ros2_ws/src/capybara-software/scripts

# Run test
python3 test_roboclaw.py
```

## python FSM used for managing ROS2_WS, here are notes about just something I thought about
## in terms of how to do error handling, without video-feed/messages 
## at 1-2hz rather than 10hz at foxglove

### gemini prompt

2. How to Detect Being "Stuck" in Code
In ROS2 and Nav2, you do not just have one way to detect a failure; you layer multiple checks. Here are the three most common programmatic ways to signal your system is stuck:

The Kinematic Check (Velocity Mismatch):
Your Nav2 controller is publishing movement commands to the /cmd_vel topic (e.g., "drive forward at 0.5 m/s"). Your code monitors the wheel encoders or your ZED 2i's visual odometry on the /odom topic. If /cmd_vel has been demanding >0.2 m/s for 5 seconds, but /odom says the rover is moving at 0.0 m/s, the rover is physically stuck.

The Hardware Check (Motor Current Spikes):
When a wheel is wedged against an immovable rock and tries to turn, the motor stalls. A stalled DC motor pulls a massive amount of electrical current (amps). If your motor controllers can publish their current draw to a ROS topic, you can write a simple node: If front-left motor current > X amps for 3 seconds, trigger STUCK status.

The Software Check (Nav2 Timeouts & Controller Failures):
Nav2 tracks progress natively. If the local planner cannot calculate a valid path around an obstacle, or if the robot hasn't made progress toward the goal within a specific time tolerance, Nav2 throws a Goal Failed or Patience Exceeded exception.

3. Executing the "Call for Help" Recovery Behavior
In ROS2 Nav2, the entire decision-making process is handled by Behavior Trees. Behavior Trees allow you to create fallback branches. If the main "Navigate to Pose" action fails because the rover is stuck, the tree automatically falls back to a "Recovery" branch.

You can write a custom Nav2 Recovery Node that does exactly what you proposed:

Clear Costmaps: The first step is to tell Nav2 to delete its local memory (clear the costmaps). Sometimes a ghost obstacle from a bad LiDAR ping is trapping the rover.

The Spin: Command a slow 360-degree rotation.

The Snapshot: While spinning, trigger a ROS service to capture a single frame from the ZED 2i.

Compress & Transmit (Crucial for 900MHz):

Option A (High Bandwidth for 900MHz): Compress that single image into a tiny, heavily artifacted black-and-white JPEG (maybe 20-30 KB) and send it over your serial radio link. It might take 10 to 30 seconds to transmit, but operators will see the rock.

Option B (Low Bandwidth - The "Image Description"): Run a lightweight YOLO object detection model on your Jetson. When the rover spins, it doesn't send an image at all. It just sends a text string: [HAZARD DETECTED: BOULDER, 0.5 METERS, HEADING 45 DEG]. Text takes mere milliseconds to transmit over 900MHz.

Wait for Teleop: The rover enters a holding state, waiting for you to hit a "Reset Nodes" or "Override" button from the base station.

Using Nav2 Behavior Trees is the cleanest way to do this because you don't have to write messy, nested if/else statements in your main control loop; the tree handles the failure logic for you.

Have you started looking into modifying the default Nav2 Behavior Tree XML files, or are you currently trying to manage state using a custom Python/C++ node?