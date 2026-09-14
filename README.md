<h1 align="center">
  capybara-software
  <br>
</h1>

<p align="center">
  Software stack for <b>Capybara</b>, the Mars rover built by <a href="https://github.com/Domer-Rover">Domer Rover</a> for the University Rover Challenge.
  <br />
  Built on <b>ROS 2 Humble</b> on a Jetson, with a ZED2i camera, LD19 LIDAR, u-blox GPS, and RoboClaw motor controllers.
</p>

<p align="center">
  <a href="https://github.com/Domer-Rover/capybara-software/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MIT-blue" alt="License"></a>
  <img src="https://img.shields.io/badge/Software%20Lead-Brandon%20Martinez-C1E1C1" alt="Software Lead">
  <img src="https://img.shields.io/badge/CI-In%20Progress-yellow" alt="CI Status">
</p>

<p align="center">
  <a href="https://github.com/Domer-Rover">Domer Rover</a>
  ·
  <a href="https://github.com/Domer-Rover/capybara-software/tree/main/docs">Documentation</a>
  ·
  <a href="https://github.com/Domer-Rover/capybara-software/issues">Report an Issue</a>
</p>

---

## Overview

Capybara is Domer Rover's entry for the **University Rover Challenge (URC)**, and this repository is its onboard software stack: ROS 2 packages for driving, hardware interfacing, navigation, and the launch/config plumbing to bring the rover up.

## Directory Structure

| Path | Description |
| --- | --- |
| `src/capybara_bringup` | Launch files, Nav2/SLAM/controller configs, maps |
| `src/capybara_description` | URDF and `ros2_control` block |
| `src/capybara_hw` | RoboClaw hardware interface for `ros2_control` |
| `src/imu_package` | BNO055 driver (unused; the ZED2i IMU is used) |
| `src/vendors` | ZED ROS 2 wrapper, `roboclaw_serial` |
| `scripts` | Developer onboarding and hardware test scripts |
| `docker` | Dockerfiles (out of date) |
| `docs` | Setup and usage guides |

## Getting Started

The rover runs natively on the Jetson. Each developer has their own account and clone ([Jetson setup](docs/jetson-setup.md)).

```bash
ssh <username>@jetsonson.dhcp.nd.edu
cd ~/domerrover/capybara-software
colcon build --symlink-install && source install/setup.bash
ros2 launch capybara_bringup capybara_foxglove.launch.py
```

Connect Foxglove to `ws://jetsonson.dhcp.nd.edu:8765`. See [Build and run](docs/build-and-run.md) for other launch files and checks.

## Built With

- **ROS 2 Humble**: middleware for every package in this repo
- **ros2_control**: hardware abstraction and control
- **Nav2**: autonomous navigation
- **ZED SDK**: visual-inertial odometry

## About Domer Rover

[Domer Rover](https://github.com/Domer-Rover) is the University of Notre Dame's rover team, competing at the University Rover Challenge (URC). This repository is maintained by the team's software subgroup.

## Contributing

Bug reports and pull requests are welcome. Check the `.github` folder for the PR template, and open an [issue](https://github.com/Domer-Rover/capybara-software/issues) if you run into a problem.

## License

This project is licensed under the [MIT License](https://github.com/Domer-Rover/capybara-software/blob/main/LICENSE).
