# capybara-software (ROS 2)

Software for Capybara, the Domer Rover URC rover: ROS 2 Humble on a Jetson, ZED2i, LD19 LIDAR, u-blox GPS, RoboClaw motor controllers.

[![Static Badge](https://img.shields.io/badge/Software_Lead-Brandon_Martinez-C1E1C1)](https://github.com/bemndy)
[![Static Badge](https://img.shields.io/badge/Arm_Lead-Noah_Janchar-58A4D6)]()
[![Static Badge](https://img.shields.io/badge/Communications_Lead-Henry_Jochaniewicz-D56DE5)](https://github.com/henryJ099123)

## Quick start

```bash
ssh <username>@jetsonson.dhcp.nd.edu
cd ~/domerrover/capybara-software
colcon build --symlink-install && source install/setup.bash
ros2 launch capybara_bringup capybara_foxglove.launch.py
```

Foxglove: `ws://jetsonson.dhcp.nd.edu:8765`

## Layout

| Path | Contents |
|---|---|
| `src/capybara_bringup` | Launch files, Nav2/SLAM/controller configs, maps |
| `src/capybara_description` | URDF and ros2_control block |
| `src/capybara_hw` | RoboClaw ros2_control hardware interface |
| `src/imu_package` | BNO055 driver (unused; ZED IMU is used) |
| `src/vendors` | ZED ROS 2 wrapper, roboclaw_serial |
| `scripts` | Dev onboarding and hardware test scripts |
| `docker` | Dockerfiles (out of date) |

## Docs

- [Jetson setup](docs/jetson-setup.md): accounts, SSH keys, serial ports
- [Build and run](docs/build-and-run.md): build, launch, teleop, checks
