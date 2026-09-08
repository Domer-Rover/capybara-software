# capybara-software (ROS 2)
Welcome to the software repo for the Capybara mars rover! This document will give you a brief description of the repo's layout and an overview of the repo.

[![Static Badge](https://img.shields.io/badge/Software_Lead-Brandon_Martinez-C1E1C1)](https://github.com/bemndy)
[![Static Badge](https://img.shields.io/badge/Arm_Lead-Noah_Janchar-58A4D6)]()
[![Static Badge](https://img.shields.io/badge/Communications_Lead-Henry_Jochaniewicz-D56DE5)](https://github.com/henryJ099123)

## Directory Structure

- **src/capybara_bringup**
  _Launch files, Nav2 / SLAM / controller configs, maps_
- **src/capybara_description**
  _URDF/xacro robot description and ros2\_control block_
- **src/capybara_hw**
  _ros2\_control hardware interface for the RoboClaw motor controllers_
- **src/imu_package**
  _BNO055 IMU driver (currently unused; ZED2i IMU is used via VIO)_
- **src/vendors**
  _Vendored external packages: ZED ROS 2 wrapper, roboclaw\_serial_
- **scripts**
  _Jetson setup, developer onboarding, and hardware test scripts_
- **docker**
  _Dockerfiles for the VNC dev container and the headless Jetson container_
- **models**
  _ONNX detection models_

See `DOCUMENTATION.md` for setup and usage, and `../ROADMAP.md` for the season plan.

