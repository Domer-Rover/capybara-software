<h1 align="center">
  capybara-software
  <br>
</h1>

<p align="center">
  Software stack for <b>Capybara</b>, the Mars rover built by <a href="https://github.com/Domer-Rover">Domer Rover</a> for the University Rover Challenge.
  <br />
  Built on <b>ROS 2</b>, this repo houses everything that lets Capybara drive, sense, and complete mission tasks.
</p>

<p align="center">
  <a href="https://github.com/Domer-Rover/capybara-software/blob/main/LICENSE"><img src="https://img.shields.io/badge/license-MIT-blue" alt="License"></a>
  <img src="https://img.shields.io/badge/Software%20Lead-Brandon%20Martinez-C1E1C1" alt="Software Lead">
  <img src="https://img.shields.io/badge/CI-In%20Progress-yellow" alt="CI Status">
</p>

<p align="center">
  <a href="https://github.com/Domer-Rover">Domer Rover</a>
  ·
  <a href="https://github.com/Domer-Rover/capybara-software/blob/main/DOCUMENTATION.md">Documentation</a>
  ·
  <a href="https://github.com/Domer-Rover/capybara-software/issues">Report an Issue</a>
</p>

---

## Overview

Capybara is Domer Rover's entry for the **University Rover Challenge (URC)**, and this repository is its onboard software stack. It's built as a set of ROS 2 packages that together handle driving, hardware interfacing, and the launch/config plumbing needed to bring the rover up in the field or in simulation.

## Directory Structure

| Path | Description |
| --- | --- |
| `.github` | CI pipeline and PR/issue templates |
| `capybara_bringup` | Launch files and configuration |
| `capybara_description` | Hardware description for `ros2_control` |
| `capybara_driving` | Driving package, built on `robot_localization` |
| `capybara_hw` | Hardware interface for `ros2_control` |
| `docker` | Container definitions for VNC and headless environments |
| `models` | Robot/hardware models |
| `scripts` | Helper scripts, mainly for testing and setup |
| `vendors` | Manually installed external libraries |
| `Dockerfile` / `docker-compose.yml` | Container build and orchestration |
| `DOCUMENTATION.md` | Deeper documentation on the packages above |

## Getting Started

Capybara's software runs in Docker to keep the ROS 2 environment consistent across every machine on the team, from laptops to the rover's onboard computer.

- Clone the repo:
  ```bash
  git clone https://github.com/Domer-Rover/capybara-software.git
  cd capybara-software
  ```
- Build and start the containers:
  ```bash
  docker compose up --build
  ```
- See [`DOCUMENTATION.md`](https://github.com/Domer-Rover/capybara-software/blob/main/DOCUMENTATION.md) for package-level details, launch instructions, and hardware setup.

## Built With

- **ROS 2** — robotics middleware powering every package in this repo
- **ros2_control** — hardware abstraction and control
- **robot_localization** — sensor fusion for driving/odometry
- **Docker** — reproducible builds for VNC and headless deployment

## About Domer Rover

[Domer Rover](https://github.com/Domer-Rover) is the University of Notre Dame's rover team, competing at the University Rover Challenge (URC). This repository is maintained by the team's software subgroup.

## Contributing

Bug reports and pull requests are welcome — check the `.github` folder for issue and PR templates, and open an [issue](https://github.com/Domer-Rover/capybara-software/issues) if you run into a problem.

## License

This project is licensed under the [MIT License](https://github.com/Domer-Rover/capybara-software/blob/main/LICENSE).
