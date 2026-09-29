#!/usr/bin/env bash
# drive.sh — manual driving only: controllers + joystick teleop, nothing else.
# No ZED, no LIDAR, no GPS, no Nav2, no Foxglove.
#
#   scripts/drive.sh              build, then launch
#   scripts/drive.sh --no-build   launch straight away (used by the systemd unit)
#
# Hold L1 on the PS5 controller and use the left stick.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"

# shellcheck disable=SC1090
source "$ROS_SETUP"
cd "$REPO_DIR"

if [[ "${1:-}" != "--no-build" ]]; then
    colcon build --symlink-install
fi

# shellcheck disable=SC1091
source "$REPO_DIR/install/setup.bash"

# Wait for the controller: on boot, Bluetooth often connects after ROS is ready.
for _ in $(seq 30); do
    [[ -e /dev/input/js0 ]] && break
    echo "waiting for a joystick on /dev/input/js0 ..."
    sleep 2
done
[[ -e /dev/input/js0 ]] || echo "WARNING: no joystick found; launching anyway"

exec ros2 launch capybara_bringup capybara.launch.xml \
    use_mock_hardware:=false \
    use_joystick:=true \
    launch_zed:=false \
    launch_lidar:=false \
    launch_gps:=false \
    launch_rviz:=false
