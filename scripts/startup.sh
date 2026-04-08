#!/bin/bash

BOLD='\033[1m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

print_step()  { echo -e "\n${CYAN}${BOLD}>>> $1${NC}"; }
print_ok()    { echo -e "${GREEN}[OK]${NC} $1"; }
print_fail()  { echo -e "${RED}[FAIL]${NC} $1" >&2; }
print_warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }

echo -e "\n${BOLD}========================================${NC}"
echo -e "${BOLD}   Capybara Rover — Startup Script${NC}"
echo -e "${BOLD}========================================${NC}"

# ── navigate ──────────────────────────────────────────────────────
print_step "Changing to workspace directory"
cd ~/capybara-software || { print_fail "Could not cd to ~/capybara-software. Exiting."; exit 1; }
print_ok "Working directory: $(pwd)"

# ── install ROS2 dependencies ──────────────────────────────────────────────────
print_step "Installing ROS2 dependencies (rosdep)"
if rosdep install --from-paths src --ignore-src -y; then
    print_ok "rosdep install complete"
else
    print_fail "rosdep install failed"
    exit 1
fi

# ── build workspace ────────────────────────────────────────────────────────────
print_step "Building workspace (colcon)"
if colcon build --symlink-install; then
    print_ok "colcon build complete"
else
    print_fail "colcon build failed"
    exit 1
fi

# ── source workspace ───────────────────────────────────────────────────────────
print_step "Sourcing workspace"
if [ -f install/setup.bash ]; then
    # shellcheck disable=SC1091
    source install/setup.bash
    print_ok "Workspace sourced (install/setup.bash)"
else
    print_fail "install/setup.bash not found — did the build succeed?"
    exit 1
fi

# ── launch loop ────────────────────────────────────────────────────────────────
print_step "Starting ROS2 launch loop"
echo -e "  ${YELLOW}Press Ctrl+C to exit, or [Enter] to restart after a crash.${NC}"

LAUNCH_COUNT=0
while true; do
    LAUNCH_COUNT=$((LAUNCH_COUNT + 1))
    echo -e "\n${BOLD}--- Launch attempt #${LAUNCH_COUNT} ---${NC}"

    ros2 launch capybara_bringup capybara_foxglove.launch.py \
    use_joystick:=false use_mock_hardware:=false      \
	launch_rviz:=false launch_zed:=true
    EXIT_CODE=$?

    if [ $EXIT_CODE -eq 0 ]; then
        print_ok "Launch exited cleanly (code 0)"
    else
        print_warn "Launch exited with code $EXIT_CODE"
    fi

    sleep 2
    read -rp $'\nPress [Enter] to restart, or Ctrl+C to quit...'
done
