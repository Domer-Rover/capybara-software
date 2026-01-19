FROM tiryoh/ros2-desktop-vnc:humble

# Switch to root to install software:
USER root

# Update and install "plugins" or dependencies:
RUN apt-get update && apt-get install -y \
    nano \
    git \
    python3-pip \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# (Optional) Install Python packages:
# RUN pip3 install numpy pandas