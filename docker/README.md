# Docker Images

## Dockerfile.dev
Development environment with GUI support.

Features:
- Ubuntu desktop via noVNC (http://localhost:6080)
- RViz, Gazebo (Intel/AMD only)
- Navigation2 packages

Usage:
```bash
# Development container (foreground, aka your not on the Jetson, but need ros2)
docker-compose up ros-dev # Access via browser: http://localhost:6080
```

## Dockerfile.jetson
Headless environment for Jetson hardware.

Features:
- ARM64 optimized
- ZED2i with Jetson SDK
- Hardware access (UART, USB)
- Ideally running during comp.

Usage:
```bash
# Jetson container (detached/background mode) (use this when sshed into Jetson)
docker-compose up -d ros-jetson # -d flag runs container in background without blocking terminal

# Enter running Jetson container
docker exec -it capybara-jetson bash
```
