# capybara_imu

BNO055 IMU driver over I2C (bus 7, address 0x28). Publishes `sensor_msgs/Imu` on `/imu/data`.

**Not used on Capybara**: the ZED2i IMU is used instead. Kept for Gerbil/reference.

```bash
ros2 run capybara_imu bno055_imu
ros2 topic hz /imu/data
```

The launch file is broken (passes undeclared serial params); run the node directly.
