# capybara_hw

ros2_control hardware interface for the RoboClaw motor controllers. Based on [roboclaw_hardware_interface](https://github.com/dumbotics/roboclaw_hardware_interface) (Apache 2.0) by Eric Cox.

- 3 RoboClaws on one serial port (addresses 128, 129, 130), 2 wheels each
- Velocity command in, position state out
- Config lives in `capybara_description/urdf/mobile_base.ros2_control.xacro`

| Param | Meaning |
|---|---|
| `serial_port` | e.g. `/dev/rover_roboclaw` |
| `use_duty_cycle` | `true` = open-loop PWM, `false` = RoboClaw velocity PID (needs encoders) |
| `address`, `motor_type`, `qppr` | Per joint: board address, `M1`/`M2`, encoder ticks per rev |

RoboClaws must be set to packet serial, 38400 baud, in Motion Studio.

```bash
colcon test --packages-select roboclaw_serial capybara_hw
```
