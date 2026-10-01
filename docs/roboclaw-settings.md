# RoboClaw settings (Motion Studio)

Capybara: **three** RoboClaws at addresses 128, 129, 130 on one serial bus,
two wheels each. Nothing was recorded for this robot — fill in what is
actually on the boards.

Battery: **TODO** — record model, chemistry, cell count, and full-charge
voltage. The voltage thresholds below are placeholders until that is known.

## Communication — must match the code

| Setting | Value | Why |
|---|---|---|
| Control mode | Packet Serial | `roboclaw_serial` speaks only this |
| Baud rate | 38400 | hardcoded in `roboclaw_serial/device.hpp` |
| Packet Serial Address | 128 / 129 / 130 | `mobile_base.ros2_control.xacro` |
| Multi-Unit Mode | **On** | three boards share one bus; without it they talk over each other |
| Serial timeout | 0.2 s | motors stop if ROS dies; `scripts/serial_timeout.py --set 0.2` |
| 128 | M1 rear_right, M2 front_left | `mobile_base.ros2_control.xacro` |
| 129 | M1 mid_right, M2 mid_left | |
| 130 | M1 front_right, M2 rear_left | |
| Encoder mode | Quadrature (both) | interface reads `READ_M1_M2_ENC` |

## Main battery (TODO: confirm pack)

| Setting | Value | Reasoning |
|---|---|---|
| Maximum | TODO | must clear a full charge |
| Minimum | TODO | above the pack's damage floor, low enough to survive sag |

## Logic battery

The "Logic Battery High" fault (error LED blinks twice) means the measured
logic voltage exceeded this maximum. Measure LB+ to LB− before setting it.

| If LB+ measures | Minimum | Maximum |
|---|---|---|
| ~12 V (from the DC-DC converter) | 10.0 V | 16.0 V |
| ~22-26 V (fed from the pack, or jumpered to main) | 20.0 V | 27.0 V |

## Current limits

| Setting | Value |
|---|---|
| M1 / M2 current limit | **5 A each to start** |

Capybara runs in duty-cycle (open loop) mode today, so there is no velocity
PID to tune — but the current limits and voltage thresholds still apply. Set
the electronic limit low and raise it only after reading real currents:

```bash
python3 -c "
from basicmicro import Basicmicro
c = Basicmicro('/dev/rover_roboclaw', 38400); c.Open()
for a in (0x80, 0x81, 0x82):
    print(hex(a), c.ReadError(a), c.ReadMainBatteryVoltage(a), c.ReadCurrents(a))
c.close()"
```

## Velocity PID (closed loop only)

Measure first, tune second:

1. **qppr** — mark the tire, zero the encoder, turn the wheel 10 full
   revolutions, read the count, divide by 10. Put it in the xacro (both joints).
2. **QPPS** — run the motor at full duty in Motion Studio and read live
   counts/sec. That is the maximum for the PID.
3. **Encoder polarity** — rolling a wheel forward must make the count go **up**.
   If it counts down, swap that channel's A/B encoder wires. In velocity mode a
   reversed encoder becomes positive feedback and the motor runs away at full
   throttle.
4. Auto-tune velocity PID, or start at P 0.2 / I 0.1 / D 0.

Use a shielded USB cable with a ferrite bead for tuning. Motor noise drops the
USB link mid-tune, which can leave half-written settings on the board.

## Measured values (fill in)

| Value | 128 M1/M2 | 129 M1/M2 | 130 M1/M2 |
|---|---|---|---|
| qppr | | | |
| Current at cruise | | | |
| Current limit set | | | |
| Voltage thresholds | | | |
