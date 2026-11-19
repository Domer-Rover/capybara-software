#!/usr/bin/env python3
"""Talk to Capybara's three RoboClaws directly, bypassing ros2_control.

Two modes:

  --probe   (default, READ-ONLY, safe)  ask each board for its firmware
            version at one or more baud rates and report who answers
  --drive   spin every wheel at one speed, open loop

Why probe first: on 2026-11-19 the hardware interface logged a continuous
storm of "Read timeout!" and "CRCs do not match" from all three boards at
once. Mixed timeouts AND CRC errors mean garbage on the wire, not a missing
device -- usually a baud mismatch or two boards answering on the same
address. Driving into that tells you nothing, so this refuses to drive until
the boards answer.

The ros2_control stack MUST BE STOPPED first -- two processes on one serial
port is itself a cause of CRC errors.

    # 1. who is out there, and at what baud?
    python3 scripts/diagnostics/drive_all_wheels.py --scan-baud

    # 2. once they answer, with the ROVER ON BLOCKS:
    python3 scripts/diagnostics/drive_all_wheels.py --drive --confirm
    python3 scripts/diagnostics/drive_all_wheels.py --drive --confirm --speed 0.3
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('pyserial missing:  pip3 install pyserial')

PORT = '/dev/rover_roboclaw'
BAUD = 38400                 # matches vendors/roboclaw_serial device.hpp:157
SCAN_BAUDS = [38400, 115200, 57600, 19200, 9600, 230400, 460800, 2400]

# address -> (M1 wheel, M2 wheel), from mobile_base.ros2_control.xacro
BOARDS = {
    0x80: ('rear_right', 'front_left'),   # 128, top
    0x81: ('mid_right', 'mid_left'),      # 129, middle
    0x82: ('front_right', 'rear_left'),   # 130, bottom
}

WHEEL_RADIUS = 0.127      # m, capybara_controllers.yaml
MAX_DUTY_SPEED = 10.0     # rad/s at full duty, from the xacro's max_duty_speed

CMD_M1_DUTY = 32
CMD_M2_DUTY = 33
CMD_READ_VERSION = 21


def crc16(data):
    """RoboClaw's CRC16-CCITT: poly 0x1021, init 0x0000, MSB first."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def read_version(port, address):
    """Command 21. Returns the version string, or None. Read-only and harmless."""
    port.reset_input_buffer()
    port.reset_output_buffer()
    port.write(bytes([address, CMD_READ_VERSION]))
    port.flush()

    body = bytearray()
    while len(body) < 64:
        ch = port.read(1)
        if not ch:
            return None, 'timeout'
        body += ch
        if ch == b'\x00':
            break
    else:
        return None, 'no terminator'

    crc_bytes = port.read(2)
    if len(crc_bytes) != 2:
        return None, 'truncated CRC'

    expected = crc16(bytes([address, CMD_READ_VERSION]) + bytes(body))
    actual = (crc_bytes[0] << 8) | crc_bytes[1]
    text = bytes(body).rstrip(b'\x00\r\n').decode('ascii', 'replace')
    if expected != actual:
        return text, f'CRC mismatch (got {actual:#06x}, want {expected:#06x})'
    return text, None


def write_duty(port, address, command, duty):
    """Signed 16-bit duty, -32767..32767. Expects a 0xFF ack."""
    duty = max(-32767, min(32767, int(duty)))
    payload = bytes([address, command]) + duty.to_bytes(2, 'big', signed=True)
    port.write(payload + crc16(payload).to_bytes(2, 'big'))
    port.flush()
    ack = port.read(1)
    return ack == b'\xff'


def probe(port_name, bauds, timeout):
    print(f'Port {port_name}')
    any_ok = False
    good_baud = None
    for baud in bauds:
        print(f'\n--- {baud} baud ---')
        try:
            with serial.Serial(port_name, baud, timeout=timeout) as port:
                time.sleep(0.1)
                for address, (m1, m2) in BOARDS.items():
                    label = f'  {address:#04x} ({address}) M1={m1:<12} M2={m2:<12}'
                    version, err = read_version(port, address)
                    if version and not err:
                        print(f'{label} OK   "{version}"')
                        any_ok = True
                        good_baud = good_baud or baud
                    elif version:
                        print(f'{label} BAD  {err}  raw="{version}"')
                    else:
                        print(f'{label} --   {err}')
        except serial.SerialException as exc:
            print(f'  cannot open: {exc}')
            return False, None
    return any_ok, good_baud


def verdict(any_ok, good_baud, scanned):
    print()
    print('=' * 70)
    if any_ok and good_baud == BAUD:
        print(f'Boards answer at {BAUD} baud, which is what the driver uses.')
        print('So the wiring and addressing are fine and the error storm has')
        print('another cause. The most likely one left: the ros2_control stack')
        print('was still running and competing for the port. Confirm with')
        print('  ros2 node list    (must be empty of controller_manager)')
    elif any_ok:
        print(f'Boards answer at {good_baud} baud, but the driver is hardcoded')
        print(f'to {BAUD} in src/vendors/roboclaw_serial/include/roboclaw_serial/')
        print('device.hpp:157. THAT is your error storm.')
        print()
        print('Fix one of two ways:')
        print(f'  a) set the boards back to {BAUD} in Motion Studio (preferred --')
        print('     it is the RoboClaw packet-serial default and the vendored')
        print('     driver is not ours to patch)')
        print(f'  b) change device.hpp to B{good_baud} and rebuild, accepting')
        print('     that an upgrade of the vendored tree reverts it')
    else:
        print('NO board answered at any baud tried.')
        print()
        print('That is not an addressing or baud problem, it is lower level:')
        print(f'  - wrong device? ls -l /dev/rover_roboclaw /dev/ttyACM* /dev/ttyUSB*')
        print('  - is the ros2_control stack holding the port? ros2 node list')
        print('  - boards powered, and set to Packet Serial (not RC/Analog)?')
        print('  - logic ground between the Jetson and the boards?')
        print('  - S1/S2 TX and RX not swapped?')
    print('=' * 70)


def drive(port_name, baud, speed, duration, timeout):
    omega = speed / WHEEL_RADIUS
    duty = int(round(omega / MAX_DUTY_SPEED * 32767))
    print(f'{speed} m/s / {WHEEL_RADIUS} m radius = {omega:.3f} rad/s')
    print(f'open loop: {omega:.3f} / {MAX_DUTY_SPEED} rad/s full duty '
          f'= {duty} of 32767 ({duty / 32767 * 100:.0f}%)')
    if abs(duty) >= 32767:
        print('WARNING: saturated. The wheels cannot reach this speed open loop.')
    print()
    print('NOTE open loop means duty, not speed. Without encoder feedback the')
    print('     actual speed depends on battery voltage and load, so treat')
    print('     this as "all six at the same effort", not "all six at 0.5 m/s".')
    print()

    acked = 0
    with serial.Serial(port_name, baud, timeout=timeout) as port:
        time.sleep(0.1)
        end = time.time() + duration
        try:
            while time.time() < end:
                for address in BOARDS:
                    if write_duty(port, address, CMD_M1_DUTY, duty):
                        acked += 1
                    if write_duty(port, address, CMD_M2_DUTY, duty):
                        acked += 1
                time.sleep(0.05)   # well inside the ~200 ms firmware failsafe
        except KeyboardInterrupt:
            print('\ninterrupted')
        finally:
            # Stop, repeatedly -- a dropped stop packet leaves the rover driving
            # until the firmware failsafe catches it.
            for _ in range(5):
                for address in BOARDS:
                    write_duty(port, address, CMD_M1_DUTY, 0)
                    write_duty(port, address, CMD_M2_DUTY, 0)
            print('stopped')
    print(f'{acked} commands acknowledged')
    if acked == 0:
        print('None were acknowledged -- nothing was listening. Run --probe.')
        return 1
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--scan-baud', action='store_true',
                    help='probe every common baud rate, not just --baud')
    ap.add_argument('--drive', action='store_true', help='spin the wheels')
    ap.add_argument('--confirm', action='store_true',
                    help='required with --drive: rover is on blocks')
    ap.add_argument('--speed', type=float, default=0.5, help='m/s (default 0.5)')
    ap.add_argument('--duration', type=float, default=3.0, help='seconds (default 3)')
    ap.add_argument('--timeout', type=float, default=0.1, help='serial read timeout s')
    args = ap.parse_args()

    if args.drive:
        if not args.confirm:
            sys.exit('--drive spins all six wheels. Get the rover on blocks, '
                     'then add --confirm.')
        sys.exit(drive(args.port, args.baud, args.speed, args.duration, args.timeout))

    bauds = SCAN_BAUDS if args.scan_baud else [args.baud]
    any_ok, good_baud = probe(args.port, bauds, args.timeout)
    verdict(any_ok, good_baud, bauds)
    sys.exit(0 if any_ok else 1)


if __name__ == '__main__':
    main()
