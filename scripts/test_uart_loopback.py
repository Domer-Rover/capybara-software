"""
UART loopback test - run BEFORE connecting RoboClaws.

Step 1: Physically jumper pin 8 (TX) to pin 10 (RX) on the Jetson 40-pin header.
Step 2: Run this script. If it passes, the port works and your wiring is the issue.
Step 3: Remove the jumper and reconnect RoboClaws.
"""

import serial
import time

PORT = "/dev/ttyTHS1"
BAUD = 38400
TEST_BYTES = b'\xaa\x55\x01\xff'

print(f"Opening {PORT} at {BAUD} baud...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"FAILED to open port: {e}")
    raise SystemExit(1)

ser.reset_input_buffer()
ser.reset_output_buffer()

print(f"Sending: {TEST_BYTES.hex()}")
ser.write(TEST_BYTES)
time.sleep(0.1)

received = ser.read(len(TEST_BYTES))
ser.close()

if received == TEST_BYTES:
    print(f"LOOPBACK PASS - received: {received.hex()}")
    print("The Jetson serial port is working correctly.")
    print("If RoboClaws still don't respond, the problem is wiring or RoboClaw config.")
else:
    print(f"LOOPBACK FAIL - received: {received.hex() if received else '(nothing)'}")
    print("The Jetson serial port itself has an issue.")
    print("Check: is nvgetty disabled? (sudo systemctl disable nvgetty && sudo reboot)")
