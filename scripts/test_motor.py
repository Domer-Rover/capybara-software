import logging
import time
from basicmicro import Basicmicro

# Enable logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

PORT = "/dev/ttyTHS1"  # Update as needed for your system 
BAUDRATE = 38400 

# Using context manager for automatic resource handling
with Basicmicro(PORT, BAUDRATE) as controller: 
    address = 0x80  # Default address
    print(f"Connected to Roboclaw on {PORT} at {BAUDRATE} baud")

    # Simple motor control
    message = (input("Enter \"test\" to go forward for 5 seconds"));
    if (message == "test"):
        controller.DutyM1(address, 8192)  
        controller.DutyM2(address, 8192)  
        time.sleep(5)
        controller.DutyM1(address, 0)
        controller.DutyM2(address, 0)

    # Read encoder values
    # enc1 = controller.ReadEncM1(address)
    # if enc1[0]:  # Check if read was successful
    #     print(f"Encoder 1 count: {enc1[1]}")

    # Set velocity PID values
    # controller.SetM1VelocityPID(address, kp=1.0, ki=0.5, kd=0.25, qpps=44000)

    print("Test complete.")