#!/usr/bin/env python3

# Also get CPU %, motor temps, battery info, etc.
import time
import datetime
from basicmicro import Basicmicro

PORT = "/dev/ttyUSB0"

# RoboClaw supports these baud rates
BAUD_RATES = [38400]

# Our three RoboClaws: top=128, middle=129, bottom=130
ADDRESSES = [0x80, 0x81, 0x82]  # 128, 129, 130

def read_motor_temp(controller, address):
    temp = controller.ReadTemp(address)
    if temp[0]:  # Check if read was successful
        return temp[1] / 10.0  # Convert to °C
    else:
        return None

def read_processor_temp(path):
    with open(path, 'r') as f:
        return int(f.read().strip()) / 1000.0 # Convert to °C

if __name__ == "__main__":
    date = datetime.date.today()

    cpu_path = "/sys/class/thermal/thermal_zone0/temp"
    gpu_path = "/sys/class/thermal/thermal_zone1/temp"
    with open('temp_log.txt', 'w') as log_file:
        log_file.write(f"Testing Baud Rate {BAUD_RATES[0]} on Port: {PORT}\n")
        log_file.write(f"Date: {date}\n")
        with Basicmicro(PORT, BAUD_RATES[0]) as controller:
            while True:
                cpu_temp = read_processor_temp(cpu_path)
                gpu_temp = read_processor_temp(gpu_path)
                motor_temps = []

                motor_temps.append(read_motor_temp(controller, ADDRESSES[0]))
                motor_temps.append(read_motor_temp(controller, ADDRESSES[1]))
                motor_temps.append(read_motor_temp(controller, ADDRESSES[2]))

                timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                timestamp = str(timestamp).split()[-1]
                print(f"CPU Temp: {cpu_temp} m°C")
                print(f"GPU Temp: {gpu_temp} m°C")
                log_file.write(f"[{timestamp}] CPU Temp: {cpu_temp} m°C, GPU Temp: {gpu_temp} °C\n")
                log_file.write(f"Motor Temps: {motor_temps} °C\n\n")

                time.sleep(5)
