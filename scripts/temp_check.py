#!/usr/bin/env python3

# Also get CPU %, motor temps, battery info, etc.
import time
import datetime
from basicmicro import Basicmicro
from jtop import jtop

PORT = "/dev/ttyUSB0"

# RoboClaw supports these baud rates
BAUD_RATES = [38400]

# Our three RoboClaws: top=128, middle=129, bottom=130
ADDRESSES = [0x80, 0x81, 0x82]  # 128, 129, 130

# How many seconds between temperature checks
wait_time = 5

def read_motor_temp(controller, address):
    temp1, temp2 = controller.ReadTemp(address), controller.ReadTemp2(address)
    temps = [None, None]
    if temp1[0]:  # Check if read was successful
        temps[0] = temp1[1] / 10.0  # Convert to °C
    if temp2[0]:  # Check if read was successful
        temps[1] = temp2[1] / 10.0  # Convert to °C
    return temps

def read_processor_temp(path):
    with open(path, 'r') as f:
        return int(f.read().strip()) / 1000.0 # Convert to °C

def monitor_temp_levels(component, temp):
    # Critical threshold: 100 C, warning: 85 C.
    TEMP_CRIT = 100
    TEMP_WARN = 85
    
    if temp > TEMP_CRIT:
        print(f"{component} exceeded critical threshold {TEMP_CRIT}")
    elif temp > TEMP_WARN:
        print(f"{component} dangerously hot ( > {TEMP_WARN})")
        

if __name__ == "__main__":
    date = datetime.date.today()

    cpu_path = "/sys/class/thermal/thermal_zone0/temp"
    gpu_path = "/sys/class/thermal/thermal_zone1/temp"
    # gpu_usage_path = "/sys/class/"
    with open('temp_log.txt', 'w') as log_file:
        log_file.write(f"Testing Baud Rate {BAUD_RATES[0]} on Port: {PORT}\n")
        log_file.write(f"Date: {date}\n")
        with Basicmicro(PORT, BAUD_RATES[0]) as controller:
            with jtop() as jetson:
                while True:
                    if jetson.ok():
                        gpu_load = jetson.gpu['gpu']['status']['load']
                        cpu_load = jetson.cpu
                        print(cpu_load)

                    cpu_temp = read_processor_temp(cpu_path)
                    gpu_temp = read_processor_temp(gpu_path)
                    motor_temps = []

                    motor_temps.append(read_motor_temp(controller, ADDRESSES[0]))
                    motor_temps.append(read_motor_temp(controller, ADDRESSES[1]))
                    motor_temps.append(read_motor_temp(controller, ADDRESSES[2]))

                    
                    monitor_temp_levels("CPU", cpu_temp)
                    monitor_temp_levels("GPU", gpu_temp)
                    for i in range(len(motor_temps)):
                        monitor_temp_levels(f"MC {i}, M1", motor_temps[i][0])
                        monitor_temp_levels(f"MC {i}, M2", motor_temps[i][1])


                    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    timestamp = str(timestamp).split()[-1]
                    print(f"CPU Temp: {cpu_temp} °C")
                    print(f"GPU Temp: {gpu_temp} °C; GPU load: {gpu_load}")
                    log_file.write(f"[{timestamp}] CPU Temp: {cpu_temp} m°C, GPU Temp: {gpu_temp} °C; GPU Load: {gpu_load}\n")
                    log_file.write(f"Motor Temps: {motor_temps} °C\n\n")

                    time.sleep(wait_time)
