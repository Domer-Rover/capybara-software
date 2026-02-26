import minimalmodbus
import serial
import matplotlib.pyplot as plt
import time
from datetime import datetime

# --- Sensor Setup (Windows) ---
sensor = minimalmodbus.Instrument('COM5', 1)  # Change COM3 to your actual port
sensor.serial.baudrate = 9600
sensor.serial.bytesize = 8
sensor.serial.parity = serial.PARITY_NONE
sensor.serial.stopbits = 1
sensor.serial.timeout = 1
sensor.mode = minimalmodbus.MODE_RTU

def read_sensor():
    # Read 4 registers starting at 0x0000 in one request
    raw = sensor.read_registers(0x0000, 4, 3)

    humidity    = raw[0] / 10.0   # %RH
    temperature = raw[1] / 10.0   # °C
    ec          = raw[2]          # µS/cm (no division)
    ph          = raw[3] / 10.0   # pH

    return temperature, humidity, ec, ph

# --- Data Storage ---
timestamps = []
temp_vals  = []
hum_vals   = []
ec_vals    = []
ph_vals    = []

NUM_READINGS = 20

print("Reading sensor...")
for i in range(NUM_READINGS):
    try:
        temp, hum, ec, ph = read_sensor()
        timestamps.append(datetime.now().strftime("%H:%M:%S"))
        temp_vals.append(temp)
        hum_vals.append(hum)
        ec_vals.append(ec) 
        ph_vals.append(ph)
        print(f"[{i+1}/{NUM_READINGS}] Temp: {temp}°C | Humidity: {hum}% | EC: {ec} µS/cm | pH: {ph}")
    except Exception as e:
        print(f"Read error: {e}")
    time.sleep(2)

# --- Save Graph ---
fig, axes = plt.subplots(2, 2, figsize=(14, 8))
fig.suptitle("SEN0604 Soil Sensor Readings", fontsize=14)

axes[0,0].plot(timestamps, temp_vals, color="red")
axes[0,0].set_title("Temperature (°C)")
axes[0,0].tick_params(axis='x', rotation=45)

axes[0,1].plot(timestamps, hum_vals, color="blue")
axes[0,1].set_title("Humidity (%RH)")
axes[0,1].tick_params(axis='x', rotation=45)

axes[1,0].plot(timestamps, ec_vals, color="orange")
axes[1,0].set_title("EC (µS/cm)")
axes[1,0].tick_params(axis='x', rotation=45)

axes[1,1].plot(timestamps, ph_vals, color="green")
axes[1,1].set_title("pH")
axes[1,1].tick_params(axis='x', rotation=45)

plt.tight_layout()
filename = f"soil_SEN0604_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
plt.savefig(filename, format="jpg", dpi=150)
plt.close()
print(f"Graph saved as {filename}")