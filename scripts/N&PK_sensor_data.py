import minimalmodbus
import serial
import matplotlib.pyplot as plt
import time
from datetime import datetime

# --- Sensor Setup (Windows) ---
sensor = minimalmodbus.Instrument('COM5', 1)  # Change to right COM you are using
sensor.serial.baudrate = 9600
sensor.serial.bytesize = 8
sensor.serial.parity = serial.PARITY_NONE
sensor.serial.stopbits = 1
sensor.serial.timeout = 1
sensor.mode = minimalmodbus.MODE_RTU

def read_sensor():
    nitrogen   = sensor.read_register(0x1E, 0, 3)  # mg/kg
    phosphorus = sensor.read_register(0x1F, 0, 3)  # mg/kg
    potassium  = sensor.read_register(0x20, 0, 3)  # mg/kg
    return nitrogen, phosphorus, potassium

# --- Data Storage ---
timestamps = []
n_vals     = []
p_vals     = []
k_vals     = []

NUM_READINGS = 50 # number of readings 

print("Reading sensor...")
for i in range(NUM_READINGS):
    try:
        n, p, k = read_sensor()
        timestamps.append(datetime.now().strftime("%H:%M:%S"))
        n_vals.append(n)
        p_vals.append(p)
        k_vals.append(k)
        print(f"[{i+1}/{NUM_READINGS}] Nitrogen: {n} mg/kg | Phosphorus: {p} mg/kg | Potassium: {k} mg/kg")
    except Exception as e:
        print(f"Read error: {e}")
    time.sleep(1)  # time between readings

# --- Save Graph ---
fig, axes = plt.subplots(1, 3, figsize=(14, 5))
fig.suptitle("SEN0605 NPK Soil Sensor Readings", fontsize=14)

axes[0].plot(timestamps, n_vals, color="green")
axes[0].set_title("Nitrogen (mg/kg)")
axes[0].tick_params(axis='x', rotation=45)

axes[1].plot(timestamps, p_vals, color="orange")
axes[1].set_title("Phosphorus (mg/kg)")
axes[1].tick_params(axis='x', rotation=45)

axes[2].plot(timestamps, k_vals, color="blue")
axes[2].set_title("Potassium (mg/kg)")
axes[2].tick_params(axis='x', rotation=45)

plt.tight_layout()
filename = f"soil_NPK_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
plt.savefig(filename, format="jpg", dpi=150)
plt.close()
print(f"Graph saved as {filename}")