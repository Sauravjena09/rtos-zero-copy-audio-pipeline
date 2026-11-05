import serial
import time

try:
    s = serial.Serial('COM3', 115200, timeout=2)
    s.dtr = True
    print("Opened COM3:")
    for _ in range(20):
        line = s.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(line)
    s.close()
except Exception as e:
    print(f"Error: {e}")
