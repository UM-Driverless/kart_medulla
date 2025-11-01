#!/usr/bin/env python3
import serial
import time

ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)
ser.reset_input_buffer()

print("Monitoring ESP32 serial output... (Ctrl+C to stop)\n")

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline()
            try:
                print(data.decode('utf-8').strip())
            except:
                print(data.hex())
        time.sleep(0.01)
except KeyboardInterrupt:
    print("\n\nStopping monitor...")
finally:
    ser.close()
