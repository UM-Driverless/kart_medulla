#!/usr/bin/env python3
"""Simple serial monitor to stream steering angles from the ESP32.

Usage:
  python3 monitor_serial.py               # auto-detect port
  python3 monitor_serial.py /dev/cu.xxx   # specify port

Outputs the target and actual steering angles along with error, PID, and sensor status.
"""

import sys
import time
import serial
from serial.tools import list_ports


def autodetect_port() -> str:
    """Pick the first USB serial port if none specified."""
    ports = list_ports.comports()
    # Prefer USB serial style devices
    for p in ports:
        name = p.device.lower()
        if any(tok in name for tok in ("usbserial", "usbmodem", "ttyusb", "ttyacm")):
            return p.device
    # Fallback to any available port
    return ports[0].device if ports else ""


def open_serial(port: str, baud: int = 115200) -> serial.Serial:
    return serial.Serial(port, baudrate=baud, timeout=1)


def monitor(port: str):
    if not port:
        print("No serial ports found. Plug in the ESP32 and try again.")
        sys.exit(1)

    print(f"Connecting to {port} @ 115200...")
    try:
        with open_serial(port) as ser:
            ser.reset_input_buffer()
            while True:
                line = ser.readline()
                if not line:
                    continue
                if not line.startswith(b"DATA,"):
                    continue  # ignore non-data lines

                try:
                    parts = line.decode("utf-8", errors="ignore").strip().split(",")
                    if len(parts) < 14:
                        continue
                    target = float(parts[10])
                    actual = float(parts[11])
                    error = float(parts[12])
                    pid = float(parts[13])
                    status = parts[14] if len(parts) > 14 else ""
                    print(f"target={target:6.2f}°  actual={actual:6.2f}°  error={error:6.2f}°  pid={pid:5.3f}  {status}")
                except Exception as e:  # keep running on parse errors
                    print(f"Parse error: {e}")
    except Exception as e:
        print(f"ERROR: {e}")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else autodetect_port()
    monitor(port)


if __name__ == "__main__":
    main()

