# Claude Development Notes

## Serial Monitoring Issues

### Problem: Terminal Hangs with Interactive Serial Monitors
When monitoring ESP32 serial output, the following commands cause the terminal to hang:
- `screen /dev/ttyUSB0 115200` - Creates persistent session that doesn't exit
- `platformio device monitor` - Expects TTY input, fails in non-interactive mode
- `picocom` - Requires interactive terminal

### Solution: Use Python with pyserial
Use a Python script with timeout to read serial data without blocking:

```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser.reset_input_buffer()

start_time = time.time()
while time.time() - start_time < 5:  # Read for 5 seconds
    if ser.in_waiting > 0:
        data = ser.readline()
        try:
            print(data.decode('utf-8').strip())
        except:
            print(data.hex())
    time.sleep(0.01)

ser.close()
```

This approach:
- Doesn't require TTY input
- Exits automatically after timeout
- Can be used in automated scripts
- Works in non-interactive environments