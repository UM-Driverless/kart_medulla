# Claude Development Notes

## PID Tuning Results

### Optimal Values Found
- **Kp=10, Ki=0, Kd=0** - PASSED accuracy test
  - Average error: 2.6°
  - Max error: 4.4°
  - Good response without oscillation

### Test Results Summary
| Kp | Ki | Kd | Avg Error | Max Error | Result | Notes |
|----|----|----|-----------|-----------|--------|-------|
| 1.5 | 0.01 | 0 | 9.0° | 19.9° | FAIL | Original - too weak |
| 5 | 0 | 0 | 5.7° | 9.0° | FAIL | Better but still weak |
| 10 | 0 | 0 | 2.6° | 4.4° | **PASS** | Good balance |
| 10 | 0.5 | 0 | 84.0° | 114.0° | FAIL | Integral windup |
| 10 | 0.05 | 0 | 79.0° | 109.0° | FAIL | Still windup |

### Recommendations
1. Use **Kp=10** as base value
2. Avoid Ki unless anti-windup is implemented
3. Add small Kd (0.1-0.5) only if oscillation occurs
4. Battery voltage affects performance significantly

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