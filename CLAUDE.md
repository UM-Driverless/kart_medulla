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

### Important: ESP32 Serial Output Timing
**CRITICAL QUIRK:** ESP32 prints startup messages ONCE during boot, then goes silent unless events occur.

When monitoring serial:
1. **Don't panic if you see no output** - ESP32 only prints when something happens
2. **Startup messages appear immediately after reset** - If you connect after boot, you'll miss them
3. **To see startup messages:** Reset the ESP32 (DTR toggle or physical button) while monitoring
4. **Application runs silently** - Unless there are events (controller connects, errors, heartbeat messages)

Example of normal behavior:
```
[After upload, immediate connection shows:]
E (5007) ps5_L2CAP: L2CA_CONNECT_REQ ret=64
⏳ Waiting for controller...
[Then silence... this is NORMAL, not a problem!]
```

**Do NOT assume the code isn't working just because serial is quiet.** The ESP32 is event-driven.

### Preventing ESP32 Reset on Serial Port Open

**IMPORTANT:** By default, opening a serial port with Python's `pyserial` toggles DTR/RTS lines, which **resets the ESP32**. This causes the controller to disconnect and requires re-pairing.

**Solution:** Disable DTR/RTS toggling when opening serial ports:

```python
import serial

# WRONG - This will reset the ESP32
ser = serial.Serial('/dev/cu.usbserial-0001', 115200)

# CORRECT - This won't reset the ESP32
ser = serial.Serial(
    '/dev/cu.usbserial-0001',
    115200,
    timeout=1,
    dsrdtr=False,  # Don't toggle DTR (prevents ESP32 reset)
    rtscts=False   # Don't use hardware flow control
)
```

**All Python scripts that open the serial port MUST use these flags** to prevent unwanted ESP32 resets and controller disconnections.

This fix has been applied to:
- `controller_gui.py` - GUI monitor
- `monitor_serial.py` - Serial debug tool

## PS5 DualSense Controller Pairing (2025-11-02)

### SUCCESS: PS5 Controller Works on macOS with ESP32

**Controller:** DualSense Wireless Controller
**MAC Address:** `AC:36:1B:09:F0:28`
**Library:** ps5Controller @ 2.1.0 (BLACKROBOTICS fork)
**Platform:** macOS (development machine) + ESP32-WROOM

### Pairing Procedure (CRITICAL - Must Follow Exactly)

The controller must be properly reset and put into pairing mode:

#### Step 1: Reset Controller (Unpair from Previous Device)
**REQUIRED if controller was previously paired to PS5/PC/other device**

1. Locate tiny reset hole on BACK of controller (near L2 button)
2. Insert paperclip or pin
3. Press and hold for **5 seconds**
4. Controller will turn OFF completely
5. Light bar should be dark

#### Step 2: Enter Pairing Mode
1. Press and hold **SHARE + PS buttons** together
2. Keep holding for **3-5 seconds**
3. Look for **RAPID BLUE FLASHING** (multiple flashes per second)
4. This is TRUE pairing mode - not slow flashing!

**Light Bar States:**
- **Slow blue flash (every 2 sec)** = Already paired to something else → Need to reset
- **Rapid blue flashing (fast)** = Pairing mode → Correct!
- **Constant blue** = Connected successfully → Success!

#### Step 3: Verify Connection
After pairing, the ESP32 will show:
```
✓ Connected - Reading data...
  Left Stick X:    0
  Buttons: ---
```

Press buttons or move sticks to verify data is being received.

### Troubleshooting

**Problem:** Controller shows slow blue flashing
**Solution:** Controller is still paired to previous device. Must reset first (Step 1).

**Problem:** No connection after pairing
**Solution:**
1. Check MAC address in code matches controller: `AC:36:1B:09:F0:28`
2. Reset ESP32 (press physical reset button)
3. Reset controller and try pairing again

**Problem:** Serial output shows nothing
**Solution:** This is normal! ESP32 only prints on events. Press a button on the controller to trigger output.

### Key Learnings

1. **macOS works fine** - No need for Ubuntu/Linux for PS5 controller
2. **Controller reset is critical** - Must unpair from previous device first
3. **Pairing mode is specific** - Must see rapid flashing, not slow flashing
4. **Library works correctly** - ps5Controller library has no L2CAP issues when pairing is done correctly
5. **Serial is event-driven** - Quiet output doesn't mean failure

### Troubleshooting GUI Freezes

**If the GUI freezes or shows stale data:**
1. **Tap the PS button** on the controller to wake it up (controller may have gone to sleep)
2. **Restart the Python GUI script** (`python3 controller_gui.py`)

The controller goes into power-saving mode after a period of inactivity. Simply pressing the PS button wakes it up, and restarting the GUI re-establishes the connection. This is normal behavior.

## Remote Control Implementation (2025-11-02)

### Acceleration Control via DAC

**Hardware Setup:**
- The kart uses an AliExpress electronic kit that expects an analog voltage (0-3.3V) from a hall sensor pedal
- ESP32 GPIO 25 (DAC1) outputs analog voltage to simulate the pedal sensor
- Direct connection: GPIO 25 → Kart's acceleration input

**Implementation:**
```cpp
const int ACCEL_DAC_PIN = 25;  // GPIO 25 (DAC1)

// In loop() when controller connected:
uint8_t r2Value = ps5.R2Value();  // 0-255
dacWrite(ACCEL_DAC_PIN, r2Value); // Output 0-3.3V analog
```

**Behavior:**
- R2 trigger position (0-255) directly controls DAC output voltage
- 0% R2 = 0V (no acceleration)
- 100% R2 = 3.3V (full acceleration)
- Controller disconnect automatically sets output to 0V for safety

**Testing Procedure:**
1. Upload code to ESP32
2. Use multimeter to measure voltage on GPIO 25:
   - R2 released: 0V
   - R2 half-pressed: ~1.65V
   - R2 fully pressed: 3.3V
3. Connect GPIO 25 to kart's pedal sensor input
4. Test acceleration response

**Safety:**
- Output automatically set to 0V when controller disconnects
- No deadman switch or emergency stop implemented yet (kept simple)
- Future enhancements can add L1/R1 deadman switch and PS button emergency stop

**Steering:**
- Not yet implemented
- Future: Left stick X will control steering via separate motor driver
- AS5600 sensor on I2C (GPIO 21/22) ready for angle feedback