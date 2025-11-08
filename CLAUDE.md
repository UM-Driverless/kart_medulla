# Claude Development Notes

## Repository Structure - CRITICAL CONVENTIONS

### Standard ESP-IDF Project Layout

**This is an ESP-IDF project with Arduino as a component, managed by PlatformIO.**

```
/Users/rubenayla/repos/kart_medulla/
├── main/                    # Source code (ESP-IDF convention)
│   ├── sketch.cpp          # Main Bluepad32 application
│   ├── main.c              # ESP-IDF entry point
│   └── CMakeLists.txt
│
├── components/              # Custom components
│   ├── motor_control/      # DAC/PWM motor controllers
│   ├── pid_controller/     # PID control logic
│   └── as5600_sensor/      # AS5600 magnetic encoder (disabled)
│
├── managed_components/      # ESP-IDF managed components
│   └── espressif__bluepad32/
│
├── patches/                 # Bluepad32 patches
├── CMakeLists.txt          # ESP-IDF project configuration
├── platformio.ini          # PlatformIO build configuration
├── sdkconfig.esp32dev      # ESP32 SDK configuration
├── dependencies.lock       # Locked component versions
└── controller_gui.py       # Python visualization GUI
```

### Critical Rules - DO NOT VIOLATE

1. **NEVER create a `src/` directory** - PlatformIO will compile it instead of `main/`
2. **NEVER nest ESP-IDF project in subdirectories** - Keep `main/`, `components/`, `CMakeLists.txt` at root level
3. **Source code location**: `main/sketch.cpp` (ESP-IDF convention, NOT `src/`)
4. **Build system**: `framework = espidf` (NOT `framework = arduino`)
5. **Source directory override**: `src_dir = main` (tells PlatformIO to use `main/` instead of `src/`)

### Build Configuration (`platformio.ini`)

```ini
[platformio]
src_dir = main  # CRITICAL: Use main/ directory, NOT src/

[env]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/54.03.21/platform-espressif32.zip
framework = espidf  # CRITICAL: ESP-IDF framework, NOT arduino
monitor_speed = 115200
monitor_filters = direct
```

### Historical Issue: Wrong Code Being Compiled

**Problem**: PS5 controller unable to pair due to I2C errors flooding serial output.

**Root Cause**: A legacy `src/` directory existed containing an old Arduino project. PlatformIO was compiling this old code instead of the Bluepad32 project in `main/`. All changes to `main/sketch.cpp` had zero effect because the wrong code was being compiled.

**Solution**:
1. Deleted `src/` directory (old Arduino project)
2. Moved ESP-IDF project from nested `esp32_code/` to root level
3. Updated `platformio.ini` with correct ESP-IDF configuration
4. Deleted redundant `esp32_code/` directory

**Lesson**: Repository must have ONLY ONE project structure. Never allow conflicting build systems.

## PID Tuning Results

### Optimal Values Found
- **Kp=10, Ki=0, Kd=0** - PASSED accuracy test
  - Average error: 2.6°
  - Max error: 4.4°
  - Good response without oscillation

### Test Results Summary
| Kp  | Ki   | Kd  | Avg Error | Max Error | Result   | Notes                 |
| --- | ---- | --- | --------- | --------- | -------- | --------------------- |
| 1.5 | 0.01 | 0   | 9.0°      | 19.9°     | FAIL     | Original - too weak   |
| 5   | 0    | 0   | 5.7°      | 9.0°      | FAIL     | Better but still weak |
| 10  | 0    | 0   | 2.6°      | 4.4°      | **PASS** | Good balance          |
| 10  | 0.5  | 0   | 84.0°     | 114.0°    | FAIL     | Integral windup       |
| 10  | 0.05 | 0   | 79.0°     | 109.0°    | FAIL     | Still windup          |

### Recommendations
1. Use **Kp=10** as base value
2. Avoid Ki unless anti-windup is implemented
3. Add small Kd (0.1-0.5) only if oscillation occurs
4. Battery voltage affects performance significantly

## Hardware Configuration

### AS5600 Steering Sensor - DISABLED

The AS5600 magnetic encoder sensor is **disabled in code** because the hardware is not yet physically connected.

**Files Modified**:
- `main/sketch.cpp:4` - Sensor include commented out
- `main/sketch.cpp:21` - Sensor object commented out
- `main/sketch.cpp:77-81` - Sensor reading disabled, `currentAngle` always returns 0.0°

**Important**: Do NOT uncomment sensor initialization until AS5600 hardware is physically connected. I2C polling without hardware causes errors that prevent PS5 controller pairing.

**To Re-enable** (when hardware is connected):
1. Uncomment `#include "as5600_sensor.h"` (line 4)
2. Uncomment `AS5600Sensor steeringSensor;` (line 21)
3. Uncomment sensor initialization in `setup()` (lines 165-171)
4. Uncomment sensor reading in `processGamepad()` (replace lines 77-78)

### Steering Angle Convention

**Coordinate System**: X forward, Y left, Z up (right-hand rule)

**Steering Mapping**:
- Left joystick **LEFT** (negative axisX) → **POSITIVE** angle (CCW rotation around Z-axis)
- Left joystick **RIGHT** (positive axisX) → **NEGATIVE** angle (CW rotation around Z-axis)

**Implementation** (`main/sketch.cpp:107`):
```cpp
targetAngle = -(float)axisX * MAX_STEERING_ANGLE_DEG / 511.0f;
// axisX: -511 (left) to +512 (right) → targetAngle: +45° to -45°
```

### Motor Control Hardware

| Motor    | GPIO                          | Type | Range         | Component       |
| -------- | ----------------------------- | ---- | ------------- | --------------- |
| Throttle | GPIO 26                       | DAC2 | 0-255 (8-bit) | `ThrottleMotor` |
| Brake    | GPIO 25                       | DAC1 | 0-255 (8-bit) | `BrakeMotor`    |
| Steering | GPIO 27 (PWM) + GPIO 14 (DIR) | LEDC | -1.0 to +1.0  | `SteeringMotor` |

## Serial Monitoring

### Cross-Platform Serial Ports

| Platform         | Port Pattern                     | Example                  |
| ---------------- | -------------------------------- | ------------------------ |
| **macOS**        | `/dev/cu.*`                      | `/dev/cu.usbserial-0001` |
| **Ubuntu/Linux** | `/dev/ttyUSB*` or `/dev/ttyACM*` | `/dev/ttyUSB0`           |
| **Windows**      | `COM*`                           | `COM3`                   |

**Finding Your Port**:
```bash
# macOS
ls /dev/cu.*

# Ubuntu/Linux
ls /dev/ttyUSB* /dev/ttyACM*

# Windows
mode  # Lists COM ports
```

### Serial Output Format

ESP32 sends CSV data every 20ms (50Hz):
```
DATA,LX,LY,RX,RY,L2,R2,BUTTONS,BATTERY,CHARGING,TARGET,ACTUAL,ERROR,PID,SENSOR_STATUS
```

Example:
```
DATA,-8,4,4,12,0,0,0x0000,176,0,0.0,0.0,0.00,0.000,NO_SENSOR
```

### Monitoring Tools

**Controller GUI** (real-time visualization):
```bash
python3 controller_gui.py                      # Auto-detect port
python3 controller_gui.py /dev/cu.usbserial-0001  # Specify port
```

**Simple Serial Monitor**:
```bash
python3 monitor_serial.py                      # Auto-detect port
python3 monitor_serial.py /dev/cu.usbserial-0001  # Specify port
```

**PlatformIO Monitor**:
```bash
pio device monitor  # Auto-detects port
```

## PS5 Controller (Bluepad32)

### Controller Pairing

1. Ensure ESP32 firmware is running (serial shows "System ready!")
2. Press and hold **PS button + SHARE button** simultaneously for 3 seconds
3. Controller LED will blink rapidly (pairing mode)
4. ESP32 serial output will show: `CALLBACK: Controller is connected, index=0`
5. Controller LED will turn solid color when paired

### Common Issues

#### Issue: Controller Turns Off After ~10 Seconds

**Symptom**: PS5 DualSense LEDs go completely off when idle.

**Root Cause**: `hasData()` function returns `false` when controller is idle, blocking firmware from processing gamepad data and preventing keepalive signals.

**Solution**: Remove `hasData()` checks - only check `isConnected()`:

```cpp
// ❌ WRONG - blocks data when controller is idle
void processGamepad(ControllerPtr ctl) {
    if (!ctl->isConnected() || !ctl->hasData()) return;  // BAD!
    // ...
}

// ✅ CORRECT - always process when connected
void processGamepad(ControllerPtr ctl) {
    if (!ctl->isConnected()) return;  // Only check connection
    // ...
}
```

**Files Fixed**:
- `main/sketch.cpp:72` - Removed `hasData()` check from `processGamepad()`
- `main/sketch.cpp:143` - Removed `hasData()` check from `processControllers()`

#### Issue: Controller Cannot Pair (Fast Blinking Blue)

**Symptom**: Controller stuck in pairing mode, serial shows I2C errors.

**Root Cause**: AS5600 sensor I2C polling without hardware connected floods error messages, blocking Bluetooth communication.

**Solution**: Disable AS5600 sensor in code (see "AS5600 Steering Sensor - DISABLED" section above).

### ESP32 Hardware Buttons

The ESP32 DevKit V1 has two buttons:

| Button   | Function        | Usage                                               |
| -------- | --------------- | --------------------------------------------------- |
| **EN**   | Enable/Reset    | Press once to restart ESP32 (exits bootloader mode) |
| **BOOT** | Bootloader Mode | Hold during power-on or reset to enter flash mode   |

**Common Issue**: After firmware upload, ESP32 may remain in bootloader mode showing "waiting for download". Press **EN** button once to restart and run firmware.

## Build and Upload Workflow

### Clean Build Process

When making code changes, follow this workflow:

1. **Clean build cache** (if problems occur):
```bash
rm -rf .pio/build/esp32dev
```

2. **Build and upload**:
```bash
# Using PlatformIO CLI
pio run --target upload --environment esp32dev

# OR use VS Code PlatformIO sidebar → Upload button
```

3. **Verify upload success** - Look for:
```
Writing at 0x00010000... (100 %)
Wrote 829600 bytes (or similar)
Hash of data verified.
```

4. **Reset ESP32** - Press **EN** button if needed

5. **Monitor serial output**:
```bash
python3 monitor_serial.py /dev/cu.usbserial-0001
```

6. **Verify firmware running** - Look for:
```
=== Kart Medulla System Initialization ===
Firmware: v4.2.0
System ready! Waiting for controller...
```

7. **Check for clean DATA packets** (no errors):
```
DATA,-8,4,4,12,0,0,0x0000,176,0,0.0,0.0,0.00,0.000,NO_SENSOR
```

### Troubleshooting Build Issues

| Problem                         | Solution                                         |
| ------------------------------- | ------------------------------------------------ |
| Upload hangs at "Connecting..." | Hold BOOT button, plug USB, release BOOT         |
| Changes not reflected           | Clean build: `rm -rf .pio/build/esp32dev`        |
| I2C errors in serial            | AS5600 sensor not disabled in code               |
| Controller won't pair           | Check for serial errors, verify firmware running |
| Wrong code compiling            | Verify NO `src/` directory exists                |

## Component Architecture

### Custom Components

All custom components follow ESP-IDF component structure:

```
components/<component_name>/
├── include/
│   └── <component_name>.h    # Public header
├── <component_name>.cpp       # Implementation
└── CMakeLists.txt            # Component build config
```

**Current Components**:
- `motor_control/` - DAC (throttle/brake) and PWM (steering) motor controllers
- `pid_controller/` - PID control logic for steering
- `as5600_sensor/` - I2C magnetic encoder sensor (disabled)

**Using Components in sketch.cpp**:
```cpp
#include "motor_control.h"
#include "pid_controller.h"
// #include "as5600_sensor.h"  // DISABLED
```

## Current System Status

### Working Features
- ✅ PS5 controller pairing via Bluepad32
- ✅ Joystick input reading (left/right sticks)
- ✅ Trigger input reading (L2/R2)
- ✅ Button state reading
- ✅ Battery level monitoring
- ✅ DAC throttle/brake control (GPIO 25/26)
- ✅ PWM steering motor control (GPIO 27/14)
- ✅ PID steering controller (Kp=10)
- ✅ Real-time controller GUI (`controller_gui.py`)
- ✅ CSV serial data output

### Disabled Features
- ❌ AS5600 steering angle sensor (hardware not connected)
- ❌ Closed-loop steering control (requires sensor)

### PID Configuration

**Current Values** (`main/sketch.cpp:34-36`):
```cpp
const float KP = 10.0f;  // Proportional gain
const float KI = 0.0f;   // Integral gain
const float KD = 0.0f;   // Derivative gain
```

**Tested and Verified**:
- Kp=10 provides good accuracy (avg error 2.6°, max error 4.4°)
- Ki should remain 0 to avoid integral windup
- Kd not needed unless oscillation occurs

**Note**: PID currently runs in open-loop mode (no sensor feedback). When AS5600 is connected, system will switch to closed-loop control.