# Kart Medulla - ESP32 Controller System

ESP32-based control system for autonomous go-kart using Bluepad32 for PS4/PS5 controller input, with PID-controlled steering and analog throttle/brake output.

- Hardware docs: https://um-driverless.github.io/kart_docs/assembly/electronics/kart-medulla/
- PCB design: https://u.easyeda.com/account/user/projects/index/detail?project=90f6158fb62a4e79af68560e3b41af01&listType=all
## Hardware Overview

* **Microcontroller:** ESP32 WROOM 32
* **Controller Input:** PS4/PS5 DualShock/DualSense via Bluetooth (Bluepad32)
* **Steering Sensor:** AS5600 Magnetic Angle Sensor (currently disabled - hardware not connected)
* **Motor Control:**
  - DAC analog outputs for throttle/brake
  - PWM/DIR outputs for steering motor

## Pin Configuration

### Motor Control Outputs

| Function | ESP32 Pin | Type | Range | Description |
|----------|-----------|------|-------|-------------|
| **Throttle (CMD_ACC)** | GPIO 25 | DAC1 | 0-255 (8-bit) | Analog throttle output |
| **Brake (CMD_BRAKE)** | GPIO 26 | DAC2 | 0-255 (8-bit) | Analog brake output |
| **Steering PWM** | GPIO 18 | PWM (LEDC) | 0-255 | Steering motor speed |
| **Steering DIR** | GPIO 19 | Digital | 0/1 | Steering motor direction |

### Sensor Inputs (AS5600 - Currently Disabled)

| AS5600 Pin | ESP32 Pin | Signal |
|------------|-----------|--------|
| SCL | GPIO 22 | I2C Clock |
| SDA | GPIO 21 | I2C Data |
| VCC | 3.3V | Power |
| GND | GND | Ground |

**Note:** AS5600 sensor is disabled in firmware (`main/sketch.cpp:4`) until hardware is physically connected.

### Other Pins

* **LED:** GPIO 2 (Onboard LED)
* **USB Serial (UART0):** GPIO 1 (TX0), GPIO 3 (RX0) - reserved for console

## ESP32-DevKitC V4 Header Pin Map

Numbering used below:
- USB-C at the bottom, H1 is the right header. H1 pin 1 is bottom-right and counts up to pin 19 at top-right.
- H2 is the left header. H2 pin 1 is top-left and counts down to pin 19 at bottom-left.
- If you count around the chip, H2 pin 1 corresponds to logical pin 20.

| ESP32 Pin # | Header Pin | ESP32 Pin | Kart Net | Notes |
|-------------|------------|-----------|----------|-------|
| 1 | H1 pin 1 | GPIO 6 (SCK) | RESERVED (FLASH/SDIO) | Do not use |
| 2 | H1 pin 2 | GPIO 7 (SD0) | RESERVED (FLASH/SDIO) | Do not use |
| 3 | H1 pin 3 | GPIO 8 (SD1) | RESERVED (FLASH/SDIO) | Do not use |
| 4 | H1 pin 4 | GPIO 15 (MTDO) | NC (strap) | Boot config risk |
| 5 | H1 pin 5 | GPIO 2 | NC (strap) | Boot config risk |
| 6 | H1 pin 6 | GPIO 0 (BOOT) | NC (strap) | Boot mode pin |
| 7 | H1 pin 7 | GPIO 4 | NC (strap) | Boot config risk |
| 8 | H1 pin 8 | GPIO 16 | MOTOR_HALL_3 | Input |
| 9 | H1 pin 9 | GPIO 17 | MOTOR_HALL_1 | Input |
| 10 | H1 pin 10 | GPIO 5 (VSPI_SS) | NC (strap) | Boot config risk |
| 11 | H1 pin 11 | GPIO 18 (VSPI_SCK) | CMD_STEER_PWM | PWM output |
| 12 | H1 pin 12 | GPIO 19 (VSPI_MISO) | CMD_STEER_DIR | Digital output |
| 13 | H1 pin 13 | GND | GND | Ground |
| 14 | H1 pin 14 | GPIO 21 (SDA) | STEER_SDA (I2C) | I2C data |
| 15 | H1 pin 15 | GPIO 3 (U0RXD) | RESERVED (USB UART0 RX) | Console UART |
| 16 | H1 pin 16 | GPIO 1 (U0TXD) | RESERVED (USB UART0 TX) | Console UART |
| 17 | H1 pin 17 | GPIO 22 (SCL) | STEER_SCL (I2C) | I2C clock |
| 18 | H1 pin 18 | GPIO 23 (VSPI_MOSI) | SPARE_3V3 | Spare GPIO |
| 19 | H1 pin 19 | GND | GND | Ground |
| 20 | H2 pin 1 | 3V3 | 3V3 | Power |
| 21 | H2 pin 2 | EN | EN | Reset/enable |
| 22 | H2 pin 3 | GPIO 36 (SVP) | PRESSURE_1 (0-3V3) | ADC input only |
| 23 | H2 pin 4 | GPIO 39 (SVN) | PRESSURE_2 (0-3V3) | ADC input only |
| 24 | H2 pin 5 | GPIO 34 | PRESSURE_3 (0-3V3) | ADC input only |
| 25 | H2 pin 6 | GPIO 35 | PEDAL_ACC (0-3V3) | ADC input only |
| 26 | H2 pin 7 | GPIO 32 | PEDAL_BRAKE (0-3V3) | ADC input |
| 27 | H2 pin 8 | GPIO 33 | MOTOR_HALL_2 | Input |
| 28 | H2 pin 9 | GPIO 25 (DAC1) | CMD_ACC (0-3V3) | DAC output |
| 29 | H2 pin 10 | GPIO 26 (DAC2) | CMD_BRAKE (0-3V3) | DAC output |
| 30 | H2 pin 11 | GPIO 27 | HYDRAULIC_1 (0-3V3) | GPIO |
| 31 | H2 pin 12 | GPIO 14 | HYDRAULIC_2 (0-3V3) | GPIO |
| 32 | H2 pin 13 | GPIO 12 (MTDI) | NC (strap) | Boot config risk |
| 33 | H2 pin 14 | GND | GND | Ground |
| 34 | H2 pin 15 | GPIO 13 (MTCK) | SDC_NOT_EMERGENCY | Digital input |
| 35 | H2 pin 16 | GPIO 9 (SD2) | RESERVED (FLASH/SDIO) | Do not use |
| 36 | H2 pin 17 | GPIO 10 (SD3) | RESERVED (FLASH/SDIO) | Do not use |
| 37 | H2 pin 18 | GPIO 11 (CMD) | RESERVED (FLASH/SDIO) | Do not use |
| 38 | H2 pin 19 | 5V | 5V | Power |

## System Architecture

### Framework & Components

```
kart_medulla/
├── main/
│   ├── sketch.cpp              # Main application (Bluepad32 + PID control)
│   └── main.c                  # ESP-IDF entry point
├── components/
│   ├── motor_control/          # DAC/PWM motor controllers
│   ├── pid_controller/         # PID control logic
│   └── as5600_sensor/          # I2C magnetic encoder (disabled)
├── managed_components/         # ESP-IDF dependencies (Bluepad32, Arduino)
├── CMakeLists.txt             # ESP-IDF project config
└── platformio.ini             # Build configuration
```

**Framework:** ESP-IDF 5.x with Arduino as component
**Build System:** PlatformIO
**Controller Library:** Bluepad32 (Bluetooth Classic HID)

### Current Features

- ✅ PS4/PS5 controller pairing via Bluepad32
- ✅ Real-time joystick input (left/right sticks)
- ✅ Trigger input (L2/R2) → Brake/Throttle DAC output
- ✅ PID steering control (Kp=10, verified optimal)
- ✅ PWM steering motor control
- ✅ CSV serial data output (50Hz)
- ✅ Real-time visualization GUI (`controller_gui.py`)
- ✅ Battery level monitoring
- ❌ AS5600 sensor (disabled - hardware not connected)

## Software Setup

### Prerequisites

1. **PlatformIO** - Install via VS Code extension or CLI:
   ```bash
   pip install platformio
   ```

2. **Python 3** (for GUI) - with `pyserial` and `tkinter`:
   ```bash
   pip install pyserial
   # tkinter usually included with Python
   ```

### Building and Flashing

```bash
# Build firmware
pio run --environment esp32dev

# Upload to ESP32
pio run --target upload --environment esp32dev

# Build + Upload + Monitor (one command)
pio run --target upload --environment esp32dev && pio device monitor
```

**Common Issue:** If ESP32 stays in bootloader mode after upload, press the **EN (reset)** button once.

### Serial Monitoring

**Simple Monitor** (raw CSV data):
```bash
python3 monitor_serial.py                      # Auto-detect port
python3 monitor_serial.py /dev/cu.usbserial-0001  # Specify port
```

**Controller GUI** (real-time visualization):
```bash
python3 controller_gui.py                      # Auto-detect port
python3 controller_gui.py /dev/cu.usbserial-0001  # Specify port
```

**PlatformIO Monitor:**
```bash
pio device monitor  # 115200 baud, auto-detects port
```

### Cross-Platform Serial Ports

| Platform | Port Pattern | Example |
|----------|--------------|---------|
| macOS | `/dev/cu.*` | `/dev/cu.usbserial-0001` |
| Linux | `/dev/ttyUSB*` or `/dev/ttyACM*` | `/dev/ttyUSB0` |
| Windows | `COM*` | `COM3` |

## Controller Pairing

### PS5 DualSense / PS4 DualShock

1. **Ensure ESP32 is running** - Serial should show:
   ```
   === Kart Medulla System Initialization ===
   System ready! Waiting for controller...
   ```

2. **Put controller in pairing mode:**
   - Hold **PS button + SHARE button** for 3 seconds
   - LED will flash rapidly (blue for PS5, white for PS4)

3. **Wait for pairing:**
   ```
   CALLBACK: Controller is connected, index=0
   Controller model: PS5 DualSense, VID=0x054c, PID=0x0ce6
   ```

4. **Controller LED turns solid** - Paired successfully!

### Supported Controllers

- PS5 DualSense
- PS4 DualShock 4
- Xbox One/Series controllers
- Nintendo Switch Pro Controller
- Generic Bluetooth gamepads

All use the same API - no code changes needed to switch controllers.

## Serial Data Format

ESP32 outputs CSV data every 20ms (50Hz):

```csv
DATA,LX,LY,RX,RY,L2,R2,BUTTONS,BATTERY,CHARGING,TARGET,ACTUAL,ERROR,PID,SENSOR_STATUS
```

**Example:**
```
DATA,-8,4,4,12,0,512,0x0100,176,0,15.2,0.0,15.20,0.523,NO_SENSOR
```

**Fields:**
- `LX,LY` - Left stick X/Y (-511 to +512)
- `RX,RY` - Right stick X/Y (-511 to +512)
- `L2,R2` - Trigger values (0-1023)
- `BUTTONS` - Button bitmask (hex)
- `BATTERY` - Battery level (0-255)
- `TARGET` - Target steering angle (degrees)
- `ACTUAL` - Current steering angle (degrees, requires sensor)
- `ERROR` - Steering error (TARGET - ACTUAL)
- `PID` - PID controller output (-1.0 to +1.0)
- `SENSOR_STATUS` - `NO_SENSOR` or `SENSOR_OK`

## Controller GUI

Real-time visualization of controller inputs:

```bash
python3 controller_gui.py
```

**Features:**
- Left/Right joystick position visualizers
- L2/R2 trigger bars with percentages
- Button state display (all buttons)
- Steering angle gauge (target vs actual)
- PID output visualization
- Battery level indicator
- Sensor status

**Performance:** 20 FPS update rate with smart canvas redrawing for smooth visualization.

## Steering Control

### Coordinate System

- **Body Frame:** X forward, Y left, Z up (right-hand rule)
- **Steering Convention:**
  - Left joystick **LEFT** (negative axisX) → **Positive** angle (CCW rotation around Z)
  - Left joystick **RIGHT** (positive axisX) → **Negative** angle (CW rotation around Z)
  - Range: ±45° (configurable in `sketch.cpp:39`)

### PID Configuration

**Current Values** (`main/sketch.cpp:34-36`):
```cpp
const float KP = 10.0f;  // Proportional gain (verified optimal)
const float KI = 0.0f;   // Integral gain (disabled)
const float KD = 0.0f;   // Derivative gain (not needed)
```

**Tuning Results:**
- Kp=10 provides good accuracy (avg error 2.6°, max error 4.4°)
- Ki should remain 0 to avoid integral windup
- Kd not needed unless oscillation occurs

**Note:** PID currently runs in **open-loop mode** (no sensor feedback). When AS5600 is connected, system will switch to closed-loop control.

## Enabling AS5600 Sensor

When AS5600 hardware is physically connected:

1. **Uncomment sensor code** in `main/sketch.cpp`:
   ```cpp
   #include "as5600_sensor.h"  // Line 4
   AS5600Sensor steeringSensor;  // Line 21
   ```

2. **Uncomment sensor initialization** in `setup()` (lines 165-171)

3. **Uncomment sensor reading** in `processGamepad()` (replace lines 77-78)

4. **Clean build and upload:**
   ```bash
   rm -rf .pio/build/esp32dev
   pio run --target upload --environment esp32dev
   ```

**Important:** DO NOT enable sensor code without hardware connected - I2C errors will flood serial output and prevent controller pairing.

## Troubleshooting

### Controller Won't Pair

1. **Check serial output** - Should show "System ready! Waiting for controller..."
2. **Verify no I2C errors** - If sensor code is enabled without hardware
3. **Reset ESP32** - Press EN button
4. **Try different controller** - PS4/PS5/Xbox all supported

### Controller Disconnects When Idle

**Symptom:** Controller LEDs turn off after ~10 seconds of no input.

**Solution:** Already fixed in firmware - removed `hasData()` checks that blocked keepalive signals.

### Build Fails or Wrong Code Compiling

**Check these:**
- No `src/` directory exists (should only have `main/`)
- `platformio.ini` has `framework = espidf` (NOT `arduino`)
- `platformio.ini` has `src_dir = main`
- Clean build cache: `rm -rf .pio/build/esp32dev`

### Serial Port Issues

**macOS:**
```bash
ls /dev/cu.*  # Should show /dev/cu.usbserial-XXXX
```

**Linux:**
```bash
ls /dev/ttyUSB* /dev/ttyACM*
sudo usermod -a -G dialout $USER  # Add user to dialout group
# Log out and back in after this
```

**Windows:**
```cmd
mode  # Lists COM ports
```

## Development Notes

See [`AGENTS.md`](AGENTS.md) for comprehensive documentation:
- Repository structure and critical conventions
- Build system configuration
- Common issues and solutions
- PID tuning history
- Hardware configuration details

## ESP32 Hardware Buttons

| Button | Function | Usage |
|--------|----------|-------|
| **EN** | Reset | Press once to restart ESP32 |
| **BOOT** | Bootloader | Hold during power-on/reset to enter flash mode |

## License

This project is part of the UM Driverless autonomous kart system.

## Contributing

When making changes:
1. Never create a `src/` directory (use `main/` only)
2. Keep `framework = espidf` in `platformio.ini`
3. Test with clean build: `rm -rf .pio/build && pio run`
4. Verify controller pairing works after firmware updates
5. Document any pin changes in this README

## Contact

For issues or questions, see the UM Driverless team.
