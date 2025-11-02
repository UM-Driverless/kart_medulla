# Kart Medulla - ESP32 Go-Kart Controller

ESP32-based control system for electric go-kart with PS5 DualSense controller input and PID steering control.

## Hardware

- **Microcontroller:** ESP32 WROOM 32
- **Sensor:** AS5600 Magnetic Angle Sensor (I²C, address: `0x36`)
- **Motor Driver:** MD30C 30A Motor Controller
- **Controller:** PS5 DualSense Wireless (Bluetooth)
- **MCU:** ESP32-D0WD-V3 (Dual-core, 4MB Flash, 520KB RAM)

## Wiring Connections

### ESP32 to AS5600 Angle Sensor

| AS5600 Pin | ESP32 Pin | Wire Color |
|------------|-----------|------------|
| SCL        | GPIO 22   | Blue       |
| SDA        | GPIO 21   | Green      |
| VCC        | 3.3V      | White      |
| GND        | GND       | Grey       |

### ESP32 to Motor Driver (MD30C)

| Motor Driver Pin | ESP32 Pin |
|------------------|-----------|
| PWM              | GPIO 25   |
| DIR              | GPIO 26   |
| VCC              | 5V        |
| GND              | GND       |

### ESP32 to Orin (UART)

| Orin Pin | ESP32 Pin |
|----------|-----------|
| TX       | GPIO 18   |
| RX       | GPIO 19   |
| GND      | GND       |

### Other Pins

- **LED:** GPIO 2 (Onboard LED)

## Quick Start

### 1. Build and Upload

```bash
~/.platformio/penv/bin/pio run --target upload
```

Or add PlatformIO to PATH and use:
```bash
pio run --target upload
```

### 2. Pair PS5 Controller

**First time / Re-pairing:**

1. **Reset controller** (unpair from previous device):
   - Find tiny reset hole on BACK of controller (near L2)
   - Press with paperclip for 5 seconds
   - Controller turns OFF

2. **Enter pairing mode**:
   - Hold SHARE + PS buttons together for 3-5 seconds
   - Look for RAPID blue flashing (multiple times per second)
   - Not slow flashing!

3. **Verify connection**:
   - Light turns constant blue when connected
   - Monitor serial output to see controller data

**Controller MAC Address:** `AC:36:1B:09:F0:28`

**Light Bar States:**
- Slow blue flash (every 2 sec) = Already paired → Need to reset
- Rapid blue flashing = Pairing mode → Correct!
- Constant blue = Connected → Success!

### 3. Monitor Serial Output

**Using Python script** (recommended):
```bash
python3 monitor_serial.py
```

**Note:** `platformio device monitor` doesn't work in non-interactive environments. Use the Python script included in the repo.

### 4. PS5 Controller GUI Monitor

**Real-time graphical visualization of controller inputs:**

```bash
python3 controller_gui.py
```

**Features:**
- Live visualization of analog sticks with crosshairs
- Trigger pressure bars (L2/R2)
- Button press indicators (face buttons, D-pad, shoulder buttons)
- Real-time numeric values
- Connection status and debug info
- 50Hz refresh rate for smooth updates

**Requirements:**
- Python 3 with `tkinter`, `pyserial`, and `threading` (usually pre-installed)
- ESP32 connected via USB

The GUI automatically connects to `/dev/cu.usbserial-0001` at 115200 baud. It shows debug serial output when waiting for the controller to connect and switches to the full visualization once paired.

## PID Tuning

### Current Optimal Values

- **Kp = 10**
- **Ki = 0**
- **Kd = 0**

**Performance:**
- Average error: 2.6°
- Max error: 4.4°
- No oscillation

See `CLAUDE.md` for detailed tuning results and recommendations.

### Manual PID Testing via Serial

You can tune PID values without recompiling:

```
kp=20        # Set proportional gain
ki=0         # Set integral gain
kd=0         # Set derivative gain
sequence     # Run full test sequence (0°, ±10°, ±20°, ±30°)
steady=15    # Run steady-state test at 15°
mode=2       # Set to constant angle mode
amp=10       # Set target angle to 10°
help         # Show all commands
```

**Tuning Guidelines:**
- Start with P-only: `ki=0` and `kd=0`
- If error too high: Increase `kp`
- If oscillating: Decrease `kp` or add small `kd` (0.1-0.5)
- If steady-state error: Add small `ki` (0.01-0.1)
- Target: Average error < 3°, Max error < 5°

## Controller Mapping

```
Left Stick X  → Steering angle (-30° to +30°)
R2 Trigger    → Throttle (0-100%)
L2 Trigger    → Brake (0-100%)
```

## Project Structure

```
├── src/
│   └── main.cpp          # Main application code
├── lib/                  # Project libraries
├── platformio.ini        # PlatformIO configuration
├── controller_gui.py     # PS5 controller GUI monitor (real-time visualization)
├── monitor_serial.py     # Serial monitoring helper
├── CLAUDE.md            # Development notes and tuning data
├── stuff.md             # Build troubleshooting notes
└── stuff/               # Archive of old documentation
```

## Reference Systems

- **Body reference system:** X is forward, Y is left, Z is up
- **Steering angle:** 0 rad is straight ahead, positive turns left, negative turns right
- **AS5600 output:** Positive angle when magnet rotates with vector pointing to PCB side without chip

## Development Notes

**See `CLAUDE.md` for:**
- PID tuning results and test data
- PS5 controller pairing procedures
- Serial monitoring tips
- ESP32 behavior quirks

**See `stuff.md` for:**
- Build system issues (ninja, CMake, miniforge conflicts)
- Toolchain configuration
- macOS-specific troubleshooting

## Platform

Developed and tested on **macOS**. No Linux/Ubuntu required for PS5 controller support.

## Chip Information

**ESP32 WROOM 32:**
- Dual-core Tensilica LX6
- 4MB Flash, 520KB RAM
- Operating Voltage: 3.3V
- Built-in Wi-Fi & Bluetooth

**AS5600:**
- 12-bit Magnetic Angle Sensor
- I²C Interface (address: `0x36`)
- 360° contactless measurement
