# Agent Development Notes

## Repository Structure

**ESP-IDF project with PlatformIO. Source in `main/`, custom libraries in `components/`.**

```
kart_medulla/
├── main/
│   ├── main.c              # ESP-IDF entry point (app_main + FreeRTOS tasks)
│   ├── sketch.cpp          # Legacy Bluepad32 gamepad app (NOT used by main.c)
│   └── CMakeLists.txt
├── components/
│   ├── km_act/             # Actuator control (DAC for throttle/brake, PWM+DIR for steering)
│   ├── km_coms/            # UART framed protocol (Orin ↔ ESP32)
│   ├── km_gpio/            # GPIO/DAC/PWM abstraction
│   ├── km_objects/         # Shared variable store (thread-safe get/set)
│   ├── km_pid/             # PID controller
│   ├── km_rtos/            # FreeRTOS task manager
│   ├── km_sdir/            # AS5600 steering angle sensor (I2C)
│   ├── km_sta/             # State machine
│   └── bluepad32/          # Gamepad library (used by sketch.cpp only)
├── platformio.ini          # Build config
└── sdkconfig.esp32dev      # ESP32 SDK config
```

### Critical Rules

1. **NEVER create a `src/` directory** — PlatformIO compiles it instead of `main/`
2. **Framework is `espidf`** (NOT arduino)
3. **`src_dir = main`** in platformio.ini tells PIO to use `main/`

## Flashing

```bash
cd ~/Desktop/kart_medulla && ~/.local/bin/pio run --target upload --environment esp32dev
```

- **Upload baud must be 115200** — the CP2102 USB-UART bridge fails at higher speeds during flash (460800 works fine for runtime comms, just not for esptool upload)
- `upload_speed = 115200` is set in `platformio.ini`
- Runtime UART baud is 460800 (set in `km_coms.c` KM_COMS_Init)
- If flash hangs at "Connecting...", hold BOOT button, press EN, release BOOT
- After flash, press EN to restart if needed

## Architecture (main.c)

Three FreeRTOS tasks:

| Task        | Period | Priority | Function                                              |
|-------------|--------|----------|-------------------------------------------------------|
| comms       | 10ms (100 Hz) | 2 | UART RX/TX — receive commands from Orin, send telemetry |
| control     | 10ms (100 Hz) | 1 | Read AS5600 sensor, run PID, drive actuators, send steering feedback |
| heartbeat   | 1000ms (1 Hz) | 1 | Send heartbeat to Orin                                |

### UART Protocol (`km_coms`)

Frame format: `| SOF(0xAA) | LEN | TYPE | PAYLOAD... | CRC8 |`

- UART0 @ 460800 baud — protocol comms with Orin (via CP2102 USB)
- UART2 — debug log output (ESP_LOG redirected here so UART0 stays clean)
- CRC8 poly 0x07 over LEN + TYPE + PAYLOAD

**Message types (Orin → ESP32):**
| Type | ID   | Payload |
|------|------|---------|
| ORIN_TARG_THROTTLE  | 0x20 | u8 [0-255] |
| ORIN_TARG_BRAKING   | 0x21 | u8 [0-255] |
| ORIN_TARG_STEERING  | 0x22 | int16 big-endian, radians × 1000 |
| ORIN_HEARTBEAT      | 0x25 | (empty) |
| ORIN_COMPLETE       | 0x27 | 7 bytes: throttle, brake, steering(2), mission, state, shutdown |

**Message types (ESP32 → Orin):**
| Type | ID   | Payload |
|------|------|---------|
| ESP_ACT_STEERING | 0x04 | int16 big-endian, radians × 1000 (actual angle from AS5600) |
| ESP_HEARTBEAT    | 0x08 | 4 bytes (0xDEADBEEF) |

### Steering Pipeline

1. Orin sends `ORIN_TARG_STEERING` (target angle in radians × 1000)
2. `control_task` reads target from `km_objects` store
3. AS5600 sensor read via I2C (400kHz, 5ms timeout) → actual angle in radians
4. PID computes output (Kp=0.03, Ki=0, Kd=0.0004) → [-1.0, 1.0]
5. `km_act` drives PWM (magnitude) + DIR pin (sign) on steering motor
6. Actual angle sent back to Orin as `ESP_ACT_STEERING`

### Hardware

| Actuator | GPIO | Type | Notes |
|----------|------|------|-------|
| Throttle | 26   | DAC2 | 0-255 output |
| Brake    | 25   | DAC1 | 0-255 output |
| Steering PWM | 27 | LEDC PWM | duty 0-255 |
| Steering DIR | 14 | Digital | 1=positive, 0=negative |
| AS5600 SDA | 21 | I2C | 400kHz, addr 0x36 |
| AS5600 SCL | 22 | I2C | |

### Safety

- Steering motor limited to **40%** output (`KM_ACT_SetLimit(&dir_act, 0.4)`) for testing
- Increase to 1.0 when system is validated
- ESP32 watchdog: if no Orin heartbeat received, should apply brakes (TODO — not yet implemented)

## Debugging

- **Debug logs**: connect to UART2 pins (see `km_gpio.h` for PIN_ORIN_UART_TX/RX) at 460800 baud
- **Protocol monitor**: `python3 monitor_serial.py` on /dev/ttyUSB0
- **ROS2 side**: `ros2 topic echo /esp32/steering` to see feedback from ESP32
