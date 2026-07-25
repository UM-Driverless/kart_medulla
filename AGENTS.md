# Agent Development Notes

## Hardware is ESP32-S3 (READ THIS FIRST)

**The physical bench/kart board is an ESP32-S3.** The firmware in this repo now fully builds for the ESP32-S3 (`platformio.ini` uses `[env:esp32-s3-devkitc-1]`; `components/km_gpio/km_gpio.h` uses the `CONFIG_IDF_TARGET_ESP32S3` pin map). The legacy classic ESP32 build (`esp32dev`) is still kept for fallback purposes, but the S3 is the primary target.

- **Authoritative pin map for the real board:** [`.agents/esp32s3-pinmap.md`](.agents/esp32s3-pinmap.md). Key pins that differ from the classic map:
  - **I²C (AS5600 steering encoder):** SDA = **GPIO 8**, SCL = **GPIO 9** (classic map says 21/22).
  - **AS5600 PWM angle output** is read on **GPIO 1** (the ex-`PRESSURE_3` terminal CN5.2). This is the pin we use to read the steering-sensor PWM.
  - **EBS compressor MOSFET** is on **GPIO 3** (the ex-`BUZZER` net, now `CMD_COMPRESSOR_PWM`, CN8.2).
- **USB bridge is a WCH CH343** (VID 0x1A86 / PID 0x55D3) → shows up as `/dev/cu.usbmodem*`. This does NOT mean native-USB / does NOT tell you classic-vs-S3 on its own. Serial goes over UART0 through the bridge, so bench builds use **`ARDUINO_USB_CDC_ON_BOOT=0`** (see `history.md` / error-log on Mac bench flashing).
- When bench-testing the steering sensor: talk to the AS5600 over **I²C on GPIO 8/9** (gives RAW angle + MD/ML/MH status flags); the GPIO 1 path is the production PWM read.

## Branch Workflow (READ THIS)

**All day-to-day work happens on `dev`.** `main` is a protected release branch — it only receives merges from `dev` (or feature branches) *after* the change has been physically validated on the kart. Same convention as `kart-brain` and the other UM-Driverless repos.

- **Default working branch on the Mac and the Orin is `dev`.** Every `git checkout` / `git pull` should be on `dev` unless you have a specific reason (e.g. inspecting `main`).
- **Commit and push to `dev` first**, every time. Never push directly to `main` — the protection rule will reject you and create cleanup work.
- **Merge `dev` → `main` only after the ESP32 has been flashed and driven**, and the change has been confirmed working on the kart. Open a PR (`gh pr create --base main --head dev`), validate, merge. Admins can bypass approval but peer review is preferred.
- **If you discover `main` is ahead of `dev`** (someone pushed straight to main, or a merge bypassed `dev`), merge `main` into `dev` immediately before adding new commits so `dev` stays the "latest + in-progress" snapshot.
- **Flashing the ESP32 does NOT replace the validation step.** A flash puts code on the chip; validation means the kart actually drove with it and nothing regressed. Only then does `main` move.

## Sensor Validity (READ THIS)

**A sensor that is absent, failed, or out of range must not produce a number.** This firmware is the *source*
of every reading, so this is where invalidity has to be representable — patching it at the dashboard only fixes
one consumer, and the steering PID is another. Live example, 2026-07-25: with nothing on the I²C bus,
`KM_SDIR_ReadRaw()` returns `lastRawValue` (0) on failure, `KM_SDIR_ReadAngle()` converts that to
`-(0 - 2250)/4096 x 2pi = 3.451 rad`, and the kart dashboard drew a confident **90 deg LEFT** on a sensor that was
not plugged in. Raw 0 is a legal encoder position, so nothing downstream could tell. `KM_SDIR_isConnected()`
exists and was never consulted.

Return `NAN` (or a validity flag) when the read failed, and check it before publishing or feeding a controller.
Verify by unplugging the sensor and looking at the output — not by reading the code.

## Files
- `tasks.md` (repo root) — **the repo's task board, the only one.** Read it before starting work.
  Only Rubén moves a task to Done; on this repo that gate means flashed *and* driven, per the branch
  workflow above. (Replaced the old `TODO.md` on 2026-07-16.)
- `.agents/error-log.md` — **consult selectively** (grep for relevant entries before working on an area)
- `history.md` (repo root, NOT `.agents/`) — **MANDATORY: All progress, issues encountered, how they were solved, and what was learned MUST be noted here** (append-only, newest at the end)
- `.agents/adding-messages.md` — **reference** (read when adding new message types)

## Repository Structure

**ESP-IDF project with PlatformIO. Source in `main/`, custom libraries in `components/`.**

```
kart-medulla/
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
4. **Always check return values from hardware write functions.** Silent failures (like `ESP_ERR_INVALID_ARG` from DAC writes) waste hours. Log or assert on error returns.
5. **Debug pipelines with binary search, not end-to-end.** Find a test that splits the pipeline in half — the result tells you which half has the bug, guaranteeing progress. For hardware issues, hardcode output at the boundary (e.g., `dac_output_voltage()` directly in `main.c`) to isolate software vs hardware.

## Flashing

**Flash from the Orin** (the Mac has no toolchain — see the stale-path warning below):
```bash
ssh orin-remote 'echo 0 | sudo -S systemctl stop kart-brain'   # KB_Coms_micro holds the port
cd ~/kart_medulla && ~/.local/bin/pio run -e esp32-s3-devkitc-1 --target upload --upload-port /dev/ttyACM0
ssh orin-remote 'echo 0 | sudo -S systemctl start kart-brain'
```
The S3 enumerates on the Orin as **`/dev/ttyACM0`** (the CH343 is a CDC-ACM device), *not* `/dev/ttyUSB0`
as the classic board's CP2102 did. Uploading while `kart-brain` is running fails — the ROS node owns the port.

> **The build can link stale objects and still report SUCCESS (found 2026-07-25).** `components/km_gpio/km_gpio.c`
> was a week newer than its `.o`, `touch` did not trigger a rebuild, and `pio run` exited 0 having compiled
> nothing — so the flashed firmware silently kept week-old behaviour. Deleting the artefacts forced a correct
> rebuild. **Until this is fixed, `rm -rf .pio/build/esp32-s3-devkitc-1` before any build you intend to trust,
> and check that a `Compiling .../<yourfile>.o` line actually appears.** `main/` tracks fine; `components/` does
> not. Details and the proposed fix: `tasks.md` in kart-brain.

> **Stale-path warning (checked 2026-07-10):** this used to read `~/Desktop/kart-medulla` and
> `~/.local/bin/pio`. The repo lives at `~/repos/kart-medulla`, and **no PlatformIO or ESP-IDF
> is currently installed on the Mac** — only `esptool` (5.2.0, at `~/.local/bin/esptool.py`).
> Install a toolchain before expecting the command above to work.
>
> The S3 board's USB-UART bridge is a **WCH CH343/CH9102** (VID `0x1A86`), not a CP2102 — it
> enumerates as `/dev/cu.usbmodem*`, not `/dev/cu.SLAB_USBtoUART`. The board's two USB-C ports
> are silkscreened `COM` (the bridge) and `USB` (native USB-OTG / USB-Serial-JTAG on GPIO 19/20).

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

> **The table below is the CLASSIC-ESP32 pin map (ESP32-WROOM-32E).**
> The kart-medulla PCB now carries an ESP32-S3 (WROOM-1-N16R8), and the firmware fully supports it via the `esp32-s3-devkitc-1` environment in `platformio.ini`.
>
> **When working on the S3 board, use the authoritative S3 pinout:**
> `dv-hardware/projects/kart-medulla/docs/pinout-esp32-s3.md` (the schematic wins where they disagree).
>
> *Note:* The S3 board assigns critical functions to different pins (e.g. `PIN_CMD_COMPRESSOR` is GPIO 3, whereas the classic map lacks this). Ensure you use the proper target environment.

| Actuator | GPIO | Type | Notes |
|----------|------|------|-------|
| Throttle | 26   | DAC2 | 0-255 output |
| Brake    | 25   | DAC1 | 0-255 output |
| Steering PWM | 27 | LEDC PWM | duty 0-255 |
| Steering DIR | 14 | Digital | 1=positive, 0=negative |
| AS5600 SDA | 21 | I2C | 400kHz, addr 0x36 |
| AS5600 SCL | 22 | I2C | |

### AS5600 Steering Sensor Wiring (2026-03)

| Wire Color | Signal | ESP32 Pin |
|------------|--------|-----------|
| White | 3.3V (power) | 3V3 |
| Black | GND | GND |
| Green | SDA (I2C data) | GPIO 21 |
| Blue | SCL (I2C clock) | GPIO 22 |

### Safety

- Steering motor limited to **40%** output (`KM_ACT_SetLimit(&dir_act, 0.4)`) for testing
- Increase to 1.0 when system is validated
- **Comms watchdog IS implemented** (`COMMS_WATCHDOG_MS = 1000`, `main/main.c:100-115`). This line
  previously claimed it was not — corrected 2026-07-10. But note *what* it does: on stale comms or
  `MISSION_MANUAL` it calls `KM_ACT_Stop()` on throttle, brake **and** steering, which zeroes the
  brake command. It therefore **releases the brake and coasts** rather than braking.
- **Still TODO: make loss-of-comms assert braking / drop the SDC chain**, rather than zeroing
  outputs. On the S3 board the SDC is GPIO 18; firmware does not drive it at all yet, so the kart
  currently cannot be armed *or* commanded to brake by the medulla.

## Debugging

- **Debug logs**: connect to UART2 pins (see `km_gpio.h` for PIN_ORIN_UART_TX/RX) at 460800 baud
- **Protocol monitor**: `python3 monitor_serial.py` on /dev/ttyUSB0
- **ROS2 side**: `ros2 topic echo /esp32/steering` to see feedback from ESP32
