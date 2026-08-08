# Agent Development Notes

## Hardware is ESP32-S3 (READ THIS FIRST)

**The physical bench/kart board is an ESP32-S3.** The firmware in this repo now fully builds for the ESP32-S3 (`platformio.ini` uses `[env:esp32-s3-devkitc-1]`; `components/km_gpio/km_gpio.h` uses the `CONFIG_IDF_TARGET_ESP32S3` pin map). The legacy classic ESP32 build (`esp32dev`) is still kept for fallback purposes, but the S3 is the primary target.

- **Authoritative pin map for the real board:** [`.agents/esp32s3-pinmap.md`](.agents/esp32s3-pinmap.md). Key pins that differ from the classic map:
  - **I²C:** SDA = **GPIO 8**, SCL = **GPIO 9** (classic map says 21/22). On-board PCF8574 at 0x20; the MT6701 also answers at **0x06** when its I²C pins are wired.
  - **Steering-sensor PWM angle output** is read on **GPIO 1** (the ex-`PRESSURE_3` terminal CN5.2). This pad belongs to MCPWM capture, so it is deliberately NOT set up as an ADC channel — nothing else may claim it.
  - **EBS compressor MOSFET** is on **GPIO 3** (the ex-`BUZZER` net, now `CMD_COMPRESSOR_PWM`, CN8.2).
- **The physical board this firmware targets is dv-hardware commit `84d6dd0` (tag `kart-medulla-v1`), plus the rework listed in that repo's README under "Boards in existence".** dv-hardware `main` HEAD carries post-fab v2 design changes that do NOT exist on the physical board (throttle gain stage ×1.51, brake gain 3, MCP4922 moved to +3V3, WAGO connectors). When reading the schematic to answer questions about the real board, netlist from the tag, not HEAD: `git -C ~/repos/dv-hardware archive kart-medulla-v1 projects/kart-medulla | tar -x -C /tmp/kmv1` then run kicad-cli on that copy. Update this line when a new board is built. **Any electrical claim (pin wiring, part value, voltage range) must name the dv-hardware commit it was read from** — mixing HEAD with the physical board produced a string of wrong conclusions on 2026-08-08 (see history.md).
- **The schematic and PCB live in a different repo: `~/repos/dv-hardware/projects/kart-medulla/`.** `kart-medulla.kicad_sch` (sheet `kart-medulla_P1.kicad_sch` holds the circuitry), `kart-medulla.kicad_pcb`, `docs/pinout-esp32-s3.md` (module pin → signal), `docs/pinout-cn-connectors.md` (**which signal is on which CN1–CN10 screw terminal** — read this when wiring or probing the outside world, currently at commit `61f5a1c9`), `datasheets/`, `parts.md`. **The schematic is the authority on anything electrical** — what a pin physically connects to, part values, signal voltage ranges. When a question is "what is this pin wired to", export a fresh netlist and read it rather than trusting any table: `kicad-cli sch export netlist --format kicadxml -o /tmp/net.xml kart-medulla.kicad_sch`. The checked-in `output/netlist.net` is dated 2026-05-07 and is stale. Note the PCB silkscreen and the schematic disagree on some connector designators — trust the silkscreen for wiring (see `tasks.md`).
- **USB bridge is a WCH CH343** (VID 0x1A86 / PID 0x55D3) → shows up as `/dev/cu.usbmodem*`. This does NOT mean native-USB / does NOT tell you classic-vs-S3 on its own. Serial goes over UART0 through the bridge, so bench builds use **`ARDUINO_USB_CDC_ON_BOOT=0`** (see `history.md` / error-log on Mac bench flashing).

## The Steering Sensor Is an MT6701, Not an AS5600

The AS5600 was retired on 2026-07-12 — it could not detect the kart's large shaft magnet. The kart now uses an **MT6701**, permanently configured (EEPROM) for PWM output at 994.4 Hz, high-valid. **`km_sdir.c`'s angle functions cannot read the kart's sensor**: they are an AS5600 I²C driver fixed at address 0x36, and the MT6701's slave address is 0x06 (datasheet §7.7.2), so every call fails. They are still the steering source on the classic ESP32 fallback build, and `KM_SDIR_Begin()` is still called on the S3 to bring the I²C bus up for the PCF8574 — but on the S3 the angle does not come from there.

- **Production read path: `components/km_sdir/km_sdir_pwm.h`** — MCPWM capture on GPIO 1, decoding the MT6701's 4119-clock frame (16 clocks high, 12-bit angle, ≥8 clocks low; datasheet §7.6, PDF in `datasheets/`). Non-blocking, so it cannot stall the control loop the way an unanswered I²C read did.
- **Bench tool for the sensor itself: `~/dv/kart/steering/mt6701-bench/`** — I²C console for reading the raw 14-bit angle and checking/setting reg 0x38 (OUT_MODE). Use it to confirm the sensor is still in PWM mode before blaming the firmware.

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

**BEFORE ANY FLASH: the steering/actuator power must be off, or the kart in manual mode.**
Flashing hard-resets the ESP32 into the download bootloader, and in that window GPIO 40
(`CMD_STEER_PWM`) and GPIO 17 (`CMD_STEER_DIR`) float — they have no hardware pulldowns (unlike
GPIO 3 and GPIO 18), so the Cytron can drive the steering motor uncontrolled. This broke the
steering gear on 2026-08-08 (see `.agents/error-log.md`). Confirm the power state with Rubén
before resetting the chip; software cannot protect a window where the firmware is not running.

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
> nothing — so the flashed firmware silently kept week-old behaviour. Deleting the artefacts fixed it.
>
> **Always check that a `Compiling .../<yourfile>.o` line actually appears.** That check is free and catches
> the failure whatever its cause; keep doing it regardless of everything below.
>
> **Do NOT reflexively `rm -rf .pio/build/...` before every build.** That was the original advice here and it is
> too broad: it turns every build into a clean one (~100 s instead of ~30 s) to work around something that is
> probably a one-time condition. Leading hypothesis, 2026-07-26 — **a moved build tree**. CMake and ninja bake
> *absolute* paths into `build.ninja`, `.ninja_deps` and `CMakeCache.txt`, and this repo has moved more than
> once (`~/Desktop/kart_medulla` → `~/repos/kart-medulla` on the Mac; the Orin workspace was renamed on
> 2026-07-06). A build directory generated under the old path keeps checking the old path, so edits at the new
> one are invisible — which matches every symptom, including `touch` doing nothing, and explains why deleting
> the directory cures it permanently rather than temporarily. The component `CMakeLists.txt` files use explicit
> `SRCS "<file>.c"` rather than `SRC_DIRS` globbing, so a glob that missed the file is ruled out.
>
> **Test it in one command** before assuming you must nuke anything:
> ```bash
> grep -m1 CMAKE_HOME_DIRECTORY .pio/build/esp32-s3-devkitc-1/CMakeCache.txt   # should equal $PWD
> ```
> Mismatch → delete that build directory once, and it should stay healthy at the new path. Match, yet a source
> edit still does not recompile → the hypothesis is wrong, so record what you saw in `history.md` and reopen the
> task in `tasks.md`.

> **Path/toolchain note (rechecked 2026-07-26):** this used to read `~/Desktop/kart-medulla`; the
> repo lives at `~/repos/kart-medulla`. **PlatformIO IS now installed on the Mac**, at
> `~/.platformio/penv/bin/pio` — not on `PATH`, so call it by full path. Verified by building both
> `esp32-s3-devkitc-1` and `esp32dev` to a linked firmware.bin on 2026-07-26, which supersedes the
> earlier "no toolchain on the Mac" note. The Mac can therefore compile-check a change before it
> goes near the kart; flashing still happens from the Orin, which is where the board is.
>
> The S3 board's USB-UART bridge is a **WCH CH343/CH9102** (VID `0x1A86`), not a CP2102 — it
> enumerates as `/dev/cu.usbmodem*`, not `/dev/cu.SLAB_USBtoUART`. The board's two USB-C ports
> are silkscreened `COM` (the bridge) and `USB` (native USB-OTG / USB-Serial-JTAG on GPIO 19/20).

- **Upload baud depends on which board.** The classic board's CP2102 fails flashing above **115200**. The S3 board's bridge is a **CH343**, rated 50 bps – 6 Mbps, so it is not bound by that — `[env:esp32-s3-devkitc-1]` uses **921600** as of 2026-07-26 (untested on hardware at time of writing; fall back to 115200 in `platformio.ini` if a flash refuses to connect). The two envs deliberately carry different numbers; do not "unify" them.
- **Runtime UART baud is 115200**, set at `km_coms.c` KM_COMS_Init (`.baud_rate`). This line said 460800 until 2026-07-26 and was simply wrong — grep confirms 115200 is the only live baud in `components/` and `main/`; the `km_gpio.c` UART blocks that mention other values are commented out. `monitor_speed` in both envs matches it.
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

- UART0 @ **115200** baud — protocol comms with Orin, over the board's USB-serial bridge
- UART2 — **not in use.** The ESP_LOG redirect to UART2 was removed (it caused crashes, and on the PCB those pins collide with MOTOR_HALL_1/3). ESP_LOG on UART0 is disabled so the binary protocol stays clean, which means the firmware is effectively silent on the console — do not wait for log output that cannot arrive
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

- **Debug logs**: there are none. The UART2 log redirect was removed and ESP_LOG on UART0 is off (see the UART Protocol section). Observe the firmware through the telemetry frames instead — `ESP_PNEUMATIC` in particular carries `control_iters`, the LEDC duty readback, the `KM_GPIO_Init()` error code and the SDC pin level precisely because there is no console to print to
- **Protocol monitor**: `python3 monitor_serial.py` on /dev/ttyUSB0
- **ROS2 side**: `ros2 topic echo /esp32/steering` to see feedback from ESP32
