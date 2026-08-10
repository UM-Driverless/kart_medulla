# Kart Medulla - ESP32 Controller System

ESP32-based control system for the UM Driverless autonomous go-kart. Receives commands from the Orin via UART binary protocol, runs PID-controlled steering, and outputs analog throttle/brake signals.

## Hardware this firmware targets

**PCB: `dv-hardware` commit `84d6dd0`.** A manufactured board is identified by the commit its gerbers
were exported from; the convention and the board's rework list live in
[`dv-hardware/projects/kart-medulla/README.md`](https://github.com/UM-Driverless/dv-hardware/blob/main/projects/kart-medulla/README.md).

Every branch in this repo — `main`, `dev`, `spi-fix` — targets `84d6dd0`. This file deliberately does
not record its own repo's commit: git already knows which one you have checked out, and any commit
editing this line would invalidate the hash written in it. The pairing that has to be written down is
the one pointing *out* of the repo, at hardware.

Other software paired to the same board:

| Repo | Commit | Notes |
|---|---|---|
| kart-brain | `main` = `c200e56` | Orin side. Recorded 2026-07-31 in its `history.md` |

**The board is not the current schematic.** `dv-hardware` HEAD is ahead of `84d6dd0`, and at least
one commit since (`f68cc1f`) changed circuitry: CN10.2's brake output moved to the LM358's amplified
side and the DAC-to-amplifier copper was restored. The physical board still has the old behaviour —
unamplified 0-5 V brake, amplifier input possibly unrouted — and is due a cut-and-jumper patch. So a
netlist exported from `dv-hardware` HEAD describes the *design*, not the board on the kart.

Two reworks are outstanding or pending on this board; both are listed in the dv-hardware README above,
and neither is reflected by the hash alone, because rework exists on a physical board and in no commit.

## Hardware Overview

* **Microcontroller:** ESP32-S3 (DevKitC-1, kart-medulla PCB)
* **Steering Sensor:** MT6701 magnetic angle encoder, read as PWM on GPIO 1 (see "The Steering Sensor Is an MT6701, Not an AS5600" in `AGENTS.md`; the AS5600 I²C driver in `km_sdir` is the classic-ESP32 fallback only)
* **Steering Motor:** Cytron H-bridge (PWM + DIR)
* **Throttle/Brake:** DAC analog outputs
* **Comms to Orin:** UART0 binary protocol (int32 encoding, 115200 baud)

## Pin Configuration

All pin assignments are defined in [`components/km_gpio/km_gpio.h`](components/km_gpio/km_gpio.h).

### Repurposed pins (current hardware — read this first)

Two nets on the actual kart-medulla PCB no longer do what their original name says. The firmware header carries both (checked 2026-07-31); the authoritative map is [`.agents/esp32s3-pinmap.md`](.agents/esp32s3-pinmap.md).

- **BUZZER (old name) → EBS compressor MOSFET.** The old buzzer output now drives the compressor gate (net renamed `CMD_COMPRESSOR_PWM`, terminal CN8.2). The buzzer itself was dropped. On the S3 board this is GPIO 3.
- **PRESSURE_3 → steering-sensor PWM input.** The Pressure-3 terminal (CN5.2) now reads the **MT6701**'s single-wire PWM angle output (~994 Hz, angle in the duty cycle) instead of a pressure transducer. On the S3 board this is GPIO 1, decoded with MCPWM capture. **Rework: remove R10 only** — the pulldown to GND — keeping R8 + R9. The net is `CN5.2 —[R8 10k]— node —[R9 10k]— GPIO 1 —[R10 10k]— GND`, so R10 is the only shunt to ground; R8 + R9 remain as a 20 kΩ series into the pin, which is too high-impedance for the ADC and is why this pin is read as digital PWM rather than an analog voltage.

The firmware now supports the ESP32-S3 board natively via the `esp32-s3-devkitc-1` PlatformIO target, which automatically applies the correct pinmap from [`.agents/esp32s3-pinmap.md`](.agents/esp32s3-pinmap.md). (The classic ESP32-WROOM-32E pinmap is still available for legacy testing using `esp32dev`).

### ESP32-S3 — the board on the kart

This is the map that matters. Taken from `km_gpio.h`'s S3 block, checked 2026-08-10.

**Actuator outputs**

| Function | GPIO | Type | Notes |
|---|---|---|---|
| Throttle (CMD_ACC) | — | MCP4922 ch. A | Not a GPIO. Over SPI to U13, then U14.8. The S3 has no built-in DAC |
| Brake (CMD_BRAKE) | — | MCP4922 ch. B | Not a GPIO. Nothing writes it yet — the proportional braking valve is unwired |
| Steering PWM | 40 | LEDC PWM | Cytron H-bridge |
| Steering DIR | 17 | Digital | Cytron H-bridge direction |
| EBS compressor | 3 | LEDC PWM | Ex-BUZZER net, terminal CN8.2 |
| Throttle source select | 15 | Digital | MAX4660 mux. LOW = driver's pedal, HIGH = throttle from the DAC |
| **SDC (not-emergency)** | **18** | **Digital — SAFETY** | **Gate of Q3. HIGH closes the shutdown chain. Driving this pin arms or kills the kart** |
| Status LED | 48 | RMT | Addressable RGB; plain GPIO will not drive it |

**Sensor inputs**

| Function | GPIO | Notes |
|---|---|---|
| Steering angle | 1 | MT6701 PWM output on CN5.2, read by MCPWM capture — **not** an ADC channel |
| Hydraulic 2 | 2 | ADC1 |
| Pedal Acc | 4 | ADC1 |
| Pedal Brake | 5 | ADC1 |
| Pressure 1 | 6 | ADC1, terminal CN7.1 — the tank sensor |
| Pressure 2 | 7 | ADC1, terminal CN7.2 |
| Hydraulic 1 | 10 | ADC1 |
| Motor Hall 1 / 2 / 3 | 16 / 47 / 21 | Through the U5 level shifter; all three usable |

**Buses**

| Function | GPIO | Notes |
|---|---|---|
| UART0 TX / RX | 43 / 44 | Console *and* the Orin binary protocol, both on this one port |
| I2C SDA / SCL | 8 / 9 | On-board PCF8574. The AS5600 is not the kart's steering sensor |
| SPI MOSI / SCLK / MISO / CS | 11 / 12 / 13 / 14 | To the MCP4922. MISO goes nowhere — the chip has no data output |

Free pins: **38 and 39**. GPIO 38 briefly carried a filtered-PWM throttle output on `dev` (commit
`e12f6b5`) and was reverted when the throttle went back to the MCP4922; check the board for a leftover
flying wire before reusing it.

On the S3, ADC1 is GPIO 1-10 and ADC2 is unusable while WiFi is active. The classic ESP32's
"GPIO 6-11 reserved for flash" and "GPIO 34-39 input only" rules do **not** apply here — 6 and 7 are
the pressure inputs.

<details>
<summary><b>Classic ESP32-WROOM-32E — the previous board. Do not wire from this.</b></summary>

Kept because `platformio.ini`'s `esp32dev` target still builds for it. **Every number below is wrong
for the kart.** The overlaps are what make it dangerous rather than merely useless: GPIO 18 is
steering PWM here and the *shutdown-circuit gate* on the S3, and GPIO 13 is the SDC line here but SPI
MISO on the S3. Following this table on the S3 board can fire or disable the emergency brake.

| Function | GPIO | Type |
|---|---|---|
| Throttle (CMD_ACC) | 25 | DAC1 |
| Brake (CMD_BRAKE) | 26 | DAC2 |
| Steering PWM | 18 | LEDC PWM |
| Steering DIR | 19 | Digital |

| Function | GPIO | ADC Channel |
|---|---|---|
| Pedal Brake | 32 | ADC1_CH4 |
| Pedal Acc | 35 | ADC1_CH7 |
| Pressure 1 | 36 (VP) | ADC1_CH0 |
| Pressure 2 | 39 (VN) | ADC1_CH3 |
| Pressure 3 | 34 | ADC1_CH6 |
| Hydraulic 1 | 27 | ADC2_CH7 |
| Hydraulic 2 | 14 | ADC2_CH6 |

| Function | GPIO | Notes |
|---|---|---|
| I2C SDA / SCL | 21 / 22 | AS5600 steering encoder |
| USB UART TX / RX (UART0) | 1 / 3 | Binary comms to Orin |
| UART2 TX / RX | 17 / 16 | Conflicts with HALL1/HALL3 on the PCB — unused |
| Motor Hall 2 | 33 | Hall 1/3 disabled (UART2 conflict) |
| SDC Emergency | 13 | Shutdown circuit status |
| Status LED | 2 | Keep LOW at boot (strap pin) |

Restrictions that apply to *this* chip only: GPIO 6-11 reserved for SPI flash, GPIO 34-39 input only.

</details>

## Project Structure

```
kart-medulla/
├── main/
│   └── main.c                     # Entry point, FreeRTOS tasks
├── components/
│   ├── km_gpio/                   # GPIO hardware abstraction (pin definitions, ADC, DAC, PWM, I2C)
│   ├── km_act/                    # Actuator controllers (steering, throttle, brake)
│   ├── km_pid/                    # PID controller
│   ├── km_sdir/                   # AS5600 steering encoder driver
│   ├── km_coms/                   # UART binary protocol (int32 encoding)
│   ├── km_objects/                # Shared object store (targets, actuals)
│   ├── km_rtos/                   # FreeRTOS task manager
│   ├── km_sta/                    # State machine
│   ├── km_gamc/                   # Gamepad controller (Bluepad32)
│   ├── bluepad32/                 # Bluetooth controller library
│   └── btstack/                   # Bluetooth stack
├── test/
│   ├── test_km_pid/               # PID unit tests
│   ├── test_km_act/               # Actuator unit tests
│   ├── test_km_coms/              # Comms unit tests
│   ├── test_km_objects/           # Object store unit tests
│   └── fakes/                     # Hardware stubs for native tests
├── platformio.ini                 # Build config (ESP-IDF + native test env)
└── CMakeLists.txt                 # ESP-IDF project config
```

**Framework:** ESP-IDF 5.x (via PlatformIO, espressif32@6.4.0)
**Build System:** PlatformIO
**Test Environment:** Native (runs on host with hardware fakes)

## FreeRTOS Tasks

| Task | Rate | Stack | Description |
|---|---|---|---|
| comms | 20 Hz | 4096 B | Receives/processes UART messages from Orin |
| control | 10 Hz | 4096 B | Reads AS5600, runs steering PID, sets actuators |
| heartbeat | 1 Hz | 2048 B | Sends uptime to Orin |
| health | 1 Hz | 4096 B | Monitors magnet, I2C, heap; reports to Orin |

## Steering Control

### Coordinate Convention

- **Body frame:** X forward, Y left, Z up
- **Positive steering** = left turn (matches ROS REP 103)
- AS5600 natural frame matches this convention - no negation needed

### PID Configuration (current)

| Parameter | Value |
|---|---|
| Kp | 0.15 |
| Ki | 0.0 |
| Kd | 0.01 |
| PWM limit | 0.15 (15%) |

PWM limit is kept low to protect steering gears during testing. Increase gradually as PID is tuned.

## Building and Flashing

```bash
# Build for ESP32-S3
pio run -e esp32-s3-devkitc-1

# Flash (from Orin, where ESP32 is connected via USB)
pio run -t upload -e esp32-s3-devkitc-1 --upload-port /dev/ttyACM0

# Monitor serial
pio device monitor

# Run unit tests (native, no hardware needed)
pio test -e native
```

**Upload speed:** 115200 baud (CP2102 USB-UART bridge limitation).

**Flash tip:** If ESP32 is in a crash loop, hold the **BOOT** button during flash.

## Development Notes

See [`AGENTS.md`](AGENTS.md) for detailed project documentation, error log, and conventions.
