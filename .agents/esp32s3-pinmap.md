<!-- reference — read when porting firmware to the ESP32-S3 -->

# ESP32-S3 pin map for the kart-medulla PCB

Derived 2026-07-10 by exporting a fresh netlist from `dv-hardware/projects/kart-medulla/kart-medulla.kicad_sch`
(`kicad-cli sch export netlist`) and cross-checking every pin against
`dv-hardware/projects/kart-medulla/docs/pinout-esp32-s3.md`. All 44 module pins agreed. The
schematic is authoritative; this file is a convenience for firmware work.

**This IS the pin map the firmware uses** (corrected 2026-08-10; it previously said the opposite).
`components/km_gpio/km_gpio.h` carries this map under `CONFIG_IDF_TARGET_ESP32S3`, and
`platformio.ini`'s `esp32-s3-devkitc-1` env is what builds and flashes. The classic-ESP32
(WROOM-32E) map is still in the same header under the `#else`, for the `esp32dev` target only.

The reason the distinction matters has not changed: the classic map puts `PIN_STEER_PWM` on
GPIO 18, which on the S3 board is the gate of Q3, the shutdown-circuit MOSFET. Wiring or reasoning
from the wrong branch can fire or disable the emergency brake. See `.agents/error-log.md`
2026-07-10.

## Signals

| Signal | GPIO | Notes |
|---|---|---|
| `PRESSURE_3` | 1 | **NOT an ADC input.** CN5.2 carries the MT6701's PWM angle output, read by MCPWM capture and deliberately excluded from ADC setup — see line 66 below and the 2026-08-08 correction |
| `HYDRAULIC_2` | 2 | analog in (ADC1) |
| `CMD_COMPRESSOR_PWM` (ex-`BUZZER`) | 3 | LEDC PWM out → compressor MOSFET gate. Strap pin (JTAG src select), not driven by firmware until `KM_GPIO_Init()` runs ~200 ms into boot — but the gate has a pulldown, so the MOSFET is held off through that window and the compressor cannot self-start on boot or on a firmware crash (same arrangement as R23 for Q3 on GPIO 18). Duty runs at **100%** — `COMPRESSOR_DUTY_RUN = 255` in `main/main.c`. This line used to say the duty was capped at 60% to keep a 7.5 V motor off a 12 V rail; that was true of the bare IRLZ44N whose gate came straight off the GPIO, and stopped being true when the gate moved to a 3D-printer hotbed MOSFET module that cannot hold a continuous PWM waveform. Average power is now limited by a slow on/off cycle instead: 15 s maximum burst, 15 s forced cooldown, with a 1 s soft-start ramp per burst for inrush. The reasoning is written out at the top of `main/main.c`. Also gated by `COMPRESSOR_DISABLED` (operator latch from the dashboard) |
| `PEDAL_ACC` | 4 | analog in (ADC1) |
| `PEDAL_BRAKE` | 5 | analog in (ADC1) |
| `PRESSURE_1` | 6 | analog in (ADC1) |
| `PRESSURE_2` | 7 | analog in (ADC1) |
| `SDA` | 8 | I²C — AS5600 (off-board, 0x36) + U25 PCF8574 (on-board, 0x20) |
| `SCL` | 9 | I²C |
| `HYDRAULIC_1` | 10 | analog in (ADC1) |
| `MOSI` | 11 | SPI → MCP4922 DAC |
| `CLK` | 12 | SPI → MCP4922 DAC |
| `MISO` | 13 | SPI → MCP4922 DAC |
| `CMD_DAC_CS` | 14 | SPI chip-select → MCP4922 DAC |
| `SELECT_THROTTLE` | 15 | digital out → MAX4660 (U14) pin 6. 10 kΩ pulldown (R32) on the net |
| `MOTOR_HALL_1` | 16 | digital in (via U5 level shifter) |
| `CMD_STEER_DIR` | 17 | digital out → Cytron H-bridge |
| `SDC_NOT_EMERGENCY` | 18 | **SAFETY.** Gate of Q3 (IRLZ44N) via R22 100 Ω. HIGH = Q3 conducts = shutdown chain closed = *no emergency*. R23 100 kΩ pulldown holds it OFF at boot |
| `MOTOR_HALL_3` | 21 | digital in (via U5) |
| `CMD_STEER_PWM` | 40 | LEDC PWM → Cytron H-bridge |
| `MOTOR_HALL_2` | 47 | digital in (via U5) |

Every analog input lands on GPIO 1–10, which is exactly ADC1 on the S3. Deliberate — ADC2 is
unusable when Wi-Fi is active.

## Do not use

| GPIO | Why |
|---|---|
| 33, 34, 35, 36, 37 | Octal PSRAM. The fitted module is an **N16R8** — these are wired to the in-package PSRAM die and are physically unusable. Never assign them |
| 43, 44 | UART0 — the dev board's USB-UART bridge owns these |
| 48 | Dev board's on-board RGB LED |
| 19, 20 | USB D−/D+ |
| 0, 45, 46 | Strap pins (BOOT, VDD_SPI, ROM-print). Reclaimable post-boot only if the signal's idle state matches the strap default |
| 41, 42 | Held for future CAN |

## Free

**GPIO 38** and **GPIO 39** are unconstrained and unconnected in the schematic (verified: netlist
shows `unconnected-(U24-Pad13)` and `unconnected-(U24-Pad14)`).

**Correction 2026-07-11:** GPIO 38 is *not* the compressor pin. The EBS compressor PWM was
**finalized on CN8.2 / GPIO 3** (the ex-`BUZZER` MOSFET — buzzer dropped, net renamed
`CMD_COMPRESSOR_PWM`) because the compressor gate has to reach a screw terminal and GPIO 38 isn't on
a CN — see `dv/kart/pneumatics/history.md` 2026-07-10. So **both GPIO 38 and 39 are genuinely free.**

**Correction 2026-07-11 (supersedes the GPIO 39 plan):** the AS5600 steering-angle PWM lands on
**CN5.2 → GPIO 1** (the ex-`PRESSURE_3` input; keep R8 series, remove R9+R10 pulldown) — decided in
`dv/kart/steering/as5600-pwm-burn-runbook.md` because the signal must arrive on an existing screw
terminal, and GPIO 39 isn't on a CN. **GPIO 38 and 39 both remain spare.** Capture routes via the
GPIO matrix (MCPWM/PCNT/RMT), so the choice of pin doesn't constrain the peripheral.

**Correction 2026-08-08 (amends the 2026-07-11 entry above on two points; CN5.2 → GPIO 1 stands):**

1. **The sensor is an MT6701, not an AS5600.** The AS5600 was retired 2026-07-12 — it could not
   detect the kart's large shaft magnet. The MT6701 is EEPROM-configured for 994.4 Hz PWM output,
   high-valid, and is mounted and validated on the kart. See "The Steering Sensor Is an MT6701, Not
   an AS5600" in `AGENTS.md`.
2. **Remove R10 only, not R9+R10.** R8 and R9 both stay. The net is
   `CN5.2 —[R8 10k]— node —[R9 10k]— GPIO 1 —[R10 10k]— GND`, so R10 is the only shunt to ground and
   R8 + R9 remain as a 20 kΩ series into the pin. That series impedance is too high for the ADC,
   which is the reason the pin is read as digital PWM via MCPWM capture rather than as an analog
   voltage — removing R9 as well would defeat that.

## Known firmware gaps against this hardware

1. ~~**The S3 build does not exist.**~~ **Closed — it was already false when written, and stayed on
   this list until 2026-08-10.** `platformio.ini` has an `esp32-s3-devkitc-1` env that builds, links
   and flashes, and it is the primary target. The four SPI pins the original text called "defined
   nowhere" are at `km_gpio.h:83-86` — GPIO 11 (MOSI) / 12 (SCLK) / 13 (MISO) / 14 (CS), matching the
   table above. MISO is not wired to U13: the MCP4922 has no data output, so nothing the firmware
   reads can confirm a write arrived.
2. ~~**Nothing drives `SELECT_THROTTLE` (GPIO 15).**~~ **Closed 2026-08-01/08-08.** `control_task`
   drives it every cycle: LOW (pedal pass-through) whenever comms are stale or the mission is manual,
   HIGH (throttle from the DAC) only past that safety gate. It is re-asserted each cycle rather than
   latched, so any path returning early leaves the driver's pedal in control. The schematic's 10 kΩ
   pulldown still gives pedal pass-through before firmware runs.
3. ~~**Nothing drives `SDC_NOT_EMERGENCY` (GPIO 18).**~~ **Closed 2026-07-26.** `control_task()` in
   `main/main.c` now decides the level on every cycle and is the only thing that drives it HIGH. It is
   a whitelist: the chain closes only while the Orin reports `AS_READY` or `AS_DRIVING`, comms are
   fresh, the steering-fault latch is clear, and the operator has not disabled the compressor. Every
   other case — including any state nobody anticipated — leaves it open, so a forgotten condition
   fails safe. The pin is configured `GPIO_MODE_INPUT_OUTPUT` so its real level can be read back;
   that readback is field 8 of the `ESP_PNEUMATIC` frame and shows on the dashboard's EBS page.
   **The gate is still not wired to anything downstream**, so this changes no physical behaviour yet:
   verify it by the readback or a meter on the pin, not by expecting the kart to brake.
4. **No PCF8574 driver.** `CMD_REVERSE` is PCF8574 P0 over I²C; no code writes it.
5. **Comms-watchdog releases the brake rather than applying it.** On stale comms or
   `MISSION_MANUAL`, `main.c` calls `KM_ACT_Stop()` on throttle, brake and steering — zeroing the
   brake command. For a driverless kart, loss of comms should *assert* braking / drop the SDC.
