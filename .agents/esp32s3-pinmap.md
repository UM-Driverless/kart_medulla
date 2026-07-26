<!-- reference — read when porting firmware to the ESP32-S3 -->

# ESP32-S3 pin map for the kart-medulla PCB

Derived 2026-07-10 by exporting a fresh netlist from `dv-hardware/projects/kart-medulla/kart-medulla.kicad_sch`
(`kicad-cli sch export netlist`) and cross-checking every pin against
`dv-hardware/projects/kart-medulla/docs/pinout-esp32-s3.md`. All 44 module pins agreed. The
schematic is authoritative; this file is a convenience for firmware work.

**This is NOT the pin map the firmware currently uses.** `components/km_gpio/km_gpio.h` still holds
the classic-ESP32 (WROOM-32E) map, which this repo builds. See `.agents/error-log.md` 2026-07-10 —
most importantly, the classic map puts `PIN_STEER_PWM` on GPIO 18, which on the S3 board is the
gate of Q3, the shutdown-circuit MOSFET.

## Signals

| Signal | GPIO | Notes |
|---|---|---|
| `PRESSURE_3` | 1 | analog in (ADC1) |
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

## Known firmware gaps against this hardware

1. **The S3 build does not exist.** `platformio.ini` has only `esp32dev` and `native`. The
   `CONFIG_IDF_TARGET_ESP32S3` branch in `km_gpio.c` references `SPI_MOSI_PIN` / `SPI_SCLK_PIN` /
   `SPI_CS_PIN`, which are defined nowhere. Those are GPIO 11 / 12 / 14 (MISO 13) per the table above.
2. **Nothing drives `SELECT_THROTTLE` (GPIO 15).** The schematic routes it to the MAX4660 mux with a
   10 kΩ pulldown, so the default state is pedal pass-through (safe). Firmware must drive it HIGH to
   hand throttle to the DAC.
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
