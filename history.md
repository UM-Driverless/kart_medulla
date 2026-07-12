<!-- consult-selectively — grep for the topic you're working on; append-only, newest at the end -->

# kart-medulla history

## 2026-07-12 — AS5600 PWM bench session (runbook Phase 0 + Phase 1 done, no burn yet)

Followed `~/dv/kart/steering/as5600-pwm-burn-runbook.md` on the bench: ESP32-S3 devkit socketed on the medulla, sensor wired to CN4, logic analyzer on the AS5600 OUT pin.

**Bench tool.** Wrote a standalone PlatformIO/Arduino firmware at `~/dv/kart/steering/as5600-bench/` (kept next to the runbook, not in this repo). Raw AS5600 register I/O over serial commands: `s` health check, `a` RAW_ANGLE sweep with min/max, `p` Phase-1 PWM RAM write, `c` re-read CONF, `d` GPIO float-vs-connected diagnostic, `BURN` double-guarded Phase-2 burn (preflight-checks magnet status + RAM config, then requires typing YES).

**Serial-port gotcha (cost ~1 h).** The devkit was plugged in via its **UART connector**, which on this board is a **CH343 bridge that enumerates on macOS as `/dev/cu.usbmodem…` ("USB Single Serial")** — the name looks like the S3's native USB but is not. With `ARDUINO_USB_CDC_ON_BOOT=1` the sketch's `Serial` went to the unplugged native-USB connector: flashing and ROM/IDF log output worked over the cable, but `Serial` prints were invisible and commands never arrived. Fix: leave `Serial` on UART0 (no CDC flags) when using the UART connector. Identify which port you really have with `ioreg -p IOUSB` ("USB Single Serial" = CH343 UART bridge; "USB JTAG/serial debug unit" = native).

**I²C bring-up.** First scans found nothing — pin diagnostic (read GPIO under internal pull-up then pull-down; external 4.7 k wins) showed SDA held high but **SCL pulled hard to GND**, killing the bus for every device including the on-board PCF8574. Resolved by redoing the CN4 wiring (root cause not pinned down — most likely a misplaced/pinched wire at the terminal). Useful facts confirmed against a fresh netlist export: **CN4.1 = SCL → GPIO 9, CN4.2 = SDA → GPIO 8** (matches kart-docs), pull-ups R33/R34 = 4.7 kΩ to +3V3, and **CN4 carries no power** — sensor VCC/GND must come from neighbouring CNs (+3V3 on CN6.3/CN1.1, GND on CN1.3/CN9.3/CN10.3).

**Phase 0 numbers (recorded pre-burn, magnet handheld near chip, not mounted):** bus scan finds AS5600 at 0x36 + PCF8574 at 0x20. ZMCO = 0 (virgin). CONF = 0x0000 (factory). STATUS = 0x37: MD=1, ML=1 (weak), MH=0. AGC = 128, MAGNITUDE = 1415. Note: **at 3.3 V the AGC range is 0–128, not 0–255**, so 128 = gain maxed out — the handheld magnet was marginal. Rubén decided that's fine for the bench (goal is PWM config, not magnet quality; the real magnet is mechanically mounted on the kart shaft). Full-travel RAW_ANGLE sweep skipped for the same reason.

**Phase 1 done:** CONF low byte 0x00 → 0xE0 written in RAM and read back verified — **OUTS=PWM, PWMF=920 Hz**. Logic-analyzer confirmation of the ~920 Hz output was pending when this entry was written. **No burn performed yet** — Phase 2 still requires the runbook's acceptance test (MCPWM capture on GPIO 1 tracking I²C RAW_ANGLE).

### Full test-by-test chronology (same session, for the record)

1. **Flash #1** (bench tool, `ARDUINO_USB_MODE=1` + `ARDUINO_USB_CDC_ON_BOOT=1`): flash OK. First serial read caught only ESP-IDF error spam — `E i2c.master: probe device timeout. Please check if xfer_timeout_ms and pull-ups are correctly set up` on every scan address, then `i2cWriteReadNonStop(): ... [259] ESP_ERR_INVALID_STATE` / `Wire.cpp requestFrom(): Error 259`. Never any `Serial.print` output (banner/scan results missing) — first clue two consoles were in play, misread at the time as "bus dead, tool works".
2. **Serial rabbit hole** (~1 h). Symptoms: commands never answered; opening the port caused `rst:0x1 (POWERON)` + ROM banner then silence. Tried: DTR/RTS combinations on open (deasserted, asserted-after-open) — ROM banner only; close-reopen after boot — nothing; repeated command sends — nothing; **tick/echo minimal sketch** (prints every 1 s) — zero bytes even at raw read level, proving the problem was infrastructure, not the bench sketch; **`ARDUINO_USB_MODE=0` (TinyUSB)** — no output AND no USB re-enumeration, proving `Serial` wasn't reaching the plugged port at all; `pio device monitor` from a script — crashes (`termios.error 19`, needs a real tty; useless headlessly). Diagnosis via `ioreg -p IOUSB`: device is **"USB Single Serial" = CH343 UART bridge** — the cable is in the UART connector, `Serial` was going to the unplugged native-USB connector. Fix: drop the CDC build flags, `Serial` on UART0. Everything worked immediately after.
3. **I²C attempt #1** (serial fixed, `s` command): scan finds **nothing** — not even the on-board PCF8574. Same probe-timeout spam.
4. **Pin diagnostic `d` #1**: `GPIO8 (SDA): pullup→1 pulldown→1 open→1` = held high externally (rail + pull-up + socket path OK). `GPIO9 (SCL): pullup→0 pulldown→0 open→0` = **held hard LOW** — dead clock explains the whole bus being mute. (Diag method: read pin under internal pull-up then pull-down; the external 4.7 k wins over the ~45 k internal, so "follows the internal pull" = floating, "high both ways" = connected, "low both ways" = grounded.)
5. Netlist check (fresh `kicad-cli sch export netlist`, committed `output/netlist.net` known stale): **CN4.1 = SCL__I2C → R34 4.7 k → socket → GPIO 9; CN4.2 = SDA__I2C → R33 4.7 k → socket → GPIO 8**; also on the bus: U25 PCF8574 (0x20). **CN4.3 = REVERSE_WIRE (Q4 drain) — CN4 has no 3V3/GND.** Matches kart-docs connector table and Rubén's reading (CN4.2 → module pin 34 = GPIO8, CN4.1 → pin 37 = GPIO9).
6. **Rewire + diag #2**: both lines held high. Scan: **PCF8574 found at 0x20; AS5600 still absent** (NACK at 0x36 → `Error 259` on the STATUS read). Bus itself proven good.
7. User measured at the module: power present, SCL idle 3.275 V. (One intermediate scan ran while the user had SCL deliberately disconnected — only 0x20 found, as expected.)
8. **Everything replugged → scan #3: AS5600 ACKs at 0x36.** Root cause of the SCL-low episode never pinned to a component — resolved by redoing the CN4 wiring; a misplaced/pinched wire at the terminal is the best guess.
9. **Health check, no magnet**: STATUS 0x13 (MD=0, ML=1, MH=0), AGC=128, MAGNITUDE=2, ZMCO=0, CONF=0x0000, RAW_ANGLE garbage (also exposed a bench-tool bug: RAW_ANGLE printed unmasked as 20384/4095 — 12-bit mask added, reflashed).
10. **Magnet held near**: MAGNITUDE 2 → 20 (still ML=1). **Closer**: STATUS 0x37 (MD=1, ML=1, MH=0), AGC=128, MAGNITUDE=1415. Learned: **at 3.3 V AGC's range is 0–128** (not 0–255), so AGC=128 = gain maxed, field still weak. Rubén: good enough for the bench — the goal is the PWM config, not magnet quality; the real magnet is mounted on the kart shaft. Full-travel sweep (`a`) skipped accordingly.
11. **Phase 1 (`p`)**: CONF lo 0x00 → 0xE0, read-back CONF = 0x00E0 → PWMF=11 (920 Hz), OUTS=10 (PWM), HYST=0, PM=0, SF=0, FTH=0, WD=0. Verified in RAM; volatile until burned.

## 2026-07-12 — ESP32 (legacy board) failure report

**Board:** Legacy / spare ESP32-WROOM dev board (UM-Driverless kart project). **Reported symptom:** board "stopped working several days ago"; not detected when connected to the computer. Diagnosed on Jorge's laptop (`jorge-Aspire-A315-51`, Linux 6.17.0-35-generic / Ubuntu).

### TL;DR — Final Conclusion

- ❌ **The onboard USB-to-serial bridge chip (CH340 / CP2102) is DEAD.** The computer cannot see the board over USB at all.
- 🟡 **The ESP32 core itself is most likely still ALIVE** (powered and running), but can no longer be reached over USB.
- **This is a hardware failure, not a software/driver/kernel problem.**
- **Most probable cause:** a ground-loop or a motor/actuator voltage transient that destroyed the exposed USB-serial front-end chip while sparing the ESP32.

---

### Evidence Collected

All tests run on the host laptop: `jorge-Aspire-A315-51`, Linux 6.17.0-35-generic (Ubuntu).

#### 1. No serial port is ever created
- `ls /dev/ttyUSB* /dev/ttyACM*` → **no device present**, before or after plugging in.

#### 2. USB-serial bridge chip does not enumerate
- `lsusb` shows only the internal Bluetooth adapter and webcam.
- The ESP32's USB-serial bridge chip (CP210x / CH340 / FTDI) **never appears** in the USB device list.

#### 3. Zero kernel activity on plug-in (the decisive test)
- `udevadm monitor --kernel` watched during **multiple physical unplug/replug cycles** → **zero kernel uevents**.
- `journalctl -k` (entire boot log) contains **no serial-enumeration line** at all.
- This is the rawest hardware-detection layer, below every driver, permission, or setting. A healthy board *always* fires events here on plug-in. This board fires none.

#### 4. No USB-serial driver loaded
- `lsmod | grep -E "cp210|ch34|ftdi|usbserial|cdc_acm"` → **none loaded** (nothing triggered them, because nothing enumerated).

#### 5. Board IS receiving power
- **Power LED lights up** when USB is connected → 5V reaches the board; the power path (USB connector + regulator) is physically intact.

#### 6. Thermal readings (Milwaukee temperature sensor)
- Most of the board: **~30 °C** (essentially room temperature).
- One localized warm spot: **56 °C**, on the **back/underside, between GND and pins GPIO32–35** — i.e. directly beneath the ESP32-WROOM module.
- Interpretation: this heat is the **ESP32 SoC itself drawing current and running**. 56 °C is a normal running temperature for a powered ESP32, **not** a runaway short (a dead short would be 80–100 °C+ and climbing). Note: GPIO34/35 are input-only pins, so this is core heat, not a driven-output short.

---

### Why It Is NOT a Software / Kernel / Driver Problem

1. **Detection happens below software.** USB enumeration is a pure electrical handshake performed by the laptop's USB host-controller hardware the instant a device pulls the data line high — *before* any driver, permission, `brltty`, or `/dev/ttyUSB0`. We watched that raw layer (`udevadm monitor --kernel`) and got nothing. No software can suppress or fake that.
2. **Even with everything software broken, the kernel would still log "new USB device found."** It didn't. So the device never electrically announced itself.
3. **Software cannot make a chip warm.** A missing driver produces no device node; it does not push current through silicon. The measured heat proves real current is flowing — that is physics, not code.

Ruled out by the above: missing/wrong driver, `dialout` permissions, `brltty` port hijacking (it is installed on this laptop but is irrelevant here since nothing enumerates), wrong baud rate, wrong port.

---

### Why It Is a Hardware Failure — Component Breakdown

The board has two independent chips relevant here:

| Component | Status | Reasoning |
|-----------|--------|-----------|
| **USB-serial bridge (CH340/CP2102)** | ❌ **Dead** | Never enumerates at kernel level (tested 5×). This is the chip that translates between the ESP32 and the laptop's USB. |
| **3.3 V regulator (AMS1117) / power path** | ✅ OK | Power LED on; board is powered; no widespread heat. |
| **ESP32 SoC (WROOM module)** | 🟡 Likely alive | Powered and warm (56 °C at the module) = drawing current / running. Cannot be confirmed via USB (that path is dead). |

The USB-serial chip and the ESP32 are **separate chips**. The USB front-end can die while the ESP32 keeps working — which is exactly what the evidence shows.

---

### Probable Root Cause — What May Have Killed the USB Chip

Ordered by likelihood for a kart (motors, H-bridge, DAC, vibration, shared grounds):

1. **Ground loop / ground-potential difference (most likely).** If the ESP32 was ever connected to the laptop USB *while also powered or grounded through the kart*, and the two grounds sat at different voltages, current flows through the USB chip's ground/data pins to equalize. The USB-serial chip sits right at that junction and takes the damage, while the ESP32 (further from the USB port) survives. Matches the symptom perfectly.

2. **Voltage spike / back-EMF from motors coupling into TX/RX.** The bridge chip's data lines connect to the ESP32 UART pins (GPIO1/GPIO3). A transient from the steering motor, H-bridge, or a solenoid (inductive back-EMF) traveling down those lines can punch through the bridge chip's I/O.

3. **Over-voltage on the serial/data lines.** Something 5 V touching the TX/RX or USB data lines (the bridge I/O is 3.3 V and does not tolerate 5 V).

4. **ESD (static discharge).** The USB connector and data pins are the most exposed part of the board; a static zap during handling commonly kills only the bridge chip. Fits the "stopped working several days ago" timing.

---

### How to Confirm the ESP32 Core Is Alive (USB path is unusable)

Free checks:
- Is the LED **steady** (power only) or **blinking** (firmware running on GPIO2)?
- Does any **WiFi network / AP** appear when the board is powered? (if the firmware uses WiFi)

Definitive check — external USB-to-UART adapter (bypasses the dead onboard chip):
```
USB-UART adapter        ESP32 board
   GND      ───────────►  GND
   RX       ───────────►  TX0 (GPIO1)
   TX       ───────────►  RX0 (GPIO3)
   3V3      ───────────►  3V3   (power from adapter; do NOT also plug in USB)
```
- Adapter MUST be set to **3.3 V** logic (never 5 V).
- Open a serial monitor @ 115200 and tap EN/RST → a live ESP32 prints its ROM bootloader message.
- Or query directly (hold BOOT/IO0 to GND, tap EN, then):
  ```
  esptool.py --port /dev/ttyUSB0 chip_id
  ```
  A valid response (chip type, MAC, flash size) = ESP32 + flash fully functional and re-flashable via this adapter.

---

### Recommendation

- **Retire the onboard USB path** on this legacy board — it is conclusively dead.
- If the ESP32 core confirms alive (LED blink / WiFi / esptool responds), the board is still usable/flashable via an external UART adapter.
- Given this is the legacy/spare board, retiring it entirely is also reasonable; use the working ESP32-WROOM-32E on the kart.

#### Prevention (protect the working board)
- **Never** connect the ESP32 to laptop USB while it is also powered/grounded by the kart — this ground loop is the #1 risk. Use USB isolation, or unplug kart power before connecting USB.
- Add **flyback diodes / isolation** on any inductive load (motors, solenoids) near the ESP32's lines.
- Keep a common, solid ground; never put 5 V on any 3.3 V pin.
