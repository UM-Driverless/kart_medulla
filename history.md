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

### Session outcome (end of day): AS5600 abandoned, switching to MPS MagAlpha MA732

The PWM output was **never observed working**, and the session established why the AS5600 is the wrong part for this installation:

- **The magnet physics is the blocker, not the electronics.** The AS5600 senses the *variation* of Bz across a 1 mm Hall circle (spec: 30–90 mT of variation, detect threshold 8 mT, magnet centered ±0.25 mm, air gap 0.5–3 mm — datasheet saved at `~/dv/kart/steering/resources/as5600-datasheet-DS000365-v1.06.pdf`). The kart's shaft magnet is two large magnets stuck sideways; large magnets produce a locally uniform field (surface field is fixed by remanence at ~0.3–0.6 T regardless of size, so variation across 1 mm ≈ B × 1 mm/L). Even touching the chip, MAGNITUDE peaked ~274–2000 with MD mostly 0, ML=1 always. With MD=0 the chip gates/garbages OUT by design ("output driven low… without regard to output mode", datasheet p.31; in practice we also saw the analog DAC pinned at garbage-ANGLE≈4095 → 3.3 V).
- **Volatility proven** (runbook step 10): power-cycle reverted CONF to 0x0000; rewrote fine. RAM config write path is solid.
- **Unresolved anomalies, moot after the switch:** (1) with MD=1/MAG≈700, analog OUT measured 2.65 V where ANGLE=149 predicted 0.12 V; (2) at session end GPIO 1 read a stiff ~3.2 V that the internal pulldown couldn't budge — i.e. a hard 3V3 rail, not a sensor output through the 20 k series path. Prime suspect: the OUT wire re-landed in a +3V3 terminal (CN6.3 is a neighbour) after the rework unplug/replug; not confirmed before stopping.
- **Decision (Rubén): stop, buy an MPS MagAlpha MA732** (board TBMA732-Q-RD-00A, Mouser/DigiKey ES; fallback Melexis MLX90316 factory-PWM variant) + a proper 6 mm diametric magnet. The MA732 senses in-plane field *direction* at a point → tolerant of big magnets and sloppy centering, native PWM ~1 kHz duty=angle/360 at 3.3 V, no OTP burn needed. Full comparison: `~/dv/kart/steering/sensor-alternatives-research.md`.

**Sourcing correction (2026-07-12): the part actually going in the AliExpress ~€10 cart is the MT6701 module, NOT the MA732.** Link: `https://es.aliexpress.com/item/1005004216051325.html` — the "MT6701 magnetic encoder, perfectly replace AS5600" board, which is the same **-22 mm / -42 mm** breakout the very-first question in this session was about. The MA732 was the sensor-research *pick* (Mouser/DigiKey), but the MT6701 is the same in-plane-direction sensing class (verified `datasheets/MT6701_datasheet.pdf`), so it solves the identical big-uniform-field-magnet problem and is far cheaper/available on AliExpress. MA732 kept as the documented alternative/fallback. Verify on arrival: (1) genuine part, (2) whether it ships with a **diametric** magnet (many of these boards include only a small one or an axially-magnetized one).
- **Everything reusable survives:** CN5.2 → GPIO 1 path with R10 removed (R8+R9 = 20 k series, full-swing verified topology), the bench tool (`~/dv/kart/steering/as5600-bench/`, works for any PWM sensor via `w`/`d`; I²C commands are AS5600-specific), and the kart-brain firmware plan (MCPWM capture on GPIO 1, period sanity check, median-of-5) — unchanged by the sensor swap.

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

## 2026-07-12 — magnet sizing idea for encoders (big magnet + large air gap for vibration tolerance)

Idea for **vector / multi-Hall angle sensors like the MT6701** (3-axis-style sensing — measures the *direction* of the in-plane field, not its magnitude): you can deliberately run a **bigger, stronger magnet at a larger air gap** to buy mechanical robustness on a vibrating kart, as long as the flux at the die still lands in the sensor's recommended band. Why it helps, both axes:
- **Axial (gap) wobble:** at a large gap the magnet sits on the flatter part of the field-vs-distance curve, so a given wobble changes die flux by a smaller fraction → more margin before it drifts out of band.
- **Lateral (off-center) wobble:** angle error from a sideways shift δ scales roughly as δ / gap, so a bigger gap means less angular error per unit of sideways play.

Requirements that don't change: magnet must stay **diametrically magnetized** (poles across the diameter) and **centered on the rotation axis, facing the chip** (on-axis). Caveats: going well outside the datasheet's recommended magnet size / gap range means the 14-bit accuracy isn't characterized (bench-verify); and a physically bigger/stronger magnet couples more to nearby ferrous parts (chassis, motor iron), which can skew the field direction at the die — geometry-dependent, check in place.

**Correction to an earlier wrong instinct:** at *equal die flux*, a far magnet does NOT have worse signal-to-stray-field ratio than a close one — signal amplitude is the same, so stray-field robustness is the same. The "far magnet = weaker signal" reasoning only holds if you *don't* size the magnet up to compensate.

**Why the AS5600 is different — VERIFIED from its datasheet** (`datasheets/AS5600_datasheet.pdf`, ams v1-06 2018-Jun-20). The AS5600 does NOT get the big-magnet-large-gap benefit, and the datasheet says why:
- **Detailed Description (p9):** "The AS5600 is a Hall-based rotary magnetic position sensor using **planar sensors that convert the magnetic field component perpendicular to the surface of the chip** into a voltage." So it senses **Bz — the axial (perpendicular) field component**, not the in-plane direction.
- **Magnetic Characteristics (Fig 11, p8):** the spec is **Bz = required orthogonal component of the field "measured at the die's surface along a circle of 1 mm", 30–90 mT** (min for magnet detection Bz_ERROR = 8 mT). Note on p5 calls out "typical magnetic field (60 mT)".

Mechanism: the AS5600 has planar Hall plates arranged around a **1 mm-diameter circle** and reconstructs the angle from how the *axial* field Bz varies azimuthally around that tiny circle (a diametric magnet makes Bz go +/− across the die; the phase of that pattern = the angle). For this to work the field must have real **spatial structure at the ~1 mm scale** over the die. A **small magnet, close in** gives strong Bz variation over the 1 mm footprint → in the 30–90 mT band. A **big magnet far away** makes the field too spatially uniform over 1 mm, so the orthogonal component over the circle drops below 30 mT → weak/failing signal, even if the raw field magnitude elsewhere is high. That is the physical reason the AS5600 wants a small magnet at a close, specified gap — the "big-and-far for vibration tolerance" trick does NOT transfer to it.

**MT6701 mechanism — VERIFIED from its datasheet** (`datasheets/MT6701_datasheet.pdf`, MagnTek Rev 1.5 2021.03). General Description (p1): *"an IC based on Hall sensing technology. A **rotating magnetic field in the x-y sensor plane** delivers two sinusoidal output signals indicating the **angle (α) between the sensor and the magnetic field direction**."* So it measures the **in-plane (x-y) field direction at a point** — no 1 mm gradient required, unlike the AS5600's Bz-over-a-circle scheme. Magnetic Input Specs (§5, p8): Bpk = in-plane field amplitude at the IC surface **200–1000 Gauss (20–100 mT)**, recommended magnet **Ø6 × 2.5 mm** diametric, air gap **0.5–2.0 mm** (typ 1.0), off-axis misalignment **≤ 0.3 mm**.

**Correction to my earlier "big magnet + large gap" framing (above):** the MT6701 datasheet recommends essentially the *same* small magnet, tight air gap and centering as the AS5600 — it does **not** license running an arbitrarily large magnet at a big gap. The robustness that matters is not "bigger/farther is fine" but the **sensing principle**: direction-sensing doesn't collapse when the field over the die is strong-and-uniform, whereas the AS5600's gradient-sensing does. Treat the big-and-far idea as a mild vibration-margin lever within the spec'd range, not a licence to ignore the recommended magnet/gap.

### Team comparison — why the MA732 (and the MT6701 class) fits the kart and the AS5600 does not

Both families are 3.3 V magnetic rotary angle chips reading a diametric magnet on the shaft end, and both datasheets recommend a similar Ø6 mm magnet at ~0.5–2 mm gap. The difference is **what physical quantity each one measures**, and that decides everything on our crude mount:

| | AS5600 (tried, abandoned) | MA732 (our pick) / MT6701 (same class) |
|---|---|---|
| Sensing principle | Variation of the **perpendicular** field (Bz) sampled around a **1 mm circle** on the die | **Direction** of the **in-plane** (x-y) field at a point |
| Needs a field *gradient* over ~1 mm? | **Yes** — that gradient *is* the signal | **No** — only the field's direction matters |
| Big / uniform-field magnet | **Breaks it** — uniform field = no Bz variation → chip reports "no magnet" (MD=0) → output gated to garbage | **Fine** — a strong uniform field still has a well-defined direction |
| Off-center / sloppy mount | Punishing (±0.25 mm) | More forgiving in practice; direction survives offset better |
| Output for us | I²C / analog / PWM, **needs OTP burn** to set PWM | Native **PWM ~1 kHz at 3.3 V, no burn** (MA732); MT6701 adds ABZ/SSI/UVW, 14-bit |
| Verified from | `datasheets/AS5600_datasheet.pdf` p8–9 | `datasheets/MT6701_datasheet.pdf` p1, p8 |

**Why this is the whole story for our kart:** our shaft "magnet" is two large magnets stuck sideways, which produces a locally **uniform** field over the chip. The AS5600 needs the field to *change* across its 1 mm sensing circle to compute an angle — a uniform field gives it nothing, so it read MD=0 and never produced a valid output on the bench. The MA732/MT6701 only need the field's **direction**, which a big magnet defines strongly and cleanly, so the same crude mount that starves the AS5600 works for them. That mechanism difference — not resolution or price — is why we switch. (Caveat kept honest: the MA732/MT6701 still want a reasonably centered magnet at a sane gap; they tolerate our *big-uniform-field* problem, not arbitrary sloppiness.)

### Off-axis tolerance — what happens past the misalignment spec (and MA732's edge)

Question that came up: if the magnet sits outside the MT6701's ≤ 0.3 mm off-axis spec, does it fail? **No — it degrades gracefully, not a cliff.** Off-axis misalignment on a direction sensor adds a **smooth, systematic angle error** (mostly a once-per-rev sinusoidal distortion) that grows gradually with displacement; the angle stays continuous, repeatable, and usable — it does *not* drop out or go to noise. This is a different failure class from the AS5600's magnet problem, which was a *detection* failure (MD=0 → gated garbage). The ≤ 0.3 mm number is only where the datasheet **guarantees rated accuracy**; past it you lose degrees of accuracy, not function. Two mitigations already in play: a **bigger magnet is more off-axis-tolerant** (more uniform field direction across the offset), and the error is systematic so it can be **calibrated out** if ever needed. For kart steering (a few degrees is acceptable), this is fine.

**MA732 fallback — VERIFIED from its datasheet** (`datasheets/MA732_datasheet.pdf`, MPS Rev 1.1, 2022-08-08). Notably more tolerant than the MT6701 for our crude mount:
- Description (p1): *"detects the absolute angular position of a permanent magnet, typically a diametrically magnetized cylinder… supports a **wide range of magnetic field strengths and spatial configurations. Both end-of-shaft and off-axis (side-shaft mounting) configurations are supported.**"* So for the MA732, **off-axis is a designed-for mode, not out-of-spec** — a real edge over the MT6701 (which specs on-axis, ≤ 0.3 mm). But it is **not automatic**: side-shaft use requires configuring the **BCT** register (8-bit bias-current-trim) plus the **ETX/ETY** enables (Table 9, p18) to rebalance the X vs Y Hall-element gains for the off-axis field asymmetry (an elliptical field locus). It's a one-time manual calibration — a trim value tuned empirically (MPS eval board + GUI), stored in NVM — not a distance you enter. The MT6701 has **no equivalent trim**, so if the mount ends up off-axis, the MA732 is the only one of the two that can be corrected.
- Electrical (p4): Applied magnetic field **B = 40 (min) / 60 (typ) mT**; VDD 3.0–3.6 V, IDD ~11.7 mA typ. Same in-plane direction-sensing principle as the MT6701.
- Outputs (pins, p3): native **PWM (14-bit) on pin 9**, ABZ incremental (12-bit, 1–1024 PPR) on A/B/Z, SPI + SSI for absolute readout, plus **MGL/MGH** field-strength flags (pins 11/16) for magnet diagnostics. No OTP burn needed. QFN-16 3×3 mm.
- Trade-off vs the €2.59 MT6701 board: MA732 is a bare QFN chip from Mouser/DigiKey (needs a breakout/PCB), pricier and slower to get. **Plan: run the cheap MT6701 module first; if its on-axis mounting fights our sideways-magnet geometry, the MA732's off-axis support is the fallback.**

## 2026-07-12 — why the PlatformIO venv is pinned to setuptools<81 (kept on purpose)

`pio run` for any **espidf** env (e.g. `esp32dev`) was failing instantly with `ModuleNotFoundError: No module named 'pkg_resources'`. Root cause: the pinned **espressif32@6.4.0** platform builder does `import pkg_resources` (used once, at `~/.platformio/platforms/espressif32/builder/frameworks/espidf.py:1157`, to enumerate installed packages via `pkg_resources.working_set`). `pkg_resources` ships with setuptools, and **setuptools 81 removed it**; the venv had setuptools 82.0.0, so the import blew up before compiling anything. (Arduino-framework builds were unaffected — only the espidf builder imports it, which is why the standalone AS5600 bench sketches flashed fine while the firmware build didn't.)

**Fix applied and kept:** `pip install "setuptools<81"` into `~/.platformio/penv`. This restores `pkg_resources`; `pio run -e esp32dev` then builds (verified: SUCCESS, RAM 7.2% / Flash 26.3%).

**Why we keep the pin instead of "modernizing":**
- The `pkg_resources` line is **not our code** — it's inside the installed espressif32@6.4.0 platform under `~/.platformio/`. Hand-patching it is untracked and gets overwritten on any platform reinstall/update, so it's not durable.
- The real modernize path is **upgrading the espressif32 platform** to a release that dropped `pkg_resources`. But `platformio.ini` pins `espressif32@6.4.0` deliberately: that pin locks the **ESP-IDF 5.1 toolchain the firmware was validated against**. Bumping it moves the compiler/IDF under the firmware — a rebuild-and-re-validate-on-the-kart event (per the branch workflow, that gates `main`), not a dependency cleanup.
- Pinning `setuptools<81` is **isolated to `~/.platformio/penv`** (doesn't touch system Python or other projects), reversible, and changes nothing about the firmware toolchain.

Decision (Rubén, 2026-07-12): keep the setuptools pin as-is. Revisit only as a deliberate platform-upgrade task if/when we choose to move off espressif32@6.4.0, at which point the pin can be dropped.

## 2026-07-12 — AS5600 PWM bench: chip + firmware good, but OUT→GPIO1 is an OPEN circuit

Bench state during this session (medulla PCB, ESP32-S3 = U24, AS5600 module external): I²C on GPIO8/9, AS5600 OUT wired into **CN5.2**. **R10 already removed** (per the steering runbook rework); **R8 + R9 remain**.

**Verified board topology (from live netlist export of `dv-hardware/.../kart-medulla.kicad_sch`):**
`CN5.2 —[R8 10k]— nodeA —[R9 10k]— GPIO1 (U24.19) —[R10 10k]— GND`. So R8 **and R9 are both series**; **only R10 was the shunt/pulldown to GND**. (Correcting the runbook, which called "R9+R10 the pulldown pair" — wrong; removing R9 would break the signal path. The rework actually done — remove R10 only, keep R8+R9 — is the correct one.)

**Diagnosis (all software, no scope):** AS5600 is healthy — I²C responds at 0x36, STATUS MD=1/ML=0/MH=0, AGC=53 (mid-range), RAW tracks the magnet. CONF `OUTS` was set to PWM (and toggled PWM↔analog) — register readback confirms. **But GPIO1 never follows OUT:** it reads a constant HIGH in both PWM and analog modes (in analog at ~21° it should read LOW), and the decisive test — enabling the ESP32 **internal pulldown** on GPIO1 — pulls it to 0, i.e. the node is **floating, nothing driving it**. Conclusion: the OUT signal does not reach GPIO1 → an **open connection** in `OUT pad → wire → CN5.2 → R8 → R9 → GPIO1`. Most likely an R8/R9 joint disturbed when R10 was desoldered, or the OUT wire not truly clamped at CN5.2. Needs a physical continuity check / reflow — not a firmware fix.

**Also fixed this session:** PlatformIO espidf builds (setuptools<81 pin — see earlier entry). Bench test harness = standalone PlatformIO arduino project in scratchpad, flashed to `/dev/cu.usbmodem*`.

**Context flag:** the steering runbook (`dv/kart/steering/as5600-pwm-burn-runbook.md`) is marked SUPERSEDED 2026-07-12 — AS5600 retired for the kart (can't handle the large shaft magnet), MA732 chosen. This PWM bench work is proving the path/firmware on a small bench magnet, not committing the AS5600 to the kart.

### 2026-07-12 (cont.) — AS5600 CONFIG burned to PWM; confirms the fault is wiring, not config

At Rubén's request (module expendable — MT6701 modules already bought), performed the one-shot **BURN_SETTING** (0x40→0xFF) to commit OUTS=PWM + PWMF=920 Hz to OTP. Preconditions clean: ZMCO=0, MD=1. Verified with the datasheet's OTP-reload sequence (write 0x01,0x11,0x10 → 0xFF, then read CONF): **CONF persisted = 0x00E0 (OUTS=2 PWM, PWMF=3 920 Hz).** Burn successful and permanent.

**Result: GPIO 1 still floats (lvl=1, no PWM edges) even with PWM permanently burned.** This is the definitive proof — the sensor is healthy and permanently emitting PWM on OUT, yet nothing reaches GPIO 1 → the fault is a physical **open connection** in `OUT pad → wire → CN5.2 → R8 → R9 → GPIO 1` (most likely an R8/R9 joint disturbed when R10 was desoldered, or the OUT-wire contact). Confirmed by datasheet p24 "Output Stage": a volatile OUTS write already drives OUT ("effective at the output ≥1 ms later"), so OTP was never the blocker.

Note: module is still I²C-usable (ANGLE readable regardless of output mode) and now hard-wired to PWM on OUT. Per the superseded runbook, the kart path is the **MT6701** anyway. To actually get PWM here: bypass test (land OUT directly on a bare GPIO) to localize the open, then reflow R8/R9 / reseat — or move to MT6701.

## 2026-07-12 — MT6701 PWM signal: cabling distance (decided: <2 m, fine)

The MT6701 PWM output (`datasheets/MT6701_datasheet.pdf` §6.6) is a **single-ended 3.3 V CMOS square wave**, frame rate **994.4 Hz** (or 497.2 Hz via PWM_FREQ=1). Angle is encoded in the **duty cycle**: frame = 4119 clock periods, 12-bit, **1 clock period = 0.088° = 244 ns** (or 488 ns if slowed). So the **angle LSB is a 244 ns slice of edge timing** — that, not signal presence, is what limits cable length.

Distance is capped by **edge-timing integrity**, not whether the wave arrives: cable capacitance (~50–100 pF/m) rounds edges → systematic duty (angle) offset; EMI jitter on the threshold crossing → direct angle noise (a kart is noisy). It degrades *gracefully* — you lose LSBs / gain jitter before you lose the signal. No max length is spec'd; it's application-dependent.

Guidance: short harness ~1–3 m plain wire is fine in a quiet setting; **on the kart keep it short + shielded/twisted-pair (signal twisted with its ground) + solid common ground.** To stretch: drop to 497 Hz (488 ns LSB = 2× timing margin) and/or a Schmitt buffer at the receiver. For a truly long run: use ABZ-into-counter or digitize-at-sensor over CAN — don't run PWM across the kart. The ESP32 MCPWM capture (~12.5 ns tick on GPIO 1) resolves the 244 ns LSB easily, so the bottleneck is the cable/noise, not the MCU.

**Decision (Rubén): the steering sensor → medulla run is < 2 m, so plain PWM is fine** — no CAN/line-driver needed. Keep the lead short and ideally shielded.

### 2026-07-12 (correction) — AS5600 OUT is STUCK HIGH (3.3 V), not open; output stage is the fault

Earlier this session I concluded the OUT→GPIO1 path was an open circuit. **That was wrong** — corrected by Rubén's multimeter: OUT pad measures a constant **3.3 V** while powered, and the OUT→GPIO1 path is a continuous **20 kΩ** (R8+R9, R10 removed). Both together mean OUT is being **held at 3.3 V**, not floating.

Why my "floating" test misled me: I read GPIO1 with the ESP32's internal pulldown (~45 kΩ). A hard 3.3 V source through the 20 kΩ series, divided by 45 kΩ, sits at **2.28 V** — just under the S3's V_IH (0.7×3.3 = 2.31 V) — so it read as 0 and I mis-called it a float. Lesson: **a high-value series R (20 kΩ) makes a driven line look floating under an internal pulldown — account for the divider before concluding "open."**

Corrected diagnosis: the AS5600 digital core is healthy (I²C, RAW angle tracks, MD=1), but its **OUT output stage is stuck at 3.3 V** — no PWM toggling (confirmed: no edges on any pin, ADC pinned 4095, DMM = 3.3 V), and not valid analog either (should be ~1.8 V at 195°). Everything on our side is verified good — wire (20 kΩ), pin (GPIO 1 = physical pin 19 per kart-docs `assembly/electronics/kart-medulla/index.md`), config (OUTS=PWM burned), firmware. The fault is the module's OUT driver (defective, or a module pull-up its ±0.5 mA output can't beat). Note: this was true **before** the burn too, so the burn didn't cause it.

**Resolution:** AS5600 OUT is unusable → switch to the **MT6701** (already in hand, plan of record). This matches the runbook's superseded decision.

## 2026-07-15 — AS5600 re-wired via I²C: whole bus reads NACK (shared-bus fault, physical)

Rubén re-wired the AS5600 to the medulla PCB over I²C (SDA=GPIO8, SCL=GPIO9) and asked to
confirm it. Flashed the existing Arduino bench tool (`~/dv/kart/steering/as5600-bench/`, board on
`/dev/cu.usbmodem5C372070281`) and ran the `s` health check. Note: `pio` IS installed at
`~/.platformio/penv/bin/pio` — the AGENTS.md "no PlatformIO on the Mac" note is stale for the
Arduino-framework bench project (the espidf `pkg_resources` bug is separate).

**Symptom:** I²C address scan ACKs the real devices (`0x20` PCF8574, `0x36` AS5600) but **every
register read NACKs** — first transaction gives a genuine "I2C hardware NACK detected", then the
ESP-IDF new-`i2c_master` driver wedges into `ESP_ERR_INVALID_STATE` (err 259) for the rest of the
run. Consistent across 3 power-cycles. Scan also grows **phantom addresses** (`0x08`, then
`0x08`+`0x09`) — a marginal/noisy bus.

**Key diagnostic:** the **on-board PCF8574 (0x20) fails its reads too**, not just the AS5600. The
PCF8574 wasn't touched by the re-wire, and this same bench tool read AS5600 registers fine on
2026-07-12 (STATUS MD=1, AGC=53, RAW tracking). So the fault is the **shared I²C bus, introduced by
the re-wiring** — not the AS5600 module and not firmware. Address-ACK-but-data-NACK for *all*
devices = the bus survives one lenient byte but not a repeated-start multi-byte read → weak/missing
pull-ups or a loose/shorted line.

**Physical checks to run (DMM, power off), in likelihood order:**
1. **SDA↔SCL short** (GPIO8↔GPIO9) from a solder bridge / pinched wire — must not be continuous.
2. **Pull-ups**: SDA→3V3 and SCL→3V3 should read ~2.2k–10k. Did the re-wire drop whatever module
   carried the bus pull-ups?
3. **Marginal SDA/SCL joint** — re-seat/reflow the GPIO8/GPIO9 run from module to CN.
4. **Common GND** between AS5600 module and board.

Not diagnosed further in software — no firmware flash fixes a bus-level electrical fault. Left for
a continuity/pull-up check on the bench.

(Reminder of the bigger picture: the AS5600 stays retired for the *kart* — big shaft magnet →
MD=0; MT6701 is plan of record. This I²C bench work is proving path/firmware on a small bench
magnet, so a working I²C read here is still useful.)

### 2026-07-15 (cont.) — Confirmed: the 1 m AS5600 branch was poisoning the bus

Binary-search test. Rubén checked #1 (3.3 V present; **no SDA↔SCL short** — continuity clear), then
**unplugged the AS5600** and re-scanned. Result: scan finds **only 0x20 (PCF8574)** and the **phantom
addresses 0x08/0x09 are gone**. So the ~1 m unplugged branch was the fault — with it removed the
on-board bus is clean. (The tool's trailing `STATUS read failed` is now expected: the `s` health
check reads AS5600 registers at 0x36, which is absent while unplugged.)

**Root cause: plain I²C over ~1 m.** Cable capacitance slows SDA/SCL rise times so the ACK bit is
misread → data-phase NACK for every device on the shared bus, plus phantom addresses from the sloppy
edges. On-board I²C is fine; the metre of wire is the whole problem.

Bench gotchas learned this session:
- The board's reset button is silkscreened **RST** (not EN) — same function. Buttons are RST + BOOT.
- Toggling DTR/RTS from pyserial to "reset" the board wedged it into a non-responsive state (no boot
  banner, esptool "No serial data received"). Recovery was a physical **USB unplug/replug** power
  cycle. Prefer a plain power cycle or the RST button over scripted DTR/RTS toggling for this board.

**Fix options for reading the sensor 1 m away (unchanged from the analysis):** slow I²C to 100/50 kHz,
stronger pull-ups (~1–2.2 k), an I²C buffer (P82B715 / LTC4311), or — best — use a distance-friendly
output (MT6701 SSI/ABZ/PWM) instead of I²C. The MT6701 is already plan of record for the kart.

### 2026-07-15 — Plan for tomorrow (bench layout change)

Move everything back to the **front** and run the **compressor wiring** (which stays in the back)
through an **extender**, so the steering system and compressor pressures can both be exercised on the
bench.

Implication: this puts the AS5600 **close to the medulla → short I²C run**, which should make today's
1 m bus-poisoning problem (NACKs + phantom 0x08/0x09) go away without any buffer/pull-up changes. The
long run shifts to the **compressor on GPIO 3** (`CMD_COMPRESSOR_PWM`, CN8.2) — a MOSFET-gate on/off
PWM signal that tolerates an extender fine (keep its ground solid). Goal for the session: bench the
steering loop (I²C AS5600) and play with compressor pressures.

## 2026-07-16 — Short I²C run CONFIRMED fixed; AS5600 now reads clean but sees NO MAGNET

The bench-layout change worked as predicted. With the ESP32-S3 moved to the front and the AS5600
wired over a short I²C run (SDA=GPIO8, SCL=GPIO9, bus at 100 kHz), the AS5600 bench tool
(`~/dv/kart/steering/as5600-bench/`) reports:

- **I²C scan finds `0x20` (PCF8574) + `0x36` (AS5600), no phantom `0x08`/`0x09`, and every register
  read succeeds.** This closes the 2026-07-15 fault: the data-phase NACK on *every* device was
  cable capacitance on the ~1 m branch, and shortening the run fixed it with **no buffer, no
  pull-up change, no I²C speed workaround**. `d` (pin diag) shows GPIO8/GPIO9 both held HIGH
  externally = bus connected.
- **But `STATUS = 0x13` → MD=0 (NO MAGNET), ML=1 (field TOO WEAK).** `MAGNITUDE ≈ 20` and
  `AGC = 128` (gain pushed high, i.e. hunting for a field that isn't there). `RAW_ANGLE` sits at
  ~2140–2156, jittering ±10 counts on the spot — **noise, not tracking**. With MD=0 the AS5600
  gates its output, so this angle is garbage, not a reading.
- For contrast, the same tool on 2026-07-12 with a small bench magnet read **MD=1, AGC=53, RAW
  tracking**. So the chip and the firmware are fine; the *magnetic coupling* is what's absent now.

**Two candidate causes, not yet distinguished:** (a) no magnet is physically near the die at the
moment (nothing mounted / gap far too large), or (b) the known AS5600-vs-big-shaft-magnet failure —
a large magnet makes the field spatially uniform over the 1 mm sensing circle, so Bz variation
collapses and the chip reports MD=0 (mechanism verified from the datasheet, see the 2026-07-12
entries above). MAGNITUDE ≈ 20 is low enough that (a) is the more likely reading — a big-but-uniform
field usually still registers *some* magnitude.

**Discrepancy worth flagging: `CONF` reads back `0x0000` (OUTS=analog 0-100%, PWMF=115 Hz).** On
2026-07-12 a permanent one-shot `BURN_SETTING` committed `CONF = 0x00E0` (OUTS=PWM, PWMF=920 Hz) to
OTP on the module in use then, verified after an OTP reload. An OTP burn cannot revert. Therefore
**the module now on the bench is a different physical AS5600 than the one burned on 2026-07-12** —
which also means the "OUT output stage stuck at 3.3 V" verdict from that session does not
automatically apply to this unit. `ZMCO = 0` (no angle burns) is consistent with a fresh module.

Also note `GPIO1` (the PWM-angle input, CN5.2) reads pullup=3095 mV / pulldown=12 mV / open=14 mV →
**floating**, as expected: this module's OUT is in analog mode and nothing is landed on CN5.2. The
PWM path is not in play here; this session is the **I²C** path only.

**Status: the I²C transport is proven working — the sensor is not, because no magnet reaches it.**
Next step is mechanical, not electrical: confirm what magnet (if any) is in front of the chip and at
what gap. A small diametric magnet at ~1 mm should immediately flip MD to 1 and make RAW track;
`m` (live monitor) shows this in real time. (Bigger picture unchanged: the AS5600 stays retired for
the *kart* — the MT6701 is plan of record. This bench work proves the path and firmware.)

## 2026-07-16 — Compressor control and ESP32-S3 build fixes

**Compressor Logic Implemented**:
Added hysteresis logic in `main.c` `control_task` to read `PIN_PRESSURE_1` via ADC and toggle `PIN_CMD_COMPRESSOR`. The logic turns the compressor on below 1 bar and off above 2 bar. (ADC thresholds `ADC_1_BAR` and `ADC_2_BAR` are placeholder integers assuming a linear 3.3V map and require calibration).

**S3 Build Environment Fixed**:
The S3 build (`esp32-s3-devkitc-1` target in PlatformIO) was previously broken due to unmapped pins and missing drivers for the S3 chip. Fixed the following to make the S3 the fully supported, primary build:
- Disabled the onboard DAC calls (`dac_output_voltage`) via `#ifdef CONFIG_IDF_TARGET_ESP32`, as the S3 lacks an internal DAC and relies on the external MCP4922 via SPI.
- Replaced hardcoded classic ESP32 I2C pins (`GPIO_NUM_21` and `GPIO_NUM_22`) with the macros `PIN_I2C_SDA` and `PIN_I2C_SCL` in the AS5600 `KM_SDIR_Begin` initializer so it dynamically maps correctly for the S3.
- Fixed an SPI undeclared macro error by replacing `SPI_HOST` with `SPI2_HOST` in `km_gpio.c` and including `driver/spi_master.h` to satisfy the `spi_device_handle_t` requirement.

The firmware now builds successfully for `esp32-s3-devkitc-1`.
### 2026-07-16 (cont.) — Fixed ESP32-S3 ADC readings and Compressor Logic

The `pres_adc` readings were continuously showing exactly `0`, and the compressor control loop was failing to operate. Both issues were root-caused and fixed:

1. **ADC Pin Mapping:** The `KM_GPIO_ReadADC` function in `km_gpio.c` was hardcoded to a switch statement mapping the *classic* ESP32 pins. `PIN_PRESSURE_1` (GPIO6 on the S3) was falling through to the `default` case and returning `0`. Added an `#if defined(CONFIG_IDF_TARGET_ESP32S3)` block to map the S3 GPIO pins to their correct `ADC1` channels (e.g., GPIO6 to `ADC1_CHANNEL_5`).
2. **Missing ADC Initialization:** The ESP-IDF requires explicit configuration of the internal ADC before it can return raw readings (`gpio_config` for input mode is not enough). The code was completely missing `adc1_config_width()` and `adc1_config_channel_atten()`. These were added to `KM_GPIO_Init` in `km_gpio.c`, initializing all configured analog pins to 12-bit width and `ADC_ATTEN_DB_11` (0-3.3V range).
3. **Safety Watchdog Skip:** The compressor control logic in `main.c` was originally placed at the very bottom of `control_task`. A safety watchdog placed above it was returning early `if (comms_stale || mission == MISSION_MANUAL)`. During bench-testing without an Orin connected, `comms_stale` evaluated to true, meaning the ADC read and compressor toggle logic was *never* executing. Moved the compressor logic above the safety watchdog. The compressor now builds pressure regardless of the Orin heartbeat (safety-critical since brake pressure is required even in manual/idle modes).

With these fixes flashed, the bench setup successfully reads valid analog ADC pressure values and turns the compressor on until target pressure is reached.

## 2026-07-16 — Comms drop when compressor starts (Hardware Bug / TODO)

**Symptom:** When the compressor starts running, telemetry comms from the ESP32 stop arriving immediately.
**Context:** The ESP32 is powered directly via USB from the Mac, not from the kart's 12V supply. The issue happens exactly when the compressor switches *ON* (so it is not the flyback diode failing on inductive kickback when turning OFF).
**Possible Causes (Root Cause Identified):**
1. **Ground Bounce / Ground Loop (Confirmed Highly Likely):** The compressor has a massive inrush current (locked rotor amps) when it first clicks on. Because the USB cable connects the Mac's ground to the ESP32's ground, and the circuit also ties the ESP32's ground to the kart's 12V step-down regulator ground, all grounds are linked. When the compressor surges (e.g. 30+ Amps), that dirty high-current tries to return to the regulator. Due to wire resistance (V = I * R), the local ground potential of the ESP32 is dragged up. A chunk of that heavy return current sees the Mac's USB ground as an alternate path to "zero volts" and shoves its way up the tiny USB wire. The Mac's safety circuitry detects this out-of-spec voltage/current on the USB data/ground lines and instantly drops the port to protect the motherboard.
2. **EMI (Electromagnetic Interference):** The massive inrush current creates a strong magnetic pulse that could couple into the USB cable, but the ground-loop theory perfectly explains the port dropping instantly on turn-ON.

**Resolution / Next Steps:**
- **Star Ground Topology:** Wire the compressor's ground directly back to the main 12V regulator in the rear of the kart. This gives the massive current a dedicated, low-resistance path home so it completely bypasses the sensitive logic/USB grounds.
- **Dedicated GND Nets (Wiring Harness Rule):** We need to strictly separate **"Power GND"** (dirty, high-current: motors, actuators, compressor) and **"Signal GND"** (clean, low-current: ESP32, sensors, logic). They must only meet at a single "Star" point at the regulator. Using different color codes in the harness (e.g., standard Black for Power GND, and Gray or Green/Black for Signal GND) will prevent accidental mixing in the future.
- **Bench Mitigation:** Run the ESP32 from the kart's step-down converter (no laptop USB connected) and read telemetry over wireless/isolated CAN to definitively prove the USB ground loop.

## 2026-07-16 — Compressor hardware v2 improvements & Software Soft-Start strategy

Following the ground-loop bug analysis, several mitigations were agreed upon to handle the massive inrush current from the portable Xiaomi-style compressor:

**Software Soft-Start (Chosen Solution):**
Since the compressor is driven by a MOSFET with a freewheeling diode, snapping it instantly to 100% duty cycle at 12V causes a "dead short" condition because the stationary motor has zero back-EMF, causing a 40-50A spike. We decided to implement a software soft-start:
- `PIN_CMD_COMPRESSOR` will be reconfigured from a digital GPIO to an LEDC PWM output.
- When pressure drops below 1 bar, `control_task` (running at 100 Hz) will increment the duty cycle by a small step (e.g., +2% per tick) to ramp the motor from 0% to 100% over ~500ms.
- This allows the motor to spin up and build back-EMF, choking off the current and preventing the 12V regulator brownouts and USB ground-loop disconnects entirely.

**Hardware V2 PCB Improvements (Documented in dv-hardware tasks.md):**
For the next revision of the kart-medulla PCB, the following design changes were finalized:
1. **Bulk Capacitance:** Even with soft-start, PWM switching causes high-frequency noise. V2 will include footprints for bulk electrolytic capacitors (e.g., 2x 4700µF 35V in parallel) on the 12V rail adjacent to the compressor MOSFET to act as a local energy reservoir. (SMD 22µF caps are insufficient for motor transients; 25V/35V rating is mandatory to survive inductive spikes).
2. **Dedicated Compressor MOSFET:** Add a second on-board power MOSFET with proper cooling clearance specifically for the compressor, including a built-in flyback diode (e.g., 3A+ Schottky) directly across the output terminals.
3. **Pin Repurposing:**
   - Skip the unused `BUZZER` signal (GPIO 3) and route it as `CMD_COMPRESSOR_PWM`.
   - Skip `PRESSURE_3` (GPIO 1) and explicitly route it as the PWM input for the AS5600/MT6701 steering sensor.
4. **Isolated GND Nets:** Physically separate the "Power GND" and "Signal GND" traces on the PCB, meeting only at the main power input connector to enforce Star Grounding.

## 2026-07-16 — Compressor soft-start implemented (firmware)

Implemented the software soft-start planned in the entry above. `CMD_COMPRESSOR_PWM` (GPIO 3) is no
longer a digital on/off output; it is now an LEDC PWM output on its own timer.

**What was built:**
- `km_gpio.c`: GPIO 3 moved from `gpio_config()` + `WriteDigital` to `LEDC_TIMER_1` /
  `LEDC_CHANNEL_1`, 8-bit duty, `COMPRESSOR_PWM_FREQ_HZ` = 500 Hz. It needs its own timer because
  the steering PWM on `LEDC_TIMER_0` runs at a different frequency (1 kHz), and an LEDC timer
  carries one frequency for every channel attached to it. `KM_GPIO_WritePWM()` now dispatches on
  the pin, so it drives either output.
- `main/main.c`: `control_task` ramps the duty linearly 0 → `COMPRESSOR_DUTY_RUN` over
  `COMPRESSOR_SOFT_START_MS` (1000 ms) starting from the rising edge of the pressure hysteresis.
  The ramp is computed from elapsed ticks, not a fixed per-cycle increment, so it stays a 1 s ramp
  if the control task period is ever retuned. (The original plan said "+2% per tick at 100 Hz", but
  the control task actually runs at 2 ms / 500 Hz — a per-tick step would have ramped 5x too fast.)
- Telemetry: `ESP_ACT_STEERING` payload grew from 4 to 5 int32 (duty 0-255 appended).
  `read_telemetry.py` decodes 12/16/**20**-byte variants, so the ramp is visible live during the
  bench test.

**Why 500 Hz.** The MOSFET gate is driven straight from a 3.3 V GPIO through a series resistor, so
it switches slowly and never fully enhances — every edge costs real energy. 500 Hz (2 ms period)
sits between the two limits: edges take a few µs so under 1% of the period is spent in transition
(at 20 kHz it would be ~10% and the MOSFET cooks), while the motor's electrical time constant of a
few ms still filters a 2 ms period into fairly smooth current. Below ~200 Hz the current starts
pulsing hard again, which is the 12 V rail disturbance the soft-start exists to prevent. If the
MOSFET runs hot, lower the frequency before lowering the duty cap — it cuts switching loss without
cutting airflow.

**`COMPRESSOR_DUTY_RUN` = 153 (60%) is the permanent operating point, not a cap to lift later.**
*(Corrected same day — this entry first claimed 60% was a MOSFET thermal experiment that could be
raised toward 255 once the part ran cool. That was wrong, and dangerously so. Recording the error
because the mistake is instructive: the duty had an electrical purpose that firmware alone doesn't
reveal, and I invented a plausible reason for the number instead of asking what it was for.)*

The real reason: **the compressor motor is a 7.5 V part and the rail is 12 V.** The duty cycle is
what steps the voltage down. The pulse height is always 12 V — duty changes how long it is there,
not how tall it is — so 7.2 V is not a voltage that exists anywhere in the circuit; it is the
equivalent DC. The motor integrates the pulses rather than following them, because its current
never stops: at each turn-off the winding's inductance keeps current circulating through the
flyback diode, and with a few-ms electrical time constant against an 0.8 ms off-time it barely
decays. So the motor draws smooth current set by the average — duty x rail = 0.60 x 12 V = 7.2 V,
just under its 7.5 V rating. (Average, not RMS: RMS would be right for a resistive load, which has
no inductance to carry current through the off-time, and would give 12 x sqrt(0.6) = 9.3 V.) Full
duty would put a sustained 12 V on a 7.5 V motor (+60%) and cook it.
So the compressor is *always* PWM'd; it never runs DC. Two consequences:
- Raising the duty is never the fix for anything. If the MOSFET runs hot, lower
  `COMPRESSOR_PWM_FREQ_HZ` or add a gate driver.
- The duty is tied to the rail voltage, but the rail is a regulated 12 V and Rubén confirmed it, so
  153 stands. *(This bullet originally speculated the rail might really be 13.8 V and the duty need
  lowering to 133. That was pattern-matching "12 V system" to a lead-acid battery on float charge —
  which is not what a regulated buck output does. Left here as the correction: check what actually
  generates a rail before theorising about its voltage.)*

Because the run duty is permanent, the MOSFET switches continuously forever and never gets to rest
at DC — so switching loss is a standing condition, not a transient. That is the genuine open
question for the bench test: with the gate driven from 3.3 V through a series resistor, Rds(on) is
worse than the datasheet's Vgs=10 V figure. Run a full pump-up cycle at 60% and feel the MOSFET.

**Boot-window gate state — checked, and it is fine (recording the false alarm).**
GPIO 3 is an ESP32-S3 strapping pin and firmware does not drive it until `KM_GPIO_Init()` runs,
~200 ms into boot. `.agents/esp32s3-pinmap.md` carried the note "idles high at boot — acceptable",
written when the net was a buzzer, where a boot chirp is harmless. That note made this look like a
hazard: an undriven gate on a compressor MOSFET would mean full 12 V into a stalled motor on every
reset — the exact inrush the soft-start exists to remove, and a plausible brownout-reset loop.

**There is a pulldown on the MOSFET gate**, so the MOSFET is held off through the boot window and
on a firmware crash. Not a hazard. The pinmap note has been rewritten to say so, since the old
wording invites this same wrong conclusion again.

Note the exported netlist could not settle this either way: `dv-hardware`
`projects/kart-medulla/output/netlist.net` is dated 7 May while the schematic was edited 9 May, and
the stale export shows *both* Q3's and Q4's gates on no net at all — which cannot be true of a board
where the SDC MOSFET demonstrably works. Do not trust that netlist for connectivity questions until
it is re-exported.

## 2026-07-18 — Compressor bench test: succeeded, but the MOSFET runs too hot

First hardware run of the compressor soft-start (firmware `a82c622`, flashed this session).

**Result: the test passed.** Tank went from 0 to 6 bar in about a minute of continuous running, the
motor never stalled, and the hysteresis shut the compressor off at the top. Telemetry confirmed the
shutoff: `pressure_adc ≈ 2000` with `comp_duty = 0` after the cycle. The `ESP_PNEUMATIC` (0x0C)
frame streamed throughout at a measured 20.0 Hz.

**The problem is heat.** The MOSFET on `CMD_COMPRESSOR_PWM` (GPIO 3) smoked during the run. The part
still works afterwards, and the smoke was most likely adhesive around it burning off rather than the
die failing — but it is too hot to leave alone.

**Config under test** (`main/main.c`, `components/km_gpio/km_gpio.h`):
`COMPRESSOR_DUTY_RUN = 153` (60% of 255, ~7.2 V average from the 12 V rail),
`COMPRESSOR_SOFT_START_MS = 1000` (linear 0 → 153), `COMPRESSOR_PWM_FREQ_HZ = 500`, gate driven
straight from a 3.3 V ESP32 pin with no gate driver.

**Ruled out — the soft-start ramp.** The first reading of this incident blamed the 1 s ramp, on the
theory from `tasks.md` that ramping from 0 holds a stalled motor at locked-rotor current. The run
disproves it: the motor spun up normally and never stalled, and the ramp is 1 s out of roughly 60 s
of running. The heat is steady-state conduction and/or switching loss during normal operation, which
is the ordinary case the existing decision tree addresses. Record this the right way round — the
plausible-sounding ramp theory was wrong, and only the "did it stall?" observation settled it.

**Also ruled out:** the freewheeling diode (an earlier ground-loop investigation established the
fault appears on switch-ON, not switch-OFF), the pressure sensor pin (`PIN_PRESSURE_1` = GPIO 6,
matching the verified SDE5 wiring CN7.1 → GPIO 6), and the shutoff logic.

**Next measurement** is the one already written into `tasks.md`: halve `COMPRESSOR_PWM_FREQ_HZ` to
250 Hz and repeat a comparable 0 → 6 bar run. Much cooler means switching-limited; barely changed
means conduction-limited and no frequency will save it. The compressor MOSFET's part number is
recorded nowhere in this repo, and its Rds(on) at Vgs = 3.3 V is what decides whether firmware can
fix this at all — the IRLZ44N in `.agents/error-log.md` is the SDC MOSFET on GPIO 18, a different
device.

**Calibration discrepancy found while reviewing this.** `main.c` uses `ADC_1_BAR = 819` /
`ADC_2_BAR = 1638` under a comment admitting the map is rough, but the verified wiring note says
bar = 3 × Vadc, under which those ADC values are about 2 bar and 4 bar. The run reaching 6 bar and
idle telemetry sitting at ADC ≈ 2000 (~4.8 bar under the verified map) both fit the verified map, not
the constants. Since cycle length sets the thermal duty, this feeds directly into the heat problem.
Logged as its own task.

## 2026-07-18 — Compressor MOSFET: it is the 3.3 V gate drive, and frequency cannot fix it

Follow-up to the bench run above, once the part was identified. The compressor MOSFET is an
**IRLZ44N** (confirmed from inventory). With the datasheet in hand the "500 Hz vs 250 Hz" decision
tree can be settled on paper instead of with another run.

**The numbers**, at an assumed 10 A running current, 60% duty, 12 V rail. *(Superseded: the
current was measured at 6 A later the same day — see the measurement log at the end of this
file. The 10 A figures below are left as written for the record, but the 6 A ones are the real
operating point and the ~360 °C rise quoted here is correspondingly overstated.)*

| Term | Value |
|---|---|
| Conduction loss | 5.76 W |
| Switching loss at 500 Hz | 0.07 W |
| Ratio | conduction dominates 77× |
| Saving from halving to 250 Hz | 0.64% |

So this is conduction-limited by a wide margin, and the decision tree's own rule — "barely changed →
frequency will not save it" — applies without needing the experiment. Switching loss was estimated
from ~25 nC of effective gate charge (48 nC total, only charged to 3.3 V) at ~20 mA of GPIO drive,
giving roughly 2.5 µs of combined transition time per cycle.

**Root cause: Vgs = 3.3 V.** The IRLZ44N datasheet specifies Rds(on) at Vgs = 10 V (0.022 Ω), 5 V
(0.025 Ω) and 4 V (0.035 Ω), and stops there — 3.3 V is *below the last specified point*, with
Vgs(th) max = 2.0 V. The device never fully enhances: about 0.05–0.07 Ω cold and ~1.6× that at 100 °C
(Fig 4). Against Rth(j-a) = 62 °C/W with no heatsink, 10 A implies a ~360 °C rise. The overheating is
arithmetic, not a marginal thermal design. "Logic-level" on the front page of the datasheet means it
*works* at 5 V, not that it is happy at 3.3 V carrying tens of amps.

**Caveat — the current is assumed, not measured.** The whole result scales with I², so this needs a
clamp-meter reading during a run: the same maths gives a ~92 °C rise at 5 A and ~810 °C at 15 A.
Sensitive enough that the fix should not be sized until it is measured.

**Chosen direction: the HA210N06 hotbed module** already in inventory (heatsink, control-in, DC-in,
load terminals). At 4 mΩ fully enhanced it dissipates ~0.36 W where the IRLZ44N does 5.76 W — 16×
less — and it comes heatsinked. The reason it works is the module's *driver stage*, which drives the
gate from the module's own DC rail rather than from 3.3 V logic. The bare HA210N06 is emphatically
not a drop-in: Vgs(th) is 2–4 V, so wired straight to GPIO 3 it would be worse than what is there
now. Buying the transistor instead of the module would be a step backwards.

Three open checks recorded in `tasks.md`: whether the control input is optocoupled (slow turn-off
plus 500 Hz PWM is a known way to cook these modules — drop to 200–250 Hz if so), whether a flyback
diode is present (hotbed modules assume a resistive load, and the compressor is inductive), and how
the module grounds (an optocoupled one is isolated, which would also break the USB ground loop
documented earlier in this file).

Also noted as an alternative worth weighing: the 60% duty exists only to step 12 V down to the
motor's 7.5 V rating, so a buck converter at 7.5 V plus a plain on/off switch removes the
hard-switching problem entirely rather than making it cheaper.

## 2026-07-18 — Compressor duty stepped 60% → 50% → 20%

`COMPRESSOR_DUTY_RUN` in `main/main.c` went from 153 (60%) to 128 (50%) and then to 51 (20%) on the
same day, all to get the compressor MOSFET's temperature down. Conduction loss dominates switching
loss by roughly 50× at this operating point, so duty is the lever that matters and frequency is not.
The estimate is that dissipation falls about cubically with duty (current and conduction time both
drop), which puts 20% well under 0.1 W against the ~2.1 W measured at 60%.

**The 20% value is below any point that has been tested on the bench.** It was chosen deliberately
over a more cautious step, so the first run with it needs watching. Two things are unknown:

- **The breakaway duty** — the lowest duty at which the motor actually starts turning — has never
  been measured. 20% (2.4 V equivalent from the 12 V rail, against a 7.5 V motor) may be below it.
  A motor that fails to start generates no back-EMF and sits at locked-rotor current, which heats
  the MOSFET *more* than 50% duty did. This is the failure mode to watch for: if the compressor
  does not spin up during the 1 s soft-start ramp, cut power and raise the constant.
- **The running current at this duty** has not been measured either. Since the loss estimate scales
  with I², the sub-0.1 W figure is an expectation to confirm with a clamp meter, not a result.

Firmware builds clean for `esp32-s3-devkitc-1` (322 kB flash, 30.8%). Not yet flashed — no board was
connected at the time (`/dev/cu.*` showed only Bluetooth and the debug console), so the flash and
the bench observation above are still outstanding.

## 2026-07-18 — Compressor bench measurement log (measured facts, separated from derived ones)

Everything in this section was **observed on the bench by Rubén**, not calculated. It is collected
here because several earlier entries in this file reason from estimates that these numbers replace.
Derived figures are marked as such and are only as good as the assumptions behind them.

### Measured

| Quantity | Value | Conditions |
|---|---|---|
| Motor running current | **6 A** | 60% duty (`COMPRESSOR_DUTY_RUN` = 153), 500 Hz |
| Pump-up, first run | **0 → 6 bar in about 1 minute**, continuous | 60% duty |
| Motor behaviour, first run | **Never stalled**; spun up normally | 60% duty |
| MOSFET, first run | Smoked, but **survived and still works** — Rubén judged the smoke to be adhesive around the part burning off, not the die | 60% duty |
| MOSFET temperature | **about 100 °C** | 20% duty (`COMPRESSOR_DUTY_RUN` = 51) |
| Pump-up, second run | Reached **7.5 bar**, then the hysteresis stopped it | 20% duty |
| Motor behaviour, second run | **Turns at 20% duty** — so 20% is above the breakaway point | 20% duty |
| Settled tank reading | ADC **2679** at a gauge-read **7.5 bar**, compressor off | after second run |
| Pressure reading while running | **Reads low**, recovers when the compressor stops | both runs |

### Parts on hand

- The compressor MOSFET is an **IRLZ44N** (confirmed from inventory).
- Also available: an **HA210N06 in a 3D-printer hotbed module**, with control-in, DC-in and load
  terminals, and a heatsink measuring **3 × 2 × 1 cm**.
- **5 V is available from the USB port** (though see the note below on why it is the wrong supply
  for gate drive).

### Target operating point requested

Pump up below **7 bar**, stop above **8 bar**, running at **20% duty**. Implemented in `main.c` on
2026-07-18 as ADC 2500 / 2858, calibrated from the settled 2679 ↔ 7.5 bar point above.

### What these measurements overturned

1. **The cubic duty estimate was wrong.** Dropping 60% → 20% was predicted to give under 0.1 W and
   about a 30 °C rise. It measured ~100 °C. Back-calculating against Rth(j-a) = 62 °C/W gives ~1.2 W,
   which at 20% duty implies roughly **8 A — higher than the 6 A measured at 60%**. The estimate had
   assumed motor current falls with duty; it does not, because the pump works against rising tank
   pressure, so at low duty the motor turns slowly, generates little back-EMF, and current is set by
   about (V_avg − back-EMF) / R_winding. *Lowering the duty buys far less thermal margin than it
   appears to, and 20% is near the useful floor.*
2. **The MOSFET was never destroyed**, so the 500 Hz vs 250 Hz comparison was never actually blocked
   — though it is moot regardless, since conduction dominates switching loss by roughly 50×.
3. **The running-vs-settled discrepancy is large.** The run stopped on an OFF threshold of ADC 1638
   yet settled at 2679. With duty at 0 the tank cannot gain pressure, so that ~1041-count gap is
   measurement bias while running, not a sensor calibration error. This has a safety consequence for
   the 7/8 bar thresholds, which were calibrated on a settled reading but are evaluated against a
   running one — written up as its own task in `tasks.md`.

### Derived, for contrast (not measured)

At the measured 6 A and 60% duty, with Rds(on) taken as 0.05–0.07 Ω cold at Vgs = 3.3 V and ~1.6×
that hot (datasheet Fig 4), conduction loss is **1.73–2.42 W** against roughly **0.04 W** of
switching loss at 500 Hz — conduction dominating about 50×, and a 107–150 °C rise in a bare TO-220.
A research pass proposed revising this down to 1.08–1.51 W; that was **checked and rejected**,
because it used the cold Rds(on) and dropped the temperature coefficient. The ~100 °C measured at
20% duty supports the higher range.
