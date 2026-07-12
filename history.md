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
