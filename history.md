# ESP32 (Legacy Board) Failure Report

**Date of diagnosis:** 2026-07-12
**Board:** Legacy / spare ESP32-WROOM dev board (UM-Driverless kart project)
**Reported symptom:** Board "stopped working several days ago"; not detected when connected to the computer.

---

## TL;DR — Final Conclusion

- ❌ **The onboard USB-to-serial bridge chip (CH340 / CP2102) is DEAD.** The computer cannot see the board over USB at all.
- 🟡 **The ESP32 core itself is most likely still ALIVE** (powered and running), but can no longer be reached over USB.
- **This is a hardware failure, not a software/driver/kernel problem.**
- **Most probable cause:** a ground-loop or a motor/actuator voltage transient that destroyed the exposed USB-serial front-end chip while sparing the ESP32.

---

## Evidence Collected

All tests run on the host laptop: `jorge-Aspire-A315-51`, Linux 6.17.0-35-generic (Ubuntu).

### 1. No serial port is ever created
- `ls /dev/ttyUSB* /dev/ttyACM*` → **no device present**, before or after plugging in.

### 2. USB-serial bridge chip does not enumerate
- `lsusb` shows only the internal Bluetooth adapter and webcam.
- The ESP32's USB-serial bridge chip (CP210x / CH340 / FTDI) **never appears** in the USB device list.

### 3. Zero kernel activity on plug-in (the decisive test)
- `udevadm monitor --kernel` watched during **multiple physical unplug/replug cycles** → **zero kernel uevents**.
- `journalctl -k` (entire boot log) contains **no serial-enumeration line** at all.
- This is the rawest hardware-detection layer, below every driver, permission, or setting. A healthy board *always* fires events here on plug-in. This board fires none.

### 4. No USB-serial driver loaded
- `lsmod | grep -E "cp210|ch34|ftdi|usbserial|cdc_acm"` → **none loaded** (nothing triggered them, because nothing enumerated).

### 5. Board IS receiving power
- **Power LED lights up** when USB is connected → 5V reaches the board; the power path (USB connector + regulator) is physically intact.

### 6. Thermal readings (Milwaukee temperature sensor)
- Most of the board: **~30 °C** (essentially room temperature).
- One localized warm spot: **56 °C**, on the **back/underside, between GND and pins GPIO32–35** — i.e. directly beneath the ESP32-WROOM module.
- Interpretation: this heat is the **ESP32 SoC itself drawing current and running**. 56 °C is a normal running temperature for a powered ESP32, **not** a runaway short (a dead short would be 80–100 °C+ and climbing). Note: GPIO34/35 are input-only pins, so this is core heat, not a driven-output short.

---

## Why It Is NOT a Software / Kernel / Driver Problem

1. **Detection happens below software.** USB enumeration is a pure electrical handshake performed by the laptop's USB host-controller hardware the instant a device pulls the data line high — *before* any driver, permission, `brltty`, or `/dev/ttyUSB0`. We watched that raw layer (`udevadm monitor --kernel`) and got nothing. No software can suppress or fake that.
2. **Even with everything software broken, the kernel would still log "new USB device found."** It didn't. So the device never electrically announced itself.
3. **Software cannot make a chip warm.** A missing driver produces no device node; it does not push current through silicon. The measured heat proves real current is flowing — that is physics, not code.

Ruled out by the above: missing/wrong driver, `dialout` permissions, `brltty` port hijacking (it is installed on this laptop but is irrelevant here since nothing enumerates), wrong baud rate, wrong port.

---

## Why It Is a Hardware Failure — Component Breakdown

The board has two independent chips relevant here:

| Component | Status | Reasoning |
|-----------|--------|-----------|
| **USB-serial bridge (CH340/CP2102)** | ❌ **Dead** | Never enumerates at kernel level (tested 5×). This is the chip that translates between the ESP32 and the laptop's USB. |
| **3.3 V regulator (AMS1117) / power path** | ✅ OK | Power LED on; board is powered; no widespread heat. |
| **ESP32 SoC (WROOM module)** | 🟡 Likely alive | Powered and warm (56 °C at the module) = drawing current / running. Cannot be confirmed via USB (that path is dead). |

The USB-serial chip and the ESP32 are **separate chips**. The USB front-end can die while the ESP32 keeps working — which is exactly what the evidence shows.

---

## Probable Root Cause — What May Have Killed the USB Chip

Ordered by likelihood for a kart (motors, H-bridge, DAC, vibration, shared grounds):

1. **Ground loop / ground-potential difference (most likely).** If the ESP32 was ever connected to the laptop USB *while also powered or grounded through the kart*, and the two grounds sat at different voltages, current flows through the USB chip's ground/data pins to equalize. The USB-serial chip sits right at that junction and takes the damage, while the ESP32 (further from the USB port) survives. Matches the symptom perfectly.

2. **Voltage spike / back-EMF from motors coupling into TX/RX.** The bridge chip's data lines connect to the ESP32 UART pins (GPIO1/GPIO3). A transient from the steering motor, H-bridge, or a solenoid (inductive back-EMF) traveling down those lines can punch through the bridge chip's I/O.

3. **Over-voltage on the serial/data lines.** Something 5 V touching the TX/RX or USB data lines (the bridge I/O is 3.3 V and does not tolerate 5 V).

4. **ESD (static discharge).** The USB connector and data pins are the most exposed part of the board; a static zap during handling commonly kills only the bridge chip. Fits the "stopped working several days ago" timing.

---

## How to Confirm the ESP32 Core Is Alive (USB path is unusable)

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

## Recommendation

- **Retire the onboard USB path** on this legacy board — it is conclusively dead.
- If the ESP32 core confirms alive (LED blink / WiFi / esptool responds), the board is still usable/flashable via an external UART adapter.
- Given this is the legacy/spare board, retiring it entirely is also reasonable; use the working ESP32-WROOM-32E on the kart.

### Prevention (protect the working board)
- **Never** connect the ESP32 to laptop USB while it is also powered/grounded by the kart — this ground loop is the #1 risk. Use USB isolation, or unplug kart power before connecting USB.
- Add **flyback diodes / isolation** on any inductive load (motors, solenoids) near the ESP32's lines.
- Keep a common, solid ground; never put 5 V on any 3.3 V pin.
