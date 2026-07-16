<!-- consult selectively — grep, never read in full -->
# Error Log

## 2026-03-21 - DAC output never worked: channel index vs GPIO pin number
**What happened:** `KM_ACT_SetOutput` for throttle/brake never produced DAC output. GPIO 25 always read ~0.12V (noise). The full Orin→serial→ESP32 pipeline appeared to work (frames received, no errors logged), but the actuator output was silently failing.
**Root cause:** `KM_ACT_Init(ACT_ACCEL)` set `dacChannel = 0` (a channel index). `KM_GPIO_WriteDAC` expected a `gpio_num_t` (25 for ACC, 26 for BRAKE) and compared with `PIN_CMD_ACC` (25). Since `0 != 25`, it returned `ESP_ERR_INVALID_ARG` silently. The return value was never checked.
**How we found it:** Instead of debugging the full pipeline (serial protocol, ROS topics, state machine, etc.), we wrote a minimal test: hardcode `dac_output_voltage(DAC_CHAN_0, 128)` directly in `main.c`, bypassing all abstraction. This immediately produced 1.65V on GPIO 25, proving the hardware works and the bug was in our abstraction layer. This binary search approach — test at the boundary to determine if the error is upstream or downstream — makes deterministic progress regardless of the test result.
**Fix:** Changed `dacChannel` from channel indices (0/1) to GPIO pin numbers (PIN_CMD_ACC/PIN_CMD_BRAKE).
**Prevention added:**
- Rule: **When debugging a pipeline, don't test end-to-end first. Find a test that splits the pipeline in half — the result tells you which half has the bug, making progress no matter what.** Hardcoding an output at the hardware boundary is the fastest way to isolate software vs hardware issues.
- Rule: **Always check return values from hardware write functions.** `KM_GPIO_WriteDAC` returned an error that was silently ignored.

## 2026-07-10 - Near miss: firmware pin map is classic-ESP32, but the PCB now carries an ESP32-S3. GPIO 18 collides with the SDC MOSFET gate.
**What happened:** Nothing broke — this was caught before anyone flashed. The kart-medulla PCB now has an ESP32-S3 (WROOM-1-N16R8) fitted, but this repo still only builds for the classic ESP32.
**The collision:** On the S3 board, **GPIO 18 is `SDC_NOT_EMERGENCY__3V3`** — the gate of Q3 (IRLZ44N) via R22 (100 Ω), which closes the kart's shutdown-circuit return path. This repo defines `PIN_STEER_PWM = GPIO_NUM_18`. Porting by simply switching the PlatformIO board target would make the steering PWM chop the SDC gate. Two neighbours down, `PIN_STEER_DIR = GPIO_NUM_19`, which on the S3 is USB D-. And `PIN_SDC_NOT_EMERGENCY` (GPIO 13) descends from the legacy table where the signal was a digital *input* ("SDC emergency status"); on the S3 board it is a digital *output* driving the gate. The direction inverted too.
**Why it can't happen today:** `platformio.ini` has only `esp32dev` and `native` envs live (`esp32-s3-devkitc-1` is commented out); `km_gpio.h` contains exactly one pin map, headed "ESP32-DevKitC V4 (ESP32-WROOM-32E)"; and the `#ifdef CONFIG_IDF_TARGET_ESP32S3` branch in `km_gpio.c` (MCP4922 external DAC over SPI, since the S3 has no DAC) references `SPI_MOSI_PIN` / `SPI_SCLK_PIN` / `SPI_CS_PIN`, none of which are defined anywhere in the repo. That branch cannot compile. Also `board_build.flash_size = 4MB` against the chip's actual 16 MB.
**Prevention:**
- Rule: **The pin map is board-specific. When porting to the S3, do not reuse a single flat `PIN_*` block** — guard it with `#if CONFIG_IDF_TARGET_ESP32S3` / `#elif CONFIG_IDF_TARGET_ESP32` and take the S3 values from the authoritative source, `dv-hardware/projects/kart-medulla/docs/pinout-esp32-s3.md` (and the schematic, which wins over the doc).
- Rule: **GPIO 18 on the S3 board is safety-critical.** It drives the SDC MOSFET. Driving it HIGH asserts "no emergency" and closes the kart's shutdown chain. Never assign it to an actuator, and never toggle it on a live kart.
- The S3's octal PSRAM makes **GPIO 33-37 unusable**. Do not assign them in firmware, ever.

## 2026-07-12 - Invisible serial: the devkit's UART connector is a CH343 that shows up as /dev/cu.usbmodem* on macOS
**What happened:** During the AS5600 bench session, the freshly flashed bench firmware produced no serial output and answered no commands, on any open. Flashing worked, ROM boot messages appeared, and IDF error logs appeared once — but never a `Serial.print`. Burned ~1 h on CDC/DTR/RTS theories.
**Root cause:** The USB cable was in the devkit's **UART connector** (CH343 bridge). On macOS the CH343 enumerates as `/dev/cu.usbmodem…` with product name "USB Single Serial" — a name that looks exactly like the S3's native USB-Serial-JTAG port. The build had `ARDUINO_USB_CDC_ON_BOOT=1`, which routes Arduino `Serial` to the native USB connector — which was unplugged. ROM/IDF logs still go to UART0, which is why *some* output got through and masked the problem.
**How we found it:** `ioreg -p IOUSB` showed the device as "USB Single Serial" (CH343) instead of "USB JTAG/serial debug unit" (native S3 USB).
**Prevention:**
- **Default: flash + Serial over the CH343 UART port with `-DARDUINO_USB_CDC_ON_BOOT=0` (Serial → UART0) on the first go.** That connector is the one the cable is usually in and it's the more reliable path for flashing (auto-reset via RTS pin — the flash log literally says "Hard resetting via RTS pin"). Don't start from a native-USB-CDC build. Re-confirmed 2026-07-12 bench (compressor pressure-reader): CDC_ON_BOOT=1 flashed fine but produced zero telemetry; rebuilding with CDC_ON_BOOT=0 gave clean output on the same `/dev/cu.usbmodem…` port immediately.
- Only switch to CDC flags if `ioreg -p IOUSB` shows "USB JTAG/serial debug unit" (native S3 USB) instead of "USB Single Serial" (CH343 bridge).
- Rule: **Don't infer the connector from the /dev name.** `usbmodem` ≠ native USB — both look identical as `/dev/cu.usbmodem…`.
- Rule: **"Logs appear but my prints don't" means two different consoles are in play** (UART0 vs USB-CDC), not a broken bus or driver.

## 2026-07-12 — asked for permission instead of just verifying + downloading a datasheet

**What happened.** User asked to note a magnet-sizing idea AND verify why the AS5600 needs a small magnet. I recorded the idea but left the AS5600 reason as an "open question" and *asked* "want me to pull the datasheet now?" instead of doing it. User (frustrated, has said this before): "your single task was to verify that. and make sure the datasheet is downloaded and organized! i'm tired of saying the same thing over and over."

**Root cause.** Treated "verify" as optional and split it behind a permission question. Global CLAUDE.md already says: write notes without asking, save source URLs/PDFs into the project as I find them without being asked, always verify my work, never end a turn with "want me to record this?".

**Prevention.** When a task includes verifying a claim or references a datasheet/spec: *do the verification in the same turn* — download the PDF into `datasheets/` (create it if absent), read the relevant section, cite page numbers, and update the note from unverified→verified. Never park it as an open question or a "want me to…?" offer. The datasheet lives at `datasheets/<part>_datasheet.pdf`.

## 2026-07-12 — retried a dead datasheet URL instead of finding another source

**What happened.** The MagnTek MT6701 datasheet URL (oneyac CDN) 404'd / timed out. I retried the same URL (curl, curl+user-agent, osascript-to-Chrome) instead of just searching for another host. User did it for me: "i get 404 too. it's not your issue i think. i've looked another one (like you should have) and saved it there."

**Root cause.** Treated one URL as the only source and burned attempts on it. A datasheet for a real part is mirrored on many sites (alldatasheet, LCSC, Mouser, manufacturer, distributor PDFs).

**Prevention.** When a datasheet/spec URL fails once, immediately WebSearch for "<part> datasheet pdf" and try a different host — don't retry the dead link or hand it back to the user. Two hosts max before switching strategy. (Pairs with the same-day entry above: verify in-turn, don't park.)

## 2026-07-16 — invented a rationale for a hardware number instead of asking what it was for

**What happened.** Rubén asked for a compressor soft-start, and to "test as an experiment something
like 60% pwm and see if the mosfet gets too warm or not". I implemented the ramp, then wrote the
60% figure into the code, the commit message and `history.md` as *a MOSFET thermal experiment* —
complete with the advice to "raise toward 255 once it runs cool, since 100% duty is DC with no
switching loss at all". Rubén: "the idea of 60% duty is to not cook the motor. it's designed for
7.5V, not 12V."

The real reason: the motor is a 7.5 V part on a 12 V rail, so duty is a permanent voltage divider —
0.60 x 12 = 7.2 V. Full duty would overvolt it by 60%. My advice was the exact opposite of correct,
and I had written it into three places as settled fact.

**Root cause.** His sentence contained two things: an instruction (use 60%) and a thing he wanted to
learn (does the MOSFET get warm). I collapsed them into one, assuming the observation *was* the
motive. The number had an electrical reason that firmware alone cannot reveal — nothing in the
repo states the motor's voltage rating — so the gap was unfillable by reading code, exactly the
case where a question is cheap and a guess is expensive. Plausibility did the rest: switching loss
is a real phenomenon, so the invented story sounded like analysis rather than a guess.

**Prevention.** When a spec number arrives without a stated reason — a duty cap, a current limit, a
voltage, a timeout — *ask what sets it* before writing a rationale for it into code comments, docs
or commits. "Test X and see if Y" gives X and Y; it does not give the reason for X. And note the
tell: if a rationale for a hardware constant can't be traced to a datasheet, a measurement, or the
user's own words, it is a guess no matter how good the physics sounds. Same failure family as the
2026-07-12 entries above — supply the missing fact, don't route around it.

**Related, same session:** I also flagged the compressor gate as a boot hazard (strap pin, undriven
until `KM_GPIO_Init()`) and wrote it up as an open hardware issue. There is a gate pulldown; it was
a false alarm. That one was defensible — the pinmap note said "idles high at boot — acceptable" and
the exported netlist was stale — but the lesson is the same: the schematic and the person holding
the board are the authority on hardware, not an inference from firmware plus a stale export.
