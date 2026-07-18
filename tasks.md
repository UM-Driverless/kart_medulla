<!-- read in full — kept under 150 lines -->

# Tasks

The repo's task board — the only one. Update status as you go: `TODO → In Progress → Done`. Read
before editing; claim by adding `[YYYY-MM-DD <name>]` and the section change. Big clusters get a
`tasks/<name>.md`, always linked from an index here.

Only Rubén moves a task to Done. "Looks finished" is not the same as confirmed on the hardware — on
this repo that means flashed *and* driven, per the branch workflow in `AGENTS.md`.

## TODO

### Compressor MOSFET runs too hot — run the 250 Hz comparison next

Bench run 2026-07-18 on firmware `a82c622`. **The test succeeded:** 0 → 6 bar in about a minute of
continuous running, the motor never stalled, and the hysteresis shut the compressor off. The MOSFET
on `CMD_COMPRESSOR_PWM` (GPIO 3) survived and still works — the smoke was almost certainly adhesive
around the part burning off, not the die. But it runs too hot to leave as is.

**The soft-start ramp is NOT the problem.** The ramp is 1 s out of ~60 s of running and the motor
spun up normally, so locked-rotor current is not what heats it. The heat comes from **steady-state
running**, which is the ordinary case the decision tree below was written for.

**So run the frequency comparison below as written** — halve `COMPRESSOR_PWM_FREQ_HZ` to 250 Hz and
compare MOSFET temperature after a similar 0 → 6 bar run. That is the one measurement that says
whether this is switching loss (scales with frequency) or conduction loss (does not), and everything
else depends on the answer.

**Blocking unknown: the compressor MOSFET's part number is recorded nowhere in this repo.** The Q3
IRLZ44N in `.agents/error-log.md` is the *SDC* MOSFET on GPIO 18, a different device. Its Rds(on) at
Vgs = 3.3 V is what decides whether a firmware change can fix this at all, or whether it needs a gate
driver or a different part. Identify it before buying anything.

Also worth a heatsink regardless — ~60 s of continuous conduction is the normal duty here, not a
worst case.

### Tank pressure thresholds do not match the verified sensor calibration

`main/main.c` sets `ADC_1_BAR = 819` and `ADC_2_BAR = 1638` under a comment admitting it is "a rough
map". The verified wiring note in this file says **bar = 3 × Vadc** (CN7.1 → GPIO 6). Those disagree:
under bar = 3 × Vadc, ADC 819 is about **2 bar** and ADC 1638 about **4 bar**, so the thresholds are
roughly double their labels.

This is consistent with the 2026-07-18 run reaching **6 bar**, and with idle telemetry reading
ADC ≈ 2000 (about 4.8 bar under the verified map). It also matters for the MOSFET heat above: the
longer the compressor runs per cycle, the hotter it gets, so mislabelled thresholds directly set the
thermal duty. Decide the real target pressures, then set the constants from bar = 3 × Vadc rather
than the rough map, and rename them so the number and the label agree.

### Compressor soft-start bench test — the decision tree for the run above

The soft-start is written and builds, but has never run on hardware (commits `f09bcf0`, `a95d91a`;
reasoning in `history.md` 2026-07-16). The compressor now ramps 0 → 60% duty over 1 s at 500 Hz on
GPIO 3 whenever pressure drops below the low threshold.

**Run:** flash the S3, start `read_telemetry.py`, let pressure fall below the low threshold and
watch one full pump-up cycle. The `[PNEUMATIC]` line prints tank pressure + live compressor duty.

Tank pressure and compressor state also now stream to the Orin as a dedicated `ESP_PNEUMATIC`
(0x0C) frame at ~20 Hz, and the dashboard's System-tab tank dial + compressor bar render them
(kart-brain, commit on `dev`). So the bench run can be watched either on `read_telemetry.py` or the
dashboard.

**Record three things:**
1. **MOSFET temperature** after a full cycle — the whole point of the 60% test.
2. **The duty at which the motor audibly starts turning** (the breakaway duty). Read it off the
   telemetry line at the moment it kicks. Below it the motor is a stationary resistor.
3. **Whether telemetry survives the start** — the old symptom was the USB port dropping the instant
   the compressor kicked in.

**Then, depending on the result:**

- **MOSFET too hot → halve `COMPRESSOR_PWM_FREQ_HZ` (`km_gpio.h`) to 250 Hz and re-run.** That one
  change also tells you *which* loss dominates, because the two scale differently: switching loss is
  proportional to frequency, conduction loss is not. Much cooler at 250 Hz → switching-limited,
  keep lowering (~200 Hz is the floor: below that the motor current goes discontinuous, which both
  brings back the current pulsing and breaks the average-voltage assumption the 60% duty rests on).
  Barely changed → conduction-limited, so frequency will not save it: it needs a gate driver, or a
  MOSFET with a decent Rds(on) at Vgs = 3.3 V. **Never raise the duty to fix heat** — that trades a
  hot MOSFET for a burnt motor.
- **Rail sags hard when it kicks → that is the brownout, and it is the ground-loop symptom**, not a
  duty problem. Sag pushes the motor voltage *down*, which is harmless to the motor; it is the
  regulator and the logic rail that suffer. Treat it together with the telemetry check below.
- **Breakaway duty is well above 0 (expect it) → shorten the ramp and start it near the breakaway.**
  Time spent below breakaway is time feeding a stalled motor — full current, no back-EMF, no
  airflow, no work — so ramping from 0 over a full second lengthens the worst phase. The 1 s figure
  was a conservative guess, not a measurement; the motor only needs a few times its mechanical
  spin-up (tens of ms, maybe 100-200 ms with the pump's inertia). Once the breakaway is known, add a
  start-duty floor just under it and cut `COMPRESSOR_SOFT_START_MS` to roughly 300 ms.
- **Telemetry still drops when it starts → the soft-start did not fix the ground loop**, so move to
  the hardware fixes: star grounding and separated power/signal GND (`history.md` 2026-07-16). A
  lower start duty and a slower ramp are worth one try first, since both cut the peak current.
- **Compressor never runs, or never stops → suspect the pressure thresholds, not the drive.**
  `ADC_1_BAR` / `ADC_2_BAR` in `main.c` are an admitted guess (a rough linear map, 1 bar = 819,
  2 bar = 1638). They decide when the compressor runs at all, so calibrate them against the real
  sensor before reading anything into the rest of the test.

### Compressor pump-on/off thresholds are still uncalibrated (separate from the dashboard bar)

- **`ADC_1_BAR` / `ADC_2_BAR` in `main.c` are wrong and should be recomputed from the verified
  calibration.** The tank read is now known (verified 2026-07-12, kart-brain tasks.md): PRESSURE_1
  (CN7.1 → GPIO 6) is the Festo SDE5-D10, 1 V/bar through a ÷3 divider, so `bar = 3.0 * V_adc`
  → about 414 raw ADC counts per bar. The current thresholds (819, 1638) were a guess of ~2× the
  real counts-per-bar AND target the wrong pressures: the EBS reservoir operates at 6-10 bar, so
  the compressor should run below ~6 bar (≈2480 ADC) and stop at ~10 bar (≈4130 ADC), not at the
  1-2 bar the current numbers imply. Recompute both before the reservoir is run for real. (The
  dashboard bar reading already uses the verified calibration — this is only the pump control.)
- **Dashboard bar is a linear-ADC approximation.** kart-brain `protocol.py` converts raw ADC → bar
  with a linear 12-bit/3.3 V model, which ignores the ESP32-S3 ADC nonlinearity. For an accurate
  reading, convert in firmware with `esp_adc_cal` (ESP-IDF) and send millivolts instead of raw ADC.

### kart-brain (Orin) side — needs an Orin build + has pre-existing test drift

- **Build `kb_coms_micro` + `kb_dashboard` on the Orin.** The `ESP_PNEUMATIC` C++ publisher was
  written on the Mac, which has no ROS2/colcon, so it is unbuilt. `colcon build` on the Orin, then
  confirm `/esp32/pneumatic` publishes and the dashboard tank dial moves.
- **Pre-existing kart-brain test failures, unrelated to this change (found while adding the
  pneumatic decoder test).** `test_decode.py` has 3 failures on a clean `dev`: `decode_steering_raw`
  now returns a 3-tuple but `TestDecodeSteeringRaw` still unpacks 2, and `TestMissions` is missing
  `autonomous`. These belong on kart-brain's board — noted here only so they aren't mistaken for
  fallout from the pneumatic work.

### ESP32-S3 firmware gaps (from `km_gpio.h`; block running on the real board)

- **`KM_GPIO_WriteDAC()` is not ported to the MCP4922.** It still calls the classic
  `dac_output_voltage()`, which the S3 does not have — throttle and brake do nothing on the S3.
- **Nothing drives `PIN_SDC_NOT_EMERGENCY` (GPIO 18).** R23's pulldown holds it off at boot, so the
  kart sits in emergency until firmware drives it HIGH. Until then it cannot be armed. **Safety —
  read the pin's note in `.agents/esp32s3-pinmap.md` before touching.**
- **Nothing drives `PIN_SELECT_THROTTLE` (GPIO 15).** R32's 10 kΩ pulldown makes the default pedal
  pass-through (safe); firmware must drive it HIGH for throttle-via-DAC.
- **`PIN_STATUS_LED` (GPIO 48) needs RMT**, not plain GPIO — it's an addressable RGB.

### Libraries (migrated from the old `TODO.md`, 2026-07-16 — written in Spanish, unverified)

These are inherited from the original author's list. **Several look stale** — `km_objects`,
`km_sta`, `km_gamc` and the `main` structure all exist now — but "the file exists" is not the same
as "the task is done", so none are marked Done here. Verify against the code, then let Rubén close
them.

- **`km_coms`**: test the return message to the ESP32; do payload formatting on both the Orin and
  the ESP32 side. Intended flow — receive: `UART buffer → library buffer → (format message, queue
  it) → format payload by message type`; send: `format payload by message type → UART buffer`.
- **`main`**: build out the whole structure. (Looks done — `main.c` has the task set up and
  running; confirm nothing from the original plan is missing.)
- **State machine library**: write it entirely. (`km_sta` exists — check whether it is actually
  implemented or a stub.)
- **Storage library**: a library that creates 'objects' that stay saved, so asking for information
  about something just means asking for the object. (`km_objects` exists and `main.c` uses it —
  likely done.)
- **`km_gamc`**: write the library to handle the gamepads. (`km_gamc` exists; `main.c` includes it.
  Note `sketch.cpp` is the legacy Bluepad32 gamepad app and is NOT used by `main.c`.)
- **Move everything pin-related into the gpio library** (the original entry ends in "???" — it was
  a question, not a decision. Largely true already; decide whether anything is left.)
- **Modernize the DAC and LEDC drivers now that everything is configured.** The build currently
  warns on both: "legacy adc driver is deprecated, migrate to `esp_adc/adc_oneshot.h`" and "the
  legacy DAC driver is deprecated, use `driver/dac_oneshot.h`". Worth doing alongside the MCP4922
  port above, since that rewrites the DAC path anyway.

### Cross-repo

- **`dv-hardware`'s exported netlist is stale and misleading — belongs on that repo's board.**
  `projects/kart-medulla/output/netlist.net` is dated 7 May while `kart-medulla_P1.kicad_sch` was
  edited 9 May. The stale export shows *both* Q3's and Q4's gates connected to no net at all, which
  cannot be true of a board whose SDC MOSFET works. Anyone answering a connectivity question from
  that file will get a wrong answer. Re-export it.

### `platformio.ini`'s S3 comment contradicts AGENTS.md — one of them is wrong

The comment above `[env:esp32-s3-devkitc-1]` in `platformio.ini` says the env "does NOT link yet"
and "exists so the S3 pin map is buildable-in-progress, not because a working S3 image exists".
`AGENTS.md` says the opposite: "the firmware in this repo now fully builds for the ESP32-S3".

Checked on 2026-07-18: `pio run -e esp32-s3-devkitc-1` links and produces a sized image (RAM 6.4%,
flash 30.8% of 1 MB). So **AGENTS.md is right and the `platformio.ini` comment is stale** — it
predates the S3 work in `d420ff2`. Delete or rewrite that comment so it stops telling readers the
S3 image doesn't exist.

Note the comment's *other* claims were not checked and may still hold: that the throttle/brake DAC
(MCP4922 over SPI), the PCF8574, and the safety-pin/watchdog drive are still gaps. "It links" is
not "those peripherals are implemented" — verify separately before treating them as done.

## In Progress

## Done

- [2026-07-16] **Compressor soft-start**: `CMD_COMPRESSOR_PWM` (GPIO 3) moved from a digital on/off
  output to LEDC PWM on its own timer at 500 Hz, ramping 0 → 60% duty over 1 s from the pressure
  hysteresis rising edge. Duty added to the `ESP_ACT_STEERING` telemetry payload. Builds for both
  S3 and the classic fallback. **Not validated on hardware** — see the bench test under TODO.
