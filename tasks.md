<!-- read in full — kept under 150 lines -->

# Tasks

The repo's task board — the only one. Update status as you go: `TODO → In Progress → Done`. Read
before editing; claim by adding `[YYYY-MM-DD <name>]` and the section change. Big clusters get a
`tasks/<name>.md`, always linked from an index here.

Only Rubén moves a task to Done. "Looks finished" is not the same as confirmed on the hardware — on
this repo that means flashed *and* driven, per the branch workflow in `AGENTS.md`.

## TODO

### Compressor soft-start bench test — DO THIS FIRST

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

## In Progress

## Done

- [2026-07-16] **Compressor soft-start**: `CMD_COMPRESSOR_PWM` (GPIO 3) moved from a digital on/off
  output to LEDC PWM on its own timer at 500 Hz, ramping 0 → 60% duty over 1 s from the pressure
  hysteresis rising edge. Duty added to the `ESP_ACT_STEERING` telemetry payload. Builds for both
  S3 and the classic fallback. **Not validated on hardware** — see the bench test under TODO.
