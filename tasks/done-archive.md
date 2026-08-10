<!-- reference — read only when you need the history of a shipped item -->
# Done archive — completed work items

Closed items moved out of the root `tasks.md` on 2026-08-10, following the same convention as
the partle repo. Nothing here is actionable: the root board carries only live work, while the
reasoning behind finished things stays findable.

The board is `tasks.md` at the repo root — the only task board in this repo. Narrative context
for most entries is in `history.md` (append-only, newest at the end).

Note this repo's rule that only Rubén marks a task Done, and that on this repo Done means
flashed *and* driven. Several entries below record work that is finished and pushed but still
awaiting that confirmation; each says so in its own text.

## Closed items from the board


### Closed straight off the board

- [x] **RESOLVED 2026-08-10 — the 50%-throttle bench hardcode is gone from `dev`.** The entry below
  described `control_task()` calling `KM_GPIO_SetThrottleSource(true)`, forcing throttle to `0.5f`
  and returning above the comms-watchdog / manual-mode gate. Commit `7bcd6eb` ("Remove the bench
  hardcode: throttle follows the Orin's TARGET_THROTTLE again") reverted the test commits
  (`ba35b75`, `12f8e05`), and that commit is in this branch's history. Verified against HEAD
  `8516d3c`: `main/main.c:821` takes throttle from `TARGET_THROTTLE`, and
  `KM_GPIO_SetThrottleSource(true)` at line 790 sits *below* the comms-stale / `MISSION_MANUAL`
  early return at line 771, so a stale-comms or manual build hands the mux back to the pedal.
  `-D SPI_DIAG_LOGS=1` is no longer in `platformio.ini`; `main.c:79` defaults it to 0. Rubén also
  confirmed by behaviour: the kart accelerated on command under remote control, which a constant
  0.5 f could not produce. No longer a blocker for a `dev` → `main` PR.

### btstack and bluepad32 are compiled on every clean build and then thrown away

Measured 2026-07-31. `components/btstack` (236 source files) and `components/bluepad32` (69) are
built into `libbtstack.a` (7.6 MB) and `libbluepad32.a` (2.0 MB), 189 archive members between them —
and **none of it reaches the firmware**: `xtensa-esp32s3-elf-nm firmware.elf` matches zero symbols
against `uni_hid|btstack_|bluepad` out of 5334 total. The linker drops the lot, correctly, because
nothing references it: `main/main.c` never mentions them, and `main/CMakeLists.txt` has `sketch.cpp`
(the legacy Bluepad32 gamepad app) commented out of `srcs`.

So they are pure clean-build cost. This is most of why a clean rebuild takes ~100 s against ~30 s
incremental, and it is the part of the slow-deploy story that nobody has addressed — the upload baud
and the `rm -rf .pio/build` ritual both got attention, this did not.

- [x] **Done 2026-07-31**: `set(EXCLUDE_COMPONENTS btstack bluepad32)` in the root `CMakeLists.txt`.
  Measured cold-build before/after — objects 1174 -> 985, clean build 32 s -> 27 s, `firmware.bin`
  333856 bytes both times, and the full defined-symbol set diffed **empty** at 5330 symbols. That
  empty diff is the proof nothing was lost. Reversible by deleting the one line; the components stay
  in `components/`. (`sketch.cpp` turned out not to exist in the repo at all.)
- [x] **Deleted 2026-08-08** per Rubén's decision. `git rm` of both components and the
  `EXCLUDE_COMPONENTS` line; git history keeps them. S3 build and native tests verified green after
  the removal.



## Previously under the board's `## Done` heading


- [2026-07-16] **Compressor soft-start**: `CMD_COMPRESSOR_PWM` (GPIO 3) moved from a digital on/off
  output to LEDC PWM on its own timer at 500 Hz, ramping 0 → 60% duty over 1 s from the pressure
  hysteresis rising edge. Duty added to the `ESP_ACT_STEERING` telemetry payload. Builds for both
  S3 and the classic fallback. **Not validated on hardware** — see the bench test under TODO.

### Docs across kart-medulla, dv-hardware and kart-docs contradict the code and each other

Found 2026-07-30 during a three-repo audit prompted by a pressure reading that disagreed with a
multimeter. **The pin map itself is fine** — `PRESSURE_1` = `CN7.1` -> GPIO 6 (`ADC1_CH5`) is
confirmed by four independent sources and a 2026-07-12 bench check. What the audit did find is a
pile of stale documentation, and one item of it is actively dangerous to someone holding a
soldering iron. Listed worst first.

- [x] **`README.md` presents classic-ESP32 pin tables with nothing saying so.** The repo's primary
  target is the S3, where **GPIO 18 is Q3's shutdown-circuit gate** — so following those tables could
  drive a safety output. **Done 2026-08-10**: the S3 map from `km_gpio.h` is now the first thing under
  "Pin Configuration", split into actuators / sensors / buses, with the SDC row called out as safety.
  The classic tables survive inside a collapsed block headed "the previous board. Do not wire from
  this", which names the two overlaps that make it dangerous rather than merely useless (GPIO 18 =
  steering PWM there but the SDC gate on the S3; GPIO 13 = SDC there but SPI MISO on the S3). Also
  corrected: the old "GPIO 6-11 reserved for SPI flash, do not use" restriction was presented as
  general and is false on the S3, where 6 and 7 are the two pressure inputs.
- [x] **`README.md` still calls the steering sensor an AS5600** (lines ~7 and ~22, the latter saying
  CN5.2 carries "the AS5600's PWM angle output"). It is an **MT6701** read over PWM; `AGENTS.md:20`
  already records the AS5600 as retired on 2026-07-12.
  **Done 2026-08-08** — both lines now say MT6701, with a pointer to the AGENTS.md section and a
  note that `km_sdir`'s AS5600 I²C driver is the classic-ESP32 fallback only.
- [x] **`.agents/esp32s3-pinmap.md` header and gap 1 are both false.** **Done 2026-08-10.** The
  header now says this IS the map the firmware uses, names `esp32-s3-devkitc-1` as the target that
  builds and flashes, and keeps the reason the distinction matters (the classic map's GPIO 18 is the
  S3's shutdown-circuit gate). Gap 1 is struck with a note that it was false when written; the four
  SPI pins it called "defined nowhere" are at `km_gpio.h:83-86`. Gap 2 (`SELECT_THROTTLE` undriven)
  was stale too and is struck as well — `control_task` has driven GPIO 15 since 2026-08-01/08-08.
- [x] **`.agents/esp32s3-pinmap.md:12` still lists `PRESSURE_3` on GPIO 1 as "analog in (ADC1)".**
  **Done 2026-08-10** — the row now says it is not an ADC input, that CN5.2 carries the MT6701 PWM
  angle output read by MCPWM capture, and points at the 2026-08-08 correction further down.
- [x] **Contradictory rework instruction for the ex-`PRESSURE_3` terminal (CN5.2).** kart-docs says
  "remove R10 only (keep R8 + R9)"; this repo's `.agents/esp32s3-pinmap.md:66` says "keep R8 series,
  remove R9+R10". **Do not solder CN5.2 until this is settled** — one of the two is wrong.
  **Settled 2026-08-08 (Rubén): remove R10 only.** kart-docs was right. R8 + R9 stay, leaving 20 kΩ
  in series into GPIO 1 — which is *why* that pin is read as digital PWM through MCPWM capture
  rather than as an ADC channel, so removing R9 would have defeated the design. `README.md` and a
  dated correction appended under the 2026-07-11 entry in `.agents/esp32s3-pinmap.md` now say so.
- [x] **Stale gap comments in `km_gpio.h`** and `platformio.ini`. **Done 2026-08-10**: the S3 block's
  header no longer claims "the classic ESP32 build is still the one that runs today" — it now says the
  S3 block is what runs on the kart and points remaining gaps at `tasks.md` instead of listing them
  inline. `platformio.ini`'s S3 comment no longer lists the three closed gaps (MCP4922 SPI write
  landed 2026-07-31; GPIO 18 and GPIO 15 both driven since 2026-08-01/08-08). Neither file now carries
  its own gap list, because a gap list in a build file goes stale without anyone reading it.
  The per-pin `GAP:` note beside `PIN_SDC_NOT_EMERGENCY` was fixed in the same pass — it claimed
  nothing drove the pin and the kart could not be armed, which has been false since 2026-07-26. The
  one beside `PIN_STATUS_LED` (needs RMT, not plain GPIO) is still true and stays.
- [x] **`km_gpio.h` never records the connector for `PRESSURE_1`/`PRESSURE_2`.** **Done 2026-08-10**
  — `PIN_PRESSURE_1` now carries `// CN7.1` plus the sensor and its 1 V/bar scaling, and
  `PIN_PRESSURE_2` carries `// CN7.2`. The GPIO 6 <-> CN7.1 link no longer lives only in `tasks.md`
  and `history.md`.

- [x] **`KM_PID_GetTunings` is a second setter, not a getter** — **Fixed 2026-08-10.** It now takes
  `const PID_Controller *` plus `float *kp, *ki, *kd`, writes through the pointers and skips any that
  are NULL. All five mirrored globals in `main.c` (`g_pid_kp/ki/kd/pwm_limit/override`) are deleted:
  `health_task` reads the gains via the getter, the PWM limit from `c->dir_act->outputLimit`, and the
  override flag from the object store — each value now lives in exactly one place. `pid_apply_override`
  compares the request against what the controller and actuator are actually running instead of
  against a remembered copy, so its change-detection cannot go stale either. Four new tests pin the
  getter down (reads back, does not modify, tracks SetTunings, NULL-safe); native suite 51 → 55, S3
  image builds. Original entry follows.

  Found 2026-07-30 while adding live PID
  tuning. `components/km_pid/km_pid.c:114` declares `void KM_PID_GetTunings(PID_Controller *controller,
  float kp, float ki, float kd)` and its body is byte-for-byte identical to `KM_PID_SetTunings` above it:
  it overwrites the gains. Taking the floats by value means it cannot return anything even in principle,
  so no caller can read gains out of a controller. Nothing calls it today, which is why it has survived;
  `main.c` works around it by mirroring the live gains into its own `g_pid_*` file-scope variables to
  build the `ESP_STEER_PID` echo. Fix: change the signature to `float *kp, float *ki, float *kd`, write
  through the pointers, and delete the mirrored globals in `main.c` in the same commit. Its header
  comment already flags it as "identical to SetTunings", so the duplication was known — what was never
  decided is whether to fix it or delete it.
