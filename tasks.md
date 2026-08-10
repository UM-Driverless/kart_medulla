<!-- the one task board for this repo. Read the section you're working in; it's too long to read in
     full every session. Completed work is in tasks/done-archive.md. -->

# Tasks

The repo's task board — the only one. Read before editing; claim by adding `[YYYY-MM-DD <name>]`.
Big clusters get a `tasks/<name>.md`, always linked from an index here.

**Done items do not stay on the board.** When an item closes, move it — with its date and closing
note — to `tasks/done-archive.md`, the only other task file and the one that holds nothing
actionable. Anything live is in *this* file. `- [x]` entries accumulating here is what made the
board unreadable before the 2026-08-10 split: `## TODO` had filled up with closed items while
`## Done` held open ones, so the headings meant nothing and the checkbox carried the real state.
Same convention as the partle repo.

One exception, because it costs nothing and losing it costs context: a **done step of a task that
is still open** stays put. "Put the QR/label on the board" is unreadable once "Pick the scheme" has
been filed away. A cluster moves to the archive whole, once its last step closes.

Only Rubén moves a task to Done. "Looks finished" is not the same as confirmed on the hardware — on
this repo that means flashed *and* driven, per the branch workflow in `AGENTS.md`. So an item can be
finished, pushed and still sit here awaiting that confirmation; several do.

## TODO

- [ ] **Flash the steering-authority gate and verify it on the kart** (added 2026-08-10). Commit `708f6da` on `dev` makes `control_task` in `main/main.c` refuse to drive `dir_act` unless the Orin reports `AS_DRIVING`, or the mission is remote control (7). It builds clean for `esp32-s3-devkitc-1` but has never run on hardware.

  **Why it exists.** The Orin sends a steering target continuously, in every state — its mux publishes a zero Twist whenever it has nothing to say. In PID angle mode 0 rad is a real target, "centre the wheels and hold them", so the motor was powered as soon as a mission was selected on the dashboard, with no Start pressed; this was seen moving the steering column on the kart. The protocol cannot express "no target" (`TARGET_STEERING` is an int32, every value is a valid angle), so the authority to actuate was moved to the firmware instead, where it is one condition in front of the one call that turns the motor.

  **Flash:** `cd ~/kart_medulla && ~/.local/bin/pio run -e esp32-s3-devkitc-1 -t upload --upload-port /dev/ttyACM0`, after `sudo systemctl stop kart-brain` — the running stack holds the serial port and the flash otherwise fails with "serial noise". Restart the service afterwards.

  **Verify, hands clear of the wheel:** selecting AUTO must not move the column; Start must let it steer; Stop must stop it; toggling steering algorithm Geometric ↔ None must not move it in any state; remote control must still steer. The last one is the regression risk — remote control is the carve-out, and if the mission ID or the Orin's reported state ever differs from what this assumes, remote steering goes dead rather than dangerous.

  The Orin side was independently fixed to stop sending drive-worthy zeros (kart-brain `ad7e49b`, `ff8db3e`, with an invariant test suite). The two are complementary: that stops the bad command being sent, this stops it being acted on. Keep both — the firmware gate is the one that also covers senders nobody has written yet.



- [ ] **`tasks.md` structure: the "Docs across kart-medulla, dv-hardware and kart-docs contradict
  the code and each other" section sits under `## Done` but holds open `- [ ]` items.** Found
  2026-08-08 while closing two of them. Either move the section back under TODO or split the
  closed items out — as it stands, open work is filed under Done and is easy to miss.

### Pump-stall detector: watch it, then decide whether to arm it

Added 2026-07-27, **reporting only**. A full-length compressor burst that starts below 4 bar and
raises the tank by less than 0.15 bar sets `pump_stall_observed`, which shows on the dashboard as
compressor state 4 and does nothing else.

It is not wired to the shutdown circuit on purpose. The first version was — it latched and opened the
SDC — and analysis right afterwards showed it would false-trip: the threshold was 0.5 bar derived from
a 0→6 bar fill, which is the fast part of the curve, while the check only ever runs near the top where
a healthy compressor legitimately adds very little per burst. That would have fired the EBS on a
working kart. It was pushed before that analysis was done.

**Before arming it:** watch state 4 through several normal fill cycles and confirm it stays clear,
then unplug the tank sensor and confirm it trips. Only then add `pump_stall_observed` to the
`sdc_may_close` whitelist in `control_task` and make it stop the pump.

**The compressor's ceiling is unknown, and deliberately not being tested** (Rubén, 2026-07-27: we did
not test max pressure and do not want to — it may break before it tops up; expectation is that it
reaches 10 bar fine). Note what the existing evidence does and does not say: the 2026-07-18 run shut
off at roughly 4.3–4.8 bar recomputed, but that was the *threshold* cutting in, not the compressor
running out of breath. It is a lower bound of ~4.8 bar, not a suggestion that 8 is out of reach.

**No test is needed, because normal use reveals it.** If the tank plateaus below `PRESSURE_PUMP_OFF_BAR`
the pump cycles 15 s on / 15 s off with the pressure sitting still, and the report-only stall detector
flags exactly that (`comp_state 4`). So: watch the dashboard during ordinary running. If that pattern
appears, set the cutoff just under the observed plateau. Until it appears, assume the target is fine.

### At-the-kart checklist (2026-07-26 compressor/SDC work) — items also live in kart-brain

The compressor-disable + shutdown-circuit change of 2026-07-26 spans both repos, so its verification
does too. Doing these in order matters: each later one needs the flash to have worked.

1. **Flash once** — confirms the new 921600 upload baud (this repo, task below).
2. **Check the build tree** — `grep -m1 CMAKE_HOME_DIRECTORY .pio/build/esp32-s3-devkitc-1/CMakeCache.txt`
   against `$PWD` (this repo, task below).
3. **Verify the SDC drive and decide on wiring the Q3 gate** — full steps in **kart-brain `tasks.md`**,
   "(at the kart) Verify the shutdown-circuit drive on the bench". Firmware-side summary: with an
   autonomous mission selected the dashboard's EBS page should read "Shutdown circuit: CLOSED", and
   pressing "Disable compressor" should flip it to OPEN with the compressor row reading DISABLED. Meter
   on the Q3 gate side of R22 to confirm the pin follows the readback.
4. **Settle the pressure calibration** — full steps in **kart-brain `tasks.md`**, "(at the kart) Settle
   the tank-pressure calibration". Measure R11/R12/R13 to confirm the divider is 4:1 rather than the 3:1
   the old note assumed. **Take all readings with the compressor OFF and settled** — see the ground IR
   drop task further down, which makes any running reading unusable for calibration.

Note that **commit `f156921` has never been compiled by anyone** (no CI, no Mac toolchain — see the task
below), so the first flash is also its first build.

### Confirm the faster S3 upload baud, and root-cause the stale-object build

Two build-loop items found 2026-07-26, both cheap to settle at the kart.

**1. Flashing was running 8x slower than the hardware allows.** `platformio.ini` had
`upload_speed = 115200` on the S3 env. That number is a **CP2102** limit, and the CP2102 is on the
*classic* board — the S3 board's bridge is a WCH CH343, rated 50 bps to 6 Mbps (datasheet:
`~/dv/datasheets/ch343_wch_datasheet.pdf`). The S3 env had simply inherited the wrong board's
constraint. Raised to **921600**, which should take a minute-plus flash down to seconds on an image
this size. **Not yet tested on hardware** — raised from datasheets, not
from a successful flash. Next time at the kart: flash once and confirm. If it will not connect or the
verify fails, try 460800, then fall back to 115200 and record which worked here.

**2. The stale-object bug probably was not a PlatformIO bug at all.** The old advice was to
`rm -rf .pio/build/...` before *every* build, which costs a full rebuild (~100 s vs ~30 s) forever.
Leading hypothesis is a **moved build tree**: CMake/ninja bake absolute paths into `build.ninja`,
`.ninja_deps` and `CMakeCache.txt`, and this repo has moved more than once (Mac
`~/Desktop/kart_medulla` → `~/repos/kart-medulla`; the Orin workspace renamed 2026-07-06). A build dir
generated at the old path keeps checking the old path — which matches every symptom, `touch` included,
and explains why deleting it cures things permanently. Glob-staleness is ruled out: the component
`CMakeLists.txt` use explicit `SRCS "<file>.c"`, not `SRC_DIRS`.

**Tested on the Mac 2026-07-31: the bug does not reproduce there, and the `touch` test is a false
positive.** `CMAKE_HOME_DIRECTORY` matched `$PWD`, `touch` did NOT trigger a recompile — and the
build was nonetheless completely healthy. A comment-only edit recompiled to a byte-identical `.o`
and correctly skipped the link; a functional edit (the `dt` floor) recompiled, changed the `.o`,
relinked and changed the ELF; reverting it returned the ELF to the same md5. The toolchain keys on
content, not mtime, so a no-op `touch` correctly does nothing. Full table in `history.md`.

- [ ] **Run the same three probes on the Orin**, which is where the bug was actually reported and
  whose workspace was renamed 2026-07-06, so the moved-build-tree hypothesis may still hold there.
  Do NOT use `touch` as the test — it reports "broken" on a healthy build. Use a real content change
  that alters generated code:
  ```bash
  grep -m1 CMAKE_HOME_DIRECTORY .pio/build/esp32-s3-devkitc-1/CMakeCache.txt   # should equal $PWD
  md5sum .pio/build/esp32-s3-devkitc-1/components/km_pid/km_pid.o
  # flip a constant in components/km_pid/km_pid.c, rebuild, md5sum again, then revert
  ```
  If the `.o` md5 moves, the build is fine and the `rm -rf .pio/build` ritual can be dropped there
  too — it costs a full rebuild (~100 s vs ~30 s) every single time.

**Both settled on the Orin 2026-08-08** (full data in `history.md`, that date):
- **921600 works**: 15.0 s total flash, ~3 s writing at effective ~1.06 Mbit/s, hash verified. No
  fallback needed; the number stays.
- **Build is healthy on the Orin too**: `CMAKE_HOME_DIRECTORY` matches `$PWD`; a constant flip
  changed the `km_pid.o` and ELF md5s, the revert returned both to baseline exactly. Drop the
  `rm -rf .pio/build` ritual on the Orin as well. Timings there: near-clean 80 s, no-op 13 s,
  one-file edit ~18 s. Awaiting Rubén's Done.

### Steering pins float during reset/bootloader — the window that broke the gear on 2026-08-08

The flash-time bootloader window leaves GPIO 40 (`CMD_STEER_PWM`) and GPIO 17 (`CMD_STEER_DIR`)
floating, and the Cytron drove the steering motor uncontrolled into the endstop — gear broken
(`.agents/error-log.md`, 2026-08-08). GPIO 3 (compressor) and GPIO 18 (SDC, R23) already have
pulldowns for exactly this window; the steering pins do not.

- [ ] Add pulldowns on the Cytron PWM/DIR inputs (hardware change — belongs in
  `~/repos/dv-hardware/projects/kart-medulla/`, note it there too).
- [ ] Until then, the AGENTS.md flashing section now requires actuator power off (or manual mode)
  before any flash — procedure only, not protection.
- [ ] Assess/repair the broken steering gear; check whether the motor or Cytron took damage too.

### No CI builds this firmware — but the Mac now can, so build before pushing

Noticed 2026-07-26 while pushing the compressor-latch / SDC change. **Nothing compiles this repo
automatically.** `.github/workflows/` does not exist on `dev`, and `gh run list` shows a single run
ever — "Build with ESP-IDF v5.4", 2025-11-02, on `main` — so whatever workflow produced it is gone.

**Correction, 2026-07-31: the Mac half of this is no longer true.** This entry used to say no
PlatformIO or ESP-IDF was installed locally. Both are now present and working:

    pio test -e native -f test_km_pid          # ~1 s, pure C against test/fakes
    pio run -e esp32-s3-devkitc-1              # ~5 s, links a real S3 image

The second was run on 2026-07-31 and succeeded in 5.36 s (RAM 6.4 %, flash 31.8 % of 1 MB). There is
no longer any excuse for pushing firmware nobody compiled.

Note this entry was already contradicting itself: it cited `AGENTS.md` for "no PlatformIO or ESP-IDF
is installed on the Mac", while `AGENTS.md`'s toolchain note (rechecked the same day, 2026-07-26)
says the opposite — PlatformIO IS installed at `~/.platformio/penv/bin/pio`, not on `PATH`, verified
by linking both `esp32-s3-devkitc-1` and `esp32dev`. `AGENTS.md` was right and needs no change; this
entry was the stale one.

That is how commit `f156921` went out: reviewed and reasoned about, but never built. It touches
`control_task`, the object store and the pneumatic frame, so a typo lands as a failed flash at the
kart rather than as a red mark on the PR.

Work: restore a build-only workflow (ESP-IDF, `esp32-s3-devkitc-1` target) triggered on push to `dev`
and on PRs into `main`. It does not need hardware — `idf.py build` in the Espressif container catches
the whole class of error that currently escapes. Worth checking `git log --all -- .github/workflows/`
on a full clone first; this clone shows no history for that path, so the old workflow may be
recoverable rather than needing to be rewritten.

**Done 2026-08-08**: `.github/workflows/build.yml` — PlatformIO build of `esp32-s3-devkitc-1` plus
the green native suites, on push to `dev` and PRs to `main`. (`git log --all` showed no recoverable
old workflow.) Awaiting Rubén's Done once a run is seen green on GitHub.

**Runs are green as of 2026-08-10** — three consecutive successes on `dev` (latest 31367394130,
39 s), each building the S3 image and running the native suites. The filter was dropped the same
day, so CI now runs all four suites: 51/51 in the run itself, not only locally. Ready for your Done.

### ~~Native test drift: test_km_act fails and test_km_coms does not compile~~

Found 2026-08-08 while wiring up CI. **Fixed 2026-08-10 — `pio test -e native` is now 51/51 green
across all four suites, and `.github/workflows/build.yml` runs it unfiltered.**

Neither cause was what the original entry guessed (it blamed the S3 DAC code path; that was not
involved — the native build uses the classic pin map from `test/fakes/km_gpio.h`):

- **`test_km_coms` did not compile**: `test/fakes/freertos/FreeRTOS.h` had no `TickType_t`, which
  `km_coms.h:276` and `km_coms.c:73` both use. Added `typedef uint32_t TickType_t` (matching
  `configUSE_16_BIT_TICKS = 0`) and gave `xTaskGetTickCount()` that return type. The 13 tests
  themselves were fine and all passed once it built.
- **`test_km_act`'s 4 failures were an index-vs-pin mismatch that had never been right.** The tests
  asserted `dacChannel == 0` / `== 1`, and the fake `KM_GPIO_WriteDAC()` recorded only when
  `pin == 0` or `pin == 1` — but `km_act` stores the *pin identifier* (`PIN_CMD_ACC` /
  `PIN_CMD_BRAKE`), so the fake matched nothing and both DAC values read 0. Both now dispatch on the
  `PIN_CMD_*` identifiers, which keeps them correct whichever target's pin map the fake tracks. The
  SIGILL was the fallout of the failed asserts and went with them.
- `km_act.c`'s `// GPIO 25 (DAC1)` / `// GPIO 26 (DAC2)` comments — which is what made the field
  look like a channel index — now say what the value actually is on each target.

### Compressor MOSFET runs too hot — run the 250 Hz comparison next

Bench run 2026-07-18 on firmware `a82c622`. **The test succeeded:** 0 → 6 bar in about a minute of
continuous running, the motor never stalled, and the hysteresis shut the compressor off. The MOSFET
on `CMD_COMPRESSOR_PWM` (GPIO 3) survived and still works — the smoke was almost certainly adhesive
around the part burning off, not the die. But it runs too hot to leave as is.

**The soft-start ramp is NOT the problem.** The ramp is 1 s out of ~60 s of running and the motor
spun up normally, so locked-rotor current is not what heats it. The heat comes from **steady-state
running**, which is the ordinary case the decision tree below was written for.

**The part is an IRLZ44N** (confirmed from inventory, 2026-07-18). That settles the decision tree
below on paper, so do not spend a run on it:

| | at 10 A, 60% duty |
|---|---|
| Conduction loss | **5.76 W** |
| Switching loss at 500 Hz | **0.07 W** |
| Ratio | conduction dominates **77×** |
| Saving from 500 → 250 Hz | **0.64%** |

**So 250 Hz is not enough — it is not even measurable.** This is conduction-limited, which the
decision tree already says frequency cannot fix. Run the 250 Hz comparison only if you want the
empirical confirmation; do not expect it to solve anything.

**The root cause is the 3.3 V gate drive, not the frequency.** The IRLZ44N datasheet specifies
Rds(on) at 10 V (0.022 Ω), 5 V (0.025 Ω) and 4 V (0.035 Ω) — and stops there. GPIO 3 drives it at
**3.3 V**, below the last specified point, with Vgs(th) max = 2.0 V, so it never fully enhances:
call it 0.05–0.07 Ω cold and ~1.6× that hot (datasheet Fig 4). With Rth(j-a) = 62 °C/W and no
heatsink, 10 A gives a ~360 °C rise. That is the smoke, and it is arithmetic, not bad luck.

**Unverified input to all of the above: the compressor's actual running current.** 10 A is an
estimate. It matters a lot — the same maths gives a ~92 °C rise at 5 A but ~810 °C at 15 A. Measure
it with a clamp meter during a run before sizing anything.

### Gate-drive options, researched 2026-07-18

> **Superseded for the compressor as of 2026-07-25** — the HA210N06 module (next section) was
> confirmed to carry an optocoupler and already drives the gate off its own DC-IN rail, so it *is*
> the gate driver. Do not fit a TC4420 / MCP1407 / UCC27517A for this load. This section stays for
> the part-selection reasoning and for the next PCB revision, where a discrete FET is still the plan.

**A replacement MOSFET is essentially not available, and the reason is structural.** Across ~150
datasheets checked by table text (Infineon, Vishay, Nexperia, Toshiba, ST, onsemi, Diodes, AOS), the
industry floor for a *specified* Rds(on) is **Vgs = 4.5 V**. Vgs(th) max on power dice runs
2.0–2.5 V, so a worst-case part sits at threshold with a 2.5 V gate and no vendor can guarantee
anything there. "Logic level" means it works at 5 V, not 3.3 V. Toshiba's parametric database returns
zero N-channel parts with Id ≥ 15 A specified at ≤ 2.5 V.

Beware a false positive when searching: nearly every one of these datasheets contains the strings
"VGS = 2.5 V" and "VGS = 3 V" as *curve labels on the Rds(on)-vs-Vgs graph*, never as table rows.

Two parts do qualify, and both have a catch:
- **IRF3708** — TO-220, specified **29 mΩ max at Vgs = 2.8 V**, a true pin-compatible drop-in needing
  no circuit or firmware change (0.63 W, ~39 °C rise bare). But it is **obsolete / NRND**; DigiKey
  lists it as no longer manufactured. A one-time fix off legacy stock, not a design.
- **CSD17307Q5A** — TI, 17.3 mΩ max at Vgs = 3 V, in production and cheap (LCSC C139362). But it is
  **SON 5×6 surface-mount with a bottom thermal pad** — reflow, not a hand iron.

**So the gate driver is the real answer**, and it is also the cheapest, because at Vgs = 10 V the
IRLZ44N already fitted is a *good* part: 22 mΩ max → **0.475 W → ~29 °C rise with no heatsink at
all**.

- **Use a driver with a fixed TTL input threshold**: TC4420 / MCP1407 (both **PDIP-8**, so
  perfboard-friendly), or UCC27517A (SOT-23-5). All spec VIH ≤ 2.4 V across VDD = 4.5–18 V, so a
  3.3 V GPIO drives them with ~0.9 V of margin. Non-inverting, so **no firmware change**. Low-side
  switching needs no bootstrap — just the 12 V rail and a 100 nF decoupler. Average supply draw is
  48 nC × 500 Hz = 24 µA.
- **Do NOT use UCC27518 or UCC27519.** Their inputs are CMOS and scale with VDD (VIN_H = 70% of
  VDD), so at 12 V the threshold is 8.4 V and a 3.3 V signal never registers.
- **Avoid the discrete inverting level shifter** (2N7002 common-source with a pull-up to 12 V),
  despite it being the fewest parts. It inverts, so at ESP32 boot and reset the GPIO floats, the
  small FET stays off, the pull-up drags the gate to ~11 V and **the compressor runs full-on**. A
  10 kΩ pulldown does not win against a 1 kΩ pull-up. Not worth saving a few cents on a pneumatic
  system.

**Also worth knowing: the gate never even sees a clean 3.3 V.** The ESP32-S3 datasheet specifies
IOH = 28 mA only at VOH ≥ 0.8 × VDD = **2.64 V**, so during the Miller plateau — exactly when the die
is dissipating — the pin sags toward 2.64 V. The real drive is worse than a 3.3 V analysis suggests.

**Next PCB revision:** UCC27517A (SOT-23-5) plus 12 V gate drive, which re-opens the entire rejected
list — IRLB8743, IPP034N03L, PSMN2R7-30PL are all 3–4 mΩ and in production. Add a 10 kΩ pulldown on
the driver input so the compressor is off through boot and reset, and a 10–47 Ω series gate resistor.

*(Note: the research pass also proposed correcting the ~2 W / ~130 °C figures in these notes down to
1.08–1.51 W. Checked and rejected — that recalculation used the cold Rds(on) and dropped the ~1.6×
rise at 100 °C from datasheet Fig 4. The 50–70 mΩ figure in these notes is the cold value; hot it is
80–112 mΩ, giving 1.73–2.42 W, and the bench measurement backs the higher range.)*

### Switch the compressor to the HA210N06 hotbed module — preferred fix

Available in inventory: an HA210N06 in a 3D-printer hotbed MOSFET module, with heatsink, control-in,
DC-in and load terminals. This is the better route, because it fixes the actual cause rather than
the symptom: the module drives the gate from its own DC rail instead of from 3.3 V logic.

At 10 A its 4 mΩ fully enhanced gives **0.36 W against the IRLZ44N's 5.76 W — 16× less** — and it
arrives with a heatsink.

**Datasheet confirmed 2026-07-18** (`datasheets/HA210N06_datasheet.pdf`, Rev 0.9, HL/Haolin):

| | |
|---|---|
| Rds(on) | **4 mΩ max @ Vgs = 10 V, Id = 75 A — the ONLY row in the table** |
| Vgs(th) | min 2 / typ 3 / **max 4 V** |
| Qg | **135 nC** @ Vgs = 10 V |
| Ciss | **5800 pF** |
| Rθja / Rθjc | 62.5 / 0.68 °C/W |
| Pinout (TO-3P) | 1 = Gate, 2 = Drain, 3 = Source |
| Note 3 | *"Package limitation current is 50 A"* — the 210 A headline is not the continuous rating |

Three consequences, all pointing the same way:

1. **It is emphatically not a logic-level part, and 3.3 V is below its worst-case threshold.**
   Vgs(th) max = 4 V, so a worst-case device has not even begun to conduct at 3.3 V. Wired straight
   to GPIO 3 it would be *worse* than the IRLZ44N, which at least specifies down to 4.0 V. 5 V is
   only 1 V of overdrive and still completely unspecified.
2. **Rds(on) is specified at exactly one point, Vgs = 10 V.** Below that the datasheet promises
   nothing at all — the same trap as the IRLZ44N, one step further.
3. **The gate is large: Qg = 135 nC, Ciss = 5800 pF**, roughly 3× the IRLZ44N's 48 nC / 1700 pF.
   Even ignoring voltage, an ESP32 pin at ~20 mA would need ~7 µs to move that charge. Direct GPIO
   drive is impossible on current as well as on voltage.

> **SOLVED on the bench 2026-07-26 — the compressor now runs from a 3.3 V GPIO.** Two changes,
> both required: (1) bypass the module's input bridge rectifier by tacking GPIO 3 to the bridge's
> **+** output pad and ESP32 ground to its **−** pad, leaving the bridge soldered in place and
> unused; (2) tack a **330 Ω in parallel with R3** (10 kΩ ∥ 330 Ω = 319 Ω), giving ~6.5 mA into the
> opto LED. Neither change alone works — bypassing with R3 at 10 kΩ still gives only 0.22 mA, and
> 330 Ω with the bridge still in path gives ~2.3 mA typical but ≤1 mA at the pessimistic corner.
> Component values now confirmed: opto is a **Sharp PC817** (SOP-4, marked `CW831`, designator
> **U2**); **R3 = 10 kΩ** (0603, code `1002`), *not* the 1 kΩ assumed in the table below, so every
> current in that table is ~10× too optimistic. Full write-up, including why 10 kΩ is correct for
> this module's real 12–24 V design point and what CTR collapse below 1 mA does, is in
> `history.md` under 2026-07-26. The two remaining constraints below (200–250 Hz PWM, flyback
> diode) are unaffected and still apply.

**Control-input topology confirmed by inspection, 2026-07-25: the module carries a bridge rectifier
and an optocoupler.** That settles the open question of whether the carrier boosts the gate — it
does, off its own DC-IN rail, so **the module is the gate driver and no separate TC4420 /
UCC27517A is needed**. It also means the control input is not a voltage-sensed logic pin but a
**current-driven LED sitting behind two diode drops**, which changes the drive requirement entirely.

**Symptom, 2026-07-25: wired to the module, the compressor does nothing.** GPIO 3 measured 3.3 V /
0 V alternating with the signal wire *unplugged*; it has not yet been measured while connected.

**The bridge is what breaks 3.3 V drive.** The path is Control In → bridge → series resistor → opto
LED, so ~1.3 V (two silicon drops) stacks on the LED's ~1.1 V Vf and **~2.4 V is consumed before any
current reaches the resistor**. Only the remainder drives the opto, and bypassing the bridge
recovers most of it:

| Drive at Control In | Fixed drop | If (1 kΩ assumed) |
|---|---|---|
| 5.0 V — the module's design point | ~2.4 V | 2.6 mA |
| 3.3 V nominal, as wired | ~2.4 V | 0.9 mA |
| 2.64 V — loaded ESP32-S3 pin, per the VOH note above | ~2.4 V | 0.24 mA |
| 3.3 V with the bridge bypassed | ~1.1 V | **2.2 mA** |
| 2.64 V with the bridge bypassed | ~1.1 V | **1.5 mA** |

A 1.5× drop in drive voltage costs ~3× the LED current, because the fixed drops eat the headroom
nonlinearly. At the pessimistic corner (bridge 1.4 V + Vf 1.4 V = 2.8 V) a loaded 3.3 V pin delivers
**zero** and the LED never turns on at all; bypassed, that same corner still gives ~1.2 mA.
PC817-class CTR also collapses below ~1 mA, so what little current does flow transfers badly. The
ESP32 can easily *supply* milliamps — it cannot *push them through* 2.4 V of diode drops from 3.3 V.
This is a voltage-headroom problem, not a current-capacity one.

**Fix, no added ICs — bypass the bridge by injecting past it, not by desoldering.** The bridge has
two AC pins (fed from the Control In screw terminals) and a **+** / **−** output pair feeding the
resistor and LED. Tack GPIO 3 to the **+** pad and ESP32 ground to the **−** pad, leaving the bridge
in place and unused. Two wires, reversible. That lands at ~85% of the module's own 5 V design
current. For margin, solder a second 1 kΩ across the existing series resistor — 500 Ω effective
gives 4.4 mA at 3.3 V and 3.1 mA on a sagging pin, far under the PC817's 50 mA continuous rating.
The only loss is polarity-insensitivity on the control wire; the opto barrier is untouched, so
galvanic isolation survives.

**Do these two before soldering:**

1. **Confirm the bridge sits on the control input, not on DC IN.** Trace the Control In screw
   terminals — they should land on the bridge's two AC pins. If it instead protects the 12 V input
   against reverse polarity, bypassing it is wrong and removes real protection; in that case the
   control side carries only the LED's ~1.1 V drop and 3.3 V is marginal rather than dead.
2. **Jumper test — five minutes, no soldering.** Disconnect the ESP32, wire Control In+ to 5 V and
   Control In− to GND. Compressor runs → the module is good and drive level is the whole bug.
   Nothing → the fault is DC IN, the load side, or the module itself, and the bypass is wasted
   effort. 5 V is safe here; these inputs are sized for 5–24 V precisely because of the bridge.

**Two failure modes specific to an optocoupled input, worth ruling out first — both produce exactly
"does nothing":**

- **The control input is a floating two-terminal loop, not a ground-referenced signal.** Driving
  Control In+ from GPIO 3 while leaving Control In− unconnected — relying on the shared ground a
  non-isolated module would have given — passes no current at all. Both terminals must be driven.
- **DC IN must be independently powered.** The optocoupler only gates the module's own rail; with no
  12 V on DC IN the module is inert no matter what the control side does.

**Still unverified on the module:** whether the output stage inverts. (The optocoupler part number
and series resistor value were resolved 2026-07-26 — see the banner at the top of this section.)
The symptom partly answers it — an inverting stage with
a starved LED would leave the compressor stuck **on**, and it is doing nothing, which points to
non-inverting.

**Two constraints that still apply once it is driven properly:**

1. **Run at 200–250 Hz, not 500 Hz.** Optocoupler turn-off is slow — tens of µs — and slow edges at
   500 Hz put the MOSFET in its linear region for a real fraction of every cycle. Hotbed modules are
   built to switch a bed at well under 1 Hz, and slow turn-off with fast PWM is a known way to cook
   them. More LED current speeds up turn-on but not the phototransistor's turn-off, so the bypass
   does not relax this. (~200 Hz is the floor from the note below, where motor current goes
   discontinuous.)
2. **Add or keep a flyback diode.** These modules are designed for a *resistive* heater and
   generally have none. The compressor is inductive. The existing freewheel diode must stay, or one
   must be fitted at the module's load terminals.

**Grounding.** The confirmed optocoupler makes the module galvanically isolated, which is a genuine
bonus here — it breaks the ESP32-to-compressor ground path behind the USB brownouts in `history.md`.
Keep the star-ground rule on the power side regardless: compressor return goes to power ground at
the regulator, not through signal ground.

**Worth considering instead of PWM entirely:** the 60% duty exists only to step 12 V down to the
motor's rated 7.5 V. A buck converter set to 7.5 V with a plain on/off switch removes the
hard-switching problem at its source, at the cost of a part rated for the motor current.

### Pressure reads LOW while the compressor runs — and the current thresholds are calibrated wrong because of it

Observed 2026-07-18: the measured tank pressure sags while the compressor is running and recovers
once it stops. Rubén's proposed workaround is to stop the compressor periodically so the reading is
taken with the motor off.

**This is probably not a small offset, and it likely explains the calibration puzzle below.** The
run that stopped at a reported 7.5 bar was using an OFF threshold of ADC > 1638, yet the settled
reading afterwards was **2679**. (The 7.5 itself is void — see `.agents/error-log.md` 2026-07-27 —
but the ADC-to-ADC part of this observation does not depend on it.) With duty at 0 the tank cannot gain pressure, so the sensor reading
rose ~1041 counts (~64%) between "running" and "settled" at equal or falling true pressure. That is
the running bias, not a calibration error in the sensor.

**Safety consequence — check before the next unattended run.** The 7/8 bar thresholds now in
`main.c` were derived from a **settled** reading but the control loop evaluates them
against a **running** reading, which is biased low. So the compressor stops later than intended, and
the reservoir is rated 10 bar:

- If the bias is a fixed *offset* (~1041 counts), the OFF threshold of 2858 while running is about
  3899 settled, roughly **10.9 bar — over the reservoir rating**.
- If it is a fixed *ratio* (~1.64×), 2858 running would need ~4687 settled, which is **past the
  ADC's 4095 full scale — the threshold could never be reached and the compressor would run until
  something gives**.

Neither model is confirmed and the truth is likely in between, but both point the same way: the
thresholds are non-conservative in exactly the direction that matters. Do not leave the compressor
running unattended until this is settled.

**CAUSE IDENTIFIED 2026-07-18 — ground IR drop, not the sensor.** Rubén: the pressure sensors are
powered from a **24 V regulator that feeds nothing else**, which rules out supply sag. That leaves
the shared ground path, and the arithmetic closes:

| | |
|---|---|
| Observed shift | 1041 counts = **0.84 V** at the ADC |
| Resistance needed to cause it at 8 A | **~105 mΩ** |
| A 0.25 mm x 50 mm 1 oz trace | **96 mΩ → 0.77 V at 8 A** |

The compressor MOSFET switches **low-side on the board**, so while the motor's + comes from the
battery externally, the full ~8 A return flows through the medulla's ground copper — copper drawn
for a ~1 mA logic feed (see `dv-hardware` `projects/kart-medulla/requirements.md`). That drop lifts
the ESP32's ground relative to the sensor's reference at the 24 V regulator. The ADC reads
`sensor_out − esp32_gnd`, so a rising `esp32_gnd` makes the reading **fall** — sign, magnitude and
the instant recovery when current stops all match.

**So neither the sensor nor the regulator is faulty.** The ADC is faithfully reporting a difference
against a reference that moved. This is the same mechanism as the USB brownouts in `history.md`.

**Confirm with one measurement:** DC volts between the 24 V regulator's GND and the ESP32's GND
while the compressor runs. Expect 0.5–1 V where a shared ground should read ~0 mV. It should also
scale with compressor current, so it will be smaller at lower duty.

**The fix is therefore grounding, not the sensor:** the motor return must go straight back to the
regulator and meet signal ground at exactly one star point, with copper sized for the current.
Logged against medulla-v2 in `dv-hardware` `tasks/kart-medulla.md`.

**Fixes, cheapest first:**
- **Measure with the motor off** (Rubén's suggestion): pause the compressor briefly, let the reading
  settle, sample, then decide. Simple and needs no hardware, but it lengthens each cycle and the
  pause has to be long enough for the rail and the reading to actually recover — measure how long
  that takes rather than guessing.
- **Recalibrate the thresholds against RUNNING readings** instead of settled ones. This is the
  smaller change and it makes the existing control loop correct without pausing anything, but it
  leaves the measurement wrong for display and for anything else that reads tank pressure.
- **Fix the cause**: separate the sensor supply/reference from the compressor's power path (star
  ground, per `history.md`), or move to a sensor that is not ratiometric off the sagging rail.

Distinguishing 1 from 2 is one measurement: log the sensor supply voltage, or the rail, while the
compressor runs. If the rail sags in proportion to the reading, it is cause 1.

### Tank pressure thresholds do not match the verified sensor calibration

> **BELIEVED OBSOLETE 2026-07-26 — read before doing any of this; yours to close.** This task is
> written against `ADC_1_BAR = 819` / `ADC_2_BAR = 1638`, and **those constants no longer exist**.
> `main.c` now has `ADC_PRESSURE_LOW = 2500` / `ADC_PRESSURE_HIGH = 2858` (~7 and ~8 bar), set from
> a 2026-07-18 figure that is now void (see `.agents/error-log.md` 2026-07-27 — there is no mechanical
> gauge on this kart and that number cannot be used), and the dashboard now derives bar from the
> sensor chain instead: 1 V/bar, 3:1 divider, eFuse-calibrated millivolts from the firmware. The `bar = 3 x Vadc` map this task argues from is also now known to be
> wrong on two counts — ADC full scale is 2900 mV not 3300, and the divider cannot be 3:1 — see the
> calibration note in kart-brain `src/kb_dashboard/kb_dashboard/protocol.py`. What is *not* settled
> is the running-vs-settled bias above, which is a separate and more serious problem.

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

> **BELIEVED OBSOLETE 2026-07-26 — yours to close.** Same reason as the task above: it argues from
> `ADC_1_BAR` / `ADC_2_BAR` (819 / 1638), which no longer exist, and from `bar = 3.0 * V_adc` /
> "~414 raw ADC counts per bar", which is the map now known to be wrong. Its claim that "the
> dashboard bar reading already uses the verified calibration" was true when written and is now
> backwards — the dashboard was the side that disagreed, and it was corrected on 2026-07-26.
>
> **One bullet below survives and is worth keeping**: converting in firmware with `esp_adc_cal` and
> sending millivolts instead of raw counts. That is now a *better* idea than when it was written,
> because it would replace the guessed full-scale figure with the chip's own calibration and remove
> one of the two errors that caused this whole mess. Consider re-filing it as its own task before
> closing this one.

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

### One-shot Orin commands are delivered blind, 100x, while an echo that could acknowledge them goes unused

**LOW PRIORITY (Rubén, 2026-08-10)** — the current scheme works and the burst costs nothing
measurable, so this waits behind anything that blocks driving the kart. Do not start it as filler
work: it is a protocol change across two repos and needs both sides landed together plus a drive to
validate, so picking it up cheaply is not an option.

Opened 2026-08-10 (Rubén's question). `ORIN_STEER_PID` and the other one-shot commands are sent by
`publish_steer_pid` in kart-brain `dashboard_node.py:527-553` as the same frame repeated **100 times
at 100 Hz for one second**, because there is no acknowledgement and a single frame can be lost. The
firmware meanwhile sends `ESP_STEER_PID` once a second carrying the gains actually in force. So a
confirmation channel already exists — nothing acts on it automatically, and the two mechanisms were
built without reference to each other.

The burst is wasteful rather than harmful, and the bandwidth objection does not hold: it travels
Orin -> ESP32, while the 87%-of-link steering telemetry travels ESP32 -> Orin, and UART is full
duplex. 24 bytes x 100/s is ~24 kbit/s, about 21% of an otherwise near-idle RX direction. Note too
that `pid_apply_override` re-reads the request every 2 ms, so any single frame of the 100 produces
the identical result — the other 99 change nothing.

**Why the echo is not already an acknowledgement:** it reports the *clamped* gains, not the request.
Ask for `kp = 99` and the echo returns the ceiling, so a mismatch cannot distinguish
arrived-and-clamped from never-arrived. Closing the loop on the current frame would require the Orin
to replicate the firmware's clamp constants, which moves a duplicate across a repo boundary and puts
the gear-protection limits somewhere they are not understood.

**Proposed fix — a sequence number.** Add a counter to `ORIN_STEER_PID`; the firmware stores the one
it last accepted and includes it in the `ESP_STEER_PID` echo. The Orin then sends once, waits for an
echo carrying its number, and resends on timeout. A real acknowledgement, no clamp knowledge needed
off-board, 100 frames down to typically one. Detection latency is up to 1 s (the echo rate), which is
fine for human-paced tuning and would want reconsidering before using the same scheme for anything
time-critical.

Not urgent — the current scheme works. It is a protocol change across kart-medulla and kart-brain,
so it needs both sides landed together and a drive to validate. Decide first whether to apply it to
every one-shot command (`ORIN_STEER_MODE`, `ORIN_COMPRESSOR_DISABLE`, `ORIN_CALIBRATE_STEERING`) or
only to the PID frame.

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

- ~~**`KM_GPIO_WriteDAC()` is not ported to the MCP4922.**~~ **Done** — the S3 branch calls
  `mcp4922_write()` per channel (`km_gpio.c:570-596`), and SPI → DAC → output was confirmed on the
  bench (Rubén, 2026-08-10). Brake stays unwritten until the proportional valve is wired.
- **Nothing drives `PIN_SDC_NOT_EMERGENCY` (GPIO 18).** R23's pulldown holds it off at boot, so the
  kart sits in emergency until firmware drives it HIGH. Until then it cannot be armed. **Safety —
  read the pin's note in `.agents/esp32s3-pinmap.md` before touching.**
- **Nothing drives `PIN_SELECT_THROTTLE` (GPIO 15).** R32's 10 kΩ pulldown makes the default pedal
  pass-through (safe); firmware must drive it HIGH for throttle-via-DAC.
- **`PIN_STATUS_LED` (GPIO 48) needs RMT**, not plain GPIO — it's an addressable RGB.

### Throttle-DAC bypass: is the MCP4922 actually dead, or just never written to?

Opened 2026-07-31. Rubén decided to bypass the MCP4922 and drive the throttle analog signal from an
ESP32-S3 GPIO directly (LEDC PWM + RC filter — the S3 has no DAC peripheral at all). Full reasoning,
the schematic net path and the 0-5 V vs 3.3 V range problem are in `history.md` under that date.

**The open question, and it decides how much work this is:** nobody has recorded whether the
MCP4922 was measured faulty on the bench, or whether "the DAC is not working" simply describes
`KM_GPIO_WriteDAC()` returning `ESP_OK` without doing anything on the S3 branch
(`components/km_gpio/km_gpio.c:352-363`, `TODO: Implement MCP4922 SPI write for S3`) combined with
nothing ever driving `SELECT_THROTTLE` (GPIO 15). If it is the latter, writing the SPI transaction
is far less work than cutting into the board — and it keeps the full 0-5 V range that a 3.3 V pin
cannot reach. Establish which before modifying hardware.

Follow-on decisions once that is settled: whether to accept the ~66% throttle ceiling a 3.3 V pin
gives on a net designed for 0-5 V, or add a gain stage; and whether to inject on the
`CMD_ACC_ESP32__0_5V` net (keeps the MAX4660 mux's fail-safe pedal pass-through) or at CN10.1
(discards it). The gain question has a cheap answer — **U1B (LM358DR channel B) is unused on the
board**, parked as a grounded unity follower, and rewiring it gives both the buffer the RC filter
needs and the 1.52x gain that reaches 5 V. Values and cautions in `history.md`.

**Cheap test that settles the open question:** power the board and meter U13 pin 1 (VDD) and pins
11/13 (VREFA/VREFB) — all three tie to `+5V_REG`. 5 V on all three means the chip is alive and
merely unwritten.

**Both answers are now written and building** (2026-07-31, neither on hardware yet):
- Branch **`spi-fix`** keeps the MCP4922 and implements the SPI write, with bench logging and
  `KM_GPIO_McpSelfTest()` — a channel-A sweep to meter at U13 pin 14. Flash it, watch the console,
  meter VOUTA. If the voltage moves, the chip is fine and no rework is needed at all.
- Branch **`dev`** drives throttle as filtered PWM from GPIO 38 and needs the hand wiring below.

Try `spi-fix` first — it costs one flash and no solder.

> **RESOLVED (Rubén, 2026-08-10): the MCP4922 is not dead — SPI → DAC → output was seen working on
> the bench.** So the open question at the top of this section is answered: it was never a faulty
> chip, only an unimplemented write. The SPI write is now in `km_gpio.c`. What remains open is only
> the GPIO-38 PWM route on `dev` (whether it was ever soldered — see the separate task) and whether
> that route is still wanted at all now that the DAC path works.
>
> **Brake (MCP4922 channel B) stays unwritten on purpose**: the proportional braking valve is not
> wired yet (Rubén, 2026-08-10). Nothing in `main.c` writes brake, and channel B costs no ESP32 pin
> — `PIN_CMD_BRAKE` is the stand-in value 201, not a GPIO. Leave it that way until the valve exists.

- [x] **~~GPIO 38 is now taken~~ — reverted, so 38 and 39 are both free again.** Checked 2026-08-10:
      `GPIO_NUM_38` appears nowhere in `components/`, `main/` or `platformio.ini` on `dev`. Commit
      `e12f6b5` ("drive the throttle from GPIO 38 as filtered PWM") is in `dev`'s history but a later
      commit removed it, when the throttle went back to the MCP4922 — so
      `.agents/esp32s3-pinmap.md` listing both under "Free" is correct after all, and `README.md`
      now says the same. Still worth correcting, and unaffected by the revert: kart-brain's
      `history.md` ("Where the PWM lands on medulla") says "GPIO 38 is earmarked for the EBS
      compressor PWM", which was never true — the compressor is on GPIO 3. That same entry
      recommends routing the steering sensor to GPIO 39; superseded, it went to CN5.2 / GPIO 1.
- [x] **Bug found while investigating, independent of which route is taken:** on the S3 both
  `PIN_CMD_ACC` and `PIN_CMD_BRAKE` were `GPIO_NUM_NC` (-1), so the first `if` in
  `KM_GPIO_WriteDAC()` caught every call and the function could not distinguish throttle from brake.
  **Fixed 2026-07-31**: the two are now `((gpio_num_t)200)` and `((gpio_num_t)201)` in
  `km_gpio.h:106-107` — distinct, above the S3's GPIO range so they cannot alias a real pin, and
  below 256 so `km_act`'s `uint8_t` does not truncate them. `KM_GPIO_WriteDAC()`
  (`components/km_gpio/km_gpio.c:570-596`) dispatches ACC to `mcp4922_write(MCP4922_CH_A, …)` and
  BRAKE to channel B, and logs an error for anything else.

- [ ] **Stale comment: `components/km_act/km_act.c:35`** sets `act.dacChannel = PIN_CMD_BRAKE` with
  the trailing comment `// GPIO 26 (DAC2)`, which is the classic-ESP32 pin. On the S3 that value is
  the MCP4922 channel-B stand-in, not a GPIO. Fix the comment.

### Decide how a manufactured PCB is identified, then state it in all three repos

Opened 2026-07-31 by Rubén. No repo records which hardware revision it targets, so "does this
firmware match the board on the kart?" cannot be answered. Concrete instance found the same day: the
boards were fabbed from dv-hardware `84d6dd0`, but HEAD is `f68cc1f`, which changed the brake output
(CN10.2 relabelled to the 0-10 V amplified net, and the U13.10 -> U1.3 DAC-to-amplifier copper
restored after most of it had been deleted). So the board that exists behaves differently from the
schematic, and nothing anywhere says so. Full reasoning in `history.md`, 2026-07-31.

Rubén's proposal: a code identifying the manufactured PCB, a QR of it on the board, per-board defects
logged in dv-hardware, and the code quoted from kart-medulla and kart-brain — possibly using a
dv-hardware commit hash as the code itself.

**DECIDED 2026-07-31 (Rubén): the commit hash is the code.** The QR or label on the board carries the
dv-hardware commit its gerbers were exported from, and firmware repos quote that hash. Written up in
`dv-hardware/README.md` and `dv-hardware/projects/kart-medulla/README.md`.

Git tags were proposed and rejected. The argument for them was that hashes have no order — you cannot
tell which of two is newer without a lookup — and Rubén's answer is that GitHub already does that, so
the ordering is not worth a second identifier. Agreed; the objection does not survive.

One qualification did survive and is folded into the convention: **a board stops matching its hash the
moment it is reworked.** The hash names a design, while a cut trace or a lifted pin exists on one
physical board and in no commit — and this board is about to get both (the CN10.2 brake patch, and
possibly a lifted U13 pin 14 for the throttle bypass). So each board carries a rework list beside its
hash, and a rework entry may name the commit whose behaviour it brings the board up to.

The other objection — that the last schematic edit and the gerber export are different commits — is
resolved rather than rejected: the convention says to stamp the gerber-export commit.

- [x] Pick the scheme.
- [x] Record it in dv-hardware's READMEs, with the one existing board (`84d6dd0`) and its outstanding
      rework.
- [x] First pairing logged in kart-brain's `history.md`: kart-brain `main` = `c200e56` against
      dv-hardware `84d6dd0`.
- [ ] Put the QR/label on the board itself. **Silkscreen half done 2026-07-31** (dv-hardware
      `b4fe1e2`): `kart-medulla.kicad_pcb` now carries `kart-medulla-v2`, `Design ID`, a 9.45 mm QR
      and the digits `1604 0948 4608 5574`, right of CN5 at (145.1, 51.65). DRC clean, and the QR
      decodes from the plotted silkscreen. Note what it carries is the **design ID**, not the commit
      hash — a commit cannot contain its own hash, so the board points at the kart-docs page
      `/p/1604094846085574/` and the gerber-export hash lives on that page instead (reasoning in
      dv-hardware `history.md`, 2026-07-31). Still to do: the sticker for the board that already
      exists (`84d6dd0`), which *can* carry the hash directly, and filling in that page with the
      hash, fab date and rework list.
- [x] State the target hash in this repo's README, so the firmware side declares it
      rather than only kart-brain.

### Steering sensor read over PWM — written, not yet validated on hardware

`components/km_sdir/km_sdir_pwm.{c,h}` reads the MT6701's PWM angle output on GPIO 1 (CN5.2) via
MCPWM capture: both edges timestamped in hardware, period sanity check against 994.4 Hz ±25%,
median of 5 frames, `NAN` whenever no angle is known. `main.c` feeds the PID from it and stops the
steering motor instead of acting on an unknown angle. The AS5600 I²C path is no longer the steering
source on the S3 (it answers at 0x36; this sensor is at 0x06) — the classic ESP32 build still uses
it. Both targets build; nothing has been flashed or driven.

- **Validate on the bench.** Flash the S3, watch `read_telemetry.py`: `[STEERING]` should track the
  magnet and the health frame should show `Steer:True` with `frames` climbing and `rejects` flat.
  Unplug the sensor lead and confirm the angle reads `INVALID` rather than a number — that is the
  check that matters, per the Sensor Validity rule in `AGENTS.md`.
- **Confirm the control loop stopped stalling.** The I²C timeout was making `control_task` run at
  ~8.9 Hz in bursts against a 500 Hz target (2026-07-25). The PWM read cannot block, so the
  iteration counter in the pneumatic frame should now climb evenly at roughly the nominal rate.
- **The EBS trip on steering-sensor loss does nothing yet, because the SDC is never closed.**
  Decided 2026-07-26 (Rubén): losing the angle under closed-loop steering zeroes the throttle and
  fires the EBS. That is implemented — `KM_GPIO_SetEmergency(1)` drives `PIN_SDC_NOT_EMERGENCY`
  (GPIO 18) LOW, opening the shutdown chain. **But nothing in this firmware ever drives that pin
  HIGH**, so the chain is already open from boot and asserting it is currently a no-op in effect.
  The trip becomes real the moment the arming path exists — see the `SDC_NOT_EMERGENCY` gap in this
  same section and in `.agents/esp32s3-pinmap.md`. Until then, the throttle-zeroing half is the
  only half that bites.
- **How should a latched steering trip be cleared?** Currently: reboot only, and closed-loop
  steering never comes back within a run (open-loop direct-PWM stays available so the column can
  still be moved while diagnosing). Reboot-only is the conservative placeholder, not a decision —
  a race-day dropout that needs a power cycle to clear may be the wrong trade.
- **The comms watchdog has the opposite problem and was not touched.** On stale comms or
  `MISSION_MANUAL`, `main.c` calls `KM_ACT_Stop()` on the brake, which *releases* it, and does not
  assert the SDC. That contradicts the steering-loss decision above, which fires the EBS for a
  less severe fault than losing the Orin entirely. Already noted as gap 5 in
  `.agents/esp32s3-pinmap.md`; left alone here because Rubén's decision was scoped to the steering
  sensor, but the two should be made consistent.
- **`SENSOR_CENTER = 2250` is undocumented and probably wrong.** `km_sdir.h:48` calls it "half of
  the 12-bit range", but half of 4096 is 2048, and `km_sdir.c:229` repeats the 2048 claim in a
  comment while the code uses 2250. Both the PWM and I²C paths centre on it, so it sets where zero
  steering is. It has to be measured against the column once the sensor is mechanically mounted;
  until then, straight-ahead will not read 0.
- **The dashboard will report "no sensor" even when the steering sensor is working perfectly, until
  kart-brain is changed.** `src/kb_dashboard/kb_dashboard/protocol.py` `decode_health()` reads only
  flags bit0 = `magnet_ok`, bit1 = `i2c_ok`, bit2 = `heap_ok`. Both steering-related bits it reads
  are AS5600 I²C facts, and on the S3 they are now deliberately and permanently 0 — the firmware no
  longer polls an AS5600 that cannot answer. The live steering health moved to **bit 3**
  (`Steer OK`) and **bit 4** (fault latched: EBS fired, throttle refused), with accepted/rejected
  frame counts appended as payload fields 5-6. The dashboard has to read the new bits, and should
  stop presenting `i2c_ok` as steering health at all.
- **kart-brain must also decode the steering frame's validity field.** `ESP_ACT_STEERING` grew a 4th
  int32 (1 = the angle is real, 0 = not) and sends `INT32_MIN` in the angle field when invalid.
  Appending keeps old decoders working, but until kart-brain checks the flag the dashboard will plot
  `INT32_MIN/1000` as an angle. `read_telemetry.py` in this repo already handles both.

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

**Fixed 2026-08-08** — comment rewritten to say the S3 env is the primary, working target, pointing
at the "ESP32-S3 firmware gaps" task for the still-open peripheral claims. The main.c rate comments
(control 10 Hz → 500 Hz; AS5600-caps-rate → UART-bound) were fixed in the same pass.

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

### Two stale statements about the control-loop rate, and the loop is UART-bound at 500 Hz

Found 2026-07-31 while answering "what rate does the steering PID run at". The rate itself is
settled: `control_task` is registered with a 2 ms period in `main/main.c` (`KM_COMS_CreateTask(...,
2, ...)`), `CONFIG_FREERTOS_HZ=1000` makes that exactly 2 ticks, and `history.md` records it
measured on the kart — `control_iters` advancing by exactly 25 between consecutive 20 Hz pneumatic
frames. 500 Hz, confirmed. Two comments say otherwise:

- [x] **Both fixed on 2026-08-08, confirmed 2026-08-10.** The `system_init` docstring reads
  "control (500 Hz)" (now `main.c:1016`), and the AS5600 remark (now `main.c:408`) is phrased as
  history — it explains why the old send-then-read order *existed* and states that the PWM read
  cannot block. Nothing left to change. The fix was already recorded under the `platformio.ini`
  section above while these two boxes stayed open, which is how they survived.

- [ ] **Decide whether the per-cycle steering frame should stay per-cycle.** `control_task` sends a
  20-byte `ESP_ACT_STEERING` frame every cycle (SOM + len + type + 4x int32 + CRC). At 8N1 that is
  200 bits x 500 Hz = 100 kbit/s of a 115200 bit/s link — 87% — before the 20 Hz pneumatics frame
  (~8.8 kbit/s) and any `ESP_LOG` output, which share UART0. `uart_driver_install(UART_NUM_0, 1024,
  0, 0, NULL, 0)` passes tx_buffer_size = 0, so `uart_write_bytes` blocks the control task until
  the hardware FIFO drains. The loop therefore cannot be raised toward 1 kHz without either
  decoupling telemetry from the control cycle (send feedback at 50-100 Hz) or raising the baud —
  raising the rate alone converts the shortfall into a stalled loop rather than dropped frames.
  Not urgent: nothing currently needs a faster loop (see the next item), but the headroom figure
  should be written down somewhere the next person changing the frame will see it.

- [x] **The median-of-5 was removed on 2026-07-31.** It cost a group delay of 2 samples (~2.0 ms at
  994.4 Hz) on the angle every consumer sees, and the evidence said there was nothing to filter: the
  accepted-frame counter measured 993/s with zero rejects, and one angle count is 244 ns of high
  time against MCPWM's 12.5 ns tick. `KM_SDIR_PWM_Median()` is now `KM_SDIR_PWM_Latest()`; the
  validity contract is unchanged. Full reasoning in `history.md`, 2026-07-31.

- [ ] **AT THE KART: watch `ESP_HEALTH_STATUS` field 6 (rejected frames) while driving.** This is
  the check the removal above is riding on, and it has not been done. The zero-reject measurement
  was taken with the kart quiet — the compressor MOSFET (12 V, ~6 A) and the steering H-bridge were
  not switching, and those are the noise sources. Rejects still zero with everything running means
  the removal was right. Rejects climbing means put protection back, and prefer a **slew-rate gate**
  over a median: drop any sample jumping further than the column can physically move in 1 ms
  (~45 counts per frame covers even 4000 deg/s) and pass everything else through undelayed. The one
  glitch that needs catching is a spurious falling edge during the frame's LOW phase — it inflates
  `s_high_ticks` while leaving the period valid, so it decodes to a confidently too-large angle.
  Spurious rising edges in the low phase and falling edges in the high phase are already handled by
  the ISR.

### Steering PID differentiates the error, not the measurement — worth attention

Found 2026-07-31. `KM_PID_Calculate` in `components/km_pid/km_pid.c` computes
`derivative = (error - lastError) / dt`, and `error` includes the setpoint. So a change in the
*target* produces a derivative response that a change in the measurement would not. This is worth
more attention than the sensor-noise question next to it, because sensor noise is roughly zero-mean
and random — its D-term contribution averages out and costs heat and chatter rather than tracking
error — whereas this is systematic and correlated with the command, so it does not average out.

With the gains in `main/main.c` (`kd = 0.10`) and the loop's `dt = 2 ms`, `kd/dt = 50`. Compare
`kp = 1.20`: a one-cycle jump moves the output about 42x harder than the same angle held as a
steady error. Two consequences, at different frequencies:

- **Quantization steps.** `target_raw = KM_OBJ_GetObjectValue(TARGET_STEERING) / 1000.0f`, so the
  setpoint arrives in milliradians. The smallest possible target change, 1 mrad (0.057 deg), yields
  a D term of 0.05 — 10% of the 0.50 actuator limit, from one count. A target step of 0.01 rad
  (0.57 deg) yields 0.5, and anything larger saturates the output for that cycle.
- **A ramping setpoint gives a constant offset, not a spike.** While Orin sweeps the target at a
  rate `r`, the D term contributes a steady `kd x r` in the direction of travel — at 60 deg/s
  (1.05 rad/s) that is 0.105, a persistent 21% of the limit added during every steering movement.
  This may well be baked into the gains chosen during the 2026-07-30 live tune.

Standard fix is derivative-on-measurement: differentiate `-measurement` instead of `error`. It is
identical while the setpoint is constant and removes both effects. Small change, but it alters what
the gains mean, and `kp`/`kd` were tuned on the vehicle on 2026-07-30 — so it needs a re-tune and a
drive to validate, not a quiet edit.

- [x] Switched to derivative-on-measurement on 2026-07-31, with a `primed` flag so the first cycle
  after `KM_PID_Reset()` contributes no derivative (without it, resuming control would differentiate
  the whole current angle in one dt: -54 at 1.08 rad, saturating instantly). `pio test -e native -f
  test_km_pid` passes 14/14, including a new test that a setpoint step produces no D response.
- [ ] **AT THE KART: re-tune `kp`/`kd` and drive it.** Not yet flashed. The 2026-07-30 live tune
  happened with the old derivative, so whatever the setpoint-driven D term was contributing is baked
  into the current gains — expect to re-tune. Each change is its own commit so `git revert` can name
  the cause if the kart drives worse.

  **Tuning order: `kp` first, `kd` only against something you can see.** Sluggish command-following
  is the missing `kd x rate` term the old derivative was contributing, and `kd` cannot give it back
  — the D term no longer sees the setpoint at all, so raising it adds damping on measured motion,
  which works against following a moving target rather than for it. That is a `kp` job, or real
  feed-forward. Raise `kd` only for overshoot past the target or ringing around it.

  Do not assume removing the median opened up room for a much larger `kd`. It cuts both ways: ~2 ms
  less delay buys phase margin, but the same removal roughly doubles the raw jitter reaching the D
  term (a median of 5 outputs ~0.54 sigma of Gaussian noise; a single sample is 1.0), and with
  `kd/dt = 50` that amplification is what limits `kd` in the first place. The measured noise is
  ~zero, so the net is a little more room, not a lot. Note also that `kd = 0.10` is already
  substantial: `kd/kp` is 0.083 s of derivative time, and at 60 deg/s of steering the D term is
  0.105 against a P term of 0.042 at 2 deg of error — D already dominates P during motion.

  If a much larger `kd` is ever wanted, add a low-pass on the D term before raising it.
  Derivative-on-measurement with no filtering anywhere and a large `kd` is the standard way to get
  motor chatter, and a D-path filter costs delay only in the path that can afford it — unlike the
  median, which delayed P and I too.

### Docs across kart-medulla, dv-hardware and kart-docs contradict the code and each other

Found 2026-07-30 during a three-repo audit. These sat under the board's `## Done` heading while
still open — the closed half of the section is now in `tasks/done-archive.md`.

- [x] **Fixed 2026-08-10 — connector-designator mismatch and the stale netlist behind it.**
  dv-hardware's `output/netlist.net` was last exported 2026-08-01, before the 2026-08-08 schematic
  edit that deleted R39; it still showed the dead `ACC_AMP_OUT` net and, per `pinout-cn-connectors.md`,
  Q3/Q4's gates on no net. Re-exported with `kicad-cli sch export netlist` against the current
  schematic (commit `e4a6e10`) — the pressure channels now show as `CN7.1`/`CN7.2`/`CN5.2` in the
  netlist too, matching the v1 silkscreen and PCB (where PCB `CN2` is HALL3/HALL2/+5V_REG, not
  pressure). Added an explicit warning in `pinout-cn-connectors.md` that an earlier schematic draft
  had the pressure channels on `CN2`, and that anyone wiring the kart should read the silkscreen, not
  a schematic printout, in case a future edit drifts again before a new revision is fabricated.
  kart-docs' `pinout.md` (a generated mirror of that file) was re-synced.
- [x] **Fixed 2026-08-10 — `PRESSURE_3` status conflict.** kart-docs was already correct (channel
  retired, GPIO 1 repurposed to read the MT6701 steering-angle sensor's PWM via MCPWM capture — the
  firmware confirms this: `PIN_PRESSURE_3` is `#if`'d out of `km_gpio.c`'s ADC1 list on the S3, and
  `km_sdir_pwm.h` documents GPIO 1 / CN5.2 as its PWM capture input). dv-hardware's
  `docs/pinout-esp32-s3.md` row 19 (the tie-break file kart-docs points to) was the stale side —
  still labelled "Pressure sensor 3 (input only)" even though the same file's own "As-built pin use"
  section, further down, already documented the repurpose correctly. Updated row 19 to match. The
  CN5.2 row in `pinout-cn-connectors.md` had the same stale label and got the same fix. The
  tie-break rule itself ("when this doc and the schematic disagree, the schematic wins") did not need
  rewording — it was correct, just pointing at a file with one stale row.
- [x] **Fixed 2026-08-10 — pressure channel count in kart-docs.** `wiring.md` said "three
  pneumatic-pressure sensors"; corrected to two, matching the BOM (qty 2), the wire list
  (`press1`/`press2` only), and the two remaining ADC channels. This was the only stale count —
  `wiring.md`'s own next sentence, the BOM, and `kart-medulla/index.md` already said two.
- [x] **Fixed 2026-08-10 — two latent ADC bugs in `km_gpio.c`, neither affecting pressure.**
  (a) `PIN_HYDRAULIC_2` (GPIO 2, ADC1_CH1 on the S3) was absent from `adc1_pins[]`, so its
  attenuation was never configured while `KM_GPIO_ReadADC` did have a case reading
  `ADC1_CHANNEL_1` — a channel read at whatever attenuation the driver defaulted to, not 11 dB,
  silently compressing the top of its range. Added to the list under the S3 branch only, since on
  the classic board it is GPIO 14 on ADC2. (b) The `GPIO_NUM_1` attenuation branch could never
  execute, because `PIN_PRESSURE_3` is `#if`'d out of the list on the S3 — GPIO 1 is the MT6701's
  PWM input read through MCPWM capture, not an ADC channel. Branch deleted with a comment saying
  why, rather than "fixed" by adding the pin back. S3 builds, 55/55 native tests pass. Not yet
  flashed — nothing reads hydraulic 2 today, so this changes no current behaviour. (a) On the S3,
  `PIN_HYDRAULIC_2` = GPIO 2 is an ADC1 pin but is handled inside the block commented "ADC2 pins",
  so its attenuation is never configured while `KM_GPIO_ReadADC` does have a case for it — a channel
  read without being configured. (b) The `GPIO_NUM_1` attenuation branch can never execute, since
  `PIN_PRESSURE_3` is `#if`'d out of `adc1_pins[]`; harmless but it is the same "configured is not
  used" trap already logged at `.agents/error-log.md:181`.
- [x] **Checked 2026-08-10 — module variant conflict does not exist today; this item was stale.**
  Both repos already agree the fitted module is **ESP32-S3-WROOM-1-N16R8**: dv-hardware's
  `docs/pinout-esp32-s3.md` and `.agents/esp32s3-pinmap.md` in this repo, and kart-docs'
  `kart-medulla/index.md`, all say N16R8. kart-docs' claim is the strongest evidence on file — it
  cites `esptool` reading `Embedded PSRAM 8MB (AP_3v3)` off the physical board, not just a visual
  check. N8R2 appears only as the originally-ordered part number in historical context (the supplier
  shipped N16R8 instead; see dv-hardware `history.md` 2026-04-23 / 2026-04-29). No doc currently
  asserts N8R2 is what's fitted, so there was nothing to reconcile.
- [x] **Fixed 2026-08-10 — `AGENTS.md`'s UART protocol tables described an encoding the firmware stopped using.** Both tables deleted and replaced by a pointer to `message_type_t` in `components/km_coms/km_coms.h`, with a note recording what they got wrong (int32 arrays, not u8/int16; `ORIN_COMPLETE` is 6 int32 not 7 bytes) and the eight frames they were missing. Regenerating them was rejected: a hand-maintained copy of an enum goes stale the moment someone adds a frame without looking there.  ~~Original entry:~~ — Noticed
  2026-07-30. The two "Message types" tables at `AGENTS.md:153-165` give payloads as `u8 [0-255]`,
  `int16 big-endian, radians × 1000` and `ORIN_COMPLETE | 7 bytes`. The wire format is int32 arrays and
  has been since the protobuf migration — `ORIN_COMPLETE` is 6 int32 elements in `km_coms.c`, not 7
  bytes. The tables are also missing every frame added since: `ORIN_CALIBRATE_STEERING` (0x28),
  `ORIN_STEER_MODE` (0x29), `ORIN_COMPRESSOR_DISABLE` (0x2A), `ORIN_STEER_PID` (0x2B),
  `ESP_HEALTH_STATUS` (0x0B), `ESP_PNEUMATIC` (0x0C), `ESP_STEER_PID` (0x0D). The accurate list is the
  `message_type_t` enum in `components/km_coms/km_coms.h`, whose doc comments carry the payload shapes.
  Decide whether to regenerate the tables from that enum or delete them and point at the header —
  a table that is wrong about the encoding is worse than no table, because it reads as authoritative.
- [ ] **[v2 design only — not the physical board]** The amp stages in dv-hardware HEAD (throttle U1B gain 1+R37/R38 = 1.51, brake U1A gain 1+R19/R20 = 3) are matched to the MCP4922 having moved to +3V3 (dv-hardware `16a35fb`): 3.3 V × 1.51 ≈ 5 V, 3.3 V × 3 ≈ 10 V. On the physical v1 board (`84d6dd0`) none of this exists: throttle is U13.14 → U14.8 direct, MCP4922 at +5V_REG. Verify the gain/VREF pairing stays consistent when v2 is fabbed. (This entry originally claimed a live overrange bug — wrong: it mixed the HEAD schematic with the v1 board. Corrected 2026-08-08.)
- [ ] **AT THE KART: check the board for a leftover GPIO 38 flying wire.** Reframed 2026-08-10 — the
      code half of this resolved itself. `dev`'s `km_gpio.h` no longer mentions GPIO 38 at all (the
      filtered-PWM throttle route in `e12f6b5` was reverted when throttle went back to the MCP4922),
      so there is no longer a code-vs-history contradiction to settle. What remains is purely
      physical, and the revert makes it *more* worth checking rather than less: if someone did solder
      a wire from the U24 socket's GPIO 38 pin toward U14.8, that wire is still there while no
      firmware drives the pin, leaving an undriven input floating onto the throttle net. Look for the
      wire, and check whether U13 pin 14 was lifted. `history.md` (2026-07-31/08-01) records the
      decision to do this rework but never the soldering, so the board is the only source of truth.
- [ ] **Flash dev tip `7bcd6eb` (bench hardcode removed, throttle back on TARGET_THROTTLE) to the S3.** Built clean on the Mac and pushed 2026-08-08; the Orin went unreachable over the tunnel right at the flash step, so the chip may still run the 50%-hardcode build. Then test throttle from the dashboard — mission must be non-manual and comms fresh, or the safety gate holds the mux on the pedal.
  **Probably already done**: Rubén reports (2026-08-10) the kart accelerated *on command* under remote control, which the hardcoded constant 0.5 f could not produce — so the chip was running `7bcd6eb` or later at that point. Confirm the running image the next time the Orin is reachable (`pio run -t upload` and check, or read the app description) and close this.
- [ ] **Stale comment in `platformio.ini`** (S3 env, ~line 66): says "GPIO 18 / GPIO 15 not driven"; km_gpio.c has driven both since 2026-08-01/08-08. Fix the comment.
- [x] **The steering-fault latch survives everything except an ESP32 reboot, and nothing on the
  dashboard says how to clear it.** A stale latch from one bad boot looked like a live sensor/code
  failure and cost hours on 2026-08-08. **Built 2026-08-10** — Rubén chose the timestamp option over
  an operator reset command, which would have let an operator re-arm closed-loop steering after a
  safety trip, the thing the latch exists to prevent.

  Firmware (`92a3248`): the trip time is stamped with `esp_timer_get_time()` before the flag is set,
  and the age in seconds is appended as field 7 of `ESP_HEALTH_STATUS`, or -1 while clear.
  kart-brain (`20db00f` + an uncommitted `index.html` change): the STEER chip now reads
  `STEER SIGNAL LOST 14m 22s ago — signal OK now, reboot ESP32 to clear`, or `— still no signal`
  when the sensor is genuinely down.

  **Found while doing it, and the more consequential half:** `kb_coms_micro` was copying exactly
  three numeric fields onto `/esp32/health/data`, so *every* appended field was discarded before
  reaching the dashboard. The steering frame counters had been sent since they were added and never
  arrived. It now forwards the whole tail.

- [ ] **AT THE KART: verify the trip age end to end.** Nothing above has run on hardware. Needs the
  Orin build below (`kb_coms_micro` is C++ and there is no ROS2 on the Mac). Unplug the steering lead
  under closed-loop steering, confirm the chip shows a small age that counts up, plug it back in and
  confirm it flips to "signal OK now" while still reading LOST. Then reboot and confirm it clears.

- [ ] **Decide whether to generate both sides of the UART protocol from one schema (and whether protobuf belongs anywhere).** Raised by Rubén 2026-08-10, after a past session had researched ROS2/protobuf bridges and picked `bdaiinstitute/proto2ros` as the most complete (others considered: `eclipse-ecal/rosidl_typesupport_protobuf`, `jiayou/proto2rosmsg`).
  **The problem is real and recurring: hand-written encoders and decoders drift.** Three instances on 2026-08-10 alone — `AGENTS.md`'s message tables described the pre-protobuf encoding; kart-brain's `decode_health` silently dropped appended fields, which is why the steering trip age never reached the dashboard; and `decode_steering_raw` returns a 3-tuple while its own test still unpacks 2. Field numbers and optional fields would make the appended-fields class of bug structurally impossible.
  **But protobuf on this wire is the wrong lever.** The 500 Hz `ESP_ACT_STEERING` frame is 20 bytes = 100 kbit/s of a 115200 link, ~87% before the 20 Hz pneumatic frame and any logging, and `uart_write_bytes` blocks `control_task`, so added bytes land as loop jitter rather than dropped frames. Protobuf adds a tag byte per field, which is pure cost on a payload of four int32s. And `proto2ros` helps only the Orin — the ESP32 would need nanopb, i.e. ESP-IDF build complexity, flash, and encode/decode inside a 2 ms cycle, paid on the microcontroller to gain convenience on the machine with 62 GB of RAM.
  **Recommendation: keep the compact framing, generate both sides from one schema.** One table (frame id, name, field names and types) emitting `message_type_t` plus payload accessors for the firmware and the decoders for kart-brain's `protocol.py`. Kills the whole drift class, costs nothing on the wire, roughly a day. It also makes a stale docs table impossible, since there would be one definition to point at.
  If protobuf is wanted specifically, the defensible split is by rate — protobuf for the 20 Hz telemetry frames where overhead is free, tight binary for the 500 Hz control frame. Do not adopt two encodings without a measured reason. Related: the UART headroom item under "Two stale statements about the control-loop rate".

## In Progress
