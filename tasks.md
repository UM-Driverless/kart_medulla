<!-- read in full — kept under 150 lines -->

# Tasks

The repo's task board — the only one. Update status as you go: `TODO → In Progress → Done`. Read
before editing; claim by adding `[YYYY-MM-DD <name>]` and the section change. Big clusters get a
`tasks/<name>.md`, always linked from an index here.

Only Rubén moves a task to Done. "Looks finished" is not the same as confirmed on the hardware — on
this repo that means flashed *and* driven, per the branch workflow in `AGENTS.md`.

## TODO

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
this size (it links btstack + bluepad32). **Not yet tested on hardware** — raised from datasheets, not
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

- [ ] **Stop building them.** `EXCLUDE_COMPONENTS` in the ESP-IDF build, or move the two directories
  out of `components/`. Check both envs before doing it: `esp32dev` is the classic fallback and
  `sketch.cpp` still exists in `main/`, so confirm nothing re-enables either. Verify by rebuilding
  clean and diffing `firmware.bin` — if the image is byte-identical, nothing was lost. Time a clean
  build before and after so the saving is recorded rather than assumed.
- [ ] Decide whether the two components should be in this repo at all. If the gamepad path is dead,
  deleting them removes ~10 MB of build output and 305 source files from every clone; if it is
  wanted later, it is in git history.

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

- [ ] **GPIO 38 is now taken, and two documents still say otherwise.** `dev` assigns it to the
      throttle PWM, leaving **GPIO 39 as the only unconstrained free pin on the board**. Stale
      claims to correct before someone allocates 38 twice: `.agents/esp32s3-pinmap.md` lists both 38
      and 39 under "Free", and kart-brain's `history.md` (the "Where the PWM lands on medulla" entry)
      says "GPIO 38 is earmarked for the EBS compressor PWM" — which was never true either, the
      compressor is on GPIO 3. That same entry recommends routing the steering sensor to GPIO 39;
      that is superseded, it went to CN5.2 / GPIO 1.
- [ ] **Bug found while investigating, independent of which route is taken:** on the S3 both
  `PIN_CMD_ACC` and `PIN_CMD_BRAKE` are `GPIO_NUM_NC` (-1), so the first `if` in
  `KM_GPIO_WriteDAC()` (`components/km_gpio/km_gpio.c:348-368`) catches every call and the function
  cannot distinguish throttle from brake. Anyone implementing the MCP4922 SPI write must change the
  signature or the sentinels first.

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
- [ ] Put the QR/label on the board itself — needs a silkscreen change for future runs, a sticker for
      the board that already exists.
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

- [ ] **`main/main.c:949`, the `system_init` docstring, says "control (10 Hz)".** It is 500 Hz.
  Fix the number.
- [ ] **`main/main.c:1058` says the "I2C AS5600 read caps real rate".** That was true on the
  classic-ESP32 path and is the stall `history.md` documents (~8.9 Hz in bursts). On the S3 the
  angle comes from MCPWM capture of the MT6701's PWM output and never blocks. The real cap now is
  the UART, so the comment points at the wrong bottleneck.

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

## In Progress

## Done

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

- [ ] **`README.md` presents classic-ESP32 pin tables with nothing saying so.** The "Actuator
  Outputs" and "Sensor Inputs (ADC)" tables (lines ~32-46) are pure classic map: Pressure 1 = GPIO
  36, Pedal Acc = 35, Steering PWM = 18, DAC on 25/26. The repo's primary target is the S3, where
  **GPIO 18 is Q3's shutdown-circuit gate** — so following this table can drive a safety output.
  This is also the only place in the repo where "35" and a pressure table appear together, which is
  the likely origin of the wrong-pad measurement. Either delete these tables or head them
  "classic ESP32 — previous board, do not use for the S3".
- [ ] **`README.md` still calls the steering sensor an AS5600** (lines ~7 and ~22, the latter saying
  CN5.2 carries "the AS5600's PWM angle output"). It is an **MT6701** read over PWM; `AGENTS.md:20`
  already records the AS5600 as retired on 2026-07-12.
- [ ] **`.agents/esp32s3-pinmap.md` header and gap 1 are both false.** The header says "This is NOT
  the pin map the firmware currently uses. `km_gpio.h` still holds the classic-ESP32 map"; gap 1
  says "The S3 build does not exist. `platformio.ini` has only `esp32dev` and `native`" and that the
  SPI pins are "defined nowhere". In fact `[env:esp32-s3-devkitc-1]` exists, links, and is what
  flashes; the four SPI pins are defined at `km_gpio.h:83-86`. A previously-filed task about this
  file's staleness could not be found in `tasks.md`, so it appears untracked — this entry replaces it.
- [ ] **`.agents/esp32s3-pinmap.md:12` still lists `PRESSURE_3` on GPIO 1 as "analog in (ADC1)".**
  GPIO 1 is now MCPWM capture for the MT6701's PWM output and is deliberately excluded from ADC
  setup. The same file's line 66 already says so; line 12 was never updated.
- [ ] **Contradictory rework instruction for the ex-`PRESSURE_3` terminal (CN5.2).** kart-docs says
  "remove R10 only (keep R8 + R9)"; this repo's `.agents/esp32s3-pinmap.md:66` says "keep R8 series,
  remove R9+R10". **Do not solder CN5.2 until this is settled** — one of the two is wrong.
- [ ] **dv-hardware's schematic and its fabricated PCB disagree on connector designators.** The
  schematic and `output/netlist.net` put the pressure channels on `CN2.1`/`CN2.2`/`CN2.3`; the v1
  silkscreen and PCB put them on `CN7.1`/`CN7.2`/`CN5.2`, where PCB `CN2` is HALL3/HALL2/+5V_REG.
  Anyone wiring from a schematic printout lands on the wrong header. Trust the silkscreen. That
  netlist export is dated 2026-05-07 and is already flagged stale in dv-hardware, but kart-docs
  carries no warning about it.
- [ ] **`PRESSURE_3` status conflict between repos.** kart-docs says the channel is retired and
  repurposed to steering PWM; dv-hardware's `docs/pinout-esp32-s3.md:176` — which kart-docs names as
  the tie-break authority — still lists it as "Pressure sensor 3 (input only)". The stated tie-break
  rule therefore points at the stale file.
- [ ] **Pressure channel count is wrong in kart-docs.** It says "3x Festo" and "the three
  pneumatic-pressure sensors", but the BOM has qty 2, the wire list defines only `press1`/`press2`,
  and only two ADC channels remain.
- [ ] **Stale gap comments in `km_gpio.h`** (lines ~87-88, 101, 105): `WriteDAC` unported and
  "nothing drives SDC / SELECT_THROTTLE". `control_task()` has driven the SDC pin since 2026-07-26.
  The `platformio.ini` comment saying the S3 env "does NOT link yet" is stale for the same reason.
- [ ] **Two latent ADC bugs in `km_gpio.c`, neither affecting pressure.** (a) On the S3,
  `PIN_HYDRAULIC_2` = GPIO 2 is an ADC1 pin but is handled inside the block commented "ADC2 pins",
  so its attenuation is never configured while `KM_GPIO_ReadADC` does have a case for it — a channel
  read without being configured. (b) The `GPIO_NUM_1` attenuation branch can never execute, since
  `PIN_PRESSURE_3` is `#if`'d out of `adc1_pins[]`; harmless but it is the same "configured is not
  used" trap already logged at `.agents/error-log.md:181`.
- [ ] **`km_gpio.h` never records the connector for `PRESSURE_1`/`PRESSURE_2`.** Only `PRESSURE_3`
  carries a `// CN5.2` comment. The GPIO 6 <-> CN7.1 link exists only in `tasks.md:74` and
  `history.md:526`, so the firmware alone cannot tell you which screw terminal a channel is. Add the
  comments.
- [ ] **Module variant conflict.** dv-hardware records the fitted module as
  **ESP32-S3-WROOM-1-N16R8** ("verified on hardware 2026-07-10"); kart-docs says **N8R2**. R8 means
  octal PSRAM, which consumes GPIO 33-37 — so which one is actually fitted determines whether those
  pins exist at all. Settle it by reading the can, and correct whichever doc is wrong.

- [ ] **`KM_PID_GetTunings` is a second setter, not a getter** — Found 2026-07-30 while adding live PID
  tuning. `components/km_pid/km_pid.c:114` declares `void KM_PID_GetTunings(PID_Controller *controller,
  float kp, float ki, float kd)` and its body is byte-for-byte identical to `KM_PID_SetTunings` above it:
  it overwrites the gains. Taking the floats by value means it cannot return anything even in principle,
  so no caller can read gains out of a controller. Nothing calls it today, which is why it has survived;
  `main.c` works around it by mirroring the live gains into its own `g_pid_*` file-scope variables to
  build the `ESP_STEER_PID` echo. Fix: change the signature to `float *kp, float *ki, float *kd`, write
  through the pointers, and delete the mirrored globals in `main.c` in the same commit. Its header
  comment already flags it as "identical to SetTunings", so the duplication was known — what was never
  decided is whether to fix it or delete it.

- [ ] **`AGENTS.md`'s UART protocol tables describe an encoding the firmware stopped using** — Noticed
  2026-07-30. The two "Message types" tables at `AGENTS.md:153-165` give payloads as `u8 [0-255]`,
  `int16 big-endian, radians × 1000` and `ORIN_COMPLETE | 7 bytes`. The wire format is int32 arrays and
  has been since the protobuf migration — `ORIN_COMPLETE` is 6 int32 elements in `km_coms.c`, not 7
  bytes. The tables are also missing every frame added since: `ORIN_CALIBRATE_STEERING` (0x28),
  `ORIN_STEER_MODE` (0x29), `ORIN_COMPRESSOR_DISABLE` (0x2A), `ORIN_STEER_PID` (0x2B),
  `ESP_HEALTH_STATUS` (0x0B), `ESP_PNEUMATIC` (0x0C), `ESP_STEER_PID` (0x0D). The accurate list is the
  `message_type_t` enum in `components/km_coms/km_coms.h`, whose doc comments carry the payload shapes.
  Decide whether to regenerate the tables from that enum or delete them and point at the header —
  a table that is wrong about the encoding is worse than no table, because it reads as authoritative.
