<!-- read in full — kept under 150 lines -->

# Tasks

The repo's task board — the only one. Update status as you go: `TODO → In Progress → Done`. Read
before editing; claim by adding `[YYYY-MM-DD <name>]` and the section change. Big clusters get a
`tasks/<name>.md`, always linked from an index here.

Only Rubén moves a task to Done. "Looks finished" is not the same as confirmed on the hardware — on
this repo that means flashed *and* driven, per the branch workflow in `AGENTS.md`.

## TODO

### No CI builds this firmware, and the Mac cannot either

Noticed 2026-07-26 while pushing the compressor-latch / SDC change. **Nothing compiles this repo
automatically.** `.github/workflows/` does not exist on `dev`, and `gh run list` shows a single run
ever — "Build with ESP-IDF v5.4", 2025-11-02, on `main` — so whatever workflow produced it is gone.
Combined with the fact that **no PlatformIO or ESP-IDF is installed on the Mac** (see `AGENTS.md`),
the practical situation is that firmware can be written and pushed with nobody having compiled it;
the first compile happens on the Orin, at the kart, when someone tries to flash.

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
run that stopped at a gauge-read 7.5 bar was using an OFF threshold of ADC > 1638, yet the settled
reading afterwards was **2679**. With duty at 0 the tank cannot gain pressure, so the sensor reading
rose ~1041 counts (~64%) between "running" and "settled" at equal or falling true pressure. That is
the running bias, not a calibration error in the sensor.

**Safety consequence — check before the next unattended run.** The 7/8 bar thresholds now in
`main.c` were derived from a **settled** reading (2679 ↔ 7.5 bar) but the control loop evaluates them
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

## In Progress

## Done

- [2026-07-16] **Compressor soft-start**: `CMD_COMPRESSOR_PWM` (GPIO 3) moved from a digital on/off
  output to LEDC PWM on its own timer at 500 Hz, ramping 0 → 60% duty over 1 s from the pressure
  hysteresis rising edge. Duty added to the `ESP_ACT_STEERING` telemetry payload. Builds for both
  S3 and the classic fallback. **Not validated on hardware** — see the bench test under TODO.
