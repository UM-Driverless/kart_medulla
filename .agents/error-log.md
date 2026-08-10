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

## 2026-07-19 — claimed I²C works over the 1.2 m steering run, when it had been tried, failed, and written up in this repo

**What happened.** Rubén asked whether any steering-sensor digital interface works past 1.2 m. I
first answered from a rule of thumb — "I²C is really on-board only, bad past ~1 m" — which happened
to be *correct*. He then said 1.2 m is the real shaft-to-rear-PCB run. I computed the I²C bus
capacitance (~120 pF against the 400 pF spec limit), concluded 1.2 m was comfortable, and **reversed
my own answer**, telling him all four interfaces work. I reinforced it with a fabricated piece of
evidence: *"you already run I²C to the AS5600 over that exact run."*

That was false twice over. The PCB currently sits **at the front, next to the sensor** — the run
that works today is short, not 1.2 m. And the long run was already tested and failed: `history.md`
2026-07-15 records I²C over a ~1 m branch producing address-ACK-but-register-NACK on *every* device
including the on-board PCF8574, phantom addresses 0x08/0x09, and a wedged driver — with the bus
going clean the moment the branch was unplugged. The decision that followed is recorded in the same
file: **plain PWM, run kept short and shielded.** The question I was asked had already been answered
in the repo.

**Root cause — three failures stacked:**

1. **A model of one failure mode was treated as proof the system works.** Bus capacitance is *one*
   way I²C dies. The documented failure was noise, stub reflections and marginal pull-ups on an
   unterminated branch — none of which appear in a capacitance sum. Clearing one failure mode says
   nothing about the others, and I presented the calculation as though it settled the question.
2. **I manufactured supporting evidence rather than checking.** `AGENTS.md` says the AS5600 "lives
   off-board on the steering shaft", and I turned that into "so the current run is the 1.2 m one"
   without ever confirming where the board physically sits. An inference became a stated fact in
   the same sentence.
3. **I did not grep `history.md` / `error-log.md` before making a hardware claim** — the standing
   rule in this repo, and one I had already partly read this same session. The answer was sitting
   in the file with a binary-search test behind it.

**The deeper process failure, which is the point.** Within two messages I asserted the opposite of
what I had just asserted, and both times with confidence. When my own answers contradict each other
that is a signal that a fact is missing — the correct move is to **stop and ask** ("where is the
board mounted today? has the long run been tried?"), not to assume the newer answer wins because it
has arithmetic attached. Rubén's words: *"you're confused but refuse to ask clarifying questions and
just affirm contradictory info. that's not how one thinks."*

**What was NOT the mistake — read this before drawing the wrong lesson.** Reasoning from first
principles was the *right* approach, and this entry must not be read as "look it up instead of
thinking." Rubén's correction, 2026-07-19. The capacitance calculation was sound arithmetic and it
is genuinely useful to know 1.2 m sits at ~30% of the I²C budget. The failure was reasoning
**too narrowly** and then stopping:

- I²C over a cable fails through *at least* five mechanisms — bus capacitance, EMI on the threshold
  crossing, stub reflections off an unterminated branch, undersized pull-ups, and ground offset
  eating noise margin. I modelled **one**, and it was the one least likely to bite in a vehicle.
- A thorough first-principles pass enumerates the failure modes *first*, then says which the model
  covers. That would have flagged the model as partial before it got presented as a verdict.
- Where a model and a measurement disagree, the measurement wins — not because reading beats
  thinking, but because a bench test samples every failure mode at once, including the ones absent
  from the model. Their disagreement is information: it says the model is missing a term.

So: reason from first principles, more thoroughly, *and* go looking for data that could falsify the
result. The two are complements, not alternatives.

**Prevention.**
- Before stating how this hardware behaves, **grep `history.md` and `.agents/error-log.md`** — not
  to replace the reasoning, but to find measurements that can falsify it. This project documents
  its failures; assume the question has been hit before.
- **Never cite "you already do X" as evidence** without confirming the present physical setup.
  Past-tense repo prose describes when it was written, not now.
- **Self-contradiction is a stop-and-ask trigger.** Do not silently retract and replace; say "I said
  the opposite a moment ago — which is true?" and find out.
- A calculation that clears one failure mode is **not** evidence of function. Say which mode it
  covers and which it does not.

## 2026-07-24 — Diagnosed a "fault" on a pin that was never wired (MT6701 bench)

**What happened.** First MT6701 bench session. I told Rubén to wire the sensor and gave the
wiring list with OUT→GPIO 1 marked "optionally"; the actionable instruction he acted on was
"wire I²C and plug in USB" — so he wired exactly that. When the ADC watch on GPIO 1 then read
flat 0 V while the I²C config write succeeded, I treated the dead pin as a hardware mystery:
resurrected the 2026-07-12 `CN5.2 → R8 → R9` open-joint diagnosis, laid out a two-branch fault
tree, asked for multimeter probing, and wrote a "GPIO 1 still dead" entry into `history.md`.
GPIO 1 read 0 V because **nothing was connected to it** — the only correct reading. Rubén
caught it from the wiring instructions; without that, hours of troubleshooting a nonexistent
fault, with a multimeter, on a healthy board.

**Root cause.** I diagnosed signal absence without first confirming the signal path physically
existed. My own wiring instruction was the record of what was connected — I never re-read it.
Two compounding failures: (1) an "optional" item in my instructions silently became an assumed
fact ("OUT is wired") one turn later; (2) the flat-0-V reading *matched* a known past fault
(the CN5.2 open), and pattern-matching to that story felt like progress, so the mundane
explanation — unplugged wire — was never on the fault tree at all.

**Prevention.**
- **Before diagnosing a dead signal, establish the wire exists.** First question on any
  no-signal reading: "what exactly is physically connected right now?" — asked out loud or
  answered from the wiring instructions actually given. A fault tree whose branches are all
  electrical is wrong if the layer-0 branch (not connected) was never closed.
- **"Optionally X" in my own instructions means X probably didn't happen.** Anything I marked
  optional must be confirmed done before any later reasoning depends on it.
- **A reading that perfectly matches a floating/unconnected pin is evidence of an unconnected
  pin** — the boring hypothesis outranks the interesting one that matches a past war story.

## 2026-07-26 — Twice asserted a checkable fact from a plausible inference (steering PWM work)

**What happened.** Two wrong claims in one session, both stated confidently, both cheap to check.

1. **"GPIO 1's ADC is sampling the PWM square wave and returning noise."** `KM_GPIO_Init()` does
   configure GPIO 1 as `ADC1_CHANNEL_0`, and I went from "the channel is set up" to "so it is being
   read" without grepping for the call site. One `grep -rn ReadADC` showed `main.c` reads only
   PRESSURE_1 and PRESSURE_2 — the pin is never sampled at all. Told to Rubén as fact before the
   grep, and written into `tasks.md`.
2. **"The ESP32-C3 and C6 have no MCPWM peripheral."** Written into a source comment and into
   `history.md` to justify a `SOC_MCPWM_SUPPORTED` guard. `soc_caps.h` says the C6 *does* have
   MCPWM; only the C3 lacks it. Never opened the header — the guard itself was correct, so the
   wrong reason rode along invisibly.

**Root cause.** Same shape both times: a fact that a single grep or file-open would settle, answered
instead from something adjacent that made it feel already-known. In (1) the adjacent thing was
initialisation code; in (2) it was a general sense of which chips are cut-down. Neither claim felt
like a guess while I was making it, which is exactly why the feeling cannot be the gate.

**Prevention.**
- **"Configured" is not "used."** Peripheral setup proves a channel exists, not that anything reads
  it. Grep for the call site before saying a value is being read, sampled, or acted on.
- **Per-target capability claims come from `soc_caps.h`, never from memory.** `grep SOC_<FEATURE>
  $IDF/components/soc/<target>/include/soc/soc_caps.h` is one command and it is authoritative.
- **A comment justifying correct code still has to be true.** Wrong reasoning attached to a working
  guard is invisible to the build and to tests — the compiler cannot fail on it, so it survives
  until someone acts on it. Check claims in comments at the same bar as claims in prose.

---

## 2026-07-27 — Built two days of hardware conclusions on one number from an old commit message

**What happened.** A comment in `main.c` said: *"Calibration (2026-07-18): the tank sat at a
gauge-read 7.5 bar while this ADC channel read 2679."* I treated that as a measurement — a real,
current, comparable reading of the same quantity the sensor measures — and reasoned outward from it
for two days:

1. Noticed the dashboard's ADC→bar map disagreed with it by ~16%, and "fixed" the dashboard to match
   it, rewriting the calibration in kart-brain and committing it.
2. When the ESP32-S3 datasheet's 2900 mV full scale made the numbers still not fit, concluded the
   board's divider must be ~3.95:1 instead of 3:1 — **inventing a hardware fact** to preserve the
   number. Wrote it into two `history.md` files, `protocol.py`, `tasks.md` and the shared datasheet
   index, with a confident derivation.
3. Rubén said the design is three equal resistors. The schematic confirms R11 = R12 = R13 = 10K.
   Corrected that, but *still* kept the 7.5 as valid, and pivoted to asserting "the mechanical gauge
   is wrong" — an instrument I had never confirmed exists.
4. Proposed multimeter procedures to adjudicate between the sensor and that instrument.

**There is no mechanical dial.** Rubén: it does not exist. And the number is unusable for far more
basic reasons than accuracy — the code was different then, the wiring was different, there may be a
regulator between the measurement point and the sensor, the two readings may have been taken at
different points in the circuit, and it may simply have been a value he reported verbally from
earlier while the pressure had since dropped by the time the ADC was read. **None of that is
recoverable from the note.** The note records a number, not a measurement.

**Root cause.** I treated a number written in a comment as data. A measurement is only comparable if
you know what was measured, where, when, and under what configuration; this had none of that, and the
note itself was a year of hardware changes out of date. Worse, every time reality contradicted it I
adjusted *reality* — first the divider ratio, then the credibility of a phantom instrument — rather
than questioning the number. That is the failure mode: an unsourced value became the fixed point that
verified facts were bent around.

The compounding factor: it was written in the imperative, confident register these files use, so it
read as established. My own additions then cited it, which made it look corroborated.

**Prevention.**
- **A number in a comment is a claim, not a measurement.** Before building on one, ask what was
  measured, with what, where in the circuit, and on which revision of the hardware and firmware. If
  the note does not say, it is an anecdote — usable as a hint, never as an anchor.
- **When a documented chain (datasheet + schematic + on-chip calibration) disagrees with one
  undocumented number, the number is the suspect.** I had it backwards twice.
- **Never infer a hardware value from a calibration mismatch.** The schematic is in
  `dv-hardware/projects/kart-medulla/` on this machine. Read it. Two wrong claims here — the 3.95:1
  divider and the "faulty gauge" — were both one grep from being avoided.
- **Old notes expire.** Anything predating a wiring or firmware change describes a system that no
  longer exists. Check the date of the note against the dates of the changes before quoting it.
- **Stop and ask instead of escalating.** By step 3 the sensible move was "where does this 7.5 come
  from, and is that instrument real?" — one question to the person who was there. Instead I invented
  a measurement procedure for a device that does not exist.

## 2026-08-08 — Flashed with steering power live; the gear broke in the bootloader window (Claude Opus 5)

**What happened.** Following the AGENTS.md flashing recipe (stop kart-brain → `pio run --target
upload` → start kart-brain), the agent flashed the ESP32-S3 while the kart's actuator power was on.
esptool hard-resets the chip into the download bootloader; during that window GPIO 40
(`CMD_STEER_PWM`) and GPIO 17 (`CMD_STEER_DIR`) float. The Cytron H-bridge drove the steering motor
uncontrolled and the gear broke. Rubén had to cut power, which also killed the Orin mid-flash.

**Root cause, two layers.**
1. *Hardware*: the steering pins have no pulldowns for the reset/bootloader window. GPIO 3
   (compressor) and GPIO 18 (SDC, R23) were given pulldowns for exactly this reason; the steering
   PWM/DIR were not.
2. *Process*: the flashing recipe said nothing about actuator power, and the agent executed it
   without asking what state the kart was in. Also, `kart-brain`'s systemd service autostarts the
   full autonomous stack — cone_follower was live-commanding full-lock steering on the bench before
   the flash was even attempted.

**Prevention.**
- **Never flash with the steering (actuator) rail powered.** Before any flash: confirm with Rubén
  that actuator power is off or the kart is in manual mode (firmware forces `KM_ACT_Stop()` on
  steering in MANUAL). A software check is not enough — the protection has to hold while the
  firmware is *not running*.
- Fix belongs in hardware too: pulldowns on the Cytron PWM/DIR inputs (tracked in `tasks.md`).
- Asking "is the kart safe to reset?" is necessary but not sufficient — the question has to name
  the actual hazard (actuator power during the reset window), not just "wheels clear".

## 2026-08-08 — `git checkout origin/dev -- .` run in the live repo instead of a worktree (Claude Opus 5)

**What happened.** While checking whether two native test suites failed before or after the branch
consolidation, I meant to compare against pre-merge `dev` in a scratch worktree. The command that
created the worktree was correct, but it was preceded by `git stash -u` and
`git checkout origin/dev -- .` aimed at the live checkout. The working tree was clean at that
point, so the stash captured nothing and the checkout silently overwrote every merged file with
`origin/dev` content — including the AGENTS.md runaway-throttle warning and the tasks.md blocker
written minutes earlier.

**Why it was recoverable.** The merges were already committed, so only the working tree was lost;
`git reset --hard HEAD` restored everything, verified by grepping for both warnings. Nothing was
pushed in the clobbered state.

**Root cause.** Two ways of getting a pre-merge tree — a worktree and a path-checkout — were
combined in one command line, and the path-checkout half acts on the current checkout with no
confirmation. `git checkout <ref> -- .` overwrites unconditionally and is silent on success.

**Prevention.** To inspect another revision, use `git worktree add` or `git show <ref>:<path>`
only. Never `git checkout <ref> -- .` in a live checkout — its whole effect is destructive and
indistinguishable from success. If it must be used, commit first and confirm `git status` is clean
*for the intended reason*, not incidentally.

**Result of the check itself:** pre-merge `origin/dev` fails `test_km_act` and `test_km_coms`
identically (4 of 40 cases, `TickType_t` undefined in the native build). The failures are
pre-existing and not caused by the merge.

## 2026-08-08 — Flashed a bench-only diagnostic build to the kart (Claude Fable 5)

**What happened:** After merging `feature/pedal-telemetry`, the ESP32-S3 was flashed with `dev` tip. Heartbeat died on the dashboard and pedal values lagged ~3 s. Raw serial showed ESP_LOG ASCII interleaved with binary frames → CRC mismatches → KB_Coms_micro stuck in a 5 s watchdog reconnect loop.

**Root cause:** `platformio.ini`'s S3 env carried `build_flags = -D SPI_DIAG_LOGS=1`, commented "BRANCH `spi-fix` ONLY — do not merge this flag to dev", but it had been merged to dev anyway. The flash step didn't check for bench-only flags before flashing the kart.

**Prevention:**
- Before flashing the kart, `grep -n DIAG platformio.ini main/main.c` — any bench/diagnostic flag means stop and strip it first.
- A "do not merge to dev" comment is not protection; if a flag must never fly, gate it out of the S3 env by default and require an explicit `-e` bench env to enable it.

## 2026-08-08 — Electrical analysis done on the wrong schematic revision (Claude Fable 5)

**What happened:** Asked where to probe the throttle chain, the agent netlisted the dv-hardware
*working tree* and confidently reported an op-amp gain stage (×1.51) in the throttle path, an
"overrange bug" (5 V DAC × 1.51 ≈ 7.6 V), and probe expectations of ~3.8 V. All wrong for the
physical board: the working tree carries post-fab v2 design changes; the manufactured board is
`84d6dd0` (tag `kart-medulla-v1`), where the DAC feeds the mux directly and 50% = 2.5 V. A
tasks.md entry and probe voltages had to be corrected mid-session.

**Root cause:** Nothing in this repo stated which dv-hardware commit the physical board is, and
the agent didn't ask which revision it was reading before making electrical claims.

**Prevention:** AGENTS.md now pins the physical board (`84d6dd0` / `kart-medulla-v1` + README
rework list) with the netlist-from-tag command, and requires every electrical claim to name the
dv-hardware commit it was read from.

## 2026-08-08 — Two confident wrong diagnoses before the real one during the DAC bench test (Claude Fable 5)

**What happened:** CN10.1 read 0 V with blips. Diagnosis 1: MCP4922 V_IH margin (3.5 V needed,
3.3 V driven) — plausible, pre-recorded in history, stated as near-certain. Then SELECT_THROTTLE
also read 0 V, which V_IH cannot explain; diagnosis 2: bad socket contact — the board was seated
fine. The firmware-side GPIO readback then proved the pin driving high; the 0 V turned out to be
a flashing problem (mechanism not pinned down — after later flashes by other agents the pin
measured 3.3 V, and the DAC works at 5 V supply with 3.3 V logic).

**Root cause:** Each hypothesis fit the evidence available at the time, but was reported as a
conclusion rather than a candidate; the cheap decisive test (make the firmware report its own
pin state, then make the pin identify itself) came third instead of first.

**Prevention:** On hardware "signal is dead" problems, put self-reporting instrumentation in the
firmware (init result, set-call result, pad readback) before theorizing about analog margins or
mechanical contact. The GPIO INPUT_OUTPUT readback is now permanent for SELECT_THROTTLE.

## 2026-08-08 — Asked the user to disambiguate "the dashboard is off" instead of looking (Claude Opus 5)

**What happened:** Rubén reported "dashboard is off". Instead of investigating, the agent replied
with a two-option multiple-choice question (not running vs. showing a wrong value). Rubén's answer
was "wtf". One `ssh orin-remote 'systemctl is-active kart-brain'` then produced the whole answer in
seconds: the service was `inactive`, so nothing was reading `/dev/ttyACM0` and the dashboard had no
source. It had been stopped for a flash (the documented procedure in AGENTS.md stops it to free the
port) and never restarted. `systemctl start kart-brain` fixed it.

**Root cause:** The ambiguity was real but irrelevant — both readings of "off" start with the same
cheap check, and that check distinguishes them. The question cost a round-trip and pushed diagnosis
back onto the person who reported the symptom.

**Prevention:** When a symptom is reported on the kart, run the state checks first and ask only if
they come back ambiguous. The standing first three, all read-only and one SSH call:
`systemctl is-active kart-brain`, `ls -l /dev/ttyACM*`, and the recent `journalctl -u kart-brain`.
A stopped `kart-brain` is the single most likely cause of a dead dashboard in this repo, because
every flash stops it and restarting is a manual step.

## 2026-08-10 — Read `tasks.md` aloud as current fact without checking the code (Claude Opus 5)

**What happened:** Asked "what tasks.md left?", the agent summarised the file's open items and
presented one of them as a live bug: "`KM_GPIO_WriteDAC()` can't distinguish throttle from brake on
S3 (both pins `GPIO_NUM_NC`)". Rubén pushed back. The claim was stale by ten days —
`km_gpio.h:106-107` has given the two channels the distinct stand-in values 200 and 201 since
2026-07-31, and `km_gpio.c:570-596` dispatches each to its own `mcp4922_write()` channel. The same
summary repeated two more stale entries: "`KM_GPIO_WriteDAC()` is not ported to the MCP4922" and the
open question of whether the MCP4922 was faulty, which Rubén had already answered on the bench.

**Root cause:** A task board records what was true when each entry was written. Summarising one is
therefore reporting *claims*, and the summary presented them as *findings* with no marker of which
had been verified. The failure is the same shape as stating an unverified fact directly: the reader
has no way to tell a checked line from an unchecked one, so every line inherits the agent's
credibility.

**A second, smaller error rode along with it.** Rubén's reply mentioned the dashboard showing both
pedals correctly, which is unrelated — pedals are ADC *inputs* on GPIO 4 and 5, while the task was
about the DAC *output* path. Answering the literal objection first, before the correction, would
have separated the two; instead they had to be untangled afterwards.

**Prevention:** When summarising `tasks.md`, `history.md` or any dated log, either spot-check the
claims that name a specific file, symbol or constant, or mark the summary as unverified up front
("as recorded, not rechecked"). Cheap in this repo: every stale entry found this session was one
`grep` away. Prefer checking the items the user is most likely to act on, and the ones whose entry
predates recent work in the same area.

## 2026-08-10 — Repeated the stale-`tasks.md` mistake within the same day (Claude Opus 5)

**What happened:** Asked "what tasks.md left?", the agent summarised the board and led with a
section headed "Safety blocker", stating that `dev` commands 50% throttle unconditionally and that
the kill switches were the only thing that would stop the kart. Rubén's reply was "though i doubt
it's true. last time we got it to accelerate on command in remote control. it was not hardcoded" —
and he was right. Commit `7bcd6eb` had removed the bench hardcode on 2026-08-08, two days earlier,
and it is in `dev`'s history. `main/main.c:821` takes throttle from `TARGET_THROTTLE`, and
`KM_GPIO_SetThrottleSource(true)` at line 790 sits *below* the comms-stale / `MISSION_MANUAL` early
return at line 771 — the opposite of what the entry claimed. `-D SPI_DIAG_LOGS=1` was likewise gone
from `platformio.ini`. Nobody had updated `tasks.md` when the fix landed.

**Root cause:** the same one already written up in the entry directly above this one, from earlier
the same day, which ends with the prevention "either spot-check the claims that name a specific
file, symbol or constant, or mark the summary as unverified up front". That prevention was not
followed, and the reason is structural rather than a lapse of attention: `.agents/error-log.md` is
consult-selectively, so it is grepped when working *in an area*, and "summarise the task board" does
not feel like an area. The guidance existed in a file that the task in question gives no reason to
open.

**Two aggravating details.** The claim was the most alarming one on the board, which is exactly the
kind a reader acts on immediately. And it was cheap to check — one `grep` of `main.c` settled it,
the same one-grep distance the previous entry noted.

**Prevention:** the fix belongs in `AGENTS.md`, not here, because that file is loaded for every task
while this one is not. Proposed line is under "Files", beside the `tasks.md` bullet.

## 2026-08-10 — Reported a protocol bug that was a stale flash (Claude Opus 5)

**What happened:** asked where the steering reject counter lives, the agent noticed
`/esp32/health/data` was carrying five fields where the firmware source builds seven, and reported
it as a live bug: the steering trip age "never reaches the dashboard", defeating the fix deployed the
same morning. It filed a task saying the loss was somewhere between `KM_COMS_SendMsg` and the C++
handler, listing the C++ dispatch, `SerialDriver::MAX_PAYLOAD` and stale builds as ruled out.

**Actual cause:** the chip was flashed before the feature existed. A raw dump off `/dev/ttyACM0`
showed `LEN=24` — six int32, genuinely sent that way — so nothing downstream was dropping anything.
`92a3248`, which adds `steer_trip_age_s`, is timestamped 10:58; the flash ran at about 10:45 from
`be82616`, whose `main.c` has `int32_t payload[6]` and sends 6. The firmware was behaving exactly as
written.

**Root cause:** the agent compared a just-pulled working tree against a chip flashed thirteen minutes
earlier, and read the difference as a protocol fault. It had itself run the flash, so "what is on the
chip" felt settled — but Rubén committed the firmware side of the feature after that flash, and
nothing prompted a re-check. The same shape as the stale-`tasks.md` error above: treating a stale
snapshot as current, except here the snapshot was the flashed binary rather than a document.

**Prevention:** when observed firmware behaviour disagrees with the source tree, check *which commit
is on the chip* before theorising about the protocol. `git log -1 --format=%ci <commit>` against the
flash time is one command and would have ended it immediately. Any long-running session should assume
the human is committing concurrently — this repo's tip moved twice today while work was in progress.
The raw-bytes measurement was the right instinct and settled it in one shot; it should have come
before the task was filed, not after.
