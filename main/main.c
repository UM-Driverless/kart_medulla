/******************************************************************************
 * @file    main.c
 * @brief   ESP32 application entry point and FreeRTOS task definitions for
 *          the kart-medulla firmware.
 *
 * @details Brings up all hardware peripherals, initializes controllers and
 *          sensors, and registers periodic FreeRTOS tasks for communications,
 *          control, heartbeat, and health monitoring.
 *****************************************************************************/

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/dac.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// Librerias propias
#include "km_act.h"
#include "km_coms.h"
#include "km_gamc.h"
#include "km_pid.h"
#include "km_rtos.h"
#include "km_sdir.h"
#include "km_sdir_pwm.h"
#include "km_sta.h"
#include "km_gpio.h"
#include "km_objects.h"

static const char *TAG = "MAIN";

// KM_GPIO_Init()'s result. system_init() only ESP_LOGE's a failure and carries on, and
// ESP_LOG is disabled on UART0 to keep the binary protocol clean - so a failed GPIO/LEDC
// setup is otherwise completely invisible, and the compressor pin would simply never be
// driven with no indication anywhere. Shipped in the pneumatic frame so it is observable.
static int32_t g_gpio_init_err = -1;

/* Set once the steering sensor is found invalid while closed-loop steering is
 * active, and never cleared while the firmware runs. Read by health_task, so it
 * is visible on the dashboard rather than only in the kart's behaviour — a
 * latched EBS that nothing reports looks identical to a brake fault. */
static volatile bool steer_fault_latched = false;

/* Set when a full-length compressor burst starting below PRESSURE_STALL_JUDGE_BELOW_BAR
 * fails to raise the tank pressure — a dead or stuck sensor, a compressor that is not
 * actually spinning, or a leak that outpaces it.
 *
 * REPORTED ONLY. It does NOT stop the pump and does NOT open the shutdown circuit,
 * deliberately, because this check has never run on hardware and its threshold is
 * derived from one bench figure rather than measured. A safety interlock that has
 * never been observed working must not be able to fire the EBS on its own: the first
 * version of this did latch, and analysis afterwards showed it would false-trip near
 * the top of the pressure range, where a healthy compressor legitimately adds almost
 * nothing per burst.
 *
 * Promote it to a real interlock — latch, stop pumping, hold the SDC open — once it
 * has been watched on the kart and seen to stay clear through normal fills and to
 * trip when the sensor is unplugged. Until then it shows on the dashboard as
 * comp_state 4 and nothing more. See tasks.md. */
static volatile bool pump_stall_observed = false;

/* Whether the tank is holding enough air for the EBS to be relied on. Computed in the
 * pneumatics section of control_task and read by the shutdown-circuit decision at the
 * TOP of the same function, so it is one cycle stale — 2 ms at the 500 Hz task rate,
 * against a tank that moves over seconds. Same arrangement as steer_fault_latched.
 * Starts false, so the kart boots in emergency and stays there until the tank has
 * actually been measured above the arm threshold. */
static volatile bool tank_pressure_ok = false;

#define MAX_ERROR_COUNT_SDIR 10
#define COMMS_WATCHDOG_MS    1000  // Zero outputs if no command for this long
#define MISSION_MANUAL       0     // Mission ID 0 = manual (no electronic actuation)

/* Autonomous-system states as the Orin numbers them, arriving here in
 * MACHINE_STATE_ORIN via ORIN_MACHINE_STATE. Source of truth for these values is
 * kart-brain's src/kart_control/scripts/state_machine_node.py — they are written
 * out as names here rather than as bare integers so a renumbering on that side is
 * findable from this one. Only the two "meant to be moving" states appear below;
 * AS_OFF, AS_FINISHED and AS_EMERGENCY deliberately have no constant, because the
 * shutdown chain stays open in all of them and the check is written as a
 * whitelist. */
#define AS_READY             1
#define AS_DRIVING           2

/* Sent in the steering frame's angle field when no angle is known. INT32_MIN is
 * roughly -2.1e6 radians once the consumer divides by 1000, so it cannot be
 * mistaken for a steering angle by anything that plots or acts on the number —
 * which is the point. Sending 0, or the last good reading, would look exactly
 * like a plausible measurement, and that is how an unplugged sensor drew a
 * confident 90-degrees-left on the dashboard on 2026-07-25 (see AGENTS.md,
 * "Sensor Validity"). Field 4 of the same frame carries the validity flag for
 * consumers that check it properly. */
#define STEER_ANGLE_INVALID  INT32_MIN

/* ---------- Steering PID: compiled defaults and remote-tuning limits ----------
 * These are the gains the firmware boots with and returns to whenever the
 * dashboard clears its override. Changing them still needs a reflash; the point
 * of ORIN_STEER_PID is that trying a gain does not.
 *
 * The clamps are not paperwork. A dashboard is a text box on a laptop that may
 * be on the far side of a Cloudflare tunnel, and the thing on the other end is a
 * motor geared to a steering column whose teeth have been stripped once already
 * (2026-03, an actuator limit that compared a 0-100 percentage against a 0.0-1.0
 * float and so never limited anything). Every value that arrives over the wire
 * is clamped here before it reaches the controller, and the accepted result is
 * echoed back so the dashboard shows what is running rather than what was typed.
 *
 * PID_REMOTE_MAX_LIMIT is deliberately below the 1.0 the actuator can take.
 * Raising the steering PWM ceiling past this is a decision that should come with
 * a flash and someone standing next to the kart, not a number typed remotely. */
/* 2026-08-08: reset to a conservative starting point (kp 1.20 -> 1.00, kd 0.10 -> 0.05) for the
 * re-tune that the derivative-on-measurement change requires. The previous values were tuned live
 * on the kart 2026-07-30 (kp 1.50 -> 1.20, kd 0.03 -> 0.10) — but with the OLD derivative, which
 * differentiated the error and so responded to setpoint motion; those numbers are not valid
 * starting points for the new derivative. Untested on the vehicle until the steering gear is
 * repaired. */
#define PID_DEFAULT_KP        1.00f
#define PID_DEFAULT_KI        0.0f
#define PID_DEFAULT_KD        0.05f
#define PID_DEFAULT_PWM_LIMIT 0.50f

#define PID_REMOTE_MAX_KP    20.0f
#define PID_REMOTE_MAX_KI    10.0f
#define PID_REMOTE_MAX_KD     5.0f
#define PID_REMOTE_MAX_LIMIT  0.60f  /* hard ceiling on remotely-set steering PWM */

/* Gains currently in force, kept here so health_task can echo them without
 * reaching into the PID struct. Written only by control_task. */
static volatile float g_pid_kp        = PID_DEFAULT_KP;
static volatile float g_pid_ki        = PID_DEFAULT_KI;
static volatile float g_pid_kd        = PID_DEFAULT_KD;
static volatile float g_pid_pwm_limit = PID_DEFAULT_PWM_LIMIT;
static volatile bool  g_pid_override  = false;

/**
 * @brief   Clamp a remotely-supplied gain to a range the firmware will run.
 *
 * @param   v    Requested value, already converted out of its x1000 integer form.
 * @param   max  Upper bound for this particular gain.
 *
 * @return  The value clamped to [0, max].
 *
 * @note    The lower bound is 0, not -max. A negative gain on this loop is
 *          positive feedback — it would drive the column away from the target
 *          until something hit a stop. Zero is allowed because disabling a term
 *          is a normal thing to want while tuning (ki is 0 by default).
 */
static float pid_clamp_gain(float v, float max) {
    if (isnan(v) || v < 0.0f) return 0.0f;
    if (v > max) return max;
    return v;
}

/**
 * @brief   Apply the dashboard's PID override, or restore the compiled defaults.
 *
 * @details Called once per control cycle. Reads the request out of the object
 *          store, clamps it, and pushes it into the live controller only when it
 *          differs from what is already running — so the common case (nothing
 *          changed) costs five comparisons and touches nothing.
 *
 *          A change resets the integral accumulator. Without that, a term tuned
 *          up while the integral still holds the accumulation from the previous
 *          gain applies that stale value at the new scale, which shows up as a
 *          kick on the column the moment Apply is pressed.
 *
 * @param   pid  Live steering PID controller.
 * @param   act  Steering actuator, whose output limit is tuned alongside.
 */
static void pid_apply_override(PID_Controller *pid, ACT_Controller *act) {
    bool override = (KM_OBJ_GetObjectValue(PID_OVERRIDE) == 1);

    float kp, ki, kd, limit;
    if (override) {
        kp    = pid_clamp_gain((float)KM_OBJ_GetObjectValue(PID_KP) / 1000.0f, PID_REMOTE_MAX_KP);
        ki    = pid_clamp_gain((float)KM_OBJ_GetObjectValue(PID_KI) / 1000.0f, PID_REMOTE_MAX_KI);
        kd    = pid_clamp_gain((float)KM_OBJ_GetObjectValue(PID_KD) / 1000.0f, PID_REMOTE_MAX_KD);
        limit = pid_clamp_gain((float)KM_OBJ_GetObjectValue(PID_PWM_LIMIT) / 1000.0f,
                               PID_REMOTE_MAX_LIMIT);
    } else {
        kp    = PID_DEFAULT_KP;
        ki    = PID_DEFAULT_KI;
        kd    = PID_DEFAULT_KD;
        limit = PID_DEFAULT_PWM_LIMIT;
    }

    if (kp == g_pid_kp && ki == g_pid_ki && kd == g_pid_kd &&
        limit == g_pid_pwm_limit && override == g_pid_override) {
        return;
    }

    KM_PID_SetTunings(pid, kp, ki, kd);
    KM_ACT_SetLimit(act, limit);
    KM_PID_Reset(pid);

    g_pid_kp        = kp;
    g_pid_ki        = ki;
    g_pid_kd        = kd;
    g_pid_pwm_limit = limit;
    g_pid_override  = override;

    ESP_LOGW(TAG, "steering PID %s: kp=%.3f ki=%.3f kd=%.3f limit=%.2f",
             override ? "override" : "restored to firmware defaults", kp, ki, kd, limit);
}

/* ---------- EBS compressor drive ----------
 * The compressor is driven DC — full duty, no switching — and its average power
 * is set by a slow on/off cycle instead of by PWM: 15 s on, 15 s off. That is a
 * 30 s period, 0.033 Hz, 50% average.
 *
 * WHY NOT PWM. The gate is driven by a 3D-printer hotbed MOSFET module
 * (HA210N06). Its driver stage supplies the gate from its own rail, which is
 * what makes it usable here at all, but a module built to switch a heater bed
 * is not built to switch fast — it cannot hold a continuous PWM waveform at
 * this motor's operating point. So the duty knob is gone, and the only lever
 * left on average power is how long the motor is switched on for.
 *
 * WHY THAT IS STILL SAFE FOR THE MOTOR. The motor is rated 7.5 V and the rail
 * is a regulated 12 V, so it is deliberately over-volted while it runs. That
 * rating is a thermal/brush limit, not an insulation limit — the danger is
 * sustained heating, not the 12 V itself. Averaged over the 30 s cycle the
 * motor sees 6 V, under its rating, while each burst still gets full torque.
 * The 15 s cap IS the protection: it bounds how much heat one burst can
 * deposit, and the 15 s off-time is when that heat leaves.
 *
 * WHY THE SOFT START STAYS. The 1 s ramp is about inrush, not average power, so
 * the slow cycle does not replace it. A stationary motor has no back-EMF, so
 * only the winding resistance limits current (~40-50 A). That spike browns out
 * the 12 V regulator and drops the USB ground loop. Ramping gives the rotor
 * time to spin up and generate back-EMF, which chokes the current off on its
 * own. Every burst re-ramps, because every burst starts from rest.
 *
 * SUPERSEDED — do not resurrect this from the git history. This block used to
 * argue for a permanent 20% duty (COMPRESSOR_DUTY_RUN = 51) on the grounds that
 * PWM was the only thing keeping a 7.5 V motor off a 12 V rail. That reasoning
 * was sound for the hardware it described: a bare TO-220 IRLZ44N with its gate
 * driven straight from GPIO 3 at 3.3 V, never fully enhanced, measured at
 * ~100 C on the 2026-07-18 bench run at 20% duty and roughly 8 A. It does not
 * apply to the hotbed module, which drives its gate properly. That measurement
 * and the reasoning around it are preserved in history.md; they are out of this
 * comment because they describe hardware this code no longer talks to.
 */
/* Board divider on PRESSURE_1/2: three equal 10 k resistors (R11/R12/R13), so the
 * pin sees a third of the sensor's output. The Festo SDE5 gives 1 V/bar, hence
 * bar = 3 * V_pin. Both terms come from the schematic and the datasheet — there is
 * no calibration constant here and none is needed. */
#define PRESSURE_BAR_PER_PIN_VOLT 3.0f

/* Driver pedals. Each pedal outputs 0-5 V (nets PEDAL_ACC__0_5V on CN6.2 and
 * PEDAL_BRAKE__0_5V on CN6.1); the board halves it with a 10k/10k divider
 * (R14/R15 for the accelerator, R16/R17 for the brake, dv-hardware schematic
 * kart-medulla_P1) into the __3V3 nets on GPIO 4 / GPIO 5 (ADC1). So the pin
 * sees at most ~2.5 V, inside the 11 dB attenuation range.
 *
 * The min/max below are PROVISIONAL: derived from the divider math (full 0-5 V
 * span -> 0-2500 mV at the pin), not measured. Real pedals rest above 0 V and
 * top out below full scale — measure both ends on the kart and replace these.
 * Until then the *_effort fields of ESP_PEDALS are approximate; the *_mv fields
 * are always trustworthy (eFuse-calibrated). */
#define PEDAL_MIN_MV 0     // pin mV at pedal released — PROVISIONAL, calibrate on kart
#define PEDAL_MAX_MV 2500  // pin mV at pedal floored  — PROVISIONAL, calibrate on kart

/** Map a pedal pin reading (mV) to a 0-255 effort using the provisional span. */
static inline int32_t pedal_effort_255(int32_t mv)
{
    int32_t e = (mv - PEDAL_MIN_MV) * 255 / (PEDAL_MAX_MV - PEDAL_MIN_MV);
    if (e < 0) e = 0;
    if (e > 255) e = 255;
    return e;
}

/* Minimum pressure rise a full-length compressor burst must produce. The compressor
 * manages roughly 0.1 bar/s (0 -> 6 bar in about a minute, bench 2026-07-18), so a
 * 15 s burst should deliver well over a bar. 0.5 sits comfortably above the SDE5's
 * +/-3 %FS accuracy (0.3 bar) so sensor noise cannot trip it, and far below what a
 * healthy burst achieves. */
/* Minimum tank pressure for the kart to be allowed out of emergency. The reservoir
 * is sized for 3 EBS activations over a 10 -> 6 bar drawdown (Festo CRVZS-0.75; see
 * ~/dv/kart/pneumatics/README.md), so below 6 bar the guaranteed activation count no
 * longer holds and the kart must not present as ready to drive.
 *
 * ARM is higher than DISARM so the shutdown output cannot chatter while the tank sits
 * at the boundary — without the gap, one count of sensor noise either side of a
 * single threshold would toggle a safety line at the control-loop rate. */
#define EBS_TANK_ARM_BAR     6.5f   // must reach this before the chain may close
#define EBS_TANK_DISARM_BAR  6.0f   // falling below this opens it again

#define PRESSURE_STALL_MIN_RISE_BAR 0.15f

/* The stall check only judges bursts that STARTED below this. Above it the test
 * cannot tell "compressor dead" from "compressor near its ceiling", because a
 * compressor asymptotes as back pressure rises and a legitimate burst there adds
 * almost nothing. Judging only the easy part of the curve keeps the test meaningful:
 * below 4 bar a working compressor moves the tank fast, so no rise means broken.
 * A fault that first appears at high pressure is caught later, once the tank has
 * bled down past this line — later than ideal, but never wrongly. */
#define PRESSURE_STALL_JUDGE_BELOW_BAR 4.0f

#define COMPRESSOR_DUTY_RUN      255    // 100% — DC on, no switching (see above)
#define COMPRESSOR_SOFT_START_MS 1000   // linear 0 → COMPRESSOR_DUTY_RUN over this long
#define COMPRESSOR_MAX_RUN_MS    15000  // hard cap on one continuous burst
#define COMPRESSOR_COOLDOWN_MS   15000  // forced off-time after a burst hits the cap

/**
 * @brief Context shared between the control and health tasks.
 */
typedef struct {
    sensor_struct *sdir;
    ACT_Controller *dir_act;
    ACT_Controller *throttle_act;
    ACT_Controller *brake_act;
    PID_Controller *dir_pid;
} control_context_t;

/**
 * @brief   Read the steering angle by whichever path this board is wired for.
 *
 * @details The ESP32-S3 kart board reads the MT6701 through its PWM output on
 *          GPIO 1 (CN5.2). The classic ESP32 fallback board has no such input
 *          and keeps the original AS5600-over-I2C path. Splitting them here
 *          keeps the rest of control_task identical on both.
 *
 * @param   sdir     AS5600 sensor state (used only on the classic build).
 * @param   out_raw  Receives the raw 12-bit angle, or -1 when unknown.
 *
 * @return  Angle in radians, positive = left, or NAN when no angle is known.
 *
 * @note    Both paths return NAN rather than a substitute value when the read
 *          fails — the I2C one in particular, because KM_SDIR_ReadAngleRadians()
 *          on its own hands back the last raw value (0 from boot) on failure,
 *          which converts to a perfectly plausible 3.451 rad. See AGENTS.md,
 *          "Sensor Validity".
 */
static float steering_read_rad(sensor_struct *sdir, int32_t *out_raw)
{
#ifdef PIN_STEER_PWM_IN
    (void)sdir;
    *out_raw = KM_SDIR_PWM_ReadRaw();
    return KM_SDIR_PWM_ReadAngleRadians();
#else
    float rad = KM_SDIR_ReadAngleRadians(sdir);   // updates errorCount, so read first
    if (!KM_SDIR_isConnected(sdir)) {
        *out_raw = -1;
        return NAN;
    }
    *out_raw = (int32_t)sdir->lastRawValue;
    return rad;
#endif
}

// ===========================
// FreeRTOS task functions
// ===========================

/**
 * @brief   Communications task — receives and processes UART messages from Orin.
 *
 * @details Registered with a 10 ms period (100 Hz target) via the KM_RTOS periodic
 *          wrapper — see period_ms arg in main(). Actual rate is bounded by UART RX
 *          latency per cycle. Reads incoming bytes from UART0 and parses complete
 *          binary-encoded messages into shared objects.
 *
 * @param   ctx  Unused (NULL).
 */
void comms_task(void *ctx) {
    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();
}

/**
 * @brief   Control task — steering PID, actuator output, and sensor feedback.
 *
 * @details Registered with a 2 ms period (500 Hz target) via the KM_RTOS periodic
 *          wrapper — see period_ms arg in main(). Actual rate is bounded by UART
 *          send latency per cycle; measure before tuning. On each cycle:
 *          1. Reads the steering angle from the MT6701's PWM output, captured in
 *             hardware by MCPWM (km_sdir_pwm.h) — a non-blocking read.
 *          2. Sends steering feedback (angle + raw + last PID output + validity)
 *             to Orin.
 *          3. Applies comms watchdog / manual-mission safety (zero outputs).
 *          4. Applies throttle and brake actuator outputs from Orin targets.
 *          5. Runs the steering PID controller and sets the motor output, or
 *             stops the steering motor when the angle is unknown.
 *
 * @param   ctx  Pointer to a control_context_t with sensor, actuator, and PID references.
 */
void control_task(void *ctx) {
    control_context_t *c = (control_context_t *)ctx;

    static float last_pid_out = 0.0f;

    // Read the steering sensor BEFORE sending feedback, so the frame carries the
    // angle measured this cycle. The old order — send the previous value, then
    // read — existed because the I2C AS5600 read could stall the whole cycle on a
    // bus timeout, and frames had to escape before that happened. The PWM read
    // copies out samples the MCPWM capture ISR already timestamped, so it cannot
    // block and there is no longer anything to work around.
    //
    // Pneumatics (tank pressure + compressor duty) are NOT in this frame — they
    // ride their own ESP_PNEUMATIC frame, sent throttled further down.
    int32_t steer_raw   = -1;
    float   new_rad     = steering_read_rad(c->sdir, &steer_raw);  // NAN if unknown
    bool    steer_valid = !isnan(new_rad);

    // Only publish an angle we actually measured. When the read is invalid,
    // ACTUAL_STEERING keeps its previous contents and is NOT sent — the frame
    // carries the sentinel instead, so nothing downstream sees a stale value
    // dressed up as a fresh one.
    if (steer_valid) {
        KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(new_rad * 1000));
    }

    // Field 4 (validity) was APPENDED, not inserted: a consumer reading only the
    // first three fields decodes this frame unchanged.
    int32_t fb[4] = {
        steer_valid ? (int32_t)KM_OBJ_GetObjectValue(ACTUAL_STEERING)
                    : STEER_ANGLE_INVALID,             // angle_rad x 1000
        steer_raw,                                     // raw 12-bit angle, -1 = unknown
        (int32_t)(last_pid_out * 1000),                // PID output (PWM duty) x 1000
        steer_valid ? 1 : 0                            // 1 = fields 1-2 are real
    };
    KM_COMS_SendMsg(ESP_ACT_STEERING, fb, 4);

    TickType_t now = xTaskGetTickCount();

    // --- Comms watchdog inputs ---
    // Computed here rather than further down because the shutdown-circuit
    // decision below needs comms_stale, and that decision has to run on every
    // cycle — including the cycles where the watchdog block returns early.
    TickType_t last_cmd = KM_COMS_GetLastCmdTick();
    int mission = (int)KM_OBJ_GetObjectValue(MISION_ORIN);
    int comms_stale = (last_cmd == 0) || ((now - last_cmd) > pdMS_TO_TICKS(COMMS_WATCHDOG_MS));

    // --- Shutdown circuit (SDC) — the one place that decides Q3's gate ---
    //
    // Written as a whitelist: the chain is CLOSED only while every condition
    // below holds, and open in every other case, including any state nobody
    // thought about. That is the opposite of the usual "assert emergency when
    // something is wrong" shape, and it is deliberate — a condition someone
    // forgets to add then fails safe instead of silently leaving the kart armed.
    //
    //   - AS_READY / AS_DRIVING: the Orin's own state machine says the kart is
    //     meant to be able to move. AS_OFF, AS_FINISHED and AS_EMERGENCY all
    //     leave the chain open. Note this means the ESP32 never arms the kart on
    //     its own initiative; the Orin has to ask, every cycle, by continuing to
    //     report one of those two states.
    //   - COMPRESSOR_DISABLED: the operator's quiet-bench latch. A kart whose air
    //     supply is switched off cannot refill the EBS reservoir, so it must not
    //     also look ready to drive. This is the interlock behind the dashboard's
    //     compressor button.
    //   - tank_pressure_ok: the kart is holding enough air to guarantee its EBS
    //     activations (>= EBS_TANK_ARM_BAR, with hysteresis). This is the condition
    //     that makes starting in emergency correct rather than a fault: an empty
    //     kart sits in emergency until the compressor brings the tank up, which is
    //     how Formula Student expects it to behave.
    //     (The pump-stall detector is deliberately NOT in this list — it reports
    //     only. A broken compressor already shows up here as a tank that never
    //     reaches the arm threshold, which is the honest reason. See
    //     pump_stall_observed.)
    //   - steer_fault_latched: reads the PREVIOUS cycle's value, since the fault
    //     is detected further down this same function — a 2 ms lag at the 500 Hz
    //     task period. Harmless, because the detection sites still call
    //     KM_GPIO_SetEmergency(1) directly the instant they trip, so this block
    //     can only ever re-confirm an assertion, never delay one.
    //   - comms_stale: no fresh command from the Orin means nobody is driving.
    //
    // NOT WIRED YET (2026-07-26): the Q3 gate does not go anywhere, so nothing
    // physically brakes or arms when this changes. The level is reported back in
    // the pneumatic frame below, read off the pin, which is how the logic is
    // checked until the gate is connected.
    bool compressor_disabled = KM_OBJ_GetObjectValue(COMPRESSOR_DISABLED) != 0;
    int  as_state = (int)KM_OBJ_GetObjectValue(MACHINE_STATE_ORIN);
    bool sdc_may_close = (as_state == AS_READY || as_state == AS_DRIVING)
                      && tank_pressure_ok
                      && !compressor_disabled
                      && !steer_fault_latched
                      && !comms_stale;
    KM_GPIO_SetEmergency(sdc_may_close ? 0 : 1);

    // --- Compressor control logic (hysteresis + soft-start ramp) ---
    // Nominally 'pump below 7 bar, stop above 8' — but see the warning below.
    //
    // Thresholds are in BAR, because that is what they mean. They used to be raw
    // ADC counts (2500 and 2858) carried over from scaling a 2026-07-18 figure that
    // turned out to be void — there is no mechanical gauge on this kart, see
    // .agents/error-log.md 2026-07-27. Those counts worked out to roughly 6.0 and
    // 6.9 bar, so the band was real but nobody had chosen it; 2858 in particular was
    // just 2679 x 8/7.5 and meant nothing.
    //
    // Comparing in bar is possible now because the pressure is actually known:
    // the Festo SDE5 gives 1 V/bar, the board divides by three (R11/R12/R13 all 10k,
    // nets PRESSURE_n__0_10V -> PRESSURE_n__0_3V3), and KM_GPIO_ReadADC_mV()
    // converts counts to millivolts through the chip's eFuse calibration. No fitted
    // constant anywhere in that chain.
    // UNVERIFIED TARGET, 2026-07-27. 8 bar is what was asked for, but no run has
    // ever demonstrated this compressor reaching it. The only hardware evidence is
    // the 2026-07-18 bench run, logged as "0 -> 6 bar" using the calibration since
    // shown to be wrong; recomputed correctly that shutoff was at roughly 4.3-4.8
    // bar, and even that was the threshold cutting in, not the compressor running
    // out of breath. Its actual ceiling is unknown.
    //
    // Deliberately not tested (Rubén, 2026-07-27): a max-pressure run risks breaking
    // the compressor against a closed system, and it is not needed — normal use
    // shows the answer. If the ceiling is below 8 the tank plateaus, the pump cycles
    // 15 s on / 15 s off with the pressure static, and the report-only stall detector
    // flags it as comp_state 4. If that pattern shows up, lower this to just under
    // the observed plateau; until it does, the target stands.
    const float PRESSURE_PUMP_ON_BAR  = 7.0f;   // below this, start pumping
    const float PRESSURE_PUMP_OFF_BAR = 8.0f;   // above this, stop
    //
    // BEHAVIOUR CHANGE, 2026-07-27: the old counts stopped the pump around 6.9 bar,
    // so the tank now fills about a bar further. Deliberate — 7/8 is the intended
    // band and the reservoir is rated 10 bar — but two things bound it:
    //   - The 8 bar cutoff sits at 2667 mV and the ADC's 11 dB range ends near
    //     2900 mV, so there is only ~0.7 bar of headroom above the cutoff before
    //     readings saturate and stop being pressures at all. Do not raise
    //     PRESSURE_PUMP_OFF_BAR much past 8 without re-reading that ceiling.
    //   - The reading sags while the compressor runs (ground IR drop, ~8 A through
    //     board copper — see tasks.md). A low reading means the cutoff is reached
    //     LATER than intended, so true pressure at stop is above 8 bar by however
    //     much the sag is worth. That bias is unresolved, so treat 8 as a floor on
    //     where the pump actually stops, not a ceiling, and do not run the
    //     compressor unattended until it is measured.

    // Raw counts stay for telemetry; the control decision uses bar.
    uint16_t pres1_adc = KM_GPIO_ReadADC(PIN_PRESSURE_1);
    uint16_t pres2_adc = KM_GPIO_ReadADC(PIN_PRESSURE_2);
    uint32_t pres1_mv  = KM_GPIO_ReadADC_mV(PIN_PRESSURE_1);
    float    tank_bar  = PRESSURE_BAR_PER_PIN_VOLT * (float)pres1_mv / 1000.0f;

    // Demand latch (hysteresis): pump below LOW, stop above HIGH, hold state in
    // between. The band is what stops the motor short-cycling at the threshold.
    //
    // The operator latch is applied to DEMAND, not to the duty write further
    // down. Zeroing only the duty would leave burst_active set, so the burst
    // timer would keep running against a motor that is not turning and would
    // charge it a cooldown it never earned — and re-enabling mid-phantom-burst
    // would then skip the soft-start ramp, which is the one thing protecting the
    // 12 V rail from a stalled rotor's inrush.
    // Pegged high is not a pressure: at/above the 11 dB ceiling the ADC is saturated
    // and the true value is unknown but certainly over-range, so pumping into it
    // would be the worst case. Refuse.
    //
    // There is deliberately NO low-voltage validity floor. An earlier version
    // refused below 50 mV as "dead channel", which was wrong: the SDE5's output
    // characteristic starts at 0 V for 0 bar (datasheet, initial value 0 V), so an
    // empty tank legitimately reads 0 mV — precisely when pumping is most needed.
    // A dead sensor and an empty tank are indistinguishable by voltage alone, so
    // they are told apart by BEHAVIOUR instead: see the stall check below.
    const uint32_t PRESSURE_MAX_VALID_MV = 2900;  // 11 dB ceiling; at/above = pegged
    bool tank_pegged = (pres1_mv >= PRESSURE_MAX_VALID_MV);

    // Tank-pressure interlock for the shutdown circuit, with hysteresis. This is the
    // condition that actually matters, and it is a LIVE test of the pressure rather
    // than a heuristic about how it got there: the kart may leave emergency only
    // while it is holding enough air to guarantee its EBS activations, and it
    // re-enters emergency the moment that stops being true.
    //
    // Starting in emergency and staying there until the tank comes up is the normal,
    // intended behaviour — not a fault. It is also why the pump-stall detector does
    // not need to touch the SDC: if a broken compressor means the tank never reaches
    // EBS_TANK_ARM_BAR, this check already holds the chain open, for the reason that
    // actually matters (there is not enough air) rather than by inference from a
    // burst that underperformed.
    //
    // A pegged reading counts as not-enough: an unknown pressure is not a verified one.
    if (tank_pegged) {
        tank_pressure_ok = false;
    } else if (tank_bar >= EBS_TANK_ARM_BAR) {
        tank_pressure_ok = true;
    } else if (tank_bar < EBS_TANK_DISARM_BAR) {
        tank_pressure_ok = false;
    }

    static bool compressor_demand = false;
    if (compressor_disabled || tank_pegged) {
        compressor_demand = false;
    } else if (tank_bar < PRESSURE_PUMP_ON_BAR) {
        compressor_demand = true;
    } else if (tank_bar > PRESSURE_PUMP_OFF_BAR) {
        compressor_demand = false;
    }

    // Burst limiter — this is the thermal protection, since duty is fixed at
    // 100% and can no longer be used to limit average power. A burst runs at
    // most COMPRESSOR_MAX_RUN_MS, then the motor is forced off for
    // COMPRESSOR_COOLDOWN_MS even if the tank is still below LOW.
    //
    // Demand falling away ends a burst early WITHOUT owing a cooldown: a burst
    // that stopped because the tank filled was short, deposited little heat,
    // and the hysteresis band already prevents it from restarting immediately.
    // Only a burst that ran the full 15 s has to pay the 15 s back.
    static bool burst_active = false;
    static bool compressor_cooling = false;
    static TickType_t burst_start_tick = 0;
    static TickType_t cooldown_start_tick = 0;
    static float burst_start_bar = 0.0f;

    if (compressor_cooling &&
        (uint32_t)(now - cooldown_start_tick) * portTICK_PERIOD_MS >= COMPRESSOR_COOLDOWN_MS) {
        compressor_cooling = false;
    }

    if (!compressor_demand) {
        burst_active = false;
    } else if (!compressor_cooling) {
        if (!burst_active) {
            burst_active = true;
            burst_start_tick = now;   // new burst → restart the soft-start ramp
            burst_start_bar  = tank_bar;   // baseline for the stall check below
        } else if ((uint32_t)(now - burst_start_tick) * portTICK_PERIOD_MS >= COMPRESSOR_MAX_RUN_MS) {
            // A burst that ran the FULL length without reaching the cutoff should
            // have moved the tank a long way — the compressor does roughly
            // 0.1 bar/s, so 15 s is over a bar. If it barely moved, the air system
            // is not doing its job and the cause needs a human: a sensor stuck at
            // some constant (including 0, which is why there is no voltage floor
            // above — a dead sensor and an empty tank look identical until you try
            // to fill it), a compressor that is not actually spinning, or a leak
            // that outpaces it.
            //
            // Only full-length bursts are judged. A burst that ended early did so
            // because the tank reached the cutoff, which is success by definition.
            // PRESSURE_STALL_MIN_RISE_BAR is 0.15 bar: about 5x the sensor's
            // REPEATABILITY (0.3 %FS = 0.03 bar), which is the right spec for a
            // difference between two readings from the same sensor — the +/-3 %FS
            // precision figure is absolute error and largely cancels in a delta.
            // It asks only "did anything happen at all", not "was this productive",
            // because a working compressor slows as back pressure rises.
            if (burst_start_bar < PRESSURE_STALL_JUDGE_BELOW_BAR &&
                (tank_bar - burst_start_bar) < PRESSURE_STALL_MIN_RISE_BAR) {
                pump_stall_observed = true;
            }
            burst_active = false;     // hit the cap → forced cooldown
            compressor_cooling = true;
            cooldown_start_tick = now;
        }
    }

    // Ramp off elapsed time rather than a per-cycle step, so the profile stays a
    // 1 s ramp regardless of how the control task period is retuned.
    uint32_t comp_duty = 0;
    if (burst_active) {
        uint32_t elapsed_ms = (uint32_t)(now - burst_start_tick) * portTICK_PERIOD_MS;
        comp_duty = (elapsed_ms >= COMPRESSOR_SOFT_START_MS)
                  ? COMPRESSOR_DUTY_RUN
                  : (COMPRESSOR_DUTY_RUN * elapsed_ms) / COMPRESSOR_SOFT_START_MS;
    }

    // 0 = idle (tank satisfied), 1 = running, 2 = forced cooldown, 3 = disabled
    // by the operator. Sent so the dashboard can tell these apart, which are
    // otherwise all identical at duty 0 — and "off because someone switched it
    // off" is exactly the one you need to see, since it is also why the kart
    // will not arm. Checked first: the latch outranks the other two.
    int32_t comp_state = compressor_disabled ? 3
                       : (pump_stall_observed ? 4
                       : (tank_pegged ? 5
                       : (burst_active ? 1 : (compressor_cooling ? 2 : 0))));
#ifdef PIN_CMD_COMPRESSOR
    KM_GPIO_WritePWM(PIN_CMD_COMPRESSOR, comp_duty);
#endif

    // --- Pneumatics telemetry → Orin (throttled) ---
    // Capped at ~20 Hz: the steering frame already uses most of the 115200 UART
    // budget, so sending a second frame every cycle would overflow it. Pressure
    // moves over seconds, so 20 Hz is plenty for the dashboard.
    //
    // Throttled on ELAPSED TIME, not on a cycle count. A /25 divider only means
    // 20 Hz while the task really runs at its nominal 500 Hz, and it does not:
    // measured 2026-07-25 at ~8 Hz on the bench, because a disconnected AS5600
    // makes every cycle block on an I2C timeout. That turned a /25 divider into
    // 0.36 Hz telemetry — a compressor bar updating once per 3 s, for a burst
    // cycle that switches every 15 s. Wall-clock throttling gives 20 Hz when the
    // loop is fast and every available cycle when it is slow, with no retuning.
    //
    // Fields 2 and 3 were APPENDED, not inserted: an older Orin still reading
    // only [pressure, duty] keeps working against this frame unchanged.
    // control_task's own iteration count. Kept permanently, because frame arrival rate is NOT
    // a usable proxy for it: on 2026-07-25 this task averaged 8.9 Hz while actually running in
    // bursts of 10 iterations 0 ms apart separated by 1.1 s stalls (a blocking I2C read with no
    // sensor attached, then vTaskDelayUntil firing repeatedly to catch up). An average hides
    // that completely, and a control loop that stalls a second then takes ten steps with dt~0
    // is a very different thing from one running slowly but evenly. Compare consecutive values
    // against wall-clock arrival times to see the real shape.
    static uint32_t control_iters = 0;
    control_iters++;

    static TickType_t pneum_last_tick = 0;
    if ((uint32_t)(now - pneum_last_tick) * portTICK_PERIOD_MS >= 50) {  // 20 Hz cap
        pneum_last_tick = now;
        // Read the duty back OUT of the LEDC peripheral rather than reporting the value we
        // asked for. comp_duty only says what the firmware intended; the readback says what
        // the hardware actually holds, which is the difference between "the code commanded
        // it" and "the pin is driving it".
        int32_t ledc_readback = (int32_t)ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        // Same readback principle as ledc_readback above, for the same reason:
        // report the level the pin is holding, not the level we asked for. Needs
        // GPIO_MODE_INPUT_OUTPUT on the pin (set in KM_GPIO_Init) — on a plain
        // OUTPUT this would read 0 forever and look like a permanently open chain.
#ifdef PIN_SDC_NOT_EMERGENCY
        int32_t sdc_readback = (int32_t)gpio_get_level(PIN_SDC_NOT_EMERGENCY);
#else
        // Sentinel for a build with no SDC pin. Both current targets define one,
        // so this branch is unreachable today — it exists so that such a build
        // reports "no data" rather than a 0 the dashboard would draw as an open
        // chain, which is a real and different condition.
        int32_t sdc_readback = -1;
#endif
        // Millivolts at the pin, converted through the chip's eFuse calibration.
        // These are the fields a consumer should use for pressure: bar = 3 * V_pin
        // for the 1 V/bar Festo SDE5 behind the board's three-equal-10k divider
        // (R11/R12/R13, net PRESSURE_n__0_10V -> PRESSURE_n__0_3V3). The raw counts
        // in fields 0 and 2 stay for the control loop and for older consumers, but
        // turning THOSE into bar means inventing a full-scale voltage, which is
        // exactly the mistake that had the dial and the firmware disagreeing.
        int32_t pres1_mv = (int32_t)KM_GPIO_ReadADC_mV(PIN_PRESSURE_1);
        int32_t pres2_mv = (int32_t)KM_GPIO_ReadADC_mV(PIN_PRESSURE_2);
        int32_t pneum[10] = {
            (int32_t)pres1_adc,        // PRESSURE_1 — tank, raw ADC 0-4095
            (int32_t)comp_duty,        // compressor duty 0-255 commanded (0 = off)
            (int32_t)pres2_adc,        // PRESSURE_2 — piston/brake line, raw ADC 0-4095
            comp_state,                // 0 = idle, 1 = running, 2 = cooldown, 3 = disabled
            (int32_t)control_iters,    // control_task iteration count
            ledc_readback,             // duty actually held by LEDC ch1 (GPIO 3 / CN8.2)
            g_gpio_init_err,           // KM_GPIO_Init() result; 0 = ESP_OK
            sdc_readback,              // SDC pin readback: 1 = chain closed, 0 = emergency
            pres1_mv,                  // PRESSURE_1 in mV at the pin (calibrated)
            pres2_mv                   // PRESSURE_2 in mV at the pin (calibrated)
        };
        KM_COMS_SendMsg(ESP_PNEUMATIC, pneum, 10);

        // Driver pedals, same 20 Hz throttle as the pneumatics frame. mV first
        // (always trustworthy, eFuse-calibrated), then the normalized effort from
        // the provisional PEDAL_MIN_MV/PEDAL_MAX_MV span — see their definition.
        int32_t acc_mv   = (int32_t)KM_GPIO_ReadADC_mV(PIN_PEDAL_ACC);
        int32_t brake_mv = (int32_t)KM_GPIO_ReadADC_mV(PIN_PEDAL_BRAKE);
        int32_t pedals[4] = {
            acc_mv,                    // accelerator pedal, mV at GPIO 4 (half of 0-5 V signal)
            brake_mv,                  // brake pedal, mV at GPIO 5 (half of 0-5 V signal)
            pedal_effort_255(acc_mv),  // accelerator effort 0-255 (provisional calibration)
            pedal_effort_255(brake_mv) // brake effort 0-255 (provisional calibration)
        };
        KM_COMS_SendMsg(ESP_PEDALS, pedals, 4);
    }

    // Pick up any PID gains the dashboard has pushed. Deliberately ABOVE the manual /
    // comms-stale early return: setting a gain actuates nothing on its own (the branch
    // below still stops every actuator), but the ESP_STEER_PID echo is built from these
    // values, so skipping this while idle would make the dashboard report the compiled
    // defaults no matter what the operator had just applied. Tuning is set up with the
    // kart stationary and in manual — the one state where the old placement, after the
    // return, meant the request was silently ignored and the readback silently lied.
    pid_apply_override(c->dir_pid, c->dir_act);

    // --- Safety: comms watchdog + manual mode ---
    // last_cmd / mission / comms_stale are computed at the top of this function,
    // because the SDC decision needs comms_stale and must not be skipped by the
    // early return below.
    if (comms_stale || mission == MISSION_MANUAL) {
        // No commands received recently OR manual mode → zero all outputs
        KM_ACT_Stop(c->throttle_act);
        KM_ACT_Stop(c->brake_act);
        KM_ACT_Stop(c->dir_act);
        KM_PID_Reset(c->dir_pid);
        last_pid_out = 0.0f;
        return;
    }

    // Steering mode: 0=PID (default), 1=direct PWM
    int steer_mode = (int)KM_OBJ_GetObjectValue(STEER_MODE);

    // Once the steering-sensor fault has tripped, it stays tripped: keep the EBS
    // fired and the throttle at zero on every cycle, whether or not the sensor
    // has since come back. Re-asserting each cycle rather than only on the
    // triggering one means a later write to either output cannot quietly undo it.
    if (steer_fault_latched) {
        KM_GPIO_SetEmergency(1);
        KM_ACT_Stop(c->throttle_act);
        if (steer_mode != 1) {
            // Open-loop direct-PWM steering stays available so the column can
            // still be moved while diagnosing the fault. Closed-loop steering
            // does not come back — that would be the firmware re-arming itself.
            KM_ACT_Stop(c->dir_act);
            KM_PID_Reset(c->dir_pid);
            last_pid_out = 0.0f;
            return;
        }
    }

    // Target from Orin: interpretation depends on mode
    float target_raw = (float)KM_OBJ_GetObjectValue(TARGET_STEERING) / 1000.0f;

    // Throttle + brake: int32 effort (0-255 range from Orin). Throttle is refused
    // outright while the steering fault is latched — the Orin does not get to
    // command drive on a kart whose steering angle is unknown. Braking is still
    // passed through: the EBS is doing the stopping, but there is no reason to
    // block a brake request on top of it.
    float thr = (float)KM_OBJ_GetObjectValue(TARGET_THROTTLE) / 255.0f;
    float brk = (float)KM_OBJ_GetObjectValue(TARGET_BRAKING) / 255.0f;
    if (steer_fault_latched) {
        KM_ACT_Stop(c->throttle_act);
    } else {
        KM_ACT_SetOutput(c->throttle_act, thr);
    }
    KM_ACT_SetOutput(c->brake_act, brk);

    float steer_out;
    if (steer_mode == 1) {
        // Direct PWM mode: target_raw is PWM value [-1.0, 1.0]. Open loop, so it
        // deliberately still runs without a valid sensor — this is the mode used
        // to move the column on the bench, including while the sensor is being
        // mounted or has nothing to read.
        steer_out = target_raw;
        // Reset PID integral so it doesn't wind up while inactive
        KM_PID_Reset(c->dir_pid);
    } else if (!steer_valid) {
        // PID mode with no angle feedback: the kart is driving without knowing
        // where its wheels point. Decision (Rubén, 2026-07-26): zero the throttle
        // and fire the EBS, which brakes hard. The steering motor stops too —
        // there is no error term to act on, so any output would be a guess.
        //
        // LATCHED until reboot, deliberately. A dropout long enough to reach here
        // has already survived the 50 ms staleness window and the median filter,
        // so it is not a single glitched frame; and a kart that resumed
        // autonomous steering the instant frames returned would be re-arming
        // itself after a safety trip, which is not the firmware's call to make.
        // How the trip should be cleared is open in tasks.md.
        steer_fault_latched = true;
        KM_GPIO_SetEmergency(1);
        KM_ACT_Stop(c->throttle_act);
        KM_ACT_Stop(c->dir_act);
        KM_PID_Reset(c->dir_pid);
        last_pid_out = 0.0f;
        return;
    } else {
        // PID mode: target_raw is angle in radians
        steer_out = KM_PID_Calculate(c->dir_pid, target_raw, new_rad);
    }
    KM_ACT_SetOutput(c->dir_act, steer_out);
    last_pid_out = steer_out;
}

/**
 * @brief   Heartbeat task — sends a periodic alive signal to Orin.
 *
 * @details Registered with a 1000 ms period (1 Hz) via the KM_RTOS periodic
 *          wrapper — see period_ms arg in main(). Sends a single int32 payload
 *          containing the ESP32 uptime in milliseconds as an ESP_HEARTBEAT
 *          message over UART.
 *
 * @param   ctx  Unused (NULL).
 */
void heartbeat_task(void *ctx) {
    int32_t payload[1] = {(int32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS)};
    KM_COMS_SendMsg(ESP_HEARTBEAT, payload, 1);
}

/**
 * @brief   Health monitoring task — checks magnet, I2C, and heap status.
 *
 * @details Runs at 1 Hz in its own FreeRTOS task (not via KM_RTOS wrapper).
 *          Reports a 4-element int32 payload [flags, agc, heap_kb, i2c_errors]
 *          to Orin as an ESP_HEALTH_STATUS message.
 *
 *          Flag bits:
 *          - bit 0 (HEALTH_FLAG_MAGNET_OK): AGC in valid range [20, 235].
 *          - bit 1 (HEALTH_FLAG_I2C_OK): AS5600 I2C read succeeded.
 *          - bit 2 (HEALTH_FLAG_HEAP_OK): Free heap >= 4 KB.
 *
 * @param   ctx  Pointer to a control_context_t (needs sdir for AS5600 access).
 *
 * @note    This task contains its own infinite loop with vTaskDelay; it is NOT
 *          intended for use with KM_RTOS_TaskWrapper.
 */
#define HEALTH_FLAG_MAGNET_OK (1 << 0)
#define HEALTH_FLAG_I2C_OK    (1 << 1)
#define HEALTH_FLAG_HEAP_OK   (1 << 2)
#define HEALTH_FLAG_STEER_OK  (1 << 3)  // steering-sensor PWM read is currently valid
#define HEALTH_FLAG_STEER_TRIP (1 << 4) // steering fault latched: EBS fired, throttle refused
#define HEALTH_HEAP_MIN_BYTES 4096
#define AGC_MIN 20   // below = magnet too strong
#define AGC_MAX 235  // above = magnet too weak

void health_task(void *ctx) {
    control_context_t *c = (control_context_t *)ctx;

    while (1) {
        int32_t flags = 0;

        uint8_t as_status = 0, agc = 0;
        int8_t i2c_ok = 0;
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
        // AGC is the reliable magnet strength indicator (0=too strong, 255=too weak)
        i2c_ok = KM_SDIR_ReadStatusAGC(c->sdir, &as_status, &agc);
#else
        // The kart's sensor is an MT6701 read over PWM. The AS5600 driver answers
        // at a different I2C address, so polling it here can only ever time out —
        // once per second, blocking this task while it does. AGC and the I2C flag
        // stay 0, meaning "not measured", and steering health is reported through
        // HEALTH_FLAG_STEER_OK and the frame counters instead.
        (void)as_status;
#endif

        if (i2c_ok) flags |= HEALTH_FLAG_I2C_OK;
        if (i2c_ok && agc >= AGC_MIN && agc <= AGC_MAX) flags |= HEALTH_FLAG_MAGNET_OK;

#ifdef PIN_STEER_PWM_IN
        int8_t steer_ok = KM_SDIR_PWM_isValid();
#else
        int8_t steer_ok = KM_SDIR_isConnected(c->sdir);
#endif
        if (steer_ok) flags |= HEALTH_FLAG_STEER_OK;
        if (steer_fault_latched) flags |= HEALTH_FLAG_STEER_TRIP;

        // Heap check
        uint32_t free_heap = esp_get_free_heap_size();
        if (free_heap >= HEALTH_HEAP_MIN_BYTES) flags |= HEALTH_FLAG_HEAP_OK;
        uint16_t heap_kb = (uint16_t)(free_heap / 1024);

        // Payload: [flags, agc, heap_kb, i2c_errors, steer_frames, steer_rejects].
        // The last two were APPENDED — a consumer reading only the first four
        // fields decodes this frame unchanged. They separate the two ways the
        // steering read can be unhealthy, which the flag bit alone cannot:
        // frames flat at zero means no edges are arriving at all (dead sensor,
        // unplugged lead), while rejects climbing against flat frames means edges
        // arrive but at the wrong rate (sensor reverted out of 994 Hz PWM mode,
        // or the lead is picking up noise).
        int32_t payload[6] = {
            flags,
            (int32_t)agc,
            (int32_t)heap_kb,
            (int32_t)c->sdir->errorCount,
            (int32_t)KM_SDIR_PWM_GetFrameCount(),
            (int32_t)KM_SDIR_PWM_GetRejectCount()
        };
        KM_COMS_SendMsg(ESP_HEALTH_STATUS, payload, 6);

        // Echo the steering gains actually in force. This is the only thing that
        // tells the dashboard the truth: an ESP32 reset clears the override and
        // reverts to the compiled defaults, and the Orin deliberately does not
        // re-push the tuning afterwards. Without this frame the browser would go
        // on displaying the numbers someone typed twenty minutes ago while the
        // kart steered on entirely different ones.
        int32_t pid_payload[5] = {
            g_pid_override ? 1 : 0,
            (int32_t)(g_pid_kp * 1000.0f),
            (int32_t)(g_pid_ki * 1000.0f),
            (int32_t)(g_pid_kd * 1000.0f),
            (int32_t)(g_pid_pwm_limit * 1000.0f)
        };
        KM_COMS_SendMsg(ESP_STEER_PID, pid_payload, 5);

        // Log warnings for critical issues
        if (i2c_ok && agc < AGC_MIN)
            ESP_LOGW(TAG, "HEALTH: magnet too strong (AGC=%d)", agc);
        if (i2c_ok && agc > AGC_MAX)
            ESP_LOGW(TAG, "HEALTH: magnet too weak (AGC=%d)", agc);
        if (!steer_ok)
            ESP_LOGW(TAG, "HEALTH: no steering angle (frames=%lu rejects=%lu)",
                     (unsigned long)KM_SDIR_PWM_GetFrameCount(),
                     (unsigned long)KM_SDIR_PWM_GetRejectCount());
        if (steer_fault_latched)
            ESP_LOGE(TAG, "HEALTH: steering fault LATCHED — EBS fired, throttle refused, "
                          "reboot to clear");
        if (!(flags & HEALTH_FLAG_HEAP_OK))
            ESP_LOGW(TAG, "HEALTH: low heap! %lu bytes free", (unsigned long)free_heap);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief   Initializes all subsystems and registers FreeRTOS tasks.
 *
 * @details Performs the full system bring-up sequence:
 *          1. GPIO peripherals (ADC, DAC, PWM, I2C, direction pin).
 *          2. RTOS task manager.
 *          3. UART communications to Orin.
 *          4. Steering encoder: I2C bus up for the PCF8574, and MCPWM capture
 *             started on the MT6701's PWM angle output.
 *          5. Actuator controllers (steering, throttle, brake) with output limits.
 *          6. Steering PID controller.
 *          7. Registers periodic tasks: comms (100 Hz), control (500 Hz),
 *             heartbeat (1 Hz), and health monitoring (1 Hz).
 *
 * @note    All controller/sensor structs are copied to file-scope statics so
 *          they outlive this function. The FreeRTOS scheduler keeps tasks alive
 *          after system_init returns.
 */
void system_init(void) {

    // Initialize hardware
    g_gpio_init_err = (int32_t)KM_GPIO_Init();
    if(g_gpio_init_err != ESP_OK)
        ESP_LOGE(TAG, "Error inicializando libreria gpio\n");

    // Initialise tasks
    KM_RTOS_Init();

    // Initialise comunications on UART0 (USB to Orin)
    if (KM_COMS_Init(UART_NUM_0) != ESP_OK)
        ESP_LOGE(TAG, "Error inicializando libreria de comunicaciones");

    // The I2C bus is still brought up — the on-board PCF8574 lives on it — but the
    // steering angle no longer comes from here. km_sdir.c is an AS5600 driver
    // fixed at address 0x36; the kart's sensor is an MT6701 at 0x06, and it is
    // read through its PWM output instead (km_sdir_pwm.h).
    sensor_struct sdir = KM_SDIR_Init(MAX_ERROR_COUNT_SDIR);
    KM_SDIR_Begin(&sdir, PIN_I2C_SDA, PIN_I2C_SCL);

#ifdef PIN_STEER_PWM_IN
    esp_err_t steer_ret = KM_SDIR_PWM_Begin(PIN_STEER_PWM_IN);
    if (steer_ret != ESP_OK) {
        ESP_LOGE(TAG, "steering PWM capture failed to start: %s", esp_err_to_name(steer_ret));
    } else {
        // Wait for the median window to fill before deciding whether the sensor
        // is there. Five frames at 994 Hz take about 5 ms; 200 ms of polling is
        // generous enough that a "not responding" verdict here means the signal
        // really is absent, not that we asked too early.
        float init_rad = NAN;
        for (int i = 0; i < 20 && isnan(init_rad); i++) {
            vTaskDelay(pdMS_TO_TICKS(10));
            init_rad = KM_SDIR_PWM_ReadAngleRadians();
        }
        if (!isnan(init_rad)) {
            KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(init_rad * 1000));
            ESP_LOGI(TAG, "steering sensor OK — %.3f rad (raw %ld)",
                     init_rad, (long)KM_SDIR_PWM_ReadRaw());
        } else {
            ESP_LOGW(TAG, "steering sensor NOT reading (frames=%lu rejects=%lu) — "
                          "PID steering will hold the motor stopped",
                     (unsigned long)KM_SDIR_PWM_GetFrameCount(),
                     (unsigned long)KM_SDIR_PWM_GetRejectCount());
        }
    }
#else
    ESP_LOGW(TAG, "no steering PWM input pin on this target — no angle feedback");
#endif

    // ------------------------------------------------------
    // Initialize Motor controllers
    // *** STEERING PWM LIMIT — keep low during testing to protect gears ***
    // Increase gradually once PID is tuned. 1.0 = full power.
    ACT_Controller dir_act = KM_ACT_Init(ACT_STEER, 0.40);
    ACT_Controller throttle_act = KM_ACT_Init(ACT_ACCEL, 1.0);
    ACT_Controller brake_act = KM_ACT_Init(ACT_BRAKE, 1.0);

    KM_ACT_SetLimit(&dir_act, PID_DEFAULT_PWM_LIMIT);
    KM_ACT_SetLimit(&throttle_act, 1.0);
    KM_ACT_SetLimit(&brake_act, 1.0);

    // TEMPORARY TEST: raw ESP-IDF DAC write to GPIO 25 (DAC_CHAN_0)
    // Bypasses all our abstraction. Should produce ~1.65V.
#ifdef CONFIG_IDF_TARGET_ESP32
    dac_output_enable(DAC_CHAN_0);  // GPIO 25
    dac_output_voltage(DAC_CHAN_0, 128);  // 128/255 * 3.3V ≈ 1.65V
#endif

    // Initialise PID for steering. The gains live in one place (PID_DEFAULT_*
    // near the top of this file) because control_task restores exactly these
    // values whenever the dashboard clears its override — two copies would drift
    // and a "reset to defaults" button would quietly reset to something else.
    PID_Controller dir_pid = KM_PID_Init(PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD);
    KM_PID_SetOutputLimits(&dir_pid, -1.0f, 1.0f);
    KM_PID_SetIntegralLimits(&dir_pid, -10.0f, 10.0f);

    // Build control context (static so it outlives system_init)
    static control_context_t ctrl_ctx;
    static sensor_struct sdir_static;
    static ACT_Controller dir_act_static, throttle_act_static, brake_act_static;
    static PID_Controller dir_pid_static;

    sdir_static = sdir;
    dir_act_static = dir_act;
    throttle_act_static = throttle_act;
    brake_act_static = brake_act;
    dir_pid_static = dir_pid;

    ctrl_ctx.sdir = &sdir_static;
    ctrl_ctx.dir_act = &dir_act_static;
    ctrl_ctx.throttle_act = &throttle_act_static;
    ctrl_ctx.brake_act = &brake_act_static;
    ctrl_ctx.dir_pid = &dir_pid_static;

    // NOTE: UART2 log redirect removed — it caused crashes and UART0 protocol noise.
    // Logs go to UART0 at 115200. SerialDriver ignores non-0xAA bytes (SOF filtering).

    // Register FreeRTOS tasks
    // KM_COMS_CreateTask args: (name, fn, ctx, period_ms, stackWords, priority, active)
    //                                          ^^^^^^^^^ period is in MILLISECONDS, not Hz.
    //   comms:     10 ms  →  100 Hz target
    //   control:    2 ms  →  500 Hz, measured on the kart (control_iters). The rate cap is
    //              the UART, not the sensor: the per-cycle steering frame uses ~87% of the
    //              115200-baud link and TX is unbuffered (see tasks.md). The old AS5600 I2C
    //              stall is gone — the S3 reads the MT6701 via non-blocking MCPWM capture.
    //   heartbeat: 1000 ms →    1 Hz
    RTOS_Task t1 = KM_COMS_CreateTask("comms", comms_task, NULL, 10, 4096, 2, 1);
    RTOS_Task t2 = KM_COMS_CreateTask("control", control_task, &ctrl_ctx, 2, 4096, 1, 1);
    RTOS_Task t3 = KM_COMS_CreateTask("heartbeat", heartbeat_task, NULL, 1000, 2048, 1, 1);

    KM_RTOS_AddTask(t1);
    KM_RTOS_AddTask(t2);
    KM_RTOS_AddTask(t3);

    ESP_LOGI(TAG, "All tasks registered — scheduler running");

    // Launch health monitoring task (1 Hz, checks magnet/I2C/heap)
    xTaskCreate(health_task, "health", 4096, &ctrl_ctx, 1, NULL);

    // system_init returns, FreeRTOS scheduler keeps tasks alive
}

/**
 * @brief   Application entry point (called by ESP-IDF after boot).
 *
 * @details Initializes NVS flash (required by ESP-IDF internals such as
 *          WiFi and Bluetooth stacks), sets the global log level to INFO,
 *          and calls system_init() to bring up all subsystems and FreeRTOS tasks.
 *
 * @note    If NVS partition is corrupt or has a version mismatch, the flash
 *          is erased and re-initialized automatically.
 */
void app_main(void) {
    // Init NVS (needed by ESP-IDF internals: WiFi, BT stacks, etc.)
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Log init message before disabling logs — UART0 is shared with binary protocol
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "ESP32 starting...");

    system_init();

    // Disable all logging on UART0 to prevent ASCII text from corrupting
    // binary protocol frames. Without this, ESP_LOG output interleaves with
    // protocol bytes and causes CRC mismatches on the Orin side.
    esp_log_level_set("*", ESP_LOG_NONE);
}
