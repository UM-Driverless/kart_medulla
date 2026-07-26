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

#define MAX_ERROR_COUNT_SDIR 10
#define COMMS_WATCHDOG_MS    1000  // Zero outputs if no command for this long
#define MISSION_MANUAL       0     // Mission ID 0 = manual (no electronic actuation)

/* Sent in the steering frame's angle field when no angle is known. INT32_MIN is
 * roughly -2.1e6 radians once the consumer divides by 1000, so it cannot be
 * mistaken for a steering angle by anything that plots or acts on the number —
 * which is the point. Sending 0, or the last good reading, would look exactly
 * like a plausible measurement, and that is how an unplugged sensor drew a
 * confident 90-degrees-left on the dashboard on 2026-07-25 (see AGENTS.md,
 * "Sensor Validity"). Field 4 of the same frame carries the validity flag for
 * consumers that check it properly. */
#define STEER_ANGLE_INVALID  INT32_MIN

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

    // --- Compressor control logic (hysteresis + soft-start ramp) ---
    // Pump up below 7 bar, stop above 8 bar.
    //
    // Calibration (2026-07-18): the tank sat at a gauge-read 7.5 bar while this
    // ADC channel read 2679, giving 7.5 / 2679 = 0.0028 bar per count. The
    // thresholds below are that ratio scaled, assuming the sensor is linear
    // through zero:
    //     7 bar -> 2679 * 7/7.5 = 2500
    //     8 bar -> 2679 * 8/7.5 = 2858
    //
    // This is a ONE-POINT calibration and the zero-offset assumption is
    // unverified — many pressure senders idle at an offset (0.5 V is common)
    // rather than at 0 V, which would make both numbers read high. It is
    // deliberately anchored to the gauge rather than to the older "bar =
    // 3 x Vadc" note, because the two disagree (that map calls this same 2679
    // reading 6.5 bar, not 7.5) and the gauge-anchored figure is the
    // conservative one: if the older map turns out to be right, these
    // thresholds stop the pump EARLY, around 6.9 bar, rather than late. Getting
    // it wrong in the other direction matters, because the reservoir is rated
    // 10 bar. Confirm against the gauge on the next run and correct if needed.
    const uint16_t ADC_PRESSURE_LOW  = 2500;  // ~7 bar — below this, start pumping
    const uint16_t ADC_PRESSURE_HIGH = 2858;  // ~8 bar — above this, stop

    uint16_t pres1_adc = KM_GPIO_ReadADC(PIN_PRESSURE_1);
    uint16_t pres2_adc = KM_GPIO_ReadADC(PIN_PRESSURE_2);

    // Demand latch (hysteresis): pump below LOW, stop above HIGH, hold state in
    // between. The band is what stops the motor short-cycling at the threshold.
    static bool compressor_demand = false;
    if (pres1_adc < ADC_PRESSURE_LOW) {
        compressor_demand = true;
    } else if (pres1_adc > ADC_PRESSURE_HIGH) {
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
        } else if ((uint32_t)(now - burst_start_tick) * portTICK_PERIOD_MS >= COMPRESSOR_MAX_RUN_MS) {
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

    // 0 = idle (tank satisfied), 1 = running, 2 = forced cooldown. Sent so the
    // dashboard can tell "off because full" from "off because cooling", which
    // otherwise look identical at duty 0.
    int32_t comp_state = burst_active ? 1 : (compressor_cooling ? 2 : 0);
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
        int32_t pneum[7] = {
            (int32_t)pres1_adc,        // PRESSURE_1 — tank, raw ADC 0-4095
            (int32_t)comp_duty,        // compressor duty 0-255 commanded (0 = off)
            (int32_t)pres2_adc,        // PRESSURE_2 — piston/brake line, raw ADC 0-4095
            comp_state,                // 0 = idle, 1 = running, 2 = cooldown
            (int32_t)control_iters,    // control_task iteration count
            ledc_readback,             // duty actually held by LEDC ch1 (GPIO 3 / CN8.2)
            g_gpio_init_err            // KM_GPIO_Init() result; 0 = ESP_OK
        };
        KM_COMS_SendMsg(ESP_PNEUMATIC, pneum, 7);
    }

    // --- Safety: comms watchdog + manual mode ---
    TickType_t last_cmd = KM_COMS_GetLastCmdTick();
    int mission = (int)KM_OBJ_GetObjectValue(MISION_ORIN);
    int comms_stale = (last_cmd == 0) || ((now - last_cmd) > pdMS_TO_TICKS(COMMS_WATCHDOG_MS));

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

    // Target from Orin: interpretation depends on mode
    float target_raw = (float)KM_OBJ_GetObjectValue(TARGET_STEERING) / 1000.0f;

    // Throttle + brake: int32 effort (0-255 range from Orin)
    float thr = (float)KM_OBJ_GetObjectValue(TARGET_THROTTLE) / 255.0f;
    float brk = (float)KM_OBJ_GetObjectValue(TARGET_BRAKING) / 255.0f;
    KM_ACT_SetOutput(c->throttle_act, thr);
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
        // PID mode with no angle feedback. There is no error term to act on, so
        // the motor is stopped rather than driven off a guessed position.
        //
        // Throttle and brake are left as the Orin commanded them, above. Whether
        // losing the steering sensor should instead trigger a full stop or the
        // EBS is a vehicle-safety decision, not a firmware default — it is open
        // in tasks.md and needs Rubén's call.
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

        // Log warnings for critical issues
        if (i2c_ok && agc < AGC_MIN)
            ESP_LOGW(TAG, "HEALTH: magnet too strong (AGC=%d)", agc);
        if (i2c_ok && agc > AGC_MAX)
            ESP_LOGW(TAG, "HEALTH: magnet too weak (AGC=%d)", agc);
        if (!steer_ok)
            ESP_LOGW(TAG, "HEALTH: no steering angle (frames=%lu rejects=%lu)",
                     (unsigned long)KM_SDIR_PWM_GetFrameCount(),
                     (unsigned long)KM_SDIR_PWM_GetRejectCount());
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
 *          7. Registers periodic tasks: comms (20 Hz), control (10 Hz),
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

    KM_ACT_SetLimit(&dir_act, 0.50);
    KM_ACT_SetLimit(&throttle_act, 1.0);
    KM_ACT_SetLimit(&brake_act, 1.0);

    // TEMPORARY TEST: raw ESP-IDF DAC write to GPIO 25 (DAC_CHAN_0)
    // Bypasses all our abstraction. Should produce ~1.65V.
#ifdef CONFIG_IDF_TARGET_ESP32
    dac_output_enable(DAC_CHAN_0);  // GPIO 25
    dac_output_voltage(DAC_CHAN_0, 128);  // 128/255 * 3.3V ≈ 1.65V
#endif

    // Initialise PID for steering
    float kp = 1.50;
    float ki = 0.0;
    float kd = 0.02;
    PID_Controller dir_pid = KM_PID_Init(kp, ki, kd);
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
    //   control:    2 ms  →  500 Hz target (I2C AS5600 read caps real rate; measure it)
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
