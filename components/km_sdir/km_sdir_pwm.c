/******************************************************************************
 * @file    km_sdir_pwm.c
 * @brief   MT6701 steering-angle read over PWM, timed by MCPWM capture.
 *
 * @details See km_sdir_pwm.h for the frame format and the validity contract.
 *          Capture runs entirely in hardware: MCPWM timestamps both edges of
 *          the sensor's OUT line and the ISR turns each complete rising-to-
 *          rising interval into one 12-bit angle sample. The reader functions
 *          only ever copy out the last few samples, so the control loop never
 *          waits on the sensor.
 *****************************************************************************/

#include "km_sdir_pwm.h"
#include "km_sdir.h"          /* SENSOR_CENTER — shared with the I2C path */

#include <math.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "soc/soc_caps.h"

#if SOC_MCPWM_SUPPORTED
#include "driver/mcpwm_cap.h"
#endif

#define TAG "KM_SDIR_PWM"

#if SOC_MCPWM_SUPPORTED

/******************************* ESTADO PRIVADO *******************************/

/* Written by the capture ISR, read by control-loop callers. Everything shared
 * across that boundary sits behind s_mux; the ISR side uses the _ISR variants
 * of the critical-section macros. */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static mcpwm_cap_timer_handle_t   s_cap_timer = NULL;
static mcpwm_cap_channel_handle_t s_cap_chan  = NULL;

/* Period sanity window, in capture ticks. Computed in Begin() from the timer's
 * real resolution rather than assuming 80 MHz, so a different capture clock
 * source does not silently widen or narrow the check. */
static uint32_t s_period_min_ticks = 0;
static uint32_t s_period_max_ticks = 0;

/* Median window of accepted samples, newest at s_ring_idx - 1. */
static uint16_t s_ring[KM_SDIR_PWM_MEDIAN_N];
static uint8_t  s_ring_idx  = 0;
static uint8_t  s_ring_fill = 0;
static int64_t  s_last_good_us = 0;

/* Edge bookkeeping, ISR-private. */
static uint32_t s_last_rise_tick = 0;
static uint32_t s_high_ticks     = 0;
static bool     s_have_rise      = false;
static bool     s_have_high      = false;

static uint32_t s_frames  = 0;
static uint32_t s_rejects = 0;

/******************************* FUNCIONES PRIVADAS ***************************/

/**
 * @brief  Capture ISR — one call per edge on the sensor's OUT line.
 *
 * @details A frame is measured rising-edge to rising-edge, with the falling
 *          edge in between giving the high time. Both are raw MCPWM tick
 *          counts; the subtraction is done in uint32 so the capture timer's
 *          32-bit wrap cancels out on its own and needs no special case.
 *
 *          A frame is only decoded if its period lands inside the sanity
 *          window. That is the check that keeps noise on the sensor lead, or a
 *          sensor that came up at the wrong frame rate, from being turned into
 *          a confident wrong angle.
 *
 * @return  false — no higher-priority task is woken from here.
 */
static bool IRAM_ATTR KM_SDIR_PWM_CaptureISR(mcpwm_cap_channel_handle_t chan,
                                             const mcpwm_capture_event_data_t *edata,
                                             void *user_ctx)
{
    (void)chan;
    (void)user_ctx;

    if (edata->cap_edge == MCPWM_CAP_EDGE_NEG) {
        /* Falling edge: close out the high time of the frame in progress. */
        if (s_have_rise) {
            s_high_ticks = edata->cap_value - s_last_rise_tick;
            s_have_high  = true;
        }
        return false;
    }

    /* Rising edge: the previous frame is complete (if we saw all of it). */
    if (s_have_rise && s_have_high) {
        uint32_t period = edata->cap_value - s_last_rise_tick;

        if (period >= s_period_min_ticks && period <= s_period_max_ticks &&
            s_high_ticks > 0 && s_high_ticks < period) {

            /* data = duty x 4119 - 16, per the MT6701 frame format. */
            uint64_t scaled = (uint64_t)s_high_ticks * KM_SDIR_PWM_FRAME_CLOCKS;
            int32_t  data   = (int32_t)(scaled / period) - KM_SDIR_PWM_START_CLOCKS;

            /* Edge-timing jitter can push a legitimate end-stop reading a
             * clock period or two past the ends of the data field, so a small
             * overshoot is clamped. Anything beyond that is a malformed frame
             * and is rejected instead — clamping it would invent an end stop. */
            if (data >= -32 && data <= KM_SDIR_PWM_DATA_STEPS + 31) {
                if (data < 0) data = 0;
                if (data > KM_SDIR_PWM_DATA_STEPS - 1) data = KM_SDIR_PWM_DATA_STEPS - 1;

                portENTER_CRITICAL_ISR(&s_mux);
                s_ring[s_ring_idx] = (uint16_t)data;
                s_ring_idx = (uint8_t)((s_ring_idx + 1) % KM_SDIR_PWM_MEDIAN_N);
                if (s_ring_fill < KM_SDIR_PWM_MEDIAN_N) s_ring_fill++;
                s_last_good_us = esp_timer_get_time();
                s_frames++;
                portEXIT_CRITICAL_ISR(&s_mux);
            } else {
                s_rejects++;
            }
        } else {
            s_rejects++;
        }
    }

    s_last_rise_tick = edata->cap_value;
    s_have_rise = true;
    s_have_high = false;
    return false;
}

/**
 * @brief  Copy the median window out and reduce it to one sample.
 * @param  out_raw  Receives the median raw value when this returns 1.
 * @return 1 if a usable angle is available, 0 otherwise.
 *
 * @note   The median is taken on the linear 0..4095 scale, not a circular one.
 *         If the magnet sits exactly on the 4095/0 wrap the median can land on
 *         the far side of it, which costs at most the sample spread — under a
 *         degree at the jitter levels measured on the bench. Mount the sensor
 *         so the wrap falls outside the steering travel and it never arises.
 */
static int8_t KM_SDIR_PWM_Median(uint16_t *out_raw)
{
    uint16_t window[KM_SDIR_PWM_MEDIAN_N];
    uint8_t  fill;
    int64_t  last_us;

    portENTER_CRITICAL(&s_mux);
    memcpy(window, s_ring, sizeof(window));
    fill    = s_ring_fill;
    last_us = s_last_good_us;
    portEXIT_CRITICAL(&s_mux);

    /* Not enough frames yet, or the newest one has aged out — say so rather
     * than serving the last value we happen to be holding. */
    if (fill < KM_SDIR_PWM_MEDIAN_N) return 0;
    if ((esp_timer_get_time() - last_us) > KM_SDIR_PWM_STALE_US) return 0;

    for (uint8_t i = 1; i < KM_SDIR_PWM_MEDIAN_N; i++) {
        uint16_t key = window[i];
        int8_t   j   = (int8_t)i - 1;
        while (j >= 0 && window[j] > key) {
            window[j + 1] = window[j];
            j--;
        }
        window[j + 1] = key;
    }

    *out_raw = window[KM_SDIR_PWM_MEDIAN_N / 2];
    return 1;
}

/******************************* FUNCIONES PÚBLICAS ***************************/

/** @brief Start MCPWM capture on the sensor pin. See km_sdir_pwm.h. */
esp_err_t KM_SDIR_PWM_Begin(gpio_num_t pin)
{
    if (s_cap_timer != NULL) return ESP_ERR_INVALID_STATE;

    mcpwm_capture_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src  = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    };
    esp_err_t ret = mcpwm_new_capture_timer(&timer_cfg, &s_cap_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "capture timer alloc failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Derive the sanity window from the timer's actual resolution. */
    uint32_t resolution_hz = 0;
    ret = mcpwm_capture_timer_get_resolution(s_cap_timer, &resolution_hz);
    if (ret != ESP_OK || resolution_hz == 0) {
        ESP_LOGE(TAG, "capture resolution unavailable: %s", esp_err_to_name(ret));
        mcpwm_del_capture_timer(s_cap_timer);
        s_cap_timer = NULL;
        return (ret == ESP_OK) ? ESP_FAIL : ret;
    }
    float f_min = KM_SDIR_PWM_NOMINAL_HZ * (1.0f - KM_SDIR_PWM_FREQ_TOL);
    float f_max = KM_SDIR_PWM_NOMINAL_HZ * (1.0f + KM_SDIR_PWM_FREQ_TOL);
    s_period_min_ticks = (uint32_t)((float)resolution_hz / f_max);
    s_period_max_ticks = (uint32_t)((float)resolution_hz / f_min);

    mcpwm_capture_channel_config_t chan_cfg = {
        .gpio_num = (int)pin,
        .prescale = 1,
        .flags = {
            .pos_edge = true,
            .neg_edge = true,
            /* No internal pull: the sensor drives this line hard through the
             * board's series resistor, and a pull would mask a disconnected
             * lead by holding the pin at a valid-looking level. */
            .pull_up   = false,
            .pull_down = false,
        },
    };
    ret = mcpwm_new_capture_channel(s_cap_timer, &chan_cfg, &s_cap_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "capture channel alloc failed: %s", esp_err_to_name(ret));
        mcpwm_del_capture_timer(s_cap_timer);
        s_cap_timer = NULL;
        return ret;
    }

    mcpwm_capture_event_callbacks_t cbs = { .on_cap = KM_SDIR_PWM_CaptureISR };
    ret = mcpwm_capture_channel_register_event_callbacks(s_cap_chan, &cbs, NULL);
    if (ret == ESP_OK) ret = mcpwm_capture_channel_enable(s_cap_chan);
    if (ret == ESP_OK) ret = mcpwm_capture_timer_enable(s_cap_timer);
    if (ret == ESP_OK) ret = mcpwm_capture_timer_start(s_cap_timer);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "capture start failed: %s", esp_err_to_name(ret));
        mcpwm_del_capture_channel(s_cap_chan);
        mcpwm_del_capture_timer(s_cap_timer);
        s_cap_chan  = NULL;
        s_cap_timer = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "capture on GPIO %d, %lu Hz clock, accept %lu-%lu ticks/frame",
             (int)pin, (unsigned long)resolution_hz,
             (unsigned long)s_period_min_ticks, (unsigned long)s_period_max_ticks);
    return ESP_OK;
}

/** @brief Median-filtered raw angle, or -1 when unknown. See km_sdir_pwm.h. */
int32_t KM_SDIR_PWM_ReadRaw(void)
{
    uint16_t raw;
    if (!KM_SDIR_PWM_Median(&raw)) return -1;
    return (int32_t)raw;
}

/** @brief Centred angle in radians, or NAN when unknown. See km_sdir_pwm.h. */
float KM_SDIR_PWM_ReadAngleRadians(void)
{
    uint16_t raw;
    if (!KM_SDIR_PWM_Median(&raw)) return NAN;

    /* Centre on the measured mechanical zero, not on the AS5600's SENSOR_CENTER
     * — that constant describes a sensor and a mounting this board no longer has.
     *
     * Wrap-safe: the difference is folded into +/- half a revolution before
     * scaling, so a zero near either end of the 0..4095 range still gives a
     * continuous angle instead of jumping a full turn as the count rolls over.
     *
     * SIGN: the raw count RISES as the wheels turn LEFT on this mounting, so the
     * difference is used as-is to make positive = left (ROS REP 103). The old
     * AS5600 code negated it, and a comment in main.c used to assert "AS5600
     * already positive=left"; that was true of the retired sensor's mounting and
     * is not true of this one. Measured on the kart 2026-07-26: straight ahead
     * 1242, full left lock 2328, a difference of +1086 counts = 95.4 degrees.
     * Negating that would have reported a hard left turn as 95 degrees RIGHT —
     * a sign error the PID would act on by steering the wrong way. */
    int32_t centered = (int32_t)raw - (int32_t)KM_SDIR_PWM_CENTER_RAW;
    if (centered >= KM_SDIR_PWM_DATA_STEPS / 2) centered -= KM_SDIR_PWM_DATA_STEPS;
    if (centered < -KM_SDIR_PWM_DATA_STEPS / 2) centered += KM_SDIR_PWM_DATA_STEPS;
    return ((float)centered / (float)KM_SDIR_PWM_DATA_STEPS) * 2.0f * (float)M_PI;
}

/** @brief Centred angle in degrees, or NAN when unknown. See km_sdir_pwm.h. */
float KM_SDIR_PWM_ReadAngleDegrees(void)
{
    float rad = KM_SDIR_PWM_ReadAngleRadians();
    if (isnan(rad)) return NAN;
    return rad * 180.0f / (float)M_PI;
}

/** @brief Whether an angle is available right now. See km_sdir_pwm.h. */
int8_t KM_SDIR_PWM_isValid(void)
{
    uint16_t raw;
    return KM_SDIR_PWM_Median(&raw);
}

/** @brief Accepted frame count. See km_sdir_pwm.h. */
uint32_t KM_SDIR_PWM_GetFrameCount(void)
{
    uint32_t v;
    portENTER_CRITICAL(&s_mux);
    v = s_frames;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

/** @brief Rejected frame count. See km_sdir_pwm.h. */
uint32_t KM_SDIR_PWM_GetRejectCount(void)
{
    return s_rejects;
}

#else  /* !SOC_MCPWM_SUPPORTED */

/* Some targets have no MCPWM peripheral and so no hardware edge timer to run
 * this on — the ESP32-C3 is the one in this repo's sdkconfig set (the ESP32,
 * S3 and C6 all have it). Those targets keep building, and every read reports
 * "no angle" — which is the truth on them, not a degraded fallback. */

esp_err_t KM_SDIR_PWM_Begin(gpio_num_t pin)
{
    (void)pin;
    ESP_LOGE(TAG, "no MCPWM on this target — steering PWM capture unavailable");
    return ESP_ERR_NOT_SUPPORTED;
}

int32_t  KM_SDIR_PWM_ReadRaw(void)           { return -1; }
float    KM_SDIR_PWM_ReadAngleRadians(void)  { return NAN; }
float    KM_SDIR_PWM_ReadAngleDegrees(void)  { return NAN; }
int8_t   KM_SDIR_PWM_isValid(void)           { return 0; }
uint32_t KM_SDIR_PWM_GetFrameCount(void)     { return 0; }
uint32_t KM_SDIR_PWM_GetRejectCount(void)    { return 0; }

#endif /* SOC_MCPWM_SUPPORTED */

/******************************* FIN DE ARCHIVO ********************************/
