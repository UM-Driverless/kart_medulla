/******************************************************************************
 * @file    km_sdir_pwm.h
 * @brief   Steering-angle sensor read over its PWM output (MT6701), captured
 *          with the MCPWM capture peripheral.
 *
 * @details The kart's steering sensor is an MT6701 configured (EEPROM, so it
 *          survives a power cycle) for OUT_MODE = PWM, 994.4 Hz, high-valid.
 *          Its OUT pin lands on the medulla's CN5.2 terminal — the ex-PRESSURE_3
 *          net — which is GPIO 1 on the ESP32-S3. There is no pressure sensor
 *          fitted to that terminal.
 *
 *          This module is the production read path. It is edge-timed in
 *          hardware by MCPWM capture, so a read costs a struct copy and never
 *          blocks the caller — unlike the I2C path in km_sdir.h, where a
 *          missing sensor stalls every control cycle on a bus timeout.
 *
 *          PWM FRAME FORMAT (MT6701 datasheet rev 1.8, section 7.6 "Pulse
 *          Width Modulation (PWM) Output Mode", figure 16):
 *
 *              frame        = 4119 PWM clock periods
 *              start pattern= 16 clock periods HIGH
 *              angle data   = 12-bit, 0..4095, sent as that many further
 *                             HIGH clock periods
 *              end pattern  = at least 8 clock periods LOW
 *
 *          so   high_time / frame_time = (16 + data) / 4119
 *          and  data = duty x 4119 - 16, with data spanning 360 degrees over
 *          4096 steps (one clock period = 0.088 degrees, per the datasheet).
 *          Duty therefore ranges 0.39 % (data = 0) to 99.81 % (data = 4095) —
 *          it never reaches 0 % or 100 %, which is what makes a stuck-high or
 *          stuck-low line distinguishable from a valid reading.
 *
 * @note    VALIDITY. Every read returns NAN (or -1 for the raw form) when the
 *          angle is not currently known — no signal, wrong frame rate, or no
 *          frame decoded yet. It never falls back to a stale or invented
 *          value. See the "Sensor Validity" section of AGENTS.md for why: a
 *          sensor read that quietly substitutes a plausible number produced a
 *          confident 90-degree-left reading off an unplugged sensor on
 *          2026-07-25.
 *****************************************************************************/

#ifndef KM_SDIR_PWM_H
#define KM_SDIR_PWM_H

#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

/******************************* DEFINES PÚBLICAS *****************************/

/** @brief Total PWM clock periods in one MT6701 frame. */
#define KM_SDIR_PWM_FRAME_CLOCKS   4119

/** @brief HIGH clock periods in the frame's start pattern, before the data. */
#define KM_SDIR_PWM_START_CLOCKS   16

/** @brief Number of angle steps in the 12-bit data field (0..4095). */
#define KM_SDIR_PWM_DATA_STEPS     4096

/** @brief Frame rate the sensor is configured for (reg 0x38, PWM_FREQ = 0). */
#define KM_SDIR_PWM_NOMINAL_HZ     994.4f

/**
 * @brief Fractional tolerance on the frame rate for the period sanity check.
 *
 * A frame whose measured period falls outside +/- this fraction of
 * KM_SDIR_PWM_NOMINAL_HZ is rejected rather than decoded. This is what stops
 * unrelated edges — noise picked up on the lead, or a sensor that reverted to
 * the 497.2 Hz frame rate — from being decoded into a confident wrong angle.
 */
#define KM_SDIR_PWM_FREQ_TOL       0.25f

/**
 * @brief How long a valid frame stays usable, in microseconds.
 *
 * At 994.4 Hz a frame arrives roughly every 1006 us, so this is about 50
 * frames' worth of grace. Past it the reads report invalid.
 */
#define KM_SDIR_PWM_STALE_US       50000

/*
 * NO SMOOTHING FILTER, deliberately — there is no KM_SDIR_PWM_MEDIAN_N any more.
 * Reads return the newest decoded frame. A median over 5 frames used to sit here
 * and cost ~2 ms of group delay on the angle for every consumer; it was removed
 * on 2026-07-31 because the frame counters measured 993 accepted frames per
 * second with zero rejects, so it was filtering noise that is not there. The
 * full reasoning, and what to do if that stops being true, is in the note on the
 * private KM_SDIR_PWM_Latest() in km_sdir_pwm.c and in history.md.
 */

/**
 * @brief Raw sensor count with the road wheels pointing straight ahead.
 *
 * This is a MECHANICAL constant: it is whatever the magnet happens to read at
 * the mounted zero, so it changes any time the sensor or magnet is moved on the
 * column, and it cannot be derived — it has to be measured.
 *
 * MEASURED 2026-07-26 on the kart, wheels held straight: 1242, holding steady
 * within one count (1242-1243, about 0.09 degrees of jitter) over five samples.
 *
 * The previous value came from SENSOR_CENTER = 2250 in km_sdir.h, which belongs
 * to the retired AS5600 on a different mounting. It put straight-ahead at
 * -(1242-2250)/4096 x 2pi = 1.55 rad, and the dashboard gauge clamped that to a
 * confident "90 LEFT" while the wheels pointed forwards.
 *
 * TO RE-MEASURE: hold the wheels straight and read the raw field (field 2) of
 * the steering frame — `read_telemetry.py`, or `ros2 topic echo /esp32/steering`
 * on the Orin — then put that number here and reflash. Re-check the sign at the
 * same time (see KM_SDIR_PWM_LEFT_LOCK_RAW): remounting can reverse which way
 * the count runs, and a silent sign flip steers the kart the wrong way.
 */
#define KM_SDIR_PWM_CENTER_RAW     1242

/**
 * @brief Raw sensor count at full LEFT lock — reference, not used in the maths.
 *
 * Measured 2026-07-26 alongside the centre: 2328, steady to the count. It sits
 * ABOVE the centre, which is what fixes the sign convention: the count rises as
 * the wheels go left, so the centred difference is used unnegated to make
 * positive = left (ROS REP 103).
 *
 * 2328 - 1242 = 1086 counts = 95.4 degrees of sensor rotation at full lock. Full
 * travel therefore spans roughly 156..2328 in raw counts, nowhere near the
 * 0/4095 wrap, so the wrap-fold in the angle conversion should never trigger in
 * normal driving. If it starts triggering, the sensor has been remounted and
 * both these constants need re-measuring.
 */
#define KM_SDIR_PWM_LEFT_LOCK_RAW  2328

/******************************* FUNCIONES PÚBLICAS ***************************/

/**
 * @brief  Start capturing the sensor's PWM output on the given pin.
 * @param  pin  GPIO carrying the MT6701 OUT signal (GPIO 1 / CN5.2 on the S3).
 * @return ESP_OK on success;
 *         ESP_ERR_NOT_SUPPORTED if the target has no MCPWM peripheral;
 *         ESP_ERR_INVALID_STATE if capture is already running;
 *         otherwise the failing MCPWM driver error.
 * @note   The pin must NOT also be configured as an ADC channel — the ADC would
 *         be fighting the capture peripheral for the same pad.
 */
esp_err_t KM_SDIR_PWM_Begin(gpio_num_t pin);

/**
 * @brief  Newest raw angle decoded from the PWM frames, unfiltered.
 * @return 0..4095 (12-bit, same scale as the AS5600 raw value), or -1 when the
 *         angle is not currently known.
 */
int32_t KM_SDIR_PWM_ReadRaw(void);

/**
 * @brief  Steering angle in radians, centred and signed like the I2C path.
 * @return Angle in radians, positive = left (ROS REP 103), or NAN when the
 *         angle is not currently known.
 */
float KM_SDIR_PWM_ReadAngleRadians(void);

/**
 * @brief  Steering angle in degrees, centred and signed like the I2C path.
 * @return Angle in degrees, positive = left, or NAN when the angle is not
 *         currently known.
 */
float KM_SDIR_PWM_ReadAngleDegrees(void);

/**
 * @brief  Whether a usable angle is available right now.
 * @return 1 if at least one frame has been decoded and the newest is fresh
 *         (within KM_SDIR_PWM_STALE_US), else 0.
 */
int8_t KM_SDIR_PWM_isValid(void);

/**
 * @brief  Count of frames that passed the period sanity check since boot.
 * @return Accepted frame count (wraps at 2^32).
 */
uint32_t KM_SDIR_PWM_GetFrameCount(void);

/**
 * @brief  Count of frames rejected by the period sanity check since boot.
 * @return Rejected frame count (wraps at 2^32). A steadily rising value with a
 *         flat frame count means edges are arriving but at the wrong rate.
 */
uint32_t KM_SDIR_PWM_GetRejectCount(void);

#endif /* KM_SDIR_PWM_H */
