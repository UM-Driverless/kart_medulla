/**
 * @file test_km_pid.c
 * @brief Unit tests for the km_pid PID controller module.
 *
 * Run: pio test -e native -f test_km_pid
 */
#include <unity.h>
#include <math.h>

/* Fake timer — we control time explicitly */
uint64_t fake_esp_timer_us = 0;

#include "km_pid.h"
#include "km_pid.c"  /* compile source directly */

/* ── Helpers ─────────────────────────────────────────────────────────── */

static void advance_us(uint64_t us) { fake_esp_timer_us += us; }

/* ── Tests ───────────────────────────────────────────────────────────── */

void test_init_sets_gains(void) {
    PID_Controller pid = KM_PID_Init(1.0f, 2.0f, 3.0f);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.kp);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, pid.ki);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, pid.kd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.lastError);
}

void test_proportional_only(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(2.0f, 0.0f, 0.0f);
    KM_PID_SetOutputLimits(&pid, -10.0f, 10.0f);

    advance_us(100000); /* 100 ms */
    float out = KM_PID_Calculate(&pid, 1.0f, 0.0f);
    /* error = 1.0, kp = 2.0 → output = 2.0 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, out);
}

void test_output_clamping(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(10.0f, 0.0f, 0.0f);
    KM_PID_SetOutputLimits(&pid, -1.0f, 1.0f);

    advance_us(100000);
    float out = KM_PID_Calculate(&pid, 5.0f, 0.0f);
    /* kp * error = 50, clamped to 1.0 */
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out);
}

void test_output_clamping_negative(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(10.0f, 0.0f, 0.0f);
    KM_PID_SetOutputLimits(&pid, -1.0f, 1.0f);

    advance_us(100000);
    float out = KM_PID_Calculate(&pid, -5.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, out);
}

void test_integral_accumulation(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(0.0f, 1.0f, 0.0f);
    KM_PID_SetOutputLimits(&pid, -100.0f, 100.0f);
    KM_PID_SetIntegralLimits(&pid, -100.0f, 100.0f);

    /* Two steps of 1 second each, constant error = 2.0 */
    advance_us(1000000);
    KM_PID_Calculate(&pid, 2.0f, 0.0f);
    /* integral = 2.0 * 1.0 = 2.0 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, KM_PID_GetIntegral(&pid));

    advance_us(1000000);
    KM_PID_Calculate(&pid, 2.0f, 0.0f);
    /* integral = 2.0 + 2.0 = 4.0 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f, KM_PID_GetIntegral(&pid));
}

void test_integral_antiwindup(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(0.0f, 1.0f, 0.0f);
    KM_PID_SetOutputLimits(&pid, -100.0f, 100.0f);
    KM_PID_SetIntegralLimits(&pid, -5.0f, 5.0f);

    /* Large error for a long time — integral should clamp at 5.0 */
    advance_us(1000000);
    KM_PID_Calculate(&pid, 100.0f, 0.0f);
    advance_us(1000000);
    KM_PID_Calculate(&pid, 100.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, KM_PID_GetIntegral(&pid));
}

void test_derivative_term(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(0.0f, 0.0f, 1.0f);
    KM_PID_SetOutputLimits(&pid, -100.0f, 100.0f);

    /* First call primes the controller — no previous measurement, so no D. */
    advance_us(1000000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, KM_PID_Calculate(&pid, 0.0f, 1.0f));

    /* Second call: measurement 1.0 → 3.0 over dt = 1 s, so
     * d(measurement)/dt = 2.0 and the D term is -kd x 2.0 = -2.0.
     * Negative because the plant is already moving up. */
    advance_us(1000000);
    float out = KM_PID_Calculate(&pid, 0.0f, 3.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.0f, out);
}

/* The point of derivative-on-measurement: moving the SETPOINT must not produce
 * a derivative response. Under the previous derivative-on-error this test would
 * have read +2.0. */
void test_derivative_ignores_setpoint_step(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(0.0f, 0.0f, 1.0f);
    KM_PID_SetOutputLimits(&pid, -100.0f, 100.0f);

    advance_us(1000000);
    KM_PID_Calculate(&pid, 1.0f, 0.0f);

    /* Setpoint jumps 1.0 → 3.0; the measurement has not moved. */
    advance_us(1000000);
    float out = KM_PID_Calculate(&pid, 3.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out);
}

/* Resuming control must not differentiate the whole current measurement in one
 * dt. With kd = 1 and a measurement of 5.0 arriving 1 ms after the reset, an
 * unprimed derivative would be -5000 and would clamp to the output limit. */
void test_no_derivative_kick_after_reset(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(0.0f, 0.0f, 1.0f);
    KM_PID_SetOutputLimits(&pid, -100.0f, 100.0f);

    advance_us(1000000);
    KM_PID_Calculate(&pid, 0.0f, 5.0f);
    advance_us(1000000);
    KM_PID_Calculate(&pid, 0.0f, 5.0f);

    KM_PID_Reset(&pid);
    advance_us(1000);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, KM_PID_Calculate(&pid, 0.0f, 5.0f));
}

void test_reset_clears_state(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(1.0f, 1.0f, 1.0f);
    KM_PID_SetOutputLimits(&pid, -100.0f, 100.0f);
    KM_PID_SetIntegralLimits(&pid, -100.0f, 100.0f);

    advance_us(1000000);
    KM_PID_Calculate(&pid, 5.0f, 0.0f);

    KM_PID_Reset(&pid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.lastError);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pid.lastMeasurement);
    TEST_ASSERT_FALSE(pid.primed);
}

void test_set_tunings(void) {
    PID_Controller pid = KM_PID_Init(1.0f, 2.0f, 3.0f);
    KM_PID_SetTunings(&pid, 4.0f, 5.0f, 6.0f);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, pid.kp);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, pid.ki);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, pid.kd);
}

/* KM_PID_GetTunings used to take its floats by value with a body identical to
 * SetTunings, so it silently overwrote the gains instead of reading them, and
 * no caller could get them back out. These pin both halves of that down. */
void test_get_tunings_reads_back(void) {
    PID_Controller pid = KM_PID_Init(1.5f, 2.5f, 3.5f);
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    KM_PID_GetTunings(&pid, &kp, &ki, &kd);
    TEST_ASSERT_EQUAL_FLOAT(1.5f, kp);
    TEST_ASSERT_EQUAL_FLOAT(2.5f, ki);
    TEST_ASSERT_EQUAL_FLOAT(3.5f, kd);
}

void test_get_tunings_does_not_modify(void) {
    PID_Controller pid = KM_PID_Init(1.5f, 2.5f, 3.5f);
    float kp, ki, kd;
    KM_PID_GetTunings(&pid, &kp, &ki, &kd);
    TEST_ASSERT_EQUAL_FLOAT(1.5f, pid.kp);
    TEST_ASSERT_EQUAL_FLOAT(2.5f, pid.ki);
    TEST_ASSERT_EQUAL_FLOAT(3.5f, pid.kd);
}

void test_get_tunings_tracks_set_tunings(void) {
    PID_Controller pid = KM_PID_Init(1.0f, 2.0f, 3.0f);
    KM_PID_SetTunings(&pid, 4.0f, 5.0f, 6.0f);
    float kp, ki, kd;
    KM_PID_GetTunings(&pid, &kp, &ki, &kd);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, kp);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, ki);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, kd);
}

/* NULL outputs are skipped, so a caller wanting one gain need not invent
 * throwaway variables for the other two. */
void test_get_tunings_null_safe(void) {
    PID_Controller pid = KM_PID_Init(1.0f, 2.0f, 3.0f);
    float ki = 0.0f;
    KM_PID_GetTunings(&pid, NULL, &ki, NULL);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, ki);
    KM_PID_GetTunings(NULL, NULL, NULL, NULL);  /* must not crash */
}

void test_zero_setpoint_zero_measurement(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(2.0f, 0.0f, 0.0f);
    KM_PID_SetOutputLimits(&pid, -10.0f, 10.0f);

    advance_us(100000);
    float out = KM_PID_Calculate(&pid, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out);
}

void test_negative_error(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(2.0f, 0.0f, 0.0f);
    KM_PID_SetOutputLimits(&pid, -10.0f, 10.0f);

    advance_us(100000);
    float out = KM_PID_Calculate(&pid, 0.0f, 1.0f);
    /* error = -1.0, kp = 2.0 → output = -2.0 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -2.0f, out);
}

void test_small_dt_floor(void) {
    fake_esp_timer_us = 0;
    PID_Controller pid = KM_PID_Init(1.0f, 0.0f, 0.0f);
    KM_PID_SetOutputLimits(&pid, -10.0f, 10.0f);

    /* dt = 0 — should use minimum 1ms, not divide by zero */
    float out = KM_PID_Calculate(&pid, 1.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, out);
}

/* ── Runner ──────────────────────────────────────────────────────────── */

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init_sets_gains);
    RUN_TEST(test_proportional_only);
    RUN_TEST(test_output_clamping);
    RUN_TEST(test_output_clamping_negative);
    RUN_TEST(test_integral_accumulation);
    RUN_TEST(test_integral_antiwindup);
    RUN_TEST(test_derivative_term);
    RUN_TEST(test_derivative_ignores_setpoint_step);
    RUN_TEST(test_no_derivative_kick_after_reset);
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_set_tunings);
    RUN_TEST(test_get_tunings_reads_back);
    RUN_TEST(test_get_tunings_does_not_modify);
    RUN_TEST(test_get_tunings_tracks_set_tunings);
    RUN_TEST(test_get_tunings_null_safe);
    RUN_TEST(test_zero_setpoint_zero_measurement);
    RUN_TEST(test_negative_error);
    RUN_TEST(test_small_dt_floor);
    return UNITY_END();
}
