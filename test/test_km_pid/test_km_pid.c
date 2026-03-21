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

    /* First call: error = 1.0 */
    advance_us(1000000);
    KM_PID_Calculate(&pid, 1.0f, 0.0f);

    /* Second call: error = 3.0, dt = 1s → derivative = (3-1)/1 = 2.0 */
    advance_us(1000000);
    float out = KM_PID_Calculate(&pid, 3.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, out);
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
}

void test_set_tunings(void) {
    PID_Controller pid = KM_PID_Init(1.0f, 2.0f, 3.0f);
    KM_PID_SetTunings(&pid, 4.0f, 5.0f, 6.0f);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, pid.kp);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, pid.ki);
    TEST_ASSERT_EQUAL_FLOAT(6.0f, pid.kd);
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
    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_set_tunings);
    RUN_TEST(test_zero_setpoint_zero_measurement);
    RUN_TEST(test_negative_error);
    RUN_TEST(test_small_dt_floor);
    return UNITY_END();
}
