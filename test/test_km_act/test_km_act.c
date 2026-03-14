/**
 * @file test_km_act.c
 * @brief Unit tests for the km_act actuator module.
 *
 * Run: pio test -e native -f test_km_act
 */
#include <unity.h>
#include <math.h>

/* Fake GPIO state — written by KM_GPIO stubs in km_gpio.h fake */
uint8_t fake_dac_value[2] = {0, 0};
uint8_t fake_pwm_duty = 0;
uint8_t fake_digital_pin19 = 0;

#include "km_act.h"
#include "km_act.c"

/* ── Helpers ─────────────────────────────────────────────────────────── */

static void reset_fakes(void) {
    fake_dac_value[0] = 0;
    fake_dac_value[1] = 0;
    fake_pwm_duty = 0;
    fake_digital_pin19 = 0;
}

/* ── Init tests ──────────────────────────────────────────────────────── */

void test_init_accel(void) {
    ACT_Controller act = KM_ACT_Init(ACT_ACCEL, 0.5f);
    TEST_ASSERT_EQUAL(ACT_ACCEL, act.type);
    TEST_ASSERT_EQUAL(0, act.dacChannel);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, act.outputLimit);
}

void test_init_brake(void) {
    ACT_Controller act = KM_ACT_Init(ACT_BRAKE, 1.0f);
    TEST_ASSERT_EQUAL(ACT_BRAKE, act.type);
    TEST_ASSERT_EQUAL(1, act.dacChannel);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, act.outputLimit);
}

void test_init_steer(void) {
    ACT_Controller act = KM_ACT_Init(ACT_STEER, 0.15f);
    TEST_ASSERT_EQUAL(ACT_STEER, act.type);
    TEST_ASSERT_EQUAL(PIN_STEER_PWM, act.pwmChannel);
    TEST_ASSERT_EQUAL(PIN_STEER_DIR, act.dirPin);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.15f, act.outputLimit);
}

void test_init_limit_clamp_above_one(void) {
    ACT_Controller act = KM_ACT_Init(ACT_ACCEL, 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, act.outputLimit);
}

void test_init_limit_clamp_below_zero(void) {
    ACT_Controller act = KM_ACT_Init(ACT_ACCEL, -0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, act.outputLimit);
}

/* ── Output limit tests (the bug that broke gears) ──────────────────── */

void test_output_limit_clamps_steering(void) {
    reset_fakes();
    ACT_Controller act = KM_ACT_Init(ACT_STEER, 0.15f);

    /* Request full power — should be clamped to 15% */
    KM_ACT_SetOutput(&act, 1.0f);
    /* 0.15 * 255 = 38.25 → 38 */
    TEST_ASSERT_UINT8_WITHIN(1, 38, fake_pwm_duty);
}

void test_output_limit_clamps_steering_negative(void) {
    reset_fakes();
    ACT_Controller act = KM_ACT_Init(ACT_STEER, 0.15f);

    KM_ACT_SetOutput(&act, -1.0f);
    TEST_ASSERT_UINT8_WITHIN(1, 38, fake_pwm_duty);
    TEST_ASSERT_EQUAL(0, fake_digital_pin19); /* direction = negative */
}

void test_set_limit_updates(void) {
    ACT_Controller act = KM_ACT_Init(ACT_STEER, 0.15f);
    KM_ACT_SetLimit(&act, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, act.outputLimit);
}

void test_set_limit_clamps_to_valid_range(void) {
    ACT_Controller act = KM_ACT_Init(ACT_STEER, 0.15f);
    KM_ACT_SetLimit(&act, 2.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, act.outputLimit);
    KM_ACT_SetLimit(&act, -0.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, act.outputLimit);
}

/* ── DAC output tests ────────────────────────────────────────────────── */

void test_accel_dac_output(void) {
    reset_fakes();
    ACT_Controller act = KM_ACT_Init(ACT_ACCEL, 1.0f);

    KM_ACT_SetOutput(&act, 0.5f);
    /* 0.5 * 255 = 127.5 → 127 */
    TEST_ASSERT_UINT8_WITHIN(1, 127, fake_dac_value[0]);
}

void test_brake_dac_output(void) {
    reset_fakes();
    ACT_Controller act = KM_ACT_Init(ACT_BRAKE, 1.0f);

    KM_ACT_SetOutput(&act, 0.75f);
    /* 0.75 * 255 = 191.25 → 191 */
    TEST_ASSERT_UINT8_WITHIN(1, 191, fake_dac_value[1]);
}

void test_accel_negative_clamped_to_zero(void) {
    reset_fakes();
    ACT_Controller act = KM_ACT_Init(ACT_ACCEL, 1.0f);

    KM_ACT_SetOutput(&act, -0.5f);
    /* Negative accel should be clamped to 0 */
    TEST_ASSERT_EQUAL_UINT8(0, fake_dac_value[0]);
}

/* ── Steering direction pin ──────────────────────────────────────────── */

void test_steer_positive_direction(void) {
    reset_fakes();
    ACT_Controller act = KM_ACT_Init(ACT_STEER, 1.0f);

    KM_ACT_SetOutput(&act, 0.5f);
    TEST_ASSERT_EQUAL(1, fake_digital_pin19); /* positive → dir = 1 */
    TEST_ASSERT_UINT8_WITHIN(1, 127, fake_pwm_duty);
}

void test_steer_negative_direction(void) {
    reset_fakes();
    ACT_Controller act = KM_ACT_Init(ACT_STEER, 1.0f);

    KM_ACT_SetOutput(&act, -0.5f);
    TEST_ASSERT_EQUAL(0, fake_digital_pin19); /* negative → dir = 0 */
    TEST_ASSERT_UINT8_WITHIN(1, 127, fake_pwm_duty);
}

/* ── Stop ────────────────────────────────────────────────────────────── */

void test_stop_zeroes_output(void) {
    reset_fakes();
    ACT_Controller act = KM_ACT_Init(ACT_STEER, 1.0f);
    KM_ACT_SetOutput(&act, 0.8f);
    TEST_ASSERT_TRUE(fake_pwm_duty > 0);

    KM_ACT_Stop(&act);
    TEST_ASSERT_EQUAL_UINT8(0, fake_pwm_duty);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, act.lastOutput);
}

void test_stop_accel(void) {
    reset_fakes();
    ACT_Controller act = KM_ACT_Init(ACT_ACCEL, 1.0f);
    KM_ACT_SetOutput(&act, 0.5f);
    KM_ACT_Stop(&act);
    TEST_ASSERT_EQUAL_UINT8(0, fake_dac_value[0]);
}

/* ── NULL safety ─────────────────────────────────────────────────────── */

void test_null_pointer_safe(void) {
    /* Should not crash */
    KM_ACT_SetOutput(NULL, 0.5f);
    KM_ACT_SetLimit(NULL, 0.5f);
    KM_ACT_Stop(NULL);
}

/* ── Runner ──────────────────────────────────────────────────────────── */

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init_accel);
    RUN_TEST(test_init_brake);
    RUN_TEST(test_init_steer);
    RUN_TEST(test_init_limit_clamp_above_one);
    RUN_TEST(test_init_limit_clamp_below_zero);
    RUN_TEST(test_output_limit_clamps_steering);
    RUN_TEST(test_output_limit_clamps_steering_negative);
    RUN_TEST(test_set_limit_updates);
    RUN_TEST(test_set_limit_clamps_to_valid_range);
    RUN_TEST(test_accel_dac_output);
    RUN_TEST(test_brake_dac_output);
    RUN_TEST(test_accel_negative_clamped_to_zero);
    RUN_TEST(test_steer_positive_direction);
    RUN_TEST(test_steer_negative_direction);
    RUN_TEST(test_stop_zeroes_output);
    RUN_TEST(test_stop_accel);
    RUN_TEST(test_null_pointer_safe);
    return UNITY_END();
}
