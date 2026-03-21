/**
 * @file test_km_objects.c
 * @brief Unit tests for the km_objects shared-state dictionary.
 *
 * Run: pio test -e native -f test_km_objects
 */
#include <unity.h>

#include "km_objects.h"
#include "km_objects.c"

/* ── Tests ───────────────────────────────────────────────────────────── */

void test_set_and_get(void) {
    KM_OBJ_SetObjectValue(TARGET_THROTTLE, 42);
    TEST_ASSERT_EQUAL_INT64(42, KM_OBJ_GetObjectValue(TARGET_THROTTLE));
}

void test_set_all_objects(void) {
    for (int i = 0; i < KM_OBJ_LAST; i++) {
        TEST_ASSERT_EQUAL_UINT8(1, KM_OBJ_SetObjectValue((km_objects_t)i, i * 100));
    }
    for (int i = 0; i < KM_OBJ_LAST; i++) {
        TEST_ASSERT_EQUAL_INT64(i * 100, KM_OBJ_GetObjectValue((km_objects_t)i));
    }
}

void test_overwrite(void) {
    KM_OBJ_SetObjectValue(ACTUAL_STEERING, 1000);
    KM_OBJ_SetObjectValue(ACTUAL_STEERING, -500);
    TEST_ASSERT_EQUAL_INT64(-500, KM_OBJ_GetObjectValue(ACTUAL_STEERING));
}

void test_out_of_range_set(void) {
    TEST_ASSERT_EQUAL_UINT8(0, KM_OBJ_SetObjectValue(KM_OBJ_LAST, 99));
    TEST_ASSERT_EQUAL_UINT8(0, KM_OBJ_SetObjectValue((km_objects_t)255, 99));
}

void test_out_of_range_get(void) {
    TEST_ASSERT_EQUAL_INT64(OBJECT_VALUE_ERROR, KM_OBJ_GetObjectValue(KM_OBJ_LAST));
}

void test_negative_values(void) {
    KM_OBJ_SetObjectValue(TARGET_STEERING, -32768);
    TEST_ASSERT_EQUAL_INT64(-32768, KM_OBJ_GetObjectValue(TARGET_STEERING));
}

void test_large_values(void) {
    int64_t big = 2147483647LL; /* INT32_MAX */
    KM_OBJ_SetObjectValue(ACTUAL_SPEED, big);
    TEST_ASSERT_EQUAL_INT64(big, KM_OBJ_GetObjectValue(ACTUAL_SPEED));
}

/* ── Runner ──────────────────────────────────────────────────────────── */

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_set_and_get);
    RUN_TEST(test_set_all_objects);
    RUN_TEST(test_overwrite);
    RUN_TEST(test_out_of_range_set);
    RUN_TEST(test_out_of_range_get);
    RUN_TEST(test_negative_values);
    RUN_TEST(test_large_values);
    return UNITY_END();
}
