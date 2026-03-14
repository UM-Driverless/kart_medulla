/**
 * @file test_km_coms.c
 * @brief Unit tests for km_coms: CRC, message framing, send/receive round-trip.
 *
 * Run: pio test -e native -f test_km_coms
 */
#include <unity.h>
#include <string.h>

/* Pull in FreeRTOS fakes before km_coms.c (it uses semaphores) */
#include "freertos/FreeRTOS.h"

/* Mutable mock state */
uint8_t  fake_uart_tx_buf[512];
size_t   fake_uart_tx_len = 0;
uint8_t  fake_uart_rx_buf[512];
size_t   fake_uart_rx_len = 0;
size_t   fake_uart_rx_pos = 0;

/* km_objects source — km_coms depends on it */
#include "km_objects.h"
#include "km_objects.c"

/* km_coms source */
#include "km_coms.h"
#include "km_coms.c"

/* ── Helpers ─────────────────────────────────────────────────────────── */

static void reset_uart(void) {
    memset(fake_uart_tx_buf, 0, sizeof(fake_uart_tx_buf));
    fake_uart_tx_len = 0;
    memset(fake_uart_rx_buf, 0, sizeof(fake_uart_rx_buf));
    fake_uart_rx_len = 0;
    fake_uart_rx_pos = 0;
    rx_buffer_len = 0;
}

/* ── CRC tests ───────────────────────────────────────────────────────── */

void test_crc8_deterministic(void) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t crc1 = KM_COMS_crc8(4, 0x20, data);
    uint8_t crc2 = KM_COMS_crc8(4, 0x20, data);
    TEST_ASSERT_EQUAL_UINT8(crc1, crc2);
}

void test_crc8_different_for_different_type(void) {
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t crc1 = KM_COMS_crc8(4, 0x20, data);
    uint8_t crc2 = KM_COMS_crc8(4, 0x21, data);
    TEST_ASSERT_NOT_EQUAL(crc1, crc2);
}

void test_crc8_different_for_different_data(void) {
    uint8_t d1[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t d2[] = {0x01, 0x02, 0x03, 0x05};
    uint8_t crc1 = KM_COMS_crc8(4, 0x20, d1);
    uint8_t crc2 = KM_COMS_crc8(4, 0x20, d2);
    TEST_ASSERT_NOT_EQUAL(crc1, crc2);
}

void test_crc8_empty_payload(void) {
    uint8_t crc = KM_COMS_crc8(0, 0x08, NULL);
    /* Should not crash and should produce a deterministic value */
    uint8_t crc2 = KM_COMS_crc8(0, 0x08, NULL);
    TEST_ASSERT_EQUAL_UINT8(crc, crc2);
}

/* ── SendMsg framing ─────────────────────────────────────────────────── */

void test_send_single_int32(void) {
    reset_uart();
    int32_t data[1] = {1000};  /* 0x000003E8 */
    int ret = KM_COMS_SendMsg(ORIN_TARG_STEERING, data, 1);

    TEST_ASSERT_EQUAL(1, ret);
    /* Frame: SOM(0xAA) + LEN(4) + TYPE(0x22) + 4 payload bytes + CRC = 8 bytes */
    TEST_ASSERT_EQUAL(8, fake_uart_tx_len);
    TEST_ASSERT_EQUAL_UINT8(0xAA, fake_uart_tx_buf[0]); /* SOM */
    TEST_ASSERT_EQUAL_UINT8(4,    fake_uart_tx_buf[1]);  /* LEN = 1 * 4 bytes */
    TEST_ASSERT_EQUAL_UINT8(0x22, fake_uart_tx_buf[2]);  /* TYPE */
    /* Payload: big-endian 1000 = 0x00 0x00 0x03 0xE8 */
    TEST_ASSERT_EQUAL_UINT8(0x00, fake_uart_tx_buf[3]);
    TEST_ASSERT_EQUAL_UINT8(0x00, fake_uart_tx_buf[4]);
    TEST_ASSERT_EQUAL_UINT8(0x03, fake_uart_tx_buf[5]);
    TEST_ASSERT_EQUAL_UINT8(0xE8, fake_uart_tx_buf[6]);
}

void test_send_negative_int32(void) {
    reset_uart();
    int32_t data[1] = {-1};  /* 0xFFFFFFFF */
    int ret = KM_COMS_SendMsg(ESP_ACT_STEERING, data, 1);

    TEST_ASSERT_EQUAL(1, ret);
    TEST_ASSERT_EQUAL_UINT8(0xFF, fake_uart_tx_buf[3]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, fake_uart_tx_buf[4]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, fake_uart_tx_buf[5]);
    TEST_ASSERT_EQUAL_UINT8(0xFF, fake_uart_tx_buf[6]);
}

void test_send_too_long_payload(void) {
    reset_uart();
    int32_t data[64]; /* 64 * 4 = 256 > max */
    int ret = KM_COMS_SendMsg(ESP_HEARTBEAT, data, 64);
    TEST_ASSERT_EQUAL(-1, ret);
}

/* ── Round-trip: send → receive → process ────────────────────────────── */

void test_round_trip_steering(void) {
    reset_uart();

    /* Send a steering target */
    int32_t steer_val = 500;  /* 0.5 rad * 1000 */
    KM_COMS_SendMsg(ORIN_TARG_STEERING, &steer_val, 1);

    /* Copy TX into RX (simulating loopback) */
    memcpy(fake_uart_rx_buf, fake_uart_tx_buf, fake_uart_tx_len);
    fake_uart_rx_len = fake_uart_tx_len;
    fake_uart_rx_pos = 0;

    /* Receive + process */
    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();

    /* Check object was updated */
    int64_t obj = KM_OBJ_GetObjectValue(TARGET_STEERING);
    TEST_ASSERT_EQUAL_INT64(500, obj);
}

void test_round_trip_throttle(void) {
    reset_uart();

    int32_t thr_val = 200;
    KM_COMS_SendMsg(ORIN_TARG_THROTTLE, &thr_val, 1);

    memcpy(fake_uart_rx_buf, fake_uart_tx_buf, fake_uart_tx_len);
    fake_uart_rx_len = fake_uart_tx_len;
    fake_uart_rx_pos = 0;

    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();

    TEST_ASSERT_EQUAL_INT64(200, KM_OBJ_GetObjectValue(TARGET_THROTTLE));
}

void test_round_trip_braking(void) {
    reset_uart();

    int32_t brk_val = 128;
    KM_COMS_SendMsg(ORIN_TARG_BRAKING, &brk_val, 1);

    memcpy(fake_uart_rx_buf, fake_uart_tx_buf, fake_uart_tx_len);
    fake_uart_rx_len = fake_uart_tx_len;
    fake_uart_rx_pos = 0;

    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();

    TEST_ASSERT_EQUAL_INT64(128, KM_OBJ_GetObjectValue(TARGET_BRAKING));
}

/* ── Corrupted CRC ───────────────────────────────────────────────────── */

void test_corrupted_crc_rejected(void) {
    reset_uart();

    int32_t val = 999;
    KM_COMS_SendMsg(ORIN_TARG_STEERING, &val, 1);

    memcpy(fake_uart_rx_buf, fake_uart_tx_buf, fake_uart_tx_len);
    fake_uart_rx_len = fake_uart_tx_len;
    fake_uart_rx_pos = 0;

    /* Corrupt the CRC byte */
    fake_uart_rx_buf[fake_uart_rx_len - 1] ^= 0xFF;

    /* Set a known value first */
    KM_OBJ_SetObjectValue(TARGET_STEERING, 0);

    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();

    /* Object should NOT have been updated */
    TEST_ASSERT_EQUAL_INT64(0, KM_OBJ_GetObjectValue(TARGET_STEERING));
}

/* ── Garbage before valid message ────────────────────────────────────── */

void test_garbage_before_message(void) {
    reset_uart();

    int32_t val = 777;
    KM_COMS_SendMsg(ORIN_TARG_STEERING, &val, 1);

    /* Prepend 5 garbage bytes before the valid frame */
    uint8_t combined[512];
    combined[0] = 0x00;
    combined[1] = 0xFF;
    combined[2] = 0x55;
    combined[3] = 0x12;
    combined[4] = 0x34;
    memcpy(combined + 5, fake_uart_tx_buf, fake_uart_tx_len);

    memcpy(fake_uart_rx_buf, combined, 5 + fake_uart_tx_len);
    fake_uart_rx_len = 5 + fake_uart_tx_len;
    fake_uart_rx_pos = 0;

    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();

    TEST_ASSERT_EQUAL_INT64(777, KM_OBJ_GetObjectValue(TARGET_STEERING));
}

/* ── Multiple messages in one chunk ──────────────────────────────────── */

void test_two_messages_in_sequence(void) {
    reset_uart();

    /* Send message 1: throttle */
    int32_t thr = 100;
    KM_COMS_SendMsg(ORIN_TARG_THROTTLE, &thr, 1);
    size_t msg1_len = fake_uart_tx_len;

    /* Send message 2: braking (appended to tx buf) */
    int32_t brk = 200;
    KM_COMS_SendMsg(ORIN_TARG_BRAKING, &brk, 1);

    /* Copy both into RX */
    memcpy(fake_uart_rx_buf, fake_uart_tx_buf, fake_uart_tx_len);
    fake_uart_rx_len = fake_uart_tx_len;
    fake_uart_rx_pos = 0;

    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();

    TEST_ASSERT_EQUAL_INT64(100, KM_OBJ_GetObjectValue(TARGET_THROTTLE));
    TEST_ASSERT_EQUAL_INT64(200, KM_OBJ_GetObjectValue(TARGET_BRAKING));
}

/* ── Runner ──────────────────────────────────────────────────────────── */

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_crc8_deterministic);
    RUN_TEST(test_crc8_different_for_different_type);
    RUN_TEST(test_crc8_different_for_different_data);
    RUN_TEST(test_crc8_empty_payload);
    RUN_TEST(test_send_single_int32);
    RUN_TEST(test_send_negative_int32);
    RUN_TEST(test_send_too_long_payload);
    RUN_TEST(test_round_trip_steering);
    RUN_TEST(test_round_trip_throttle);
    RUN_TEST(test_round_trip_braking);
    RUN_TEST(test_corrupted_crc_rejected);
    RUN_TEST(test_garbage_before_message);
    RUN_TEST(test_two_messages_in_sequence);
    return UNITY_END();
}
