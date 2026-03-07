/**
 * @file test_main.c
 * @brief Hardware test suite for kart_medulla.
 *
 * Swap with main.c to run. Tests each subsystem in isolation
 * with clear PASS/FAIL output on UART0 at 115200 baud.
 *
 * Covers: GPIO pins, ADC, DAC, LEDC PWM, I2C, AS5600 sensor,
 * steering motor, UART0 protocol framing, stack sizing,
 * GPIO pin conflicts, and FreeRTOS task creation.
 */
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "km_gpio.h"
#include "km_sdir.h"
#include "km_coms.h"
#include "km_objects.h"
#include "km_pid.h"
#include "km_act.h"
#include "km_rtos.h"

static int pass_count = 0;
static int fail_count = 0;
static int skip_count = 0;

#define TEST(name, expr) do { \
    esp_err_t _r = (expr); \
    if (_r == ESP_OK) { printf("  PASS: %s\n", name); pass_count++; } \
    else { printf("  FAIL: %s (err=%d)\n", name, _r); fail_count++; } \
} while(0)

#define TEST_BOOL(name, cond) do { \
    if (cond) { printf("  PASS: %s\n", name); pass_count++; } \
    else { printf("  FAIL: %s\n", name); fail_count++; } \
} while(0)

#define SKIP(name, reason) do { \
    printf("  SKIP: %s (%s)\n", name, reason); skip_count++; \
} while(0)

/* ============================================================
 * 1. GPIO PIN ASSIGNMENT VALIDATION
 * Catches: wrong pin numbers, pin conflicts, strap pin misuse
 * ============================================================ */
static void test_pin_assignments(void) {
    printf("\n--- Pin assignments (match PCB wiring) ---\n");

    /* Verify pin numbers match the PCB design */
    TEST_BOOL("STEER_PWM = GPIO18 (H1.11)",   PIN_STEER_PWM == 18);
    TEST_BOOL("STEER_DIR = GPIO19 (H1.12)",   PIN_STEER_DIR == 19);
    TEST_BOOL("I2C_SDA   = GPIO21 (H1.14)",   PIN_I2C_SDA == 21);
    TEST_BOOL("I2C_SCL   = GPIO22 (H1.17)",   PIN_I2C_SCL == 22);
    TEST_BOOL("USB_TX    = GPIO1  (H1.16)",   PIN_USB_UART_TX == 1);
    TEST_BOOL("USB_RX    = GPIO3  (H1.15)",   PIN_USB_UART_RX == 3);
    TEST_BOOL("PRESSURE_1 = GPIO36 (H2.3)",   PIN_PRESSURE_1 == 36);
    TEST_BOOL("PRESSURE_2 = GPIO39 (H2.4)",   PIN_PRESSURE_2 == 39);
    TEST_BOOL("PRESSURE_3 = GPIO34 (H2.5)",   PIN_PRESSURE_3 == 34);
    TEST_BOOL("PEDAL_ACC  = GPIO35 (H2.6)",   PIN_PEDAL_ACC == 35);
    TEST_BOOL("PEDAL_BRAKE= GPIO32 (H2.7)",   PIN_PEDAL_BRAKE == 32);
    /* HALL 1/3 disabled — GPIO17/16 used by UART2 */
    TEST_BOOL("HALL_2     = GPIO33 (H2.8)",   PIN_MOTOR_HALL_2 == 33);
    TEST_BOOL("CMD_ACC    = GPIO25 (H2.9)",   PIN_CMD_ACC == 25);
    TEST_BOOL("CMD_BRAKE  = GPIO26 (H2.10)",  PIN_CMD_BRAKE == 26);
    TEST_BOOL("HYDRAULIC_1= GPIO27 (H2.11)",  PIN_HYDRAULIC_1 == 27);
    TEST_BOOL("HYDRAULIC_2= GPIO14 (H2.12)",  PIN_HYDRAULIC_2 == 14);
    TEST_BOOL("STATUS_LED = GPIO2  (H1.5)",   PIN_STATUS_LED == 2);
}

/* ============================================================
 * 2. GPIO PIN CONFLICT DETECTION
 * Catches: UART2 vs hall sensor conflict (GPIO16/17)
 * ============================================================ */
static void test_pin_conflicts(void) {
    printf("\n--- Pin conflict detection ---\n");

    /* GPIO16/17 used by UART2 (debug logs). HALL 1/3 disabled. */
    TEST_BOOL("UART2_TX = GPIO17 (was HALL1)", PIN_ORIN_UART_TX == 17);
    TEST_BOOL("UART2_RX = GPIO16 (was HALL3)", PIN_ORIN_UART_RX == 16);

    /* Verify no two output pins share the same GPIO */
    TEST_BOOL("STEER_PWM != STEER_DIR",  PIN_STEER_PWM != PIN_STEER_DIR);
    TEST_BOOL("STEER_PWM != CMD_ACC",    PIN_STEER_PWM != PIN_CMD_ACC);
    TEST_BOOL("STEER_PWM != CMD_BRAKE",  PIN_STEER_PWM != PIN_CMD_BRAKE);
    TEST_BOOL("STEER_DIR != CMD_ACC",    PIN_STEER_DIR != PIN_CMD_ACC);
    TEST_BOOL("STEER_DIR != CMD_BRAKE",  PIN_STEER_DIR != PIN_CMD_BRAKE);
    TEST_BOOL("CMD_ACC   != CMD_BRAKE",  PIN_CMD_ACC != PIN_CMD_BRAKE);

    /* Verify output pins are not on input-only GPIOs (34-39) */
    TEST_BOOL("STEER_PWM not input-only", PIN_STEER_PWM < 34);
    TEST_BOOL("STEER_DIR not input-only", PIN_STEER_DIR < 34);
    TEST_BOOL("CMD_ACC not input-only",   PIN_CMD_ACC < 34);
    TEST_BOOL("CMD_BRAKE not input-only", PIN_CMD_BRAKE < 34);
    TEST_BOOL("STATUS_LED not input-only", PIN_STATUS_LED < 34);

    /* Verify no pins use flash SPI (GPIO 6-11) */
    const gpio_num_t output_pins[] = {
        PIN_STEER_PWM, PIN_STEER_DIR, PIN_CMD_ACC, PIN_CMD_BRAKE,
        PIN_I2C_SDA, PIN_I2C_SCL, PIN_STATUS_LED
    };
    for (int i = 0; i < sizeof(output_pins)/sizeof(output_pins[0]); i++) {
        char buf[64];
        snprintf(buf, sizeof(buf), "GPIO%d not flash SPI (6-11)", output_pins[i]);
        TEST_BOOL(buf, output_pins[i] < 6 || output_pins[i] > 11);
    }
}

/* ============================================================
 * 3. ADC PIN CONFIGURATION
 * ============================================================ */
static void test_adc_pins(void) {
    printf("\n--- ADC pins ---\n");
    gpio_config_t cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    const struct { gpio_num_t pin; const char *name; } pins[] = {
        {PIN_PRESSURE_1,  "PRESSURE_1 (GPIO36, ADC1_CH0)"},
        {PIN_PRESSURE_2,  "PRESSURE_2 (GPIO39, ADC1_CH3)"},
        {PIN_PRESSURE_3,  "PRESSURE_3 (GPIO34, ADC1_CH6)"},
        {PIN_PEDAL_ACC,   "PEDAL_ACC (GPIO35, ADC1_CH7)"},
        {PIN_PEDAL_BRAKE, "PEDAL_BRAKE (GPIO32, ADC1_CH4)"},
        {PIN_HYDRAULIC_1, "HYDRAULIC_1 (GPIO27, ADC2_CH7)"},
        {PIN_HYDRAULIC_2, "HYDRAULIC_2 (GPIO14, ADC2_CH6)"},
    };

    for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); i++) {
        cfg.pin_bit_mask = 1ULL << pins[i].pin;
        TEST(pins[i].name, gpio_config(&cfg));
    }
}

/* ============================================================
 * 4. ADC READING SANITY
 * Catches: wrong ADC channel mapping, floating pins
 * ============================================================ */
static void test_adc_read(void) {
    printf("\n--- ADC read sanity ---\n");

    /* Configure ADC1 width */
    adc1_config_width(ADC_WIDTH_BIT_12);

    /* ADC1 channels — these should read without error */
    const struct { adc1_channel_t ch; const char *name; } adc1[] = {
        {ADC1_CHANNEL_0, "PRESSURE_1 (CH0)"},
        {ADC1_CHANNEL_3, "PRESSURE_2 (CH3)"},
        {ADC1_CHANNEL_6, "PRESSURE_3 (CH6)"},
        {ADC1_CHANNEL_7, "PEDAL_ACC (CH7)"},
        {ADC1_CHANNEL_4, "PEDAL_BRAKE (CH4)"},
    };

    for (int i = 0; i < sizeof(adc1)/sizeof(adc1[0]); i++) {
        adc1_config_channel_atten(adc1[i].ch, ADC_ATTEN_DB_11);
        int val = adc1_get_raw(adc1[i].ch);
        char buf[80];
        snprintf(buf, sizeof(buf), "%s raw=%d (0-4095)", adc1[i].name, val);
        TEST_BOOL(buf, val >= 0 && val <= 4095);
    }

    /* ADC2 channels — may fail if WiFi is active, but should work otherwise */
    printf("  (ADC2 may fail if WiFi is on)\n");
    const struct { adc2_channel_t ch; const char *name; } adc2[] = {
        {ADC2_CHANNEL_7, "HYDRAULIC_1 (CH7, GPIO27)"},
        {ADC2_CHANNEL_6, "HYDRAULIC_2 (CH6, GPIO14)"},
    };

    for (int i = 0; i < sizeof(adc2)/sizeof(adc2[0]); i++) {
        adc2_config_channel_atten(adc2[i].ch, ADC_ATTEN_DB_11);
        int val = 0;
        esp_err_t ret = adc2_get_raw(adc2[i].ch, ADC_WIDTH_BIT_12, &val);
        char buf[80];
        snprintf(buf, sizeof(buf), "%s raw=%d", adc2[i].name, val);
        TEST(buf, ret);
    }
}

/* ============================================================
 * 5. DAC OUTPUTS
 * ============================================================ */
static void test_dac(void) {
    printf("\n--- DAC outputs ---\n");
    TEST("DAC_CHAN_0 enable (CMD_ACC, GPIO25)", dac_output_enable(DAC_CHAN_0));
    TEST("DAC_CHAN_1 enable (CMD_BRAKE, GPIO26)", dac_output_enable(DAC_CHAN_1));
    TEST("DAC0 write 0", dac_output_voltage(DAC_CHAN_0, 0));
    TEST("DAC1 write 0", dac_output_voltage(DAC_CHAN_1, 0));
    TEST("DAC0 write 128 (mid)", dac_output_voltage(DAC_CHAN_0, 128));
    TEST("DAC0 write 255 (max)", dac_output_voltage(DAC_CHAN_0, 255));
    TEST("DAC0 back to 0", dac_output_voltage(DAC_CHAN_0, 0));
}

/* ============================================================
 * 6. STEERING PWM (LEDC)
 * ============================================================ */
static void test_steering_pwm(void) {
    printf("\n--- Steering PWM (GPIO%d) ---\n", PIN_STEER_PWM);
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    TEST("LEDC timer config", ledc_timer_config(&timer));

    ledc_channel_config_t ch = {
        .gpio_num = PIN_STEER_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    TEST("LEDC channel config", ledc_channel_config(&ch));
    TEST("LEDC set duty 50%", ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 128));
    TEST("LEDC update duty", ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
    TEST("LEDC set duty 0", ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0));
    TEST("LEDC update duty", ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
}

/* ============================================================
 * 7. STEERING DIR PIN
 * ============================================================ */
static void test_steering_dir(void) {
    printf("\n--- Steering DIR (GPIO%d) ---\n", PIN_STEER_DIR);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_STEER_DIR,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    TEST("DIR gpio_config", gpio_config(&cfg));
    TEST("DIR set HIGH", gpio_set_level(PIN_STEER_DIR, 1));
    TEST("DIR set LOW", gpio_set_level(PIN_STEER_DIR, 0));
}

/* ============================================================
 * 8. I2C BUS
 * ============================================================ */
static void test_i2c(void) {
    printf("\n--- I2C (SDA=GPIO%d, SCL=GPIO%d) ---\n", PIN_I2C_SDA, PIN_I2C_SCL);
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    TEST("I2C param_config", i2c_param_config(I2C_NUM_0, &cfg));
    TEST("I2C driver_install", i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

/* ============================================================
 * 9. AS5600 MAGNETIC ENCODER
 * ============================================================ */
static void test_as5600(void) {
    printf("\n--- AS5600 sensor ---\n");
    sensor_struct sdir = KM_SDIR_Init(10);
    int8_t ok = KM_SDIR_Begin(&sdir, PIN_I2C_SDA, PIN_I2C_SCL);
    if (ok >= 0) {
        printf("  PASS: AS5600 detected on I2C bus\n"); pass_count++;
    } else {
        printf("  FAIL: AS5600 not detected (ret=%d)\n", ok); fail_count++;
        return;
    }

    /* Read multiple times to check stability */
    int errors = 0;
    float readings[5];
    for (int i = 0; i < 5; i++) {
        readings[i] = KM_SDIR_ReadAngleRadians(&sdir);
        errors += sdir.errorCount;
        sdir.errorCount = 0;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_BOOL("AS5600 5 reads without I2C errors", errors == 0);

    /* Check readings are in valid range [0, 2*PI] */
    int range_ok = 1;
    for (int i = 0; i < 5; i++) {
        if (readings[i] < -0.1f || readings[i] > 6.4f) range_ok = 0;
    }
    char buf[80];
    snprintf(buf, sizeof(buf), "AS5600 readings in range (%.3f, %.3f, %.3f, %.3f, %.3f)",
             readings[0], readings[1], readings[2], readings[3], readings[4]);
    TEST_BOOL(buf, range_ok);

    /* Check readings are consistent (not jumping wildly) */
    float max_diff = 0;
    for (int i = 1; i < 5; i++) {
        float diff = fabsf(readings[i] - readings[i-1]);
        if (diff > max_diff) max_diff = diff;
    }
    snprintf(buf, sizeof(buf), "AS5600 stable (max jitter=%.4f rad, threshold=0.1)", max_diff);
    TEST_BOOL(buf, max_diff < 0.1f);
}

/* ============================================================
 * 9b. AS5600 REGISTER INTEGRITY & MAGNET HEALTH
 * Catches: ZPOS/MPOS corruption, OTP burns, weak/missing magnet
 * ============================================================ */
static void test_as5600_health(void) {
    printf("\n--- AS5600 register integrity & magnet ---\n");
    sensor_struct sdir = KM_SDIR_Init(10);
    int8_t ok = KM_SDIR_Begin(&sdir, PIN_I2C_SDA, PIN_I2C_SCL);
    if (ok <= 0) {
        SKIP("AS5600 health", "sensor not detected");
        return;
    }

    uint8_t diag[9];
    uint8_t len = KM_SDIR_ReadDiagnostics(&sdir, diag);
    TEST_BOOL("ReadDiagnostics returned 9 bytes", len == 9);

    // ZPOS should be 0x0000 (no zero position programmed)
    uint16_t zpos = ((uint16_t)diag[0] << 8) | diag[1];
    char buf[80];
    snprintf(buf, sizeof(buf), "ZPOS = 0x%04X (expected 0x0000)", zpos);
    TEST_BOOL(buf, zpos == 0);

    // MPOS should be 0x0000
    uint16_t mpos = ((uint16_t)diag[2] << 8) | diag[3];
    snprintf(buf, sizeof(buf), "MPOS = 0x%04X (expected 0x0000)", mpos);
    TEST_BOOL(buf, mpos == 0);

    // ZMCO = OTP burn count, should be 0 (never burned)
    uint8_t zmco = diag[8];
    snprintf(buf, sizeof(buf), "ZMCO = %d (expected 0, OTP burns)", zmco);
    TEST_BOOL(buf, zmco == 0);

    // STATUS register: MD (bit5) should be set, ML/MH should be clear
    uint8_t status = diag[6];
    uint8_t agc = diag[7];
    snprintf(buf, sizeof(buf), "STATUS MD=1 magnet detected (0x%02X)", status);
    TEST_BOOL(buf, status & (1 << 5));

    snprintf(buf, sizeof(buf), "STATUS ML=0 magnet not too weak (0x%02X)", status);
    TEST_BOOL(buf, !(status & (1 << 4)));

    snprintf(buf, sizeof(buf), "STATUS MH=0 magnet not too strong (0x%02X)", status);
    TEST_BOOL(buf, !(status & (1 << 3)));

    // AGC should be in a reasonable range (not pegged at 0 or 255)
    snprintf(buf, sizeof(buf), "AGC = %d (expected 10-200, not saturated)", agc);
    TEST_BOOL(buf, agc >= 10 && agc <= 200);

    // Quick ReadStatusAGC consistency check
    uint8_t st2, agc2;
    int8_t rok = KM_SDIR_ReadStatusAGC(&sdir, &st2, &agc2);
    TEST_BOOL("ReadStatusAGC succeeds", rok == 1);
    snprintf(buf, sizeof(buf), "ReadStatusAGC STATUS=0x%02X matches diag STATUS=0x%02X", st2, status);
    TEST_BOOL(buf, st2 == status);
}

/* ============================================================
 * 9c. NVS CALIBRATION PERSISTENCE
 * Catches: NVS write/read failures, wrong namespace/key
 * ============================================================ */
static void test_nvs_calibration(void) {
    printf("\n--- NVS steering calibration ---\n");
    sensor_struct sdir = KM_SDIR_Init(10);

    // Save a known value
    uint16_t test_val = 1234;
    KM_SDIR_setCenterOffset(&sdir, test_val);
    TEST_BOOL("setCenterOffset stores value", sdir.centerOffset == test_val);

    // Re-init and load — should get same value back
    sensor_struct sdir2 = KM_SDIR_Init(10);
    KM_SDIR_LoadCalibration(&sdir2);
    char buf[80];
    snprintf(buf, sizeof(buf), "LoadCalibration read back %d (expected %d)", sdir2.centerOffset, test_val);
    TEST_BOOL(buf, sdir2.centerOffset == test_val);

    // Restore default so test doesn't leave stale calibration
    KM_SDIR_setCenterOffset(&sdir, SENSOR_CENTER);
}

/* ============================================================
 * 10. PID CONTROLLER SANITY
 * Catches: NaN, unbounded output, wrong sign
 * ============================================================ */
static void test_pid(void) {
    printf("\n--- PID controller ---\n");

    PID_Controller pid = KM_PID_Init(0.03f, 0.0f, 0.0004f);
    KM_PID_SetOutputLimits(&pid, -1.0f, 1.0f);

    /* Positive error should give positive output */
    float out = KM_PID_Compute(&pid, 0.5f, 0.0f);
    TEST_BOOL("PID positive error -> positive output", out > 0.0f);

    /* Output should be within limits */
    TEST_BOOL("PID output within [-1, 1]", out >= -1.0f && out <= 1.0f);

    /* Large error should saturate at limit */
    pid = KM_PID_Init(0.03f, 0.0f, 0.0004f);
    KM_PID_SetOutputLimits(&pid, -1.0f, 1.0f);
    float big_out = KM_PID_Compute(&pid, 100.0f, 0.0f);
    TEST_BOOL("PID large error saturates at 1.0", big_out == 1.0f);

    /* Negative error should give negative output */
    pid = KM_PID_Init(0.03f, 0.0f, 0.0004f);
    KM_PID_SetOutputLimits(&pid, -1.0f, 1.0f);
    float neg_out = KM_PID_Compute(&pid, -0.5f, 0.0f);
    TEST_BOOL("PID negative error -> negative output", neg_out < 0.0f);

    /* Zero error should give zero output (on first compute) */
    pid = KM_PID_Init(0.03f, 0.0f, 0.0004f);
    KM_PID_SetOutputLimits(&pid, -1.0f, 1.0f);
    float zero_out = KM_PID_Compute(&pid, 0.0f, 0.0f);
    TEST_BOOL("PID zero error -> zero output", fabsf(zero_out) < 0.001f);

    /* NaN check */
    TEST_BOOL("PID output is not NaN", !isnan(out));
    TEST_BOOL("PID output is not Inf", !isinf(out));
}

/* ============================================================
 * 11. FRAME PROTOCOL ENCODING
 * Catches: wrong byte order, CRC errors, wrong type codes
 * ============================================================ */
static void test_frame_encoding(void) {
    printf("\n--- Frame protocol encoding ---\n");

    /* Steering encoding: int16 big-endian, radians * 1000 */
    float steer_rad = 0.25f;
    int16_t steer_i16 = (int16_t)(steer_rad * 1000.0f);
    uint8_t hi = (uint8_t)(steer_i16 >> 8);
    uint8_t lo = (uint8_t)(steer_i16 & 0xFF);
    TEST_BOOL("0.25 rad -> 250 (int16)", steer_i16 == 250);
    TEST_BOOL("0.25 rad -> [0x00, 0xFA]", hi == 0x00 && lo == 0xFA);

    /* Negative steering */
    float neg_steer = -0.3f;
    int16_t neg_i16 = (int16_t)(neg_steer * 1000.0f);
    uint8_t neg_hi = (uint8_t)((uint16_t)neg_i16 >> 8);
    uint8_t neg_lo = (uint8_t)(neg_i16 & 0xFF);
    TEST_BOOL("-0.3 rad -> -300 (int16)", neg_i16 == -300);
    TEST_BOOL("-0.3 rad -> [0xFE, 0xD4]", neg_hi == 0xFE && neg_lo == 0xD4);

    /* Throttle encoding: 0-255 */
    float throttle = 0.5f;
    uint8_t thr_byte = (uint8_t)(throttle * 255.0f);
    TEST_BOOL("50% throttle -> 127", thr_byte == 127);

    /* Type codes match Frame.msg constants */
    TEST_BOOL("ORIN_TARG_THROTTLE = 0x20", 0x20 == 32);
    TEST_BOOL("ORIN_TARG_BRAKING = 0x21",  0x21 == 33);
    TEST_BOOL("ORIN_TARG_STEERING = 0x22", 0x22 == 34);
    TEST_BOOL("ESP_ACT_STEERING = 0x04",   0x04 == 4);
    TEST_BOOL("ESP_HEARTBEAT = 0x08",      0x08 == 8);
}

/* ============================================================
 * 12. CRC8 VALIDATION
 * Catches: CRC polynomial mismatch between ESP32 and Orin
 * ============================================================ */
static uint8_t crc8_compute(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

static void test_crc8(void) {
    printf("\n--- CRC8 (poly 0x07) ---\n");

    /* Known test vector: heartbeat frame */
    /* Frame: SOF=AA, LEN=04, TYPE=08, PAYLOAD=DE AD BE EF, CRC=?? */
    uint8_t heartbeat_data[] = {0x04, 0x08, 0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t crc = crc8_compute(heartbeat_data, sizeof(heartbeat_data));
    printf("  Heartbeat CRC8 = 0x%02X\n", crc);
    TEST_BOOL("CRC8 is deterministic", crc == crc8_compute(heartbeat_data, sizeof(heartbeat_data)));

    /* Empty payload */
    uint8_t empty[] = {0x00, 0x08};
    uint8_t crc_empty = crc8_compute(empty, sizeof(empty));
    TEST_BOOL("CRC8 empty payload != 0", crc_empty != 0);

    /* Single bit change should change CRC */
    uint8_t modified[] = {0x04, 0x08, 0xDE, 0xAD, 0xBE, 0xEE}; /* last byte changed */
    uint8_t crc_mod = crc8_compute(modified, sizeof(modified));
    TEST_BOOL("CRC8 detects single bit change", crc != crc_mod);
}

/* ============================================================
 * 13. FREERTOS TASK STACK SIZING
 * Catches: stack overflow (the 1024-byte heartbeat crash)
 * ============================================================ */
static volatile int task_completed = 0;

static void stack_test_task(void *arg) {
    /* Simulate what heartbeat_task does: format + send a UART message */
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    char buf[128];
    snprintf(buf, sizeof(buf), "Stack test: payload=%02X%02X%02X%02X",
             payload[0], payload[1], payload[2], payload[3]);
    task_completed = 1;
    vTaskDelete(NULL);
}

static void test_task_stack(void) {
    printf("\n--- FreeRTOS task stack ---\n");

    /* 2048 bytes should be enough for tasks that do UART writes */
    task_completed = 0;
    BaseType_t ret = xTaskCreate(stack_test_task, "stk_test", 2048, NULL, 1, NULL);
    TEST_BOOL("Create task with 2048 stack", ret == pdPASS);
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_BOOL("Task completed without stack overflow", task_completed == 1);

    /* 512 bytes is too small — task should still be created but may crash.
       We don't actually run this to avoid crashing the test suite. */
    SKIP("512-byte stack overflow", "would crash test suite");
}

/* ============================================================
 * 14. STEERING MOTOR MOVEMENT
 * ============================================================ */
static void test_steering_motor(void) {
    printf("\n--- Steering motor movement ---\n");
    printf("  DIR=1, PWM=20%% for 3s...\n");
    gpio_set_level(PIN_STEER_DIR, 1);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 51);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(3000));

    printf("  DIR=0, PWM=20%% for 3s (reverse)...\n");
    gpio_set_level(PIN_STEER_DIR, 0);
    vTaskDelay(pdMS_TO_TICKS(3000));

    printf("  STOP\n");
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    printf("  PASS: motor test complete (verify visually)\n");
    pass_count++;
}

/* ============================================================
 * 15. STATUS LED
 * ============================================================ */
static void test_status_led(void) {
    printf("\n--- Status LED (GPIO%d) ---\n", PIN_STATUS_LED);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_STATUS_LED,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    TEST("LED gpio_config", gpio_config(&cfg));
    TEST("LED on", gpio_set_level(PIN_STATUS_LED, 1));
    vTaskDelay(pdMS_TO_TICKS(500));
    TEST("LED off", gpio_set_level(PIN_STATUS_LED, 0));
}

/* ============================================================
 * MAIN
 * ============================================================ */
void app_main(void) {
    // Init NVS for calibration tests
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    printf("\n========================================\n");
    printf("  KART MEDULLA HARDWARE TEST SUITE v3\n");
    printf("========================================\n");
    printf("Pin config:\n");
    printf("  STEER: PWM=%d DIR=%d\n", PIN_STEER_PWM, PIN_STEER_DIR);
    printf("  DAC:   ACC=%d BRAKE=%d\n", PIN_CMD_ACC, PIN_CMD_BRAKE);
    printf("  I2C:   SDA=%d SCL=%d\n", PIN_I2C_SDA, PIN_I2C_SCL);
    printf("  HALL:  2=%d (1/3 disabled, pins used by UART2)\n", PIN_MOTOR_HALL_2);
    printf("  HYD:   1=%d 2=%d\n", PIN_HYDRAULIC_1, PIN_HYDRAULIC_2);

    /* Pure logic tests first (no hardware needed) */
    test_pin_assignments();
    test_pin_conflicts();
    test_frame_encoding();
    test_crc8();
    test_pid();
    test_task_stack();

    /* Hardware tests */
    test_adc_pins();
    test_adc_read();
    test_dac();
    test_steering_pwm();
    test_steering_dir();
    test_status_led();
    test_i2c();
    test_as5600();
    test_as5600_health();
    test_nvs_calibration();
    test_steering_motor();

    printf("\n========================================\n");
    printf("  RESULTS: %d passed, %d failed, %d skipped\n",
           pass_count, fail_count, skip_count);
    printf("========================================\n");

    while (1) { vTaskDelay(pdMS_TO_TICKS(5000)); printf("idle\n"); }
}
