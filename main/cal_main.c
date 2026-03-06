#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "km_gpio.h"
#include "km_sdir.h"
#include <stdio.h>

void app_main(void) {
    printf("\n=== STEERING CALIBRATION ===\n");

    // Init NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Setup PWM
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .gpio_num = PIN_STEER_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&ch);

    gpio_config_t dir_cfg = {
        .pin_bit_mask = 1ULL << PIN_STEER_DIR,
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&dir_cfg);

    // Init AS5600
    sensor_struct sdir = KM_SDIR_Init(10);
    KM_SDIR_Begin(&sdir, PIN_I2C_SDA, PIN_I2C_SCL);

    // Drive LEFT at 50% for 4s
    printf("Driving LEFT for 4s...\n");
    gpio_set_level(PIN_STEER_DIR, 0);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(4000));

    // Record left limit
    uint32_t sum = 0;
    int n = 20;
    for (int i = 0; i < n; i++) {
        sum += KM_SDIR_ReadRaw(&sdir);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    uint16_t left_raw = sum / n;

    // Stop
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Drive RIGHT at 50% for 4s
    printf("Driving RIGHT for 4s...\n");
    gpio_set_level(PIN_STEER_DIR, 1);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(4000));

    // Record right limit
    sum = 0;
    for (int i = 0; i < n; i++) {
        sum += KM_SDIR_ReadRaw(&sdir);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    uint16_t right_raw = sum / n;

    // Stop motor
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    // Compute center handling wraparound
    uint16_t center;
    uint16_t travel;
    if (right_raw >= left_raw) {
        travel = right_raw - left_raw;
        if (travel <= 2048) {
            center = left_raw + travel / 2;
        } else {
            travel = 4096 - travel;
            center = (right_raw + travel / 2) % 4096;
        }
    } else {
        travel = left_raw - right_raw;
        if (travel <= 2048) {
            center = right_raw + travel / 2;
        } else {
            travel = 4096 - travel;
            center = (left_raw + travel / 2) % 4096;
        }
    }

    // Save to NVS
    nvs_handle_t nvs;
    nvs_open("steering", NVS_READWRITE, &nvs);
    nvs_set_u16(nvs, "left_raw", left_raw);
    nvs_set_u16(nvs, "right_raw", right_raw);
    nvs_set_u16(nvs, "center", center);
    nvs_set_u16(nvs, "travel", travel);
    nvs_commit(nvs);
    nvs_close(nvs);

    printf("\n=============================\n");
    printf("LEFT   = %u\n", left_raw);
    printf("RIGHT  = %u\n", right_raw);
    printf("CENTER = %u  (saved to NVS)\n", center);
    printf("TRAVEL = %u counts (%.1f deg)\n", travel, travel * 360.0f / 4096.0f);
    printf("=============================\n");

    // Return toward center
    gpio_set_level(PIN_STEER_DIR, 0);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 64);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

    printf("Calibration complete. Flash real firmware now.\n");
    while (1) vTaskDelay(pdMS_TO_TICKS(5000));
}
