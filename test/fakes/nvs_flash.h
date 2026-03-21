/* Fake nvs_flash.h for native unit tests */
#ifndef FAKE_NVS_FLASH_H
#define FAKE_NVS_FLASH_H
#include "esp_err.h"
static inline esp_err_t nvs_flash_init(void) { return ESP_OK; }
static inline esp_err_t nvs_flash_erase(void) { return ESP_OK; }
#define ESP_ERR_NVS_NO_FREE_PAGES 0x1100
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x1101
#endif
