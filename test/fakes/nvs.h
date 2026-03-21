/* Fake nvs.h for native unit tests */
#ifndef FAKE_NVS_H
#define FAKE_NVS_H
#include "esp_err.h"
#include <stdint.h>

typedef uint32_t nvs_handle_t;
typedef enum { NVS_READONLY, NVS_READWRITE } nvs_open_mode_t;

static inline esp_err_t nvs_open(const char *ns, nvs_open_mode_t m, nvs_handle_t *h) {
    (void)ns;(void)m; *h = 1; return ESP_OK;
}
static inline esp_err_t nvs_set_u16(nvs_handle_t h, const char *k, uint16_t v) { (void)h;(void)k;(void)v; return ESP_OK; }
static inline esp_err_t nvs_get_u16(nvs_handle_t h, const char *k, uint16_t *v) { (void)h;(void)k;(void)v; return ESP_FAIL; }
static inline esp_err_t nvs_commit(nvs_handle_t h) { (void)h; return ESP_OK; }
static inline void nvs_close(nvs_handle_t h) { (void)h; }

#endif
