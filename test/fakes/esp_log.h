/* Fake esp_log.h for native unit tests */
#ifndef FAKE_ESP_LOG_H
#define FAKE_ESP_LOG_H

typedef enum {
    ESP_LOG_NONE,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE
} esp_log_level_t;

#define ESP_LOGE(tag, fmt, ...)
#define ESP_LOGW(tag, fmt, ...)
#define ESP_LOGI(tag, fmt, ...)
#define ESP_LOGD(tag, fmt, ...)

static inline void esp_log_level_set(const char *tag, esp_log_level_t level) { (void)tag; (void)level; }

#endif
