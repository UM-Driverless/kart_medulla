/* Fake esp_system.h for native unit tests */
#ifndef FAKE_ESP_SYSTEM_H
#define FAKE_ESP_SYSTEM_H

#include <stdint.h>

static inline uint32_t esp_get_free_heap_size(void) { return 65536; }

#endif
