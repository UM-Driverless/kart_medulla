/* Fake esp_timer.h for native unit tests */
#ifndef FAKE_ESP_TIMER_H
#define FAKE_ESP_TIMER_H

#include <stdint.h>

/* Controllable mock time — tests set this directly */
extern uint64_t fake_esp_timer_us;

static inline uint64_t esp_timer_get_time(void) {
    return fake_esp_timer_us;
}

#endif
