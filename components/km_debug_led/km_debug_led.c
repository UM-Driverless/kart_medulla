
#include "km_debug_led.h"

#define DEBUG_LED GPIO_NUM_2

void debug_led_init(void)
{
    gpio_reset_pin(DEBUG_LED);
    gpio_set_direction(DEBUG_LED, GPIO_MODE_OUTPUT);
}

void debug_led_on(void)
{
    gpio_set_level(DEBUG_LED, 1);
}

void debug_led_off(void)
{
    gpio_set_level(DEBUG_LED, 0);
}

void debug_led_blink(int times, int delay_ms)
{
    for(int i = 0; i < times; i++) {
        gpio_set_level(DEBUG_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(DEBUG_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}