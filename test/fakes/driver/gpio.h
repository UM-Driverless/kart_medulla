/* Fake driver/gpio.h for native unit tests */
#ifndef FAKE_DRIVER_GPIO_H
#define FAKE_DRIVER_GPIO_H

#include <stdint.h>

#ifndef gpio_num_t
typedef int gpio_num_t;
#endif

#define GPIO_NUM_1   1
#define GPIO_NUM_2   2
#define GPIO_NUM_3   3
#define GPIO_NUM_13  13
#define GPIO_NUM_14  14
#define GPIO_NUM_18  18
#define GPIO_NUM_19  19
#define GPIO_NUM_21  21
#define GPIO_NUM_22  22
#define GPIO_NUM_25  25
#define GPIO_NUM_26  26
#define GPIO_NUM_27  27
#define GPIO_NUM_32  32
#define GPIO_NUM_33  33
#define GPIO_NUM_34  34
#define GPIO_NUM_35  35
#define GPIO_NUM_36  36
#define GPIO_NUM_39  39
#define GPIO_PULLUP_ENABLE 1

#endif
