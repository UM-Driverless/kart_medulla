/* Fake km_gpio.h for native unit tests — stubs out all hardware I/O */
#ifndef FAKE_KM_GPIO_H
#define FAKE_KM_GPIO_H

#include <stdint.h>
#include "esp_err.h"

/* GPIO num type */
typedef int gpio_num_t;
#define GPIO_NUM_1   1
#define GPIO_NUM_3   3
#define GPIO_NUM_18  18
#define GPIO_NUM_19  19
#define GPIO_NUM_21  21
#define GPIO_NUM_22  22
#define GPIO_NUM_25  25
#define GPIO_NUM_26  26

/* Pin defines matching the real km_gpio.h */
#define PIN_USB_UART_TX   GPIO_NUM_1
#define PIN_USB_UART_RX   GPIO_NUM_3
#define PIN_STEER_PWM     GPIO_NUM_18
#define PIN_STEER_DIR     GPIO_NUM_19
#define PIN_CMD_ACC       GPIO_NUM_25
#define PIN_CMD_BRAKE     GPIO_NUM_26

/* UART types */
typedef int uart_port_t;
#define UART_NUM_0  0
#define UART_NUM_1  1
#define UART_NUM_2  2
#define UART_PIN_NO_CHANGE (-1)

/* Recorded calls for assertions */
extern uint8_t fake_dac_value[2];      /* [0]=channel A, [1]=channel B */
extern uint8_t fake_pwm_duty;
extern uint8_t fake_digital_pin19;     /* direction pin */

static inline esp_err_t KM_GPIO_Init(void) { return ESP_OK; }
static inline uint8_t   KM_GPIO_ReadDigital(gpio_num_t pin) { (void)pin; return 0; }

static inline esp_err_t KM_GPIO_WriteDigital(gpio_num_t pin, uint8_t level) {
    if (pin == PIN_STEER_DIR) fake_digital_pin19 = level;
    return ESP_OK;
}

static inline uint16_t  KM_GPIO_ReadADC(gpio_num_t pin) { (void)pin; return 0; }

static inline esp_err_t KM_GPIO_WriteDAC(gpio_num_t pin, uint8_t value) {
    if (pin == 0) fake_dac_value[0] = value;      /* channel A (accel) */
    else if (pin == 1) fake_dac_value[1] = value;  /* channel B (brake) */
    return ESP_OK;
}

static inline esp_err_t KM_GPIO_WritePWM(gpio_num_t pin, uint32_t duty) {
    (void)pin;
    fake_pwm_duty = (uint8_t)duty;
    return ESP_OK;
}

#endif
