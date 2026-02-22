/******************************************************************************
 * @file    km_gpio.h
 * @brief   Interfaz pública de la librería.
 * @author  Adrian Navarredonda Arizaleta
 * @date    7-2-2026
 * @version 1.0
 *****************************************************************************/

#ifndef KM_GPIO_H
#define KM_GPIO_H

/******************************* INCLUDES *************************************/
// Includes necesarios para la API pública
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_err.h"

/******************************* DEFINES PÚBLICAS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería

#ifndef LEDC_HIGH_SPEED_MODE
#define LEDC_HIGH_SPEED_MODE 0
#endif
#ifndef LEDC_LOW_SPEED_MODE
#define LEDC_LOW_SPEED_MODE 1
#endif

/* ============================================================
 *  ESP32-DevKitC V4  (ESP32-WROOM-32E)
 *  Pin assignment - NO WiFi
 * ============================================================ */

/* ---------- USB (UART0 to ORIN) ---------- */
#define PIN_USB_UART_TX         GPIO_NUM_1   // U0TXD
#define PIN_USB_UART_RX         GPIO_NUM_3   // U0RXD

/* ---------- ADC INPUTS (Sensors) ---------- */
/* ADC1 - input only, WiFi safe */
#define PIN_PEDAL_BRAKE         GPIO_NUM_32  // ADC1_CH4
#define PIN_HYDRAULIC_1         GPIO_NUM_33  // ADC1_CH5
#define PIN_PRESSURE_3          GPIO_NUM_34  // ADC1_CH6
#define PIN_PEDAL_ACC           GPIO_NUM_35  // ADC1_CH7
#define PIN_PRESSURE_1          GPIO_NUM_36  // ADC1_CH0 (VP)
#define PIN_PRESSURE_2          GPIO_NUM_39  // ADC1_CH3 (VN)

/* ADC2 - allowed (WiFi not used) */
#define PIN_HYDRAULIC_2         GPIO_NUM_13  // ADC2_CH4 (strap pin)

/* ---------- DAC OUTPUTS ---------- */
#define PIN_CMD_ACC             GPIO_NUM_25  // DAC1
#define PIN_CMD_BRAKE           GPIO_NUM_26  // DAC2

/* ---------- STEERING MOTOR ---------- */
#define PIN_STEER_PWM           GPIO_NUM_27  // PWM Steering
#define PIN_STEER_DIR           GPIO_NUM_14  // Direction steering

/* ---------- HALL SENSORS ---------- */
#define PIN_MOTOR_HALL_1        GPIO_NUM_18  // HALL 1 motor
#define PIN_MOTOR_HALL_2        GPIO_NUM_19  // HALL 2 motor
#define PIN_MOTOR_HALL_3        GPIO_NUM_23  // HALL 3 motor

/* ---------- I2C (AS5600) ---------- */
#define PIN_I2C_SDA             GPIO_NUM_21  // I2C SDA
#define PIN_I2C_SCL             GPIO_NUM_22  // I2C SCL

/* ---------- STATUS LED ---------- */
#define PIN_STATUS_LED          GPIO_NUM_2   // Strap pin (keep LOW at boot)

/* ============================================================
 *  GPIO RESTRICTIONS (DO NOT USE)
 *  GPIO 6-11 : SPI FLASH
 *  GPIO 34-39: INPUT ONLY
 * ============================================================ */

/******************************* TIPOS PÚBLICOS *******************************/
// Estructuras, enums, typedefs públicos

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

/* ---------- USB (UART0 to ORIN) ---------- */
extern const uart_config_t uart_config;

/* ---------- ADC INPUTS (Sensors) ---------- */
/* ADC1 - input only, WiFi safe */

extern const gpio_config_t pin_pressure_1;
extern const gpio_config_t pin_pressure_2;
extern const gpio_config_t pin_pressure_3;
extern const gpio_config_t pin_pedal_acc;
extern const gpio_config_t pin_pedal_brake;
extern const gpio_config_t pin_hydraulic_1;

/* ADC2 - allowed (WiFi not used) */

extern const gpio_config_t pin_hydraulic_2;

/* ---------- DAC OUTPUTS ---------- */

extern const gpio_config_t pin_cmd_acc;
extern const gpio_config_t pin_cmd_brake;

/* ---------- STEERING MOTOR ---------- */

extern const gpio_config_t pin_steer_pwm;
extern const gpio_config_t pin_steer_dir;

/* ---------- HALL SENSORS ---------- */

extern const gpio_config_t pin_motor_hall_1;
extern const gpio_config_t pin_motor_hall_2;
extern const gpio_config_t pin_motor_hall_3;

/* ---------- I2C (AS5600) ---------- */

extern const gpio_config_t pin_i2c_scl;
extern const gpio_config_t pin_i2c_sda;

/* ---------- STATUS LED ---------- */

extern const gpio_config_t pin_status_led;

/******************************* FUNCIONES PÚBLICAS ***************************/
/**
 * @brief Initialize all hardware peripherals needed by KM_GPIO.
 *
 * This function initializes:
 *  - PWM (LEDC) timers and channels
 *  - DAC channels
 *  - I2C driver
 *
 * Note: The actual GPIO pins must be configured in gpio_config_t structs
 *       before calling this function.
 *
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t KM_GPIO_Init(void);

/* ---------- Digital GPIO ---------- */

/**
 * @brief Read the digital level of a GPIO pin.
 *
 * @param pin Pointer to a gpio_config_t representing the pin to read.
 * @return 0 if LOW, 1 if HIGH.
 */
uint8_t KM_GPIO_ReadPin(const gpio_config_t *pin);

/**
 * @brief Set a GPIO pin HIGH (1).
 *
 * @param pin Pointer to a gpio_config_t representing the pin to write.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t KM_GPIO_WritePinHigh(const gpio_config_t *pin);

/**
 * @brief Set a GPIO pin LOW (0).
 *
 * @param pin Pointer to a gpio_config_t representing the pin to write.
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t KM_GPIO_WritePinLow(const gpio_config_t *pin);

/* ---------- ADC ---------- */

/**
 * @brief Read an analog input (ADC) from the specified pin.
 *
 * Only pins configured as ADC1 or ADC2 channels can be read.
 *
 * @param pin Pointer to a gpio_config_t representing the ADC pin.
 * @return 12-bit ADC value (0-4095). Returns 0 if pin is invalid.
 */
uint16_t KM_GPIO_ReadADC(const gpio_config_t *pin);

/* ---------- DAC ---------- */

/**
 * @brief Write a value to a DAC output pin.
 *
 * Only DAC1 (GPIO25) and DAC2 (GPIO26) are supported.
 *
 * @param pin Pointer to a gpio_config_t representing the DAC pin.
 * @param value 8-bit value (0-255) to output.
 * @return ESP_OK on success, or ESP_ERR_INVALID_ARG if pin is not DAC.
 */
esp_err_t KM_GPIO_WriteDAC(const gpio_config_t *pin, uint8_t value);

/* ---------- PWM ---------- */

/**
 * @brief Write a PWM duty cycle to a pin using LEDC.
 *
 * Only pins configured with PWM channels will respond.
 *
 * @param pin Pointer to a gpio_config_t representing the PWM pin.
 * @param duty Duty cycle from 0 to 255.
 * @return ESP_OK on success, or ESP_ERR_INVALID_ARG if pin is not PWM.
 */
esp_err_t KM_GPIO_WritePWM(const gpio_config_t *pin, uint8_t duty);

/* ---------- I2C ---------- */

/**
 * @brief Initialize I2C master interface.
 *
 * Uses SDA and SCL pins defined in the gpio_config_t structs.
 *
 * @return ESP_OK on success, or an ESP-IDF error code.
 */
esp_err_t KM_GPIO_I2CInit(void);

#endif /* KM_GPIO_H */
