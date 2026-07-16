/******************************************************************************
 * @file    km_gpio.h
 * @brief   Public interface for the KM_GPIO hardware abstraction library.
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
#include "driver/spi_master.h"
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

#if defined(CONFIG_IDF_TARGET_ESP32S3)
/* ============================================================
 *  ESP32-S3 — kart-medulla PCB (the real bench/kart hardware)
 *  Authoritative map: .agents/esp32s3-pinmap.md (from the schematic).
 *  Pin count/functions differ from the classic map below — this is a
 *  remap, not a renumber. Several functional gaps remain (see the
 *  GAP notes); the classic ESP32 build is still the one that runs today.
 * ============================================================ */

/* ---------- UART0 (USB bridge / CH343) — debug console + Orin binary comms ---------- */
#define PIN_USB_UART_TX         GPIO_NUM_43  // U0TXD (dev-board USB-UART bridge)
#define PIN_USB_UART_RX         GPIO_NUM_44  // U0RXD
/* No separate UART2-to-Orin on S3 (GPIO16/17 are Hall/steer-dir here).
 * Orin comms ride UART0 — these alias it so shared code still compiles. */
#define PIN_ORIN_UART_TX        GPIO_NUM_43
#define PIN_ORIN_UART_RX        GPIO_NUM_44

/* ---------- ADC1 inputs (GPIO1-10 = ADC1 on S3; ADC2 unusable with WiFi) ---------- */
#define PIN_PRESSURE_3          GPIO_NUM_1   // CN5.2 — ALSO the AS5600 PWM-angle read pin
#define PIN_HYDRAULIC_2         GPIO_NUM_2
#define PIN_PEDAL_ACC           GPIO_NUM_4
#define PIN_PEDAL_BRAKE         GPIO_NUM_5
#define PIN_PRESSURE_1          GPIO_NUM_6
#define PIN_PRESSURE_2          GPIO_NUM_7
#define PIN_HYDRAULIC_1         GPIO_NUM_10

/* ---------- EBS compressor (ex-BUZZER net, CN8.2) ---------- */
#define PIN_CMD_COMPRESSOR      GPIO_NUM_3   // LEDC PWM (soft-start), see COMPRESSOR_PWM_FREQ_HZ

/* ---------- I2C — AS5600 (0x36) + on-board PCF8574 (0x20) ---------- */
#define PIN_I2C_SDA             GPIO_NUM_8
#define PIN_I2C_SCL             GPIO_NUM_9

/* ---------- SPI → MCP4922 external DAC (throttle/brake) ---------- */
/* S3 has NO built-in DAC; throttle/brake go through the MCP4922 over SPI.
 * GAP: KM_GPIO_WriteDAC() still calls the classic dac_output_voltage() and
 * is not ported to SPI — the S3 build will not link until that is done. */
#define SPI_MOSI_PIN            GPIO_NUM_11
#define SPI_SCLK_PIN            GPIO_NUM_12
#define SPI_MISO_PIN            GPIO_NUM_13
#define SPI_CS_PIN              GPIO_NUM_14
#define PIN_CMD_ACC             GPIO_NUM_NC  // MCP4922 channel, not a GPIO (sentinel)
#define PIN_CMD_BRAKE           GPIO_NUM_NC  // MCP4922 channel, not a GPIO (sentinel)
#define PIN_SELECT_THROTTLE     GPIO_NUM_15  // MAX4660 mux; drive HIGH = throttle via DAC. GAP: not driven

/* ---------- STEERING MOTOR (Cytron H-bridge) ---------- */
#define PIN_STEER_PWM           GPIO_NUM_40  // LEDC PWM
#define PIN_STEER_DIR           GPIO_NUM_17  // direction

/* ---------- HALL SENSORS (via U5 level shifter; all three usable on S3) ---------- */
#define PIN_MOTOR_HALL_1        GPIO_NUM_16
#define PIN_MOTOR_HALL_2        GPIO_NUM_47
#define PIN_MOTOR_HALL_3        GPIO_NUM_21

/* ---------- SDC (Shutdown Circuit) — SAFETY ---------- */
/* Gate of Q3 (IRLZ44N). HIGH = Q3 conducts = chain closed = NO emergency.
 * Held OFF at boot by R23 pulldown → kart sits in emergency until firmware
 * drives it HIGH. GAP: nothing drives it yet, so the kart cannot be armed. */
#define PIN_SDC_NOT_EMERGENCY   GPIO_NUM_18

/* ---------- STATUS LED ---------- */
#define PIN_STATUS_LED          GPIO_NUM_48  // dev-board addressable RGB. GAP: needs RMT, not plain GPIO

#else  /* CONFIG_IDF_TARGET_ESP32 — classic ESP32-WROOM-32E (current build) */

/* ---------- USB (UART0 - debug console) ---------- */
#define PIN_USB_UART_TX         GPIO_NUM_1   // U0TXD
#define PIN_USB_UART_RX         GPIO_NUM_3   // U0RXD

/* ---------- UART2 (debug logs) ---------- */
/* WARNING: GPIO17/16 conflict with MOTOR_HALL_1/3 in the PCB.
 * UART2 debug logging unavailable when using the interface PCB.
 * Orin comms use UART0 (USB, GPIO1/3) for binary protocol. */
#define PIN_ORIN_UART_TX        GPIO_NUM_17  // U2TXD (conflicts HALL1 on PCB)
#define PIN_ORIN_UART_RX        GPIO_NUM_16  // U2RXD (conflicts HALL3 on PCB)

/* ---------- ADC INPUTS (Sensors) ---------- */
/* ADC1 - input only, WiFi safe */
#define PIN_PEDAL_BRAKE         GPIO_NUM_32  // ADC1_CH4
#define PIN_HYDRAULIC_1         GPIO_NUM_27  // H2-11, ADC2_CH7
#define PIN_PRESSURE_3          GPIO_NUM_34  // ADC1_CH6
#define PIN_PEDAL_ACC           GPIO_NUM_35  // ADC1_CH7
#define PIN_PRESSURE_1          GPIO_NUM_36  // ADC1_CH0 (VP)
#define PIN_PRESSURE_2          GPIO_NUM_39  // ADC1_CH3 (VN)

/* ADC2 - allowed (WiFi not used) */
#define PIN_HYDRAULIC_2         GPIO_NUM_14  // H2-12, ADC2_CH6

/* ---------- DAC OUTPUTS ---------- */
#define PIN_CMD_ACC             GPIO_NUM_25  // DAC1 !!!!!!Antes era el 25
#define PIN_CMD_BRAKE           GPIO_NUM_26  // DAC2

/* ---------- STEERING MOTOR ---------- */
#define PIN_STEER_PWM           GPIO_NUM_18  // H1-11, PWM Steering (Cytron H-bridge)
#define PIN_STEER_DIR           GPIO_NUM_19  // H1-12, Direction steering (Cytron H-bridge)

/* ---------- HALL SENSORS ---------- */
/* HALL 1/3 disabled — GPIO17/16 used by UART2 (debug to Orin) */
// #define PIN_MOTOR_HALL_1     GPIO_NUM_17  // conflicts with PIN_ORIN_UART_TX
#define PIN_MOTOR_HALL_2        GPIO_NUM_33  // H2-8, HALL 2 motor
// #define PIN_MOTOR_HALL_3     GPIO_NUM_16  // conflicts with PIN_ORIN_UART_RX

/* ---------- I2C (AS5600) ---------- */
#define PIN_I2C_SDA             GPIO_NUM_21  // I2C SDA
#define PIN_I2C_SCL             GPIO_NUM_22  // I2C SCL !!!!!!!!!!!!Antes era el 22

/* ---------- SDC (Shutdown Circuit) ---------- */
#define PIN_SDC_NOT_EMERGENCY   GPIO_NUM_13  // H2-15, SDC emergency status

/* ---------- STATUS LED ---------- */
#define PIN_STATUS_LED          GPIO_NUM_2   // Strap pin (keep LOW at boot)

#endif  /* target select */

/* ============================================================
 *  COMPRESSOR PWM
 *  The MOSFET gate is driven straight from a 3.3 V GPIO through a series
 *  resistor, so it switches slowly and never fully enhances. Both effects
 *  push us to a LOW carrier frequency: every edge costs energy, and the
 *  fewer edges per second, the less the MOSFET heats.
 *  500 Hz (2 ms period) sits between the two limits:
 *   - Switching loss: edges take a few µs, so <1% of the period is spent
 *     in transition. At 20 kHz that would be ~10% and the MOSFET cooks.
 *   - Current ripple: the motor's electrical time constant is a few ms,
 *     so a 2 ms period still filters into fairly smooth current. Going
 *     much below ~200 Hz lets current pulse hard again, which is exactly
 *     the 12 V rail disturbance the soft-start exists to avoid.
 *  If the MOSFET runs hot, lower this before touching the duty cap.
 * ============================================================ */
#define COMPRESSOR_PWM_FREQ_HZ  500

/* ============================================================
 *  GPIO RESTRICTIONS (DO NOT USE)
 *  GPIO 6-11 : SPI FLASH
 *  GPIO 34-39: INPUT ONLY
 * ============================================================ */

/******************************* TIPOS PÚBLICOS *******************************/
// Estructuras, enums, typedefs públicos

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

extern const uart_config_t uart0_config;
extern const uart_config_t uart2_config;

// /* ---------- USB (UART0 to ORIN) ---------- */
// extern const uart_config_t uart_config;

// /* ---------- ADC INPUTS (Sensors) ---------- */
// /* ADC1 - input only, WiFi safe */

// extern const gpio_config_t pin_pressure_1;
// extern const gpio_config_t pin_pressure_2;
// extern const gpio_config_t pin_pressure_3;
// extern const gpio_config_t pin_pedal_acc;
// extern const gpio_config_t pin_pedal_brake;
// extern const gpio_config_t pin_hydraulic_1;

// /* ADC2 - allowed (WiFi not used) */

// extern const gpio_config_t pin_hydraulic_2;

// /* ---------- DAC OUTPUTS ---------- */

// extern const gpio_config_t pin_cmd_acc;
// extern const gpio_config_t pin_cmd_brake;

// /* ---------- STEERING MOTOR ---------- */

// extern const gpio_config_t pin_steer_pwm;
// extern const gpio_config_t pin_steer_dir;

// /* ---------- HALL SENSORS ---------- */

// extern const gpio_config_t pin_motor_hall_1;
// extern const gpio_config_t pin_motor_hall_2;
// extern const gpio_config_t pin_motor_hall_3;

// /* ---------- I2C (AS5600) ---------- */

// extern const gpio_config_t pin_i2c_scl;
// extern const gpio_config_t pin_i2c_sda;

// /* ---------- SDC (Shutdown Circuit) ---------- */

/* ---------- STATUS LED ---------- */

// extern const gpio_config_t pin_status_led;

/******************************* FUNCIONES PÚBLICAS ***************************/

/**
 * @brief   Initializes all GPIO peripherals: ADC, DAC, PWM (LEDC), I2C, and direction pin.
 *
 * @details Configures ADC1/ADC2 input pins for sensors, enables DAC outputs for
 *          throttle/brake commands, sets up the LEDC PWM timer and channel for
 *          steering, configures the steering direction output pin, and initializes
 *          I2C master for the AS5600 encoder.
 *
 * @return  ESP_OK on success, or an esp_err_t error code on failure.
 *
 * @note    Must be called once at startup before any other KM_GPIO function.
 * @note    UART0/UART2 driver installation is currently commented out; the USB
 *          UART0 is managed by KM_COMS_Init instead.
 */
esp_err_t KM_GPIO_Init(void);

/* ---------- Digital GPIO ---------- */

/**
 * @brief   Reads the digital level of a GPIO pin.
 *
 * @param   pin  GPIO number to read.
 * @return  1 if high, 0 if low.
 */
uint8_t KM_GPIO_ReadDigital(gpio_num_t pin);

/**
 * @brief   Writes a digital level to a GPIO pin.
 *
 * @param   pin    GPIO number to write.
 * @param   level  Desired output level (0 = low, non-zero = high).
 * @return  ESP_OK on success, or an esp_err_t error code on failure.
 */
esp_err_t KM_GPIO_WriteDigital(gpio_num_t pin, uint8_t level);

/* ---------- ADC ---------- */

/**
 * @brief   Reads the raw ADC value for a given analog input pin.
 *
 * @details Maps the GPIO number to the corresponding ADC1 or ADC2 channel and
 *          returns the 12-bit raw reading. Supports the six ADC1 sensor pins
 *          and the two ADC2 hydraulic pins defined in this header.
 *
 * @param   pin  GPIO number of the analog input (must be a defined ADC pin).
 * @return  12-bit raw ADC value (0-4095), or 0 if the pin is invalid or the
 *          ADC2 read fails.
 */
uint16_t KM_GPIO_ReadADC(gpio_num_t pin);

/* ---------- DAC ---------- */

/**
 * @brief   Writes an 8-bit value to one of the two DAC channels.
 *
 * @param   pin    GPIO number of the DAC output (PIN_CMD_ACC or PIN_CMD_BRAKE).
 * @param   value  8-bit output value (0-255).
 * @return  ESP_OK on success, ESP_ERR_INVALID_ARG if the pin is not a DAC output.
 */
esp_err_t KM_GPIO_WriteDAC(gpio_num_t pin, uint8_t value);

/* ---------- PWM ---------- */

/**
 * @brief   Sets the PWM duty cycle for a PWM output.
 *
 * @details Two independent outputs are supported, each on its own LEDC timer:
 *          PIN_STEER_PWM at 1 kHz, and PIN_CMD_COMPRESSOR at
 *          COMPRESSOR_PWM_FREQ_HZ (S3 target only).
 *
 * @param   pin   GPIO number of the PWM output.
 * @param   duty  Duty cycle value (0-255, 8-bit resolution).
 * @return  ESP_OK on success, ESP_ERR_INVALID_ARG if the pin is not a PWM output.
 */
esp_err_t KM_GPIO_WritePWM(gpio_num_t pin, uint32_t duty);

#endif /* KM_GPIO_H */
