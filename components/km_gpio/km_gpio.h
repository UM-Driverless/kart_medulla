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
 *  PIN MAPS — two of them, selected by the build target.
 *
 *  esp32-s3-devkitc-1  -> the ESP32-S3 branch below. This is the
 *                         kart's board and the image that runs on it.
 *  esp32dev            -> the classic ESP32-WROOM-32E branch in the
 *                         #else. Fallback only; not flashed to the kart.
 *
 *  The two overlap dangerously rather than merely differing: GPIO 18 is
 *  steering PWM on the classic board and the gate of Q3, the
 *  shutdown-circuit MOSFET, on the S3. Never read one branch while
 *  working on the other.
 * ============================================================ */

#if defined(CONFIG_IDF_TARGET_ESP32S3)
/* ============================================================
 *  ESP32-S3 — kart-medulla PCB (the real bench/kart hardware)
 *  Authoritative map: .agents/esp32s3-pinmap.md (from the schematic).
 *  Pin count/functions differ from the classic map below — this is a
 *  remap, not a renumber. THIS is the build that runs on the kart; the
 *  classic block below is the previous board, kept for the esp32dev
 *  target only. Remaining gaps are tracked in tasks.md, not here.
 * ============================================================ */

/* ---------- UART0 (USB bridge / CH343) — debug console + Orin binary comms ---------- */
#define PIN_USB_UART_TX         GPIO_NUM_43  // U0TXD (dev-board USB-UART bridge)
#define PIN_USB_UART_RX         GPIO_NUM_44  // U0RXD
/* No separate UART2-to-Orin on S3 (GPIO16/17 are Hall/steer-dir here).
 * Orin comms ride UART0 — these alias it so shared code still compiles. */
#define PIN_ORIN_UART_TX        GPIO_NUM_43
#define PIN_ORIN_UART_RX        GPIO_NUM_44

/* ---------- ADC1 inputs (GPIO1-10 = ADC1 on S3; ADC2 unusable with WiFi) ---------- */
/* CN5.2 carries the MT6701 steering sensor's PWM output, not a pressure
 * transducer — no sensor is fitted to the PRESSURE_3 terminal. The pad is
 * driven by MCPWM capture (see km_sdir_pwm.h), so it is deliberately left out
 * of the ADC channel setup in km_gpio.c: the ADC and the capture peripheral
 * cannot both own it. PIN_PRESSURE_3 stays defined only so code shared with
 * the classic ESP32 build keeps compiling. */
#define PIN_PRESSURE_3          GPIO_NUM_1   // CN5.2 — NOT an ADC input on the S3
#define PIN_STEER_PWM_IN        GPIO_NUM_1   // CN5.2 — MT6701 OUT, ~994 Hz PWM angle frame
#define PIN_HYDRAULIC_2         GPIO_NUM_2
#define PIN_PEDAL_ACC           GPIO_NUM_4
#define PIN_PEDAL_BRAKE         GPIO_NUM_5
#define PIN_PRESSURE_1          GPIO_NUM_6   // CN7.1 — EBS tank, Festo SDE5-D10, 1 V/bar via divider
#define PIN_PRESSURE_2          GPIO_NUM_7   // CN7.2
#define PIN_HYDRAULIC_1         GPIO_NUM_10

/* ---------- EBS compressor (ex-BUZZER net, CN8.2) ---------- */
#define PIN_CMD_COMPRESSOR      GPIO_NUM_3   // LEDC PWM (soft-start), see COMPRESSOR_PWM_FREQ_HZ

/* ---------- I2C — AS5600 (0x36) + on-board PCF8574 (0x20) ---------- */
#define PIN_I2C_SDA             GPIO_NUM_8
#define PIN_I2C_SCL             GPIO_NUM_9

/* ---------- SPI → MCP4922 external DAC (throttle/brake) ---------- */
/* The S3 has no built-in DAC, so throttle and brake are produced by an
 * MCP4922 (U13) driven over SPI2. VDD and both VREF pins tie to +5V_REG, and
 * the code below selects gain 1x, so a full-scale code gives ~5.0 V.
 *
 * The MCP4922 is WRITE-ONLY: the schematic's MISO net reaches the ESP32 pads
 * and nothing else, because the chip has no data output. Nothing the firmware
 * can read confirms that a write arrived — the only proof is a voltmeter on
 * U13 pin 14 (channel A) or pin 10 (channel B). Diagnostics in km_gpio.c can
 * therefore only ever say "the ESP32 transmitted", never "the DAC received".
 *
 * LDAC# (pin 8) is strapped to GND and SHDN# (pin 9) to +5V_REG, so writes
 * take effect immediately and neither needs a control line. */
#define SPI_MOSI_PIN            GPIO_NUM_11
#define SPI_SCLK_PIN            GPIO_NUM_12
#define SPI_MISO_PIN            GPIO_NUM_13  // not wired to U13; the MCP4922 has no data output
#define SPI_CS_PIN              GPIO_NUM_14

/* The two DAC outputs are pins of the MCP4922, not GPIOs of the ESP32, but the
 * km_act layer identifies an analog output by "pin number" and stores it in a
 * uint8_t. These stand-in values are chosen to survive that: distinct from each
 * other, above the S3's GPIO range (0-48) so they can never alias a real pin,
 * and below 256 so the uint8_t does not truncate them.
 *
 * Both were GPIO_NUM_NC (-1) until 2026-07-31, which broke two ways at once:
 * the first `if` in KM_GPIO_WriteDAC() matched both channels so brake was
 * indistinguishable from throttle, and km_act's uint8_t turned -1 into 255 so
 * neither matched anything and every write returned ESP_ERR_INVALID_ARG. */
#define PIN_CMD_ACC             ((gpio_num_t)200)  // MCP4922 channel A -> U14.8 -> throttle
#define PIN_CMD_BRAKE           ((gpio_num_t)201)  // MCP4922 channel B -> U1A x2 -> brake

#define PIN_SELECT_THROTTLE     GPIO_NUM_15  // MAX4660 mux; LOW = pedal, HIGH = throttle via DAC

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
 * drives it HIGH. control_task has driven it since 2026-07-26: it decides the
 * SDC state every cycle, above the early return, so a stalled or returning
 * control path cannot leave the chain closed by omission. */
#define PIN_SDC_NOT_EMERGENCY   GPIO_NUM_18

/* ---------- STATUS LED ---------- */
#define PIN_STATUS_LED          GPIO_NUM_48  // dev-board addressable RGB. GAP: needs RMT, not plain GPIO

#else  /* CONFIG_IDF_TARGET_ESP32 — classic ESP32-WROOM-32E, the esp32dev fallback target.
        * NOT the kart's board. Do not wire or reason from these numbers. */

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
 *  This carrier runs whenever the compressor runs: the duty is what
 *  keeps a 7.5 V motor at its rated voltage off a 12 V rail, so it never
 *  reaches 100% and the MOSFET never rests at DC (see COMPRESSOR_DUTY_RUN
 *  in main.c). Switching loss is therefore permanent, not a transient.
 *  That matters because the gate is driven straight from a 3.3 V GPIO
 *  through a series resistor: it switches slowly and never fully
 *  enhances, so every edge costs real energy.
 *  500 Hz (2 ms period) sits between the two limits:
 *   - Switching loss: edges take a few µs, so <1% of the period is spent
 *     in transition. At 20 kHz that would be ~10% and the MOSFET cooks.
 *   - Current ripple: the motor's electrical time constant is a few ms,
 *     so a 2 ms period keeps current continuous — which is also what
 *     makes the motor see duty x rail as a clean average. Going much
 *     below ~200 Hz lets current go discontinuous: it pulses hard again
 *     (the 12 V rail disturbance the soft-start exists to avoid) AND the
 *     average-voltage assumption behind the 60% duty stops holding.
 *  If the MOSFET runs hot, lower this. Do not raise the duty instead.
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

/**
 * @brief   Opens or closes the shutdown circuit (SDC) — SAFETY.
 *
 * @details Drives `PIN_SDC_NOT_EMERGENCY` (GPIO 18 on the S3), the gate of Q3.
 *          Asserting emergency drives it LOW, which opens the shutdown chain
 *          and fires the EBS.
 *
 * @param   assert_emergency  Non-zero to assert emergency (open the chain, fire
 *                            the EBS); 0 to close the chain.
 *
 * @return  ESP_OK on success, ESP_ERR_NOT_SUPPORTED on a target with no SDC pin.
 *
 * @warning Closing the chain (passing 0) is NOT arming the kart and must not be
 *          used as if it were — arming has other preconditions this function
 *          knows nothing about. Nothing in this firmware currently closes the
 *          chain, so the kart boots and stays in the emergency state. Read the
 *          `SDC_NOT_EMERGENCY` row in `.agents/esp32s3-pinmap.md` before
 *          changing that.
 */
esp_err_t KM_GPIO_SetEmergency(uint8_t assert_emergency);

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

/**
 * @brief   Reads an ADC pin and returns MILLIVOLTS, not raw counts.
 *
 * @details Uses the chip's own eFuse ADC calibration, so callers never have to
 *          assume a counts-per-volt constant. Prefer this over KM_GPIO_ReadADC()
 *          for anything that means a physical quantity: a consumer given raw
 *          counts has to guess the full-scale voltage, and guessing it wrong is
 *          what made the tank-pressure dial disagree with the firmware for months
 *          (see history.md 2026-07-26).
 *
 * @param   pin  GPIO to read (must be an ADC1 channel set up by KM_GPIO_Init).
 * @return  Voltage at the pin in mV. Values near the 11 dB ceiling (~2900 mV)
 *          are saturated, not measurements.
 */
uint32_t KM_GPIO_ReadADC_mV(gpio_num_t pin);

/* ---------- DAC ---------- */

/**
 * @brief   Choose which source drives the kart's throttle line.
 *
 * Sets the MAX4660 (U14) mux on GPIO 15. LOW routes the pedal straight through
 * to CN10.1; HIGH hands the line to the MCP4922's VOUTA. The pin has a 10 k
 * pulldown, so the pedal is the state before firmware runs and the state the
 * hardware falls back to if this pin is ever left floating.
 *
 * @param   use_dac  true = DAC drives the throttle, false = pedal passes through.
 * @return  ESP_OK, or ESP_ERR_NOT_SUPPORTED if the pin is not defined for this board.
 */
esp_err_t KM_GPIO_SetThrottleSource(bool use_dac);

/**
 * @brief   Writes an 8-bit value to one of the two DAC channels.
 *
 * @param   pin    GPIO number of the DAC output (PIN_CMD_ACC or PIN_CMD_BRAKE).
 * @param   value  8-bit output value (0-255).
 * @return  ESP_OK on success, ESP_ERR_INVALID_ARG if the pin is not a DAC output.
 */
esp_err_t KM_GPIO_WriteDAC(gpio_num_t pin, uint8_t value);

#ifdef CONFIG_IDF_TARGET_ESP32S3
/**
 * @brief   Reports how the MCP4922 writes have gone since boot.
 *
 * @param   ok        Out: successful transmits. May be NULL.
 * @param   fail      Out: failed transmits. May be NULL.
 * @param   last_cmd  Out: the last 16-bit command word sent. May be NULL.
 *
 * @note    These count what the ESP32 *transmitted*. The MCP4922 has no data
 *          output, so a high `ok` count says the SPI peripheral accepted the
 *          transfers, not that the DAC received or acted on them.
 */
void KM_GPIO_McpStats(uint32_t *ok, uint32_t *fail, uint16_t *last_cmd);

/**
 * @brief   Steps MCP4922 channel A through 0 / 25 / 50 / 75 / 100% / 0,
 *          holding each level 2 s, logging the voltage to expect.
 *
 * @return  ESP_OK if every step transmitted, otherwise the first error.
 *
 * @warning Blocks for ~12 s. Meter U13 pin 14 (VOUTA) against GND while it
 *          runs. Channel A only: it stops at the MAX4660, which sits on pedal
 *          pass-through, so nothing reaches the kart. Channel B is deliberately
 *          not swept — brake reaches CN10.2 unmuxed and a sweep would brake.
 */
esp_err_t KM_GPIO_McpSelfTest(void);
#endif

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
