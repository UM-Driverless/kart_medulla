/******************************************************************************
 * @file    km_coms.h
 * @brief   Public API for the KM_COMS communication library.
 *
 * This header defines the interface for the KM_COMS library, which provides:
 *  - UART-based communication between an ESP32 and NVIDIA Orin (or other host)
 *  - Message framing, CRC checks, and payload handling
 *  - Functions to send, receive, and process messages
 *
 * The library supports periodic tasks that handle incoming/outgoing messages
 * safely in a FreeRTOS environment.
 *
 * @author  Adrian Navarredonda Arizaleta
 * @date    14-02-2026
 * @version 1.0
 *****************************************************************************/

#ifndef KM_COMS_H
#define KM_COMS_H

/******************************* INCLUDES *************************************/
// Includes necesarios para la API pública
#include "esp_log.h" // Para log
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "km_gpio.h"
#include "km_objects.h"
#include <stdint.h>

// Estructura mensaje, | es solo para visualizar los disintintos campos, no esta
// en el mensaje
//  | SOF | LEN | TYPE | PAYLOAD | CRC |

// Donde:

//     SOF → 1 byte (ej: 0xAA)

//     LEN → 1 byte (longitud del payload)

//     TYPE → 1 byte (tipo de mensaje)

//     PAYLOAD → N bytes

//     CRC → 1 byte (checksum simple XOR)

// -------------------------- Tipos de mensajes --------------------------------
// ## Orin <-> ESP32 Messaging (UTF-8)

// All inter-computer messages are UTF-8 text. Payloads are defined below; line framing and exact
// field ordering should follow the agreed message definitions when implemented.

// **ESP32 -> Orin (telemetry):**
// - `actual_speed` (m/s, float)
// - `actual_acc` (m/s^2, float, signed)
// - `actual_braking` (0-1, float; interpreted as brake pedal or hydraulic pressure)
// - `actual_steering` (rad, float)
// - `mission` (enum; state machine owner TBD)
// - `machine_state` (enum; mission sub-state)
// - `actual_shutdown` (0/1, end of SDC loop state)
// - `esp32_heartbeat`

// **Orin -> ESP32 (commands):**
// - `target_throttle` (0-1, float)
// - `target_braking` (0-1, float)
// - `target_steering` (-1 to 1, float)
// - `mission` (enum; state machine owner TBD)
// - `machine_state` (enum; mission sub-state)
// - `orin_heartbeat`

/******************************* DEFINES PÚBLICAS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería

#define KM_COMS_SOM 0xAA
#define KM_COMS_MSG_MAX_LEN 256
#define KM_COMS_RX_CHUNK 64 // Lectura de la UART por bloques
#define BUF_SIZE_TX 2048
#define BUF_SIZE_RX 2048

/******************************* TIPOS PÚBLICOS ********************************/
// Estructuras, enums, typedefs públicos

/**
 * @brief Enum defining all message types supported by KM_COMS.
 * 
 * Values are split between ESP32 -> Orin and Orin -> ESP32 messages.
 */
typedef enum
{
    // ==========================
    // ESP32 --> Orin (0x01 - 0x1F)
    // ==========================
    ESP_ACT_SPEED           = 0x01, /**< Actual speed telemetry (m/s) */
    ESP_ACT_THROTTLE        = 0x02, /**< Actual throttle telemetry (0-100) */
    ESP_ACT_BRAKING         = 0x03, /**< Actual braking telemetry (0-100) */
    ESP_ACT_STEERING        = 0x04, /**< Actual steering telemetry (-100 to 100) */
    ESP_MISION              = 0x05, /**< Current mission/state */
    ESP_MACHINE_STATE       = 0x06, /**< Current machine sub-state */
    ESP_ACT_SHUTDOWN        = 0x07, /**< Shutdown state */
    ESP_HEARTBEAT           = 0x08, /**< ESP32 heartbeat message */
    ESP_COMPLETE            = 0x09, /**< Full telemetry message */
    ESP_DIAG_STEERING       = 0x0A, /**< AS5600 diagnostic registers (debug) */
    ESP_HEALTH_STATUS       = 0x0B, /**< Periodic health, 1 Hz, 7 fields:
                                         [flags, agc, heap_kb, i2c_errors, steer_frames,
                                          steer_rejects, steer_trip_age_s].
                                         flags: bit0 magnet_ok, bit1 i2c_ok, bit2 heap_ok,
                                         bit3 steer_ok (read valid NOW), bit4 steer_trip (latched).
                                         steer_trip_age_s is seconds since the latch tripped, or -1
                                         while clear — bit4 alone cannot tell a live fault from one
                                         latched during an earlier boot. Fields are only ever
                                         APPENDED, so a consumer reading the first N keeps working. */
    ESP_PNEUMATIC           = 0x0C, /**< Pneumatics telemetry, 8 fields, ~20 Hz: [tank_adc, comp_duty,
                                         piston_adc, comp_state, control_iters, ledc_readback,
                                         gpio_init_err, sdc_level]. Fields are only ever APPENDED, so a
                                         consumer that reads the first N keeps working. */
    ESP_STEER_PID           = 0x0D, /**< Steering PID gains actually in force, 1 Hz, 5 fields:
                                         [override, kp x1000, ki x1000, kd x1000, pwm_limit x1000].
                                         Read back from the live controller after clamping, so it
                                         reports what the firmware is using, not what was asked for. */
    ESP_PEDALS              = 0x0E, /**< Driver pedal positions, ~20 Hz, 4 fields:
                                         [acc_mv, brake_mv, acc_effort, brake_effort].
                                         *_mv = calibrated millivolts at the ADC pin (GPIO 4 / GPIO 5,
                                         half the 0-5 V pedal signal through the board's 10k/10k
                                         divider). *_effort = normalized 0-255 from provisional
                                         min/max constants in main.c — trust the mV fields until
                                         those are calibrated on the kart. */

    // ==========================
    // Orin --> ESP32 (0x20 - 0x3F)
    // ==========================
    ORIN_TARG_THROTTLE      = 0x20, /**< Target throttle command (0-100) */
    ORIN_TARG_BRAKING       = 0x21, /**< Target braking command (0-100) */
    ORIN_TARG_STEERING      = 0x22, /**< Target steering command (-100 to 100) */
    ORIN_MISION             = 0x23, /**< Mission command/state */
    ORIN_MACHINE_STATE      = 0x24, /**< Machine state command */
    ORIN_HEARTBEAT          = 0x25, /**< Orin heartbeat message */
    ORIN_SHUTDOWN           = 0x26, /**< Shutdown command */
    ORIN_COMPLETE           = 0x27, /**< Complete command with all fields */
    ORIN_CALIBRATE_STEERING = 0x28, /**< Calibrate steering sensor */
    ORIN_STEER_MODE         = 0x29, /**< Steering mode: 0=PID, 1=direct PWM */
    ORIN_COMPRESSOR_DISABLE = 0x2A, /**< EBS compressor latch: 0=run normally, 1=operator disabled */
    ORIN_STEER_PID          = 0x2B, /**< Live steering PID tuning, 5 fields:
                                         [override, kp x1000, ki x1000, kd x1000, pwm_limit x1000].
                                         override=0 restores the gains compiled into main.c and
                                         ignores the other four. */

    // ==========================
    // Others (0x40 - 0xFF)
    // ==========================
} message_type_t;

/**
 * @brief Structure representing a KM_COMS message.
 *
 * Message format:
 * | SOF | LEN | TYPE | PAYLOAD | CRC |
 *
 * - SOF: 1 byte Start-of-Message (0xAA)
 * - LEN: 1 byte payload length
 * - TYPE: 1 byte message type
 * - PAYLOAD: variable-length data
 * - CRC: 1 byte checksum
 */
typedef struct {
    uint8_t len;                            /**< Length of the payload */
    uint8_t type;                           /**< Message type (see message_type_t) */
    int32_t payload[KM_COMS_MSG_MAX_LEN-5]; /**< Payload data */
    uint8_t crc;                            /**< CRC checksum of the message */
} km_coms_msg;

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

// extern int ejemplo_variable_publica;

/******************************* FUNCIONES PÚBLICAS ***************************/
/**
 * @brief Initializes the KM_COMS communication library for a given UART port.
 *
 * This function sets up the internal UART interface for the communication library,
 * creates a mutex for thread-safe access to the RX buffer, and installs/configures
 * the UART driver with the appropriate TX/RX pins and buffer sizes.
 *
 * Steps performed by the function:
 * 1. Stores the UART port to be used by the library (`km_coms_uart`).
 * 2. Creates a mutex (`km_coms_mutex`) to protect access to the internal RX buffer.
 *    - Returns `ESP_ERR_NO_MEM` if mutex creation fails.
 * 3. Installs the UART driver with predefined RX and TX buffer sizes.
 * 4. Configures UART parameters (baud rate, data bits, parity, stop bits, etc.).
 * 5. Sets the TX/RX pins based on the selected UART port.
 *
 * @param uart_port The UART port to use (e.g., UART_NUM_1, UART_NUM_2).
 * @return ESP_OK if initialization succeeded.
 * @return ESP_ERR_NO_MEM if the mutex could not be created.
 *
 * @note This function should be called once at the start of the system before
 *       sending or receiving messages.
 * @note Thread-safety for the RX buffer is ensured via `km_coms_mutex`.
 */
esp_err_t KM_COMS_Init(uart_port_t uart_port);

/**
 * @brief Sends a communication message over UART.
 *
 * This function constructs and sends a message frame according to the KM_COMS protocol:
 * [SOM | LEN | TYPE | PAYLOAD | CRC]. It calculates the CRC for the message and attempts
 * to send it via UART up to 5 times if the UART buffer is full.
 *
 * Steps performed by the function:
 * 1. Validates that the payload length does not exceed the maximum allowed.
 * 2. Fills a `km_coms_msg` structure with length, type, payload, and calculated CRC.
 * 3. Builds the UART frame: Start-of-Message (SOM), LEN, TYPE, PAYLOAD, CRC.
 * 4. Attempts to send the entire frame via `uart_write_bytes`, retrying up to 5 times
 *    if the UART buffer is temporarily full, with a short delay between attempts.
 * 5. Returns success if the full message is transmitted, or failure if it could not be sent.
 *
 * @param type    The type of message to send (message_type_t).
 * @param payload Pointer to the payload data to send.
 * @param len     Length of the payload in bytes.
 * @return 1 if the message was sent successfully, 0 if sending failed, -1 if payload is too long.
 *
 * @note This function can be called from a FreeRTOS task periodically or on-demand.
 */
int KM_COMS_SendMsg(message_type_t type, int32_t *payload, uint8_t len);

/**
 * @brief Reads bytes from the UART and stores them into the internal RX buffer.
 *
 * This function retrieves available data from the UART interface (`km_coms_uart`) 
 * and appends it to the internal communication library buffer (`rx_buffer`). 
 * It ensures thread-safe access using the `km_coms_mutex` semaphore.
 *
 * Steps performed by the function:
 * 1. Checks how many bytes are currently available in the UART buffer.
 * 2. Reads up to `KM_COMS_RX_CHUNK` bytes from the UART.
 * 3. If bytes are read successfully, acquires the `km_coms_mutex` semaphore.
 * 4. Appends the read bytes to the internal RX buffer.
 *    - If the buffer would overflow, it is reset to prevent memory corruption.
 * 5. Releases the semaphore after updating the buffer.
 *
 *
 * @note Thread-safety is provided via `km_coms_mutex`.
 * @note The internal RX buffer size is limited; excessive incoming data may reset the buffer.
 * @note This function is intended to be called periodically from a FreeRTOS task.
 */
void km_coms_ReceiveMsg(void);

/**
 * @brief Processes all received messages from the RX buffer.
 *
 * This function iterates through the `rx_buffer` to extract and process complete
 * communication messages. Each message is expected to follow the protocol:
 * [SOM | LEN | TYPE | PAYLOAD... | CRC].
 *
 * Steps performed by the function:
 * 1. Acquires the `km_coms_mutex` semaphore to ensure thread-safe access to the RX buffer.
 * 2. Iterates through the buffer while enough bytes remain for at least a minimal message.
 * 3. Searches for the Start-of-Message (SOM) byte. Skips invalid bytes.
 * 4. Reads the payload length (`LEN`) and calculates the total message length.
 * 5. If a complete message is available:
 *    - Constructs a `km_coms_msg` structure with `len`, `type`, `payload`, and `crc`.
 *    - Verifies the CRC using `KM_COMS_crc8()`.
 *    - If the CRC is valid, calls `KM_COMS_ProccessPayload()` to handle the message.
 * 6. After processing, compacts the buffer by moving unprocessed bytes to the beginning.
 * 7. Releases the `km_coms_mutex` semaphore.
 *
 * This function ensures that partially received messages are retained in the buffer
 * until complete data arrives.
 *
 * @note The minimum message length is 4 bytes: SOM + LEN + TYPE + CRC.
 * @note Thread-safety is provided by `km_coms_mutex`.
 * @note This function is intended to be called periodically from a FreeRTOS task.
 */
void KM_COMS_ProccessMsgs(void);

/**
 * @brief Returns the FreeRTOS tick count of the last received Orin command.
 *
 * Used by the control task to implement a comms watchdog: if no command has
 * been received within a timeout (e.g. 1 second), actuator outputs should be
 * zeroed for safety.
 *
 * @return Tick count of the last Orin→ESP32 message, or 0 if none received yet.
 */
TickType_t KM_COMS_GetLastCmdTick(void);

#endif /* KM_COMS_H */
