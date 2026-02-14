/******************************************************************************
 * @file    nombre_libreria.h
 * @brief   Interfaz pública de la librería.
 * @author  Adrian Navarredonda Arizaleta
 * @date    14-02-2026
 * @version 1.0
 *****************************************************************************/

#ifndef KM_COMS_H
#define KM_COMS_H

/******************************* INCLUDES *************************************/
// Includes necesarios para la API pública
#include <stdint.h>
#include "esp_log.h" // Para log
#include "km_gpio.h"


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

// All inter-computer messages are UTF-8 text. Payloads are defined below; line framing and exact field ordering should follow the agreed message definitions when implemented.

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
#define KM_COMS_PAYLOAD_MAX 255
#define KM_COMS_QUEUE_LEN   16

/******************************* TIPOS PÚBLICOS ********************************/
// Estructuras, enums, typedefs públicos

typedef enum
{
    // ==========================
    // ESP32 --> Orin (0x01 - 0x1F)
    // ==========================
    ESP_ACT_SPEED           = 0x01,
    ESP_ACT_ACCELERATION    = 0x02,
    ESP_ACT_BRAKING         = 0x03,
    ESP_ACT_STEERING        = 0x04,
    ESP_MISION              = 0x05,
    ESP_MACHINE_STATE       = 0x06,
    ESP_ACT_SHUTDOWN        = 0x07,
    ESP_HEARTBEAT           = 0x08,
    ESP_COMPLETE            = 0x09,

    // ==========================
    // Orin --> ESP32 (0x20 - 0x3F)
    // ==========================
    ORIN_TARG_THROTTLE      = 0x20,
    ORIN_TARG_BRAKING       = 0x21,
    ORIN_TARG_STEERING      = 0x22,
    ORIN_MISION             = 0x23,
    ORIN_MACHINE_STATE      = 0x24,
    ORIN_HEARTBEAT          = 0x25,
    ORIN_SHUTDOWN           = 0x26,
    ORIN_COMPLETE           = 0x27,

    // ==========================
    // Others (0x40 - 0xFF)
    // ==========================

} message_type_t;

typedef struct {
    uint8_t len;                            // Len of payload    
    message_type_t type;                    // Type of msg send
    uint8_t payload[KM_COMS_PAYLOAD_MAX];   // Payload del msg
    uint8_t crc;                            // CRC del msg
} km_coms_msg;


/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

// extern int ejemplo_variable_publica;

/******************************* FUNCIONES PÚBLICAS ***************************/
// Inicializa UART y la cola
int KM_COMS_Init(gpio_num_t uart_num);

// Agrega un mensaje a la cola
int KM_COMS_AddMsg(message_type_t type, uint8_t *payload, uint8_t len);

// Función que la tarea FreeRTOS debe ejecutar periódicamente
void KM_COMS_SendMSG(void *pvParameters);

#endif /* KM_COMS_H */
 