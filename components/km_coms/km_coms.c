/******************************************************************************
 * @file    KM_COMS.c
 * @brief   Implementación de la librería.
 *****************************************************************************/

#include "km_coms.h"

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas
#include "freertos/queue.h"
#include <string.h>
#include "driver/uart.h"

/******************************* MACROS PRIVADAS ********************************/
// Constantes internas, flags de debug

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)
static QueueHandle_t km_coms_queue;

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/
static uint8_t KM_COMS_crc8(const uint8_t *data, uint8_t len);

/******************************* FUNCIONES PÚBLICAS ***************************/

int KM_COMS_Init(gpio_num_t uart_num)
{

    // Crear cola
    km_coms_queue = xQueueCreate(KM_COMS_QUEUE_LEN, sizeof(km_coms_msg));
    if (!km_coms_queue) return -1;

    return 0;
}

int KM_COMS_AddMsg(message_type_t type, uint8_t *payload, uint8_t len)
{
    if (!km_coms_queue) return -1;
    if (len > KM_COMS_PAYLOAD_MAX) return -2;

    km_coms_msg msg;
    msg.len = len;
    msg.type = (uint8_t)type;

    memcpy(msg.payload, payload, len);
    msg.crc = KM_COMS_crc8(&msg.len, len + 2);  // LEN + TYPE + PAYLOAD

    // Enviar a la cola sin bloquear
    if (xQueueSend(km_coms_queue, &msg, 0) != pdPASS)
    {
        ESP_LOGW("KM_COMS", "Cola llena, mensaje perdido");
        return -3;
    }

    return 0;
}

// ---------------------- Tarea de envío ----------------------
void km_coms_task_process(void *pvParameters)
{
    km_coms_msg msg;

    while(1)
    {
        if (xQueueReceive(km_coms_queue, &msg, 10) == pdPASS) // AJUSTAR TIEMPO
        {
            // Armar frame
            uint8_t frame[4 + KM_COMS_PAYLOAD_MAX];
            frame[0] = KM_COMS_SOM;
            frame[1] = msg.len;
            frame[2] = msg.type;
            memcpy(&frame[3], msg.payload, msg.len);
            frame[3 + msg.len] = msg.crc;

            // Enviar solo bytes necesarios
            uart_write_bytes(PIN_USB_UART_TX, (const char *)frame, 4 + msg.len);
        }
    }
}

/******************************* FUNCIONES PRIVADAS ***************************/

static uint8_t KM_COMS_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}


/******************************* FIN DE ARCHIVO ********************************/
 