/******************************************************************************
 * @file    KM_COMS.c
 * @brief   Implementación de la librería.
 *****************************************************************************/

#include "km_coms.h"

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas
#include "driver/uart.h"
#include "freertos/queue.h"
#include <string.h>

/******************************* MACROS PRIVADAS ********************************/
// Constantes internas, flags de debug

#define KM_COMS_QUEUE_LEN 10
#define KM_COMS_WAIT_SEM_AVAILABLE 5

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)
static uint8_t rx_buffer[KM_COMS_MSG_MAX_LEN - 1]; //[0-255]
static size_t rx_buffer_len = 0;
QueueHandle_t km_coms_queue;
static SemaphoreHandle_t km_coms_mutex;

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/
static uint8_t KM_COMS_crc8(uint8_t len, uint8_t type, const uint8_t *data);

/******************************* FUNCIONES PÚBLICAS ***************************/

esp_err_t KM_COMS_Init(gpio_num_t uart_num) {

    // Crear la cola
    km_coms_queue = xQueueCreate(KM_COMS_QUEUE_LEN, sizeof(km_coms_msg));

    if(km_coms_queue == NULL)
        return 0;

    km_coms_mutex = xSemaphoreCreateMutex();
    if(km_coms_mutex == NULL)
        return 0;

    // Instala el driver UART con buffers TX/RX
    uart_driver_install(UART_NUM_0, BUF_SIZE_RX, BUF_SIZE_TX, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);

    // Asigna pines (TX/RX)
    uart_set_pin(UART_NUM_0, PIN_USB_UART_TX, PIN_USB_UART_RX, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

    return 1;
}

int KM_COMS_SendMsg(message_type_t type, uint8_t *payload, uint8_t len) {
    km_coms_msg msg;
    uint8_t frame[KM_COMS_MSG_MAX_LEN - 1];
    size_t total_sent = 0;
    int sent, attempts = 0;

    if(len > KM_COMS_MSG_MAX_LEN)
        return -1;

    msg.len = (uint8_t)len;
    msg.type = (uint8_t)type;

    memcpy(msg.payload, payload, len);
    msg.crc = KM_COMS_crc8(msg.len, msg.type, msg.payload); // LEN + TYPE + PAYLOAD

    // Armar frame
    frame[0] = (uint8_t)KM_COMS_SOM;
    frame[1] = (uint8_t)msg.len;
    frame[2] = (uint8_t)msg.type;
    memcpy(&frame[3], msg.payload, msg.len);
    frame[3 + msg.len] = (uint8_t)msg.crc;

    // Enviar mensaje, se intenta 5 veces
    while(total_sent < len+4 && attempts < 5) {
        sent = uart_write_bytes(UART_NUM_0, frame + total_sent, (len + 4) - total_sent);
        total_sent += sent;

        if(sent < (len - total_sent)) {
            // buffer lleno, espera a que se vacíe
            attempts++;
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }

    // NO se ha podido enviar el mensaje correctamente
    if(attempts >= 5 || total_sent < 4 + len){
        ESP_LOGE("KM_coms", "El msg no se ha enviado, bytes enviados: %d, bytes que habia que enviar: %d", total_sent, 4+len);
        return 0;
    }

    // Se ha enviado el mensaje correctamente
    return 1;
}

// Esta funcion sube los bytes del buffer de la uart al buffer de la libreria
void km_coms_ReceiveMsg(void) {
    uint8_t uart_chunk[KM_COMS_RX_CHUNK];
    size_t len_read = 0;
    uint8_t bytes2read = 0;

    // 1. Verificar cuantos bytes hay en el buffer UART
    size_t uart_len = 0;
    uart_get_buffered_data_len(UART_NUM_0, &uart_len);
    debug_led_off();
    if(uart_len == 0)
        return;

    // 2. Leer hasta KM_COMS_RX_CHUNK bytes de la UART
    if(uart_len > KM_COMS_RX_CHUNK)
        bytes2read = KM_COMS_RX_CHUNK;
    else
        bytes2read = uart_len;

    len_read = uart_read_bytes(UART_NUM_0, uart_chunk, bytes2read, 0);
    if(len_read == 0)
        return;

    if(xSemaphoreTake(km_coms_mutex, pdMS_TO_TICKS(KM_COMS_WAIT_SEM_AVAILABLE)) == pdTRUE) {
        // 3. Copiar bytes al buffer interno
        if(rx_buffer_len + len_read > sizeof(rx_buffer)) {
            // Overflow, reiniciar buffer
            rx_buffer_len = 0;
        }
        memcpy(rx_buffer + rx_buffer_len, uart_chunk, len_read);
        rx_buffer_len += len_read;
        xSemaphoreGive(km_coms_mutex);
    }
}

// Esta funcion procesa los bytes que se han dejado en el buffer de la libreria
// y recrea el mensaje
void KM_COMS_ProccessMsgs(void) {
    // Procesar mensajes completos
    size_t processed = 0;
    if(xSemaphoreTake(km_coms_mutex, pdMS_TO_TICKS(KM_COMS_WAIT_SEM_AVAILABLE)) == pdTRUE) {
        while(rx_buffer_len - processed >= 4) { // Mínimo tamaño de mensaje: SOM + LEN + TYPE + CRC
            if(rx_buffer[processed] != KM_COMS_SOM) {
                processed++; // Buscar siguiente SOM
                continue;
            }

            uint8_t payload_len = rx_buffer[processed + 1];
            size_t total_len = 4 + payload_len; // SOM + LEN + TYPE + PAYLOAD + CRC

            if(rx_buffer_len - processed < total_len) {
                // Mensaje incompleto, esperar más bytes
                break;
            }

            // Construir mensaje
            km_coms_msg msg;
            msg.len = payload_len;
            msg.type = rx_buffer[processed + 2];
            memcpy(msg.payload, rx_buffer + processed + 3, payload_len);
            msg.crc = rx_buffer[processed + 3 + payload_len];

            // Liberar mutex
            xSemaphoreGive(km_coms_mutex);

            // Verificar CRC
            uint8_t crc_calc = KM_COMS_crc8(msg.len, msg.type, msg.payload); // LEN + TYPE + PAYLOAD
            if(crc_calc == msg.crc) {
                // Mensaje válido, enviar a la cola
                if(km_coms_queue) {
                    xQueueSend(km_coms_queue, &msg, 0);
                }
            }

            // Avanzar processed al siguiente mensaje
            processed += total_len;
        }
    }

    if(xSemaphoreTake(km_coms_mutex, pdMS_TO_TICKS(KM_COMS_WAIT_SEM_AVAILABLE)) == pdTRUE) {
        // 5. Mover los bytes restantes al inicio del buffer
        if(processed > 0 && processed < rx_buffer_len) {
            memmove(rx_buffer, rx_buffer + processed, rx_buffer_len - processed);
            rx_buffer_len -= processed;
        } else if(processed == rx_buffer_len) {
            rx_buffer_len = 0;
        }
        xSemaphoreGive(km_coms_mutex);
    }
}

/******************************* FUNCIONES PRIVADAS ***************************/

static uint8_t KM_COMS_crc8(uint8_t len, uint8_t type, const uint8_t *data) {
    uint8_t crc = 0x00;

    crc ^= len;
    for (int j = 0; j < 8; j++) {
        crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }

    crc ^= type;
    for (int j = 0; j < 8; j++) {
        crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }

    return crc;
}

/******************************* FIN DE ARCHIVO ********************************/
