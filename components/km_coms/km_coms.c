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

#define KM_COMS_WAIT_SEM_AVAILABLE 5

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)
static uint8_t rx_buffer[KM_COMS_MSG_MAX_LEN - 1]; //[0-255]
static size_t rx_buffer_len = 0;
static SemaphoreHandle_t km_coms_mutex;
static uart_port_t km_coms_uart = UART_NUM_0;

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/
static void KM_COMS_ProccessPayload(km_coms_msg msg);
static uint8_t KM_COMS_crc8(uint8_t len, uint8_t type, const uint8_t *data);

/******************************* FUNCIONES PÚBLICAS ***************************/

esp_err_t KM_COMS_Init(uart_port_t uart_port) {
    km_coms_uart = uart_port;

    km_coms_mutex = xSemaphoreCreateMutex();
    if(km_coms_mutex == NULL)
        return ESP_ERR_NO_MEM;

    // Instala el driver UART con buffers TX/RX
    uart_driver_install(km_coms_uart, BUF_SIZE_RX, BUF_SIZE_TX, 0, NULL, 0);
    uart_param_config(km_coms_uart, &uart_config);

    // Asigna pines según el puerto
    if(km_coms_uart == UART_NUM_2) {
        uart_set_pin(km_coms_uart, PIN_ORIN_UART_TX, PIN_ORIN_UART_RX,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    } else {
        uart_set_pin(km_coms_uart, PIN_USB_UART_TX, PIN_USB_UART_RX,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }

    return ESP_OK;
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
        sent = uart_write_bytes(km_coms_uart, frame + total_sent, (len + 4) - total_sent);
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


void km_coms_ReceiveMsg(void) {
    uint8_t uart_chunk[KM_COMS_RX_CHUNK];
    size_t len_read = 0;
    uint8_t bytes2read = 0;

    // 1. Verificar cuantos bytes hay en el buffer UART
    size_t uart_len = 0;
    uart_get_buffered_data_len(km_coms_uart, &uart_len);
    if(uart_len == 0)
        return;

    // 2. Leer hasta KM_COMS_RX_CHUNK bytes de la UART
    if(uart_len > KM_COMS_RX_CHUNK)
        bytes2read = KM_COMS_RX_CHUNK;
    else
        bytes2read = uart_len;

    len_read = uart_read_bytes(km_coms_uart, uart_chunk, bytes2read, 0);
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

void KM_COMS_ProccessMsgs(void) {
    size_t processed = 0;

    if(xSemaphoreTake(km_coms_mutex, pdMS_TO_TICKS(KM_COMS_WAIT_SEM_AVAILABLE)) != pdTRUE)
        return;

    while(rx_buffer_len - processed >= 4) { // Mínimo: SOM + LEN + TYPE + CRC
        if(rx_buffer[processed] != KM_COMS_SOM) {
            processed++;
            continue;
        }

        uint8_t payload_len = rx_buffer[processed + 1];
        size_t total_len = 4 + payload_len; // SOM + LEN + TYPE + PAYLOAD + CRC

        if(rx_buffer_len - processed < total_len)
            break; // Mensaje incompleto, esperar más bytes

        // Construir mensaje
        km_coms_msg msg;
        msg.len = payload_len;
        msg.type = rx_buffer[processed + 2];
        memcpy(msg.payload, rx_buffer + processed + 3, payload_len);
        msg.crc = rx_buffer[processed + 3 + payload_len];

        // Verificar CRC
        uint8_t crc_calc = KM_COMS_crc8(msg.len, msg.type, msg.payload);
        if(crc_calc == msg.crc) {
            KM_COMS_ProccessPayload(msg);
        }

        processed += total_len;
    }

    // Compactar buffer: mover bytes no procesados al inicio
    if(processed > 0 && processed < rx_buffer_len) {
        memmove(rx_buffer, rx_buffer + processed, rx_buffer_len - processed);
        rx_buffer_len -= processed;
    } else if(processed >= rx_buffer_len) {
        rx_buffer_len = 0;
    }

    xSemaphoreGive(km_coms_mutex);
}

/******************************* FUNCIONES PRIVADAS ***************************/

/**
 * @brief Processes the payload of the incoming message and updates the corresponding application objects.
 *
 * This function interprets the payload of a received message (`km_coms_msg`) based on its `type`.
 * Depending on the message type, it extracts the relevant data from the payload, converts it to
 * a 64-bit integer (`int64_t`), and updates the appropriate shared object using
 * `KM_OBJ_SetObjectValue()`.
 *
 * Supported message types include:
 * - ORIN_TARG_THROTTLE: 1-byte payload representing the target throttle.
 * - ORIN_TARG_BRAKING: 1-byte payload representing the target braking.
 * - ORIN_TARG_STEERING: 2-byte payload where the first byte indicates direction and the
 *   second byte the magnitude. 0->positive turn, 1->negative turn
 * - ORIN_MISION: 1-byte payload representing the current mission state.
 * - ORIN_MACHINE_STATE: 1-byte payload representing the machine's current state.
 * - ORIN_HEARTBEAT: message indicating heartbeat; currently not processed.
 * - ORIN_SHUTDOWN: 1-byte payload indicating shutdown command.
 * - ORIN_COMPLETE: 7-byte payload updating all above objects in a single message.
 *
 * Invalid payloads (wrong length or unknown direction for steering) are ignored.
 *
 * @param msg The incoming message to process.
 */
static void KM_COMS_ProccessPayload(km_coms_msg msg) {
    ESP_LOGI("KM_coms", "RX msg type=0x%02X len=%d crc=0x%02X", msg.type, msg.len, msg.crc);

    int64_t object_value = 0;
    int8_t direction;

    switch (msg.type)
    {
    case ORIN_TARG_THROTTLE:
        if (msg.len != 1) return; // Invalid payload
        
        object_value = (int64_t)msg.payload[0];
        KM_OBJ_SetObjectValue(TARGET_THROTTLE, object_value);
        break;

    case ORIN_TARG_BRAKING:
        if (msg.len != 1) return; // Invalid payload
        object_value = (int64_t)msg.payload[0];
        KM_OBJ_SetObjectValue(TARGET_BRAKING, object_value);
        break;

    case ORIN_TARG_STEERING:
        if (msg.len != 2) return; // Invalid payload
        direction = msg.payload[0];
        if (direction == 0) {
            object_value = (int64_t)msg.payload[1];
        } else if (direction == 1){
            object_value -= (int64_t)msg.payload[1];
        } else {
            return; // Invalid payload
        }

        KM_OBJ_SetObjectValue(TARGET_STEERING, object_value);
        break;

    case ORIN_MISION:
        if (msg.len != 1) return; // Invalid payload
        object_value = (int64_t)msg.payload[0];
        KM_OBJ_SetObjectValue(MISION_ORIN, object_value);
        break;

    case ORIN_MACHINE_STATE:
        if (msg.len != 1) return; // Invalid payload
        object_value = (int64_t)msg.payload[0];
        KM_OBJ_SetObjectValue(MACHINE_STATE_ORIN, object_value);
        break;

    case ORIN_HEARTBEAT:
        // Ns si guardarlo en la libreria de variables o reinicar algo. ns
        break;

    case ORIN_SHUTDOWN:
        if (msg.len != 1) return; // Invalid payload
        object_value = (int64_t)msg.payload[0];
        KM_OBJ_SetObjectValue(SHUTDOWN_ORIN, object_value);
        break;

    case ORIN_COMPLETE:
        if (msg.len != 7) return; // Invalid payload
        object_value = (int64_t)msg.payload[0];
        KM_OBJ_SetObjectValue(TARGET_THROTTLE, object_value);

        object_value = (int64_t)msg.payload[1];
        KM_OBJ_SetObjectValue(TARGET_BRAKING, object_value);

        direction = msg.payload[2];
        if (direction == 0) {
            object_value = (int64_t)msg.payload[3];
        } else if (direction == 1){
            object_value -= (int64_t)msg.payload[3];
        } else {
            return; // Invalid payload
        }
        KM_OBJ_SetObjectValue(TARGET_STEERING, object_value);

        object_value = (int64_t)msg.payload[4];
        KM_OBJ_SetObjectValue(MISION_ORIN, object_value);

        object_value = (int64_t)msg.payload[5];
        KM_OBJ_SetObjectValue(MACHINE_STATE_ORIN, object_value);

        object_value = (int64_t)msg.payload[6];
        KM_OBJ_SetObjectValue(SHUTDOWN_ORIN, object_value);
        break;
    
    default:
        break;
    }
}

/**
 * @brief Calculates an 8-bit CRC checksum for a message.
 *
 * This function computes a CRC-8 checksum using the polynomial 0x07. The calculation
 * includes the message length (`len`), message type (`type`), and the actual payload data.
 * The CRC is computed sequentially:
 * 1. XOR with the message length and process 8 bits.
 * 2. XOR with the message type and process 8 bits.
 * 3. XOR each byte of the payload data and process 8 bits per byte.
 *
 * This checksum is used for detecting errors in communication messages.
 *
 * @param len  Length of the payload data in bytes.
 * @param type Message type identifier.
 * @param data Pointer to the payload data array.
 * @return The computed 8-bit CRC value.
 */
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
