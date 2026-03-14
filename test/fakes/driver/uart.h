/* Fake driver/uart.h for native unit tests */
#ifndef FAKE_DRIVER_UART_H
#define FAKE_DRIVER_UART_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

typedef int uart_port_t;

#define UART_NUM_0 0
#define UART_NUM_1 1
#define UART_NUM_2 2
#define UART_PIN_NO_CHANGE (-1)

typedef enum { UART_DATA_8_BITS = 3 } uart_word_length_t;
typedef enum { UART_PARITY_DISABLE = 0 } uart_parity_t;
typedef enum { UART_STOP_BITS_1 = 1 } uart_stop_bits_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE = 0 } uart_hw_flowcontrol_t;

typedef struct {
    int baud_rate;
    uart_word_length_t data_bits;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_hw_flowcontrol_t flow_ctrl;
} uart_config_t;

static inline esp_err_t uart_driver_install(uart_port_t p, int rx, int tx, int q, void *h, int f) {
    (void)p;(void)rx;(void)tx;(void)q;(void)h;(void)f; return ESP_OK;
}
static inline esp_err_t uart_driver_delete(uart_port_t p) { (void)p; return ESP_OK; }
static inline esp_err_t uart_param_config(uart_port_t p, const uart_config_t *c) { (void)p;(void)c; return ESP_OK; }
static inline esp_err_t uart_set_pin(uart_port_t p, int tx, int rx, int rts, int cts) {
    (void)p;(void)tx;(void)rx;(void)rts;(void)cts; return ESP_OK;
}

/* Mock UART TX — record bytes for test assertions */
extern uint8_t fake_uart_tx_buf[512];
extern size_t  fake_uart_tx_len;

static inline int uart_write_bytes(uart_port_t p, const void *src, size_t len) {
    (void)p;
    const uint8_t *s = (const uint8_t *)src;
    for (size_t i = 0; i < len && fake_uart_tx_len < sizeof(fake_uart_tx_buf); i++)
        fake_uart_tx_buf[fake_uart_tx_len++] = s[i];
    return (int)len;
}

/* Mock UART RX — tests fill this buffer */
extern uint8_t fake_uart_rx_buf[512];
extern size_t  fake_uart_rx_len;
extern size_t  fake_uart_rx_pos;

static inline esp_err_t uart_get_buffered_data_len(uart_port_t p, size_t *len) {
    (void)p;
    *len = fake_uart_rx_len - fake_uart_rx_pos;
    return ESP_OK;
}

static inline int uart_read_bytes(uart_port_t p, void *dst, uint32_t len, uint32_t ticks) {
    (void)p; (void)ticks;
    size_t avail = fake_uart_rx_len - fake_uart_rx_pos;
    if (len > avail) len = avail;
    uint8_t *d = (uint8_t *)dst;
    for (size_t i = 0; i < len; i++)
        d[i] = fake_uart_rx_buf[fake_uart_rx_pos++];
    return (int)len;
}

#endif
