/* Fake driver/i2c.h for native unit tests */
#ifndef FAKE_DRIVER_I2C_H
#define FAKE_DRIVER_I2C_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

typedef int i2c_port_t;
#define I2C_NUM_0 0
#define I2C_MODE_MASTER 1
#define I2C_MASTER_WRITE 0
#define I2C_MASTER_READ  1
#define I2C_MASTER_ACK   0
#define I2C_MASTER_NACK  1

typedef int gpio_num_t;
#define GPIO_PULLUP_ENABLE 1

typedef struct {
    int mode;
    int sda_io_num;
    int sda_pullup_en;
    int scl_io_num;
    int scl_pullup_en;
    struct { int clk_speed; } master;
} i2c_config_t;

typedef void * i2c_cmd_handle_t;

static inline esp_err_t i2c_param_config(i2c_port_t p, const i2c_config_t *c) { (void)p;(void)c; return ESP_OK; }
static inline esp_err_t i2c_driver_install(i2c_port_t p, int m, int rx, int tx, int f) {
    (void)p;(void)m;(void)rx;(void)tx;(void)f; return ESP_OK;
}
static inline esp_err_t i2c_driver_delete(i2c_port_t p) { (void)p; return ESP_OK; }

static inline i2c_cmd_handle_t i2c_cmd_link_create(void) { return (void *)1; }
static inline void i2c_cmd_link_delete(i2c_cmd_handle_t h) { (void)h; }
static inline esp_err_t i2c_master_start(i2c_cmd_handle_t h) { (void)h; return ESP_OK; }
static inline esp_err_t i2c_master_stop(i2c_cmd_handle_t h) { (void)h; return ESP_OK; }
static inline esp_err_t i2c_master_write_byte(i2c_cmd_handle_t h, uint8_t b, bool ack) { (void)h;(void)b;(void)ack; return ESP_OK; }
static inline esp_err_t i2c_master_read(i2c_cmd_handle_t h, uint8_t *d, size_t len, int ack) { (void)h;(void)d;(void)len;(void)ack; return ESP_OK; }
static inline esp_err_t i2c_master_read_byte(i2c_cmd_handle_t h, uint8_t *d, int nack) { (void)h;(void)d;(void)nack; return ESP_OK; }

/* Default: I2C always fails (sensor not connected) */
extern esp_err_t fake_i2c_result;
static inline esp_err_t i2c_master_cmd_begin(i2c_port_t p, i2c_cmd_handle_t h, uint32_t t) {
    (void)p;(void)h;(void)t; return fake_i2c_result;
}

#endif
