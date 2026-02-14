/******************************************************************************
* @file    km_sdir.c
* @brief   Librería para usar el sensor de dirección (ESP-IDF)
* @author Adrian Navarredonda
* @date 31-01-2026
*****************************************************************************/

#include "km_sdir.h"

/******************************* MACROS PRIVADAS ******************************/
#define TAG "KM_SDIR"
#define I2C_MASTER_TIMEOUT_MS 1000

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/
static int8_t KM_SDIR_ReadRegisters(sensor_struct *sensor, uint8_t reg, uint8_t* data, uint8_t len);
static float KM_SDIR_ReadAngle(sensor_struct *sensor);

/******************************* FUNCIONES PÚBLICAS ***************************/

sensor_struct KM_SDIR_Init(int8_t max_error_count) {
    sensor_struct sensor;

    sensor.centerOffset = SENSOR_CENTER;
    sensor.connected = 0;
    sensor.errorCount = 0;
    sensor.lastRawValue = 0;
    sensor.lastReadTime = 0;
    sensor.max_error_count = max_error_count;

    return sensor;
}

// Inicializa I2C
int8_t KM_SDIR_Begin(sensor_struct *sensor, gpio_num_t sda, gpio_num_t scl) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    i2c_param_config(I2C_NUM_0, &conf);
    if (i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver");
        return 0;
    }

    uint8_t testData[2];
    sensor->connected = KM_SDIR_ReadRegisters(sensor, AS5600_ANGLE_MSB, testData, 2);
    if (sensor->connected) {
        ESP_LOGI(TAG, "AS5600 sensor initialized successfully");
    } else {
        ESP_LOGW(TAG, "AS5600 sensor not found!");
    }

    return sensor->connected;
}

// Leer valor crudo (0-4095)
uint16_t KM_SDIR_ReadRaw(sensor_struct *sensor) {
    uint8_t data[2];

    if (KM_SDIR_ReadRegisters(sensor, AS5600_ANGLE_MSB, data, 2)) {
        sensor->lastRawValue = ((uint16_t)data[0] << 8) | data[1];
        sensor->lastReadTime = esp_timer_get_time(); // microsegundos desde boot
        sensor->errorCount = 0;
        return sensor->lastRawValue;
    } else {
        sensor->errorCount++;
        if (sensor->errorCount >= sensor->max_error_count) {
            ESP_LOGW(TAG, "Error reading sensor, resetting I2C");
            KM_SDIR_ResetI2C(sensor);
            sensor->errorCount = 0;
        }
        return sensor->lastRawValue;
    }
}

// Leer ángulo en radianes
float KM_SDIR_ReadAngleRadians(sensor_struct *sensor) {
    return KM_SDIR_ReadAngle(sensor);
}

// Leer ángulo en grados
float KM_SDIR_ReadAngleDegrees(sensor_struct *sensor) {
    return KM_SDIR_ReadAngle(sensor) * 180.0f / PI;
}

// Verifica si el sensor está conectado
int8_t KM_SDIR_isConnected(sensor_struct *sensor) {
    return sensor->connected && (sensor->errorCount < sensor->max_error_count);
}

// Reset de I2C
int8_t KM_SDIR_ResetI2C(sensor_struct *sensor) {
    i2c_driver_delete(I2C_NUM_0);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Reinstala I2C con los mismos parámetros guardados en el sensor
    // Necesitas que sensor tenga guardados i2c_port, sda, scl
    // Por simplicidad vamos a usar I2C_NUM_0 y pines por defecto
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 22,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_NUM_0, &conf);
    if (i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reinstall I2C driver");
        return 0;
    }

    uint8_t testData[2];
    sensor->connected = KM_SDIR_ReadRegisters(sensor, AS5600_ANGLE_MSB, testData, 2);
    return sensor->connected;
}

// Ajuste de offset
void KM_SDIR_setCenterOffset(sensor_struct *sensor, uint16_t offset) {
    sensor->centerOffset = offset;
    ESP_LOGI(TAG, "AS5600 center offset set to: %d", offset);
}

/******************************* FUNCIONES PRIVADAS ***************************/

// Leer registros I2C
static int8_t KM_SDIR_ReadRegisters(sensor_struct *sensor, uint8_t reg, uint8_t* data, uint8_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return 0;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? 1 : 0;
}

// Devuelve ángulo en radianes (-PI a PI)
static float KM_SDIR_ReadAngle(sensor_struct *sensor) {
    uint16_t raw = KM_SDIR_ReadRaw(sensor);

    int16_t centered = (int16_t)raw - sensor->centerOffset;
    float angle = ((float)centered / (float)SENSOR_MAX) * 2.0f * MAX_RAD;

    return angle;
}

/******************************* FIN DE ARCHIVO ********************************/
