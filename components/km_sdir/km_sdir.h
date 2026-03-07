/******************************************************************************
 * @file    km_dsen.h
 * @brief   Libreria para usar el sensor de la direccion.
 * @author  Adrian Navarredonda Arizaleta
 * @date    27-01-2026
 * @version 1.0
 *****************************************************************************/

#ifndef KM_SDIR_H
#define KM_SDIR_H

/******************************* INCLUDES *************************************/
#include "driver/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include "esp_timer.h"
 
/******************************* DEFINES PÚBLICAS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería


#define PI 3.1415

/******************************* TIPOS PÚBLICOS ********************************/
// Estructuras, enums, typedefs públicos
/**
 * @brief Structure that reperesents the direction sensor
 */
typedef struct {
    uint8_t connected;
    uint8_t errorCount;
    uint8_t max_error_count;
    uint64_t lastReadTime;
    uint16_t lastRawValue;
    uint16_t centerOffset;
} sensor_struct;

// Represents the conection constans
typedef enum {
    AS5600_ADDR = 0x36,
    AS5600_ANGLE_MSB = 0x0C,
    AS5600_ANGLE_LSB = 0x0D
} conection_constans;

typedef enum {
    SENSOR_MIN = 0,
    SENSOR_MAX = 4095,
    SENSOR_CENTER = 2048,
} sensor_constans;

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

// extern int ejemplo_variable_publica;

/******************************* FUNCIONES PÚBLICAS ***************************/

/**
 * @brief   Inicializa una conexion i2c.
 * @return  retorna el struct conection i2c inicializado
 */
sensor_struct KM_SDIR_Init(int8_t max_error_count);

// Initialize the sensor
int8_t KM_SDIR_Begin(sensor_struct *sensor, gpio_num_t sdaPin, gpio_num_t sclPin);

// Read raw sensor value (0-4095)
uint16_t KM_SDIR_ReadRaw(sensor_struct *sensor);

// Read angle in radians (-PI to PI)
float KM_SDIR_ReadAngleRadians(sensor_struct *sensor);

// Read angle in degrees (-180 to 180)
float KM_SDIR_ReadAngleDegrees(sensor_struct *sensor);

// Check if sensor is connected
int8_t KM_SDIR_isConnected(sensor_struct *sensor);

// Reset I2C communication if errors occur
int8_t KM_SDIR_ResetI2C(sensor_struct *sensor);

// Set center offset and save to NVS
void KM_SDIR_setCenterOffset(sensor_struct *sensor, uint16_t offset);

// Load center offset from NVS (call after KM_SDIR_Init, before KM_SDIR_Begin)
void KM_SDIR_LoadCalibration(sensor_struct *sensor);

// Read AS5600 diagnostic registers into buffer
// Buffer layout: [ZPOS_H, ZPOS_L, MPOS_H, MPOS_L, CONF_H, CONF_L, STATUS, AGC, ZMCO]
// Returns number of bytes written (9 on success)
uint8_t KM_SDIR_ReadDiagnostics(sensor_struct *sensor, uint8_t *out_buf);

#endif /* KM_SDIR_H */
