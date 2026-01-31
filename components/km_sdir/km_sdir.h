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
// Includes necesarios para la API pública

#include <Arduino.h>
#include <Wire.h>
 
/******************************* DEFINES PÚBLICAS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería

/**
 * @brief Structure that reperesents the direction sensor
 */
typedef struct {
    bool connected;
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

const float MAX_RAD = PI;

/******************************* TIPOS PÚBLICOS ********************************/
// Estructuras, enums, typedefs públicos

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
int8_t KM_SDIR_Begin(sensor_struct *sensor, int8_t sdaPin, int8_t sclPin);

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

// Get center offset for calibration
void KM_SDIR_setCenterOffset(sensor_struct *sensor, uint16_t offset);

#endif /* KM_SDIR_H */
