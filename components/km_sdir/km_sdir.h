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
 * @brief Structure that represents the steering direction sensor (AS5600).
 */
typedef struct {
    uint8_t connected;        /**< 1 if sensor is responding on I2C, 0 otherwise */
    uint8_t errorCount;       /**< Consecutive I2C read errors since last success */
    uint8_t max_error_count;  /**< Error threshold before automatic I2C reset */
    uint64_t lastReadTime;    /**< Timestamp of last successful read (microseconds) */
    uint16_t lastRawValue;    /**< Last successfully read raw angle (0-4095) */
    uint16_t centerOffset;    /**< Raw value that corresponds to center (straight ahead) */
} sensor_struct;

/** @brief AS5600 I2C address and angle register addresses. */
typedef enum {
    AS5600_ADDR = 0x36,        /**< 7-bit I2C slave address */
    AS5600_ANGLE_MSB = 0x0C,   /**< Angle register high byte */
    AS5600_ANGLE_LSB = 0x0D    /**< Angle register low byte */
} conection_constans;

/** @brief Sensor value range constants (12-bit encoder, 0-4095). */
typedef enum {
    SENSOR_MIN = 0,            /**< Minimum raw sensor value */
    SENSOR_MAX = 4095,         /**< Maximum raw sensor value */
    SENSOR_CENTER = 2048,      /**< Default center (half of 12-bit range) */
} sensor_constans;

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

// extern int ejemplo_variable_publica;

/******************************* FUNCIONES PÚBLICAS ***************************/

/**
 * @brief  Create and initialize a sensor_struct with default values.
 * @param  max_error_count  Number of consecutive I2C errors before
 *                          automatic I2C bus reset is triggered.
 * @return Initialized sensor_struct (not yet connected to hardware).
 * @note   centerOffset defaults to SENSOR_CENTER (2048). Call
 *         KM_SDIR_LoadCalibration() to restore a saved offset, then
 *         KM_SDIR_Begin() to start I2C communication.
 */
sensor_struct KM_SDIR_Init(int8_t max_error_count);

/**
 * @brief  Initialize the I2C bus and verify the AS5600 sensor is present.
 * @param  sensor  Pointer to an initialized sensor_struct.
 * @param  sdaPin  GPIO pin number for I2C SDA.
 * @param  sclPin  GPIO pin number for I2C SCL.
 * @return 1 if the sensor responded on I2C, 0 on failure.
 * @note   Configures I2C_NUM_0 as master at 400 kHz with internal pull-ups.
 */
int8_t KM_SDIR_Begin(sensor_struct *sensor, gpio_num_t sdaPin, gpio_num_t sclPin);

/**
 * @brief  Read the raw 12-bit angle value from the AS5600.
 * @param  sensor  Pointer to the sensor state.
 * @return Raw angle value in the range 0-4095.
 * @note   On I2C failure, returns the last successfully read value and
 *         increments errorCount. When errorCount reaches max_error_count,
 *         the I2C bus is automatically reset.
 */
uint16_t KM_SDIR_ReadRaw(sensor_struct *sensor);

/**
 * @brief  Read the steering angle in radians, centered around the offset.
 * @param  sensor  Pointer to the sensor state.
 * @return Angle in radians, approximately -PI to +PI.
 */
float KM_SDIR_ReadAngleRadians(sensor_struct *sensor);

/**
 * @brief  Read the steering angle in degrees, centered around the offset.
 * @param  sensor  Pointer to the sensor state.
 * @return Angle in degrees, approximately -180 to +180.
 */
float KM_SDIR_ReadAngleDegrees(sensor_struct *sensor);

/**
 * @brief  Check whether the sensor is connected and healthy.
 * @param  sensor  Pointer to the sensor state.
 * @return 1 if connected and errorCount is below threshold, 0 otherwise.
 */
int8_t KM_SDIR_isConnected(sensor_struct *sensor);

/**
 * @brief  Reset the I2C bus and re-check sensor presence.
 * @param  sensor  Pointer to the sensor state.
 * @return 1 if the sensor is reachable after reset, 0 on failure.
 * @note   Deletes and reinstalls the I2C driver on I2C_NUM_0 with
 *         hardcoded SDA=21, SCL=22 pins. Includes a 100 ms delay.
 */
int8_t KM_SDIR_ResetI2C(sensor_struct *sensor);

/**
 * @brief  Set the center offset and persist it to NVS.
 * @param  sensor  Pointer to the sensor state.
 * @param  offset  Raw sensor value that represents the center position.
 */
void KM_SDIR_setCenterOffset(sensor_struct *sensor, uint16_t offset);

/**
 * @brief  Load the center offset from NVS into the sensor struct.
 * @param  sensor  Pointer to the sensor state.
 * @note   Call after KM_SDIR_Init() and before KM_SDIR_Begin().
 *         If no calibration is found in NVS, the existing centerOffset
 *         (default SENSOR_CENTER) is kept.
 */
void KM_SDIR_LoadCalibration(sensor_struct *sensor);

/**
 * @brief  Read AS5600 diagnostic registers into a buffer.
 * @param  sensor   Pointer to the sensor state.
 * @param  out_buf  Output buffer (must hold at least 9 bytes).
 *                  Layout: [ZPOS_H, ZPOS_L, MPOS_H, MPOS_L,
 *                           CONF_H, CONF_L, STATUS, AGC, ZMCO].
 * @return Number of bytes written (9 on success).
 * @note   Failed register reads are stored as 0xFF.
 */
uint8_t KM_SDIR_ReadDiagnostics(sensor_struct *sensor, uint8_t *out_buf);

/**
 * @brief  Read AS5600 STATUS and AGC registers (two quick I2C reads).
 * @param  sensor  Pointer to the sensor state.
 * @param  status  Output: STATUS register value (set to 0 on failure).
 * @param  agc     Output: AGC register value (set to 0 on failure).
 * @return 1 if both reads succeeded, 0 if any I2C read failed.
 */
int8_t KM_SDIR_ReadStatusAGC(sensor_struct *sensor, uint8_t *status, uint8_t *agc);

#endif /* KM_SDIR_H */
