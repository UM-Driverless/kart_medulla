/******************************************************************************
* @file    km_sdir.c
* @brief   Implementación de la librería.
* @author Adrian Navarredonda Arizaleta
* @date 31-01-2026
*****************************************************************************/

#include "km_sdir.h"
#include <stdio.h>   // solo si es necesario para debug interno

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas

/******************************* MACROS PRIVADAS ********************************/
// Constantes internas, flags de debug
// #define LIBRERIA_DEBUG 1

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/
int8_t KM_SDIR_ReadRegisters(uint8_t reg, uint8_t* data, uint8_t len);

/******************************* FUNCIONES PÚBLICAS ***************************/
/**
 * @brief   Implementación de la función pública declarada en el header
 */
sensor_struct KM_SDIR_Init(int8_t max_error_count) {

    sensor_struct sensor;

    sensor.centerOffset = SENSOR_CENTER;
    sensor.connected = false;
    sensor.errorCount = 0;
    sensor.lastRawValue = 0;
    sensor.lastReadTime = 0;
    sensor.max_error_count = max_error_count;

    return sensor;
}

// Initialize the sensor
int8_t KM_SDIR_Begin(sensor_struct *sensor ,int8_t sdaPin, int8_t sclPin){
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);  // 400kHz I2C speed

    // Test connection by reading a register
    uint8_t testData[2];
    sensor->connected = readRegisters(AS5600_ANGLE_MSB, testData, 2);

    if (sensor->connected) {
        Serial.println("AS5600 sensor initialized successfully");
    } else {
        Serial.println("AS5600 sensor not found!");
    }

    return sensor->connected;
}

// Read raw sensor value (0-4095)
uint16_t KM_SDIR_ReadRaw(sensor_struct *sensor){
    uint8_t data[2];

    if (readRegisters(AS5600_ANGLE_MSB, data, 2)) {
        sensor->lastRawValue = ((uint16_t)data[0] << 8) | data[1];
        sensor->lastReadTime = millis();
        sensor->errorCount = 0;
        return sensor->lastRawValue;
    } else {
        sensor->errorCount++;
        if (sensor->errorCount >= sensor->max_error_count) {
            Serial.println("Error reading sensor. Restarting I2C communication.");
            resetI2C();
            sensor->errorCount = 0;
        }
        return sensor->lastRawValue;  // Return last good value
    }
}

// Read angle in radians (-PI to PI)
float KM_SDIR_ReadAngleRadians(sensor_struct *sensor){
    return KM_SDIR_ReadAngle(sensor);
}

// Read angle in degrees (-180 to 180)
float KM_SDIR_ReadAngleDegrees(sensor_struct *sensor){
    return readAngle(sensor) * 180.0f / PI;
}

// Check if sensor is connected
int8_t KM_SDIR_isConnected(sensor_struct *sensor){
    return sensor->connected && (sensor->errorCount < sensor->max_error_count);
}

// Reset I2C communication if errors occur
int8_t KM_SDIR_ResetI2C(sensor_struct *sensor){
    Wire.end();
    delay(100);
    Wire.begin();
    Wire.setClock(400000);

    // Test connection again
    uint8_t testData[2];
    sensor->connected = readRegisters(AS5600_ANGLE_MSB, testData, 2);

    return sensor->connected;
}

// Get center offset for calibration
void KM_SDIR_setCenterOffset(sensor_struct *sensor, uint16_t offset){
    sensor->centerOffset = offset;
    Serial.printf("AS5600 center offset set to: %d\n", offset);
}

/******************************* FUNCIONES PRIVADAS ***************************/
/**
 * @brief   Función interna no visible desde fuera
 */
int8_t KM_SDIR_ReadRegisters(uint8_t reg, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return 0; // False
    }

    if (Wire.requestFrom(AS5600_ADDR, len) != len) {
        return 0; // False
    }

    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }

    return 1; // True
}

// Devuelve el angulo en radianes
float KM_SDIR_ReadAngle(sensor_struct *sensor) {
    uint16_t raw = readRaw();

    // Convert to centered value (-2048 to 2047)
    int16_t centered = (int16_t)raw - sensor->centerOffset;

    // Convert to radians (-PI to PI)
    float angle = ((float)centered / (float)SENSOR_MAX) * 2.0f * MAX_RAD;

    return angle;
}

/******************************* FIN DE ARCHIVO ********************************/
 