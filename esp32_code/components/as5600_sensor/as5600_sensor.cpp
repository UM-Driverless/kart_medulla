#include "as5600_sensor.h"

AS5600Sensor::AS5600Sensor()
    : connected(false),
      errorCount(0),
      lastReadTime(0),
      lastRawValue(0),
      centerOffset(SENSOR_CENTER) {
}

bool AS5600Sensor::begin(int sdaPin, int sclPin) {
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);  // 400kHz I2C speed

    // Test connection by reading a register
    uint8_t testData[2];
    connected = readRegisters(AS5600_ANGLE_MSB, testData, 2);

    if (connected) {
        Serial.println("AS5600 sensor initialized successfully");
    } else {
        Serial.println("AS5600 sensor not found!");
    }

    return connected;
}

uint16_t AS5600Sensor::readRaw() {
    uint8_t data[2];

    if (readRegisters(AS5600_ANGLE_MSB, data, 2)) {
        lastRawValue = ((uint16_t)data[0] << 8) | data[1];
        lastReadTime = millis();
        errorCount = 0;
        return lastRawValue;
    } else {
        errorCount++;
        if (errorCount >= MAX_ERROR_COUNT) {
            Serial.println("Error reading sensor. Restarting I2C communication.");
            resetI2C();
            errorCount = 0;
        }
        return lastRawValue;  // Return last good value
    }
}

float AS5600Sensor::readAngle() {
    uint16_t raw = readRaw();

    // Convert to centered value (-2048 to 2047)
    int16_t centered = (int16_t)raw - (int16_t)centerOffset;

    // Convert to radians (-PI to PI)
    float angle = ((float)centered / (float)SENSOR_MAX) * 2.0f * MAX_RAD;

    return angle;
}

float AS5600Sensor::readAngleDegrees() {
    return readAngle() * 180.0f / PI;
}

bool AS5600Sensor::isConnected() {
    return connected && (errorCount < MAX_ERROR_COUNT);
}

void AS5600Sensor::resetI2C() {
    Wire.end();
    delay(100);
    Wire.begin();
    Wire.setClock(400000);

    // Test connection again
    uint8_t testData[2];
    connected = readRegisters(AS5600_ANGLE_MSB, testData, 2);
}

void AS5600Sensor::setCenterOffset(uint16_t offset) {
    centerOffset = offset;
    Serial.printf("AS5600 center offset set to: %d\n", offset);
}

bool AS5600Sensor::readRegisters(uint8_t reg, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(AS5600_ADDR, len) != len) {
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }

    return true;
}
