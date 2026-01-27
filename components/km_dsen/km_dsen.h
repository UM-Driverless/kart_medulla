#ifndef AS5600_SENSOR_H
#define AS5600_SENSOR_H

#include <Arduino.h>
#include <Wire.h>

class AS5600Sensor {
private:
    static const uint8_t AS5600_ADDR = 0x36;
    static const uint8_t AS5600_ANGLE_MSB = 0x0C;
    static const uint8_t AS5600_ANGLE_LSB = 0x0D;

    // Sensor constants
    static const int SENSOR_MIN = 0;
    static const int SENSOR_MAX = 4095;
    static const int SENSOR_CENTER = 2048;
    static constexpr float MAX_RAD = PI;  // Maximum angle in radians

    // Error handling
    bool connected;
    uint8_t errorCount;
    static const uint8_t MAX_ERROR_COUNT = 10;
    unsigned long lastReadTime;
    uint16_t lastRawValue;

public:
    AS5600Sensor();

    // Initialize the sensor
    bool begin(int sdaPin = 21, int sclPin = 22);

    // Read raw sensor value (0-4095)
    uint16_t readRaw();

    // Read angle in radians (-PI to PI)
    float readAngle();

    // Read angle in degrees (-180 to 180)
    float readAngleDegrees();

    // Check if sensor is connected
    bool isConnected();

    // Reset I2C communication if errors occur
    void resetI2C();

    // Get center offset for calibration
    void setCenterOffset(uint16_t offset);

private:
    uint16_t centerOffset;

    // Internal I2C read function
    bool readRegisters(uint8_t reg, uint8_t* data, uint8_t len);
};

#endif // AS5600_SENSOR_H