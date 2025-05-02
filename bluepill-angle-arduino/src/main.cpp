#include <Wire.h>  // Include the Wire library for I2C communication

#define AS5600_ADDR 0x36  // Default I2C address for AS5600 (check your wiring)

// AS5600 Registers
#define AS5600_ANGLE_LSB 0x0D  // Low byte of the angle register
#define AS5600_ANGLE_MSB 0x0C  // High byte of the angle register

// Function declaration (move this to the top)
int readAngle();

void setup() {
  Serial.begin(115200);    // Start serial communication
  Wire.begin();            // Start I2C communication
  delay(2000);             // Wait for the sensor to initialize

  Serial.println("AS5600 Magnetic Angle Sensor");
}

void loop() {
  int angle = readAngle();    // Read angle from AS5600

  Serial.print("Angle: ");
  Serial.println(angle);      // Print the angle value
  
  delay(500);                 // Wait for 500ms before reading again
}

// Function definition (keep it here)
int readAngle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_MSB);  // Send the address of the angle MSB register
  Wire.endTransmission();
  
  Wire.requestFrom(AS5600_ADDR, 2);  // Request 2 bytes of data (MSB and LSB)
  
  if (Wire.available() == 2) {
    byte msb = Wire.read();   // Read MSB
    byte lsb = Wire.read();   // Read LSB

    // Combine MSB and LSB to form the full 12-bit angle value
    int angle = ((msb << 8) | lsb) & 0x0FFF;  // Mask to get the lower 12 bits
    
    return angle;
  } else {
    return -1;  // Return -1 if data is not available
  }
}
