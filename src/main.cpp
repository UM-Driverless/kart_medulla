#include <Wire.h>  // Include the Wire library for I2C communication
#include <Arduino.h> 

#define AS5600_ADDR 0x36  // Default I2C address for AS5600 (check your wiring)

// AS5600 Registers
#define AS5600_ANGLE_LSB 0x0D  // Low byte of the angle register
#define AS5600_ANGLE_MSB 0x0C  // High byte of the angle register

#define PWM_PIN PA0      // PWM
#define DIR_PIN PA1     // Sentido de giro

// Parámetros del PID
float kp = 1.0;
float ki = 0.0;
float kd = 0.1;

float setpoint = 20.0;     // Valor deseado
float input = 0.0;          // Valor leído del sensor
float output = 0.0;         // Señal de control que vas a aplicar (PWM, DAC, etc.)

// Internos del PID
float lastError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;

int pwm = 0;
bool sentido = LOW;

unsigned long lastMotorChangeTime = 0;
const long motorInterval = 100; // Milliseconds between PWM changes

unsigned long lastLedToggleTime = 0;
const long ledInterval = 500; // Milliseconds between LED toggles

bool ledState = LOW;

void PID();
int readAngle();

void setup()
{
  Serial.begin(115200); // Initialize serial communication
  Wire.begin(); // Initialize I2C communication
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); // Initial direction

  pinMode(LED_BUILTIN, OUTPUT);   // Set LED pin as output
  digitalWrite(LED_BUILTIN, LOW); // Ensure LED is off initially
}

void loop()
{
  unsigned long currentMillis = millis();

  // LED blinking logic (non-blocking)
  if (currentMillis - lastLedToggleTime >= ledInterval)
  {
    lastLedToggleTime = currentMillis;
    ledState = !ledState; // Toggle LED state
    digitalWrite(LED_BUILTIN, ledState);
  }

  PID();
}

void aplicarSalida(float pid_output) {
  // Ensure output is within PWM range
  int pwm_value = constrain(abs(pid_output), 0, 255);

  if (pid_output > 0) {
    digitalWrite(DIR_PIN, HIGH); // Set direction for positive output
  } else {
    digitalWrite(DIR_PIN, LOW);  // Set direction for negative output
  }
  
  analogWrite(PWM_PIN, pwm_value);
}

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

void PID() {

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  if (dt >= 0.01) { // Ejecuta el PID cada 10 ms
    input = readAngle();

    if (input == -1) {
      // Error reading sensor, stop the motor as a safety measure
      aplicarSalida(0);
      Serial.println("Error reading sensor");
      return; // Skip PID calculation
    }

    float error = setpoint - input;
    integral += error * dt;
    float derivative = (error - lastError) / dt;

    output = kp * error + ki * integral + kd * derivative;

    aplicarSalida(output);

    lastError = error;
    lastTime = now;

    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print("  Input: ");
    Serial.print(input);
    Serial.print("  Output: ");
    Serial.println(output);
  }

}