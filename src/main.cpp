/*
 * Pill Kart Steering Control
 *
 * This program controls the steering of a go-kart using a PID controller.
 *
 * Coordinate System:
 * - Angles are measured in radians.
 * - The center position (straight) is 0 radians.
 * - Positive values correspond to a left turn.
 * - Negative values correspond to a right turn.
 */

#include <Arduino.h>
#include <Wire.h>

// Pin Definitions
#define PWM_PIN PA0
#define DIR_PIN PA1
#define LED_PIN LED_BUILTIN

// I2C Address for AS5600
#define AS5600_ADDR 0x36

// AS5600 Registers
#define AS5600_ANGLE_MSB 0x0C
#define AS5600_ANGLE_LSB 0x0D

// Sensor & Angle Constants
const int SENSOR_MIN = 0;
const int SENSOR_MAX = 4095;
const int SENSOR_CENTER = 2048;
const float MAX_RAD = PI;    // The steering shaft can turn PI radians (180 degrees) in each direction
const float PWM_LIMIT = 0.5; // Limit the maximum PWM output (0.0 to 1.0)

// PID Controller State
struct PIDController
{
  float kp, ki, kd;
  float integral;
  float lastError;
  unsigned long lastTime;
};

// Global PID instance
PIDController pid = {2, 0.09, 0.0, 0.0, 0.0, 0};

// Function Prototypes
void setupHardware();
void blinkLed();
void printDiagnostics(float target, float actual, float pid_output);

float getTargetAngle();
float getActualAngle();
float calculatePID(float target, float actual, PIDController &pid);
void setMotorOutput(float control_signal);
int readRawAngle();

void setup()
{
  setupHardware();
}

void loop()
{
  // 1. Get the desired target angle
  float targetAngle = getTargetAngle();

  // 2. Get the current angle of the steering shaft
  float actualAngle = getActualAngle();

  if (actualAngle == 0.0f && readRawAngle() == -1)
  {                    // Check for actual error, not just 0.0f
    setMotorOutput(0); // Stop motor on sensor error
    Serial.println("Error reading sensor. Restarting I2C communication.");
    Wire.end();
    delay(10);
    Wire.begin();
    return;
  }

  // 3. Calculate the control signal using the PID controller
  float pidOutput = calculatePID(targetAngle, actualAngle, pid);

  // 4. Set the motor PWM and direction based on the control signal
  setMotorOutput(pidOutput);

  // Utility functions
  blinkLed();
  printDiagnostics(targetAngle, actualAngle, pidOutput);

  delay(10); // Main loop delay
}

void setupHardware()
{
  Serial.begin(115200);
  Wire.begin();
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

/**
 * @brief Gets the target steering angle.
 * @return The target angle in radians. For now, this is a constant 0.0 for going straight.
 */
float getTargetAngle()
{
  return 0.0f; // Radians
}

/**
 * @brief Reads the raw sensor value and converts it to radians.
 * @return The actual steering angle in radians.
 */
float getActualAngle()
{
  int rawAngle = readRawAngle();
  if (rawAngle == -1)
  {
    return 0.0f; // Return a neutral value on error
  }
  // Map the sensor value [0, 4095] to the radian range [-PI, PI] using floating-point arithmetic
  return (float)(rawAngle - SENSOR_CENTER) / (float)SENSOR_CENTER * MAX_RAD;
}

/**
 * @brief Calculates the PID control signal.
 * @param target The desired angle in radians.
 * @param actual The current angle in radians.
 * @param pid A reference to the PID controller state.
 * @return A control signal from -1.0 to 1.0.
 */
float calculatePID(float target, float actual, PIDController &pid)
{
  unsigned long now = millis();
  float dt = (now - pid.lastTime) / 1000.0f;

  if (dt <= 0)
  { // Prevent division by zero or negative dt
    dt = 1e-3;
  }

  float error = target - actual;
  pid.integral += error * dt;
  float derivative = (error - pid.lastError) / dt;

  float output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);

  pid.lastError = error;
  pid.lastTime = now;

  // Constrain the output to the range [-1.0, 1.0]
  return constrain(output, -1.0f, 1.0f);
}

/**
 * @brief Sets the motor's PWM and direction based on the MD30C truth table.
 * @param control_signal A value from -1.0 (full right) to 1.0 (full left).
 */
void setMotorOutput(float control_signal)
{
  // Determine direction from the sign of the control signal
  // According to the manual: HIGH for CCW, LOW for CW
  bool direction = (control_signal > 0); // Assuming positive is one direction, negative is the other

  // Calculate PWM value from the absolute magnitude of the signal, scaled by the limit
  int pwm_value = abs(control_signal) * 255 * PWM_LIMIT;

  // Apply the logic from the MD30C truth table
  if (pwm_value < 5)
  { // Use a small threshold to prevent noise from causing movement
    // Brake the motor if the control signal is near zero
    analogWrite(PWM_PIN, 0);
  }
  else
  {
    // Set direction and PWM for movement
    digitalWrite(DIR_PIN, direction);
    analogWrite(PWM_PIN, constrain(pwm_value, 0, 255));
  }
}

/**
 * @brief Reads the raw 12-bit angle from the AS5600 sensor.
 * @return The raw angle (0-4095) or -1 on error.
 */
int readRawAngle()
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_MSB);
  if (Wire.endTransmission(false) != 0)
  {
    Serial.println("I2C Error at endTransmission");
    return -1;
  }

  if (Wire.requestFrom(AS5600_ADDR, 2) == 2)
  {
    int msb = Wire.read();
    int lsb = Wire.read();
    return ((msb << 8) | lsb) & 0x0FFF;
  }
  Serial.println("I2C Error at requestFrom");
  return -1;
}

void blinkLed()
{
  static unsigned long lastLedToggleTime = 0;
  const long ledInterval = 500;
  if (millis() - lastLedToggleTime >= ledInterval)
  {
    lastLedToggleTime = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void printDiagnostics(float target, float actual, float pid_output)
{
  static unsigned long lastPrintTime = 0;
  const long printInterval = 100;

  if (millis() - lastPrintTime >= printInterval)
  {
    lastPrintTime = millis();
    Serial.print("Target: ");
    Serial.print(target, 4);
    Serial.print(" rad  Actual: ");
    Serial.print(actual, 4);
    Serial.print(" rad  PID: ");
    Serial.print(pid_output, 4);
    Serial.print("  DIR: ");
    Serial.print(digitalRead(DIR_PIN));
    Serial.print("  PWM: ");
    Serial.println(abs(pid_output) * 255 * PWM_LIMIT);
  }
}