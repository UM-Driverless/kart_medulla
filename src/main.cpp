// Kart Medulla - Steering Control with PID

#include <Arduino.h>
#include <Wire.h>

// Pin Definitions
#define PWM_PIN 25
#define DIR_PIN 26
#define LED_PIN 2

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
const float PWM_LIMIT = 1.0; // Limit the maximum PWM output (0.0 to 1.0)

// PID Controller State
struct PIDController
{
  float kp, ki, kd;
  float integral;
  float lastError;
  unsigned long lastTime;
};

// Global PID instance
PIDController pid = {3, 0.2, 0.0, 0.0, 0.0, 0}; // { kp, ki, kd, integral, lastError, lastTime }

// Function Prototypes
void setupHardware();
void blinkLed();
void printDiagnostics(float target, float actual, float pid_output)
{
  static unsigned long lastPrintTime = 0;
  static unsigned long lastLoopTime = 0;
  static float loopFrequency = 0.0;
  const long printInterval = 100;

  // Calculate loop frequency
  unsigned long currentTime = millis();
  if (lastLoopTime > 0) {
    float deltaTime = (currentTime - lastLoopTime) / 1000.0;
    if (deltaTime > 0) {
      loopFrequency = 1.0 / deltaTime;
    }
  }
  lastLoopTime = currentTime;

  if (currentTime - lastPrintTime >= printInterval)
  {
    lastPrintTime = currentTime;
    Serial.print("Freq: ");
    Serial.print(loopFrequency, 1);
    Serial.print(" Hz  Target: ");
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

float generateSineWaveTargetAngle(float amplitude_degrees, float period_seconds);
float getTargetAngle();
float getActualAngle();
float calculatePID(float target, float actual, PIDController &pid);
void setMotorOutput(float control_signal);
int readRawAngle();

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

float generateSineWaveTargetAngle(float amplitude_degrees, float period_seconds)
{
  return (amplitude_degrees * PI / 180.0) * sin(2.0 * PI * (millis() / (period_seconds * 1000.0)));
}

float getTargetAngle()
{
  return generateSineWaveTargetAngle(20.0, 3.0);
}

float getActualAngle()
{
  int rawAngle = readRawAngle();
  if (rawAngle == -1)
  {
    return 0.0f;
  }
  return (float)(rawAngle - SENSOR_CENTER) / (float)SENSOR_CENTER * MAX_RAD;
}

float calculatePID(float target, float actual, PIDController &pid)
{
  unsigned long now = millis();
  float dt = (now - pid.lastTime) / 1000.0f;

  if (dt <= 0)
  {
    dt = 1e-3;
  }

  float error = target - actual;
  pid.integral += error * dt;
  float derivative = (error - pid.lastError) / dt;

  float output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);

  pid.lastError = error;
  pid.lastTime = now;

  return constrain(output, -1.0f, 1.0f);
}

void setMotorOutput(float control_signal)
{
  bool direction = (control_signal > 0);
  int pwm_value = abs(control_signal) * 255 * PWM_LIMIT;

  if (pwm_value < 5)
  {
    analogWrite(PWM_PIN, 0);
  }
  else
  {
    digitalWrite(DIR_PIN, direction);
    analogWrite(PWM_PIN, constrain(pwm_value, 0, 255));
  }
}

int readRawAngle()
{
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_MSB);
  if (Wire.endTransmission(false) != 0)
  {
    return -1;
  }

  if (Wire.requestFrom(AS5600_ADDR, 2) == 2)
  {
    int msb = Wire.read();
    int lsb = Wire.read();
    return ((msb << 8) | lsb) & 0x0FFF;
  }
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
}