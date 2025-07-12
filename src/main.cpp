// Kart Medulla - Steering Control with PID

#include <Arduino.h>
#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Pin Definitions
#define PWM_PIN 25
#define DIR_PIN 26
#define LED_PIN 2

// I2C Address for AS5600 Hall Sensor
#define AS5600_ADDR 0x36

// I2C Configuration for Orin (Neural Network Computer)

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
PIDController pid = {1.5, 0.01, 0.0, 0.0, 0.0, 0}; // { kp, ki, kd, integral, lastError, lastTime }

// Global State Structure


// Global Target Structure


// Global instances
float targetSteering = 0.0;
float targetAcceleration = 0.0;
float targetBrake = 0.0;
bool messageReceived = false;
unsigned long lastMessageTime = 0;

// Function Prototypes
void setupHardware();
void blinkLed();
void handleSerialCommands();
void handleOrinUartData();
void updateTargetFromOrinData();
bool readAndValidateSensorData(float &actualAngle);
void calculateAndApplyMotorControl(float targetAngle, float actualAngle, float &pidOutput);
void runPeriodicTasks(float targetAngle, float actualAngle, float pidOutput);
void printDiagnostics(float target, float actual, float pid_output)
{
  static unsigned long lastPrintTime = 0;
  static unsigned long lastLoopTime = 0;
  static float loopFrequency = 0.0;
  const long printInterval = 100;

  // Calculate loop frequency
  unsigned long currentTime = millis();
  if (lastLoopTime > 0)
  {
    float deltaTime = (currentTime - lastLoopTime) / 1000.0;
    if (deltaTime > 0)
    {
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

    SerialBT.print("Freq: ");
    SerialBT.print(loopFrequency, 1);
    SerialBT.print(" Hz  Target: ");
    SerialBT.print(target, 4);
    SerialBT.print(" rad  Actual: ");
    SerialBT.print(actual, 4);
    SerialBT.print(" rad  PID: ");
    SerialBT.print(pid_output, 4);
    SerialBT.print("  DIR: ");
    SerialBT.print(digitalRead(DIR_PIN));
    SerialBT.print("  PWM: ");
    SerialBT.println(abs(pid_output) * 255 * PWM_LIMIT);
  }
}

float generateSineWaveTargetAngle(float amplitude_degrees, float period_seconds);
float getTargetAngle();
float getActualAngle();
float calculatePID(float target, float actual, PIDController &pid);
float getConstantTargetAngle();
void setMotorOutput(float control_signal);
int readRawAngle();

void setupHardware()
{
  Serial.begin(115200);
  SerialBT.begin("KartMedulla"); // Bluetooth device name

  // Initialize I2C master for AS5600 hall sensor (pins 21/22)
  Wire.begin();

  // Initialize UART2 for Orin communication
  Serial2.begin(115200, SERIAL_8N1, 18, 19); // Baud rate, protocol, RX pin, TX pin

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

float getConstantTargetAngle()
{
  return 0.0;
}

float getTargetAngle()
{
  updateTargetFromOrinData();
  return targetSteering;
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
  int pwm_value = abs(control_signal) * 255 * PWM_LIMIT;

  if (pwm_value < 5)
  {
    analogWrite(PWM_PIN, 0);
    digitalWrite(DIR_PIN, LOW);
  }
  else
  {
    if (control_signal > 0)
    {
      digitalWrite(DIR_PIN, HIGH); // Swapped back to HIGH
      analogWrite(PWM_PIN, constrain(pwm_value, 0, 255));
    }
    else
    {
      digitalWrite(DIR_PIN, LOW); // Swapped back to LOW
      analogWrite(PWM_PIN, constrain(pwm_value, 0, 255));
    }
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

void handleOrinUartData()
{
  if (Serial2.available() >= 4)
  {
    byte buffer[4];
    // Read all available bytes from Orin UART bus
    Serial2.readBytes(buffer, 4);

    Serial.print("UART Raw Data: ");
    for (int j = 0; j < 4; j++) {
      Serial.print(buffer[j], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Verify header byte
    if (buffer[0] == 0xAA)
    {
      // Update state with received values (0-255 range)
      targetSteering = (buffer[1] - 127.5) / 127.5; // Convert to -1.0 to 1.0
      targetAcceleration = buffer[2] / 255.0;       // Convert to 0.0 to 1.0
      targetBrake = buffer[3] / 255.0;              // Convert to 0.0 to 1.0
      messageReceived = true;
      lastMessageTime = millis();

      // Update target steering angle based on received steering value
      targetSteering = targetSteering * MAX_RAD; // Scale to max steering angle
      targetAcceleration = targetAcceleration;
      targetBrake = targetBrake;

      Serial.print("UART Received - Steering: ");
      Serial.print(targetSteering, 3);
      Serial.print(", Accel: ");
      Serial.print(targetAcceleration, 3);
      Serial.print(", Brake: ");
      Serial.println(targetBrake, 3);
    }
  }
}

void updateTargetFromOrinData()
{
  // Check if we have recent UART data (within last 500ms)
  if (messageReceived && (millis() - lastMessageTime < 500))
  {
    // Use UART received target
    // Target steering is already updated in handleOrinUartData
  }
  else
  {
    // Fall back to sine wave if no recent UART data
    targetSteering = generateSineWaveTargetAngle(20.0, 3.0);
    targetAcceleration = 0.0;
    targetBrake = 0.0;

    if (messageReceived)
    {
      Serial.println("UART timeout - falling back to sine wave");
      messageReceived = false;
    }
  }
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

void handleSerialCommands()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any whitespace

    if (command.startsWith("kp="))
    {
      pid.kp = command.substring(3).toFloat();
      Serial.print("kp updated to: ");
      Serial.println(pid.kp, 4);
    }
    else if (command.startsWith("ki="))
    {
      pid.ki = command.substring(3).toFloat();
      Serial.print("ki updated to: ");
      Serial.println(pid.ki, 4);
    }
    else if (command.startsWith("kd="))
    {
      pid.kd = command.substring(3).toFloat();
      Serial.print("kd updated to: ");
      Serial.println(pid.kd, 4);
    }
  }
}

void runPeriodicTasks(float targetAngle, float actualAngle, float pidOutput)
{
  blinkLed();
  handleSerialCommands();
  handleOrinUartData();
  printDiagnostics(targetAngle, actualAngle, pidOutput);
}

void calculateAndApplyMotorControl(float targetAngle, float actualAngle, float &pidOutput)
{
  pidOutput = calculatePID(targetAngle, actualAngle, pid);
  setMotorOutput(pidOutput);
}

bool readAndValidateSensorData(float &actualAngle)
{
  actualAngle = getActualAngle();

  if (actualAngle == 0.0f && readRawAngle() == -1)
  {
    setMotorOutput(0); // Stop motor on sensor error
    Serial.println("Error reading sensor. Restarting I2C communication.");
    Wire.end();
    delay(10);
    Wire.begin();
    return false;
  }
  return true;
}

void setup()
{
  setupHardware();
}

void loop()
{
  // 1. Get the desired target angle
  float targetAngle = getConstantTargetAngle();

  // 2. Get the current angle of the steering shaft
  float actualAngle;
  if (!readAndValidateSensorData(actualAngle))
  {
    return;
  }

  // 3. Calculate and apply motor control
  float pidOutput;
  calculateAndApplyMotorControl(targetAngle, actualAngle, pidOutput);

  // 4. Run periodic tasks
  runPeriodicTasks(targetAngle, actualAngle, pidOutput);
}