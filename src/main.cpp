// Kart Medulla - Steering Control with PID

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "steering_test.h"
#include "steering_test_sequence.h"
#include "sine_wave_test.h"

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// BLE Server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Client Connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("BLE Client Disconnected");
    }
};

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
PIDController pid = {15.0, 0.0, 0.0, 0.0, 0.0, 0}; // { kp, ki, kd, integral, lastError, lastTime }

// Global State Structure


// Global Target Structure


// Global instances
float targetSteering = 0.0;
float targetAcceleration = 0.0;
float targetBrake = 0.0;
bool messageReceived = false;
unsigned long lastMessageTime = 0;

// Test mode variables
bool testModeEnabled = true;  // Start in test mode
float testAmplitude = 0.0;    // degrees - start at 0 (straight)
float testPeriod = 5.0;       // seconds
int testMode = 2;              // 1=sine, 2=constant, 3=step - start with constant

// Test objects
SteeringTest steeringTest;
SteeringTestSequence testSequence;
SineWaveTest sineTest;

// Function Prototypes
void setupHardware();
void blinkLed();
void handleSerialCommands();
void handleOrinUartData();
void updateTargetFromOrinData();
bool readAndValidateSensorData(float &actualAngle);
void calculateAndApplyMotorControl(float targetAngle, float actualAngle, float &pidOutput);
void runPeriodicTasks(float targetAngle, float actualAngle, float pidOutput);
int readRawAngle();
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
    
    // Print test mode status
    if (testModeEnabled) {
      Serial.print("[CONST ");
      Serial.print(testAmplitude, 0);
      Serial.print("°] ");
    } else if (sineTest.isActive()) {
      Serial.print("[SINE] ");
    } else if (testSequence.isActive()) {
      Serial.print("[SEQ] ");
    } else if (steeringTest.isActive()) {
      Serial.print("[STEADY] ");
    }
    
    Serial.print("Freq: ");
    Serial.print(loopFrequency, 1);
    Serial.print(" Hz  Target: ");
    Serial.print(target * 180.0 / PI, 1);  // Convert to degrees
    Serial.print("°  Actual: ");
    Serial.print(actual * 180.0 / PI, 1);  // Convert to degrees
    Serial.print("°  Error: ");
    Serial.print((target - actual) * 180.0 / PI, 1);
    Serial.print("°  PID: ");
    Serial.print(pid_output, 3);
    Serial.print("  PWM: ");
    Serial.print(abs(pid_output) * 255 * PWM_LIMIT);
    Serial.print("  Raw: ");
    Serial.println(readRawAngle());

    // Send data over BLE if connected
    if (deviceConnected) {
      String bleData = "Freq: " + String(loopFrequency, 1) + 
                       " Hz  Target: " + String(target, 4) + 
                       " rad  Actual: " + String(actual, 4) + 
                       " rad  PID: " + String(pid_output, 4) + 
                       "  DIR: " + String(digitalRead(DIR_PIN)) + 
                       "  PWM: " + String(abs(pid_output) * 255 * PWM_LIMIT);
      
      pCharacteristic->setValue(bleData.c_str());
      pCharacteristic->notify();
    }
  }
}

float generateSineWaveTargetAngle(float amplitude_degrees, float period_seconds);
float getTargetAngle();
float getActualAngle();
float calculatePID(float target, float actual, PIDController &pid);
float getConstantTargetAngle();
void setMotorOutput(float control_signal);
int readRawAngle();

void setupBLE() {
  // Create the BLE Device
  BLEDevice::init("KartMedulla");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create a BLE Descriptor for notifications
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("BLE Advertising Started - Waiting for clients to connect...");
}

void setupHardware()
{
  Serial.begin(115200);
  
  // Setup BLE
  setupBLE();

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
    else if (command == "test")
    {
      testModeEnabled = !testModeEnabled;
      Serial.print("Test mode: ");
      Serial.println(testModeEnabled ? "ENABLED" : "DISABLED");
    }
    else if (command.startsWith("mode="))
    {
      testMode = command.substring(5).toInt();
      Serial.print("Test mode changed to: ");
      Serial.println(testMode == 1 ? "SINE" : (testMode == 2 ? "CONSTANT" : "STEP"));
    }
    else if (command.startsWith("amp="))
    {
      testAmplitude = command.substring(4).toFloat();
      Serial.print("Test amplitude: ");
      Serial.print(testAmplitude);
      Serial.println(" degrees");
    }
    else if (command.startsWith("period="))
    {
      testPeriod = command.substring(7).toFloat();
      Serial.print("Test period: ");
      Serial.print(testPeriod);
      Serial.println(" seconds");
    }
    else if (command.startsWith("steady"))
    {
      float targetDeg = 15.0;  // default
      if (command.length() > 6) {
        targetDeg = command.substring(7).toFloat();
      }
      steeringTest.startSteadyStateTest(targetDeg);
      testModeEnabled = false;  // Disable other test modes
    }
    else if (command == "sequence")
    {
      testSequence.startSequenceTest();
      testModeEnabled = false;  // Disable other test modes
    }
    else if (command.startsWith("sine"))
    {
      float amp = 30.0;  // default amplitude
      float freq = 0.2;  // default frequency
      if (command.length() > 4) {
        // Parse sine=amp,freq format
        int commaIndex = command.indexOf(',');
        if (commaIndex > 5) {
          amp = command.substring(5, commaIndex).toFloat();
          freq = command.substring(commaIndex + 1).toFloat();
        } else {
          amp = command.substring(5).toFloat();
        }
      }
      sineTest.startTest(amp, freq, 20000);  // 20 second test
      testModeEnabled = false;  // Disable other test modes
    }
    else if (command == "help")
    {
      Serial.println("\n=== Commands ===");
      Serial.println("kp=X.X     - Set P gain");
      Serial.println("ki=X.X     - Set I gain");
      Serial.println("kd=X.X     - Set D gain");
      Serial.println("test       - Toggle test mode");
      Serial.println("mode=N     - Set test mode (1=sine, 2=constant, 3=step)");
      Serial.println("amp=X      - Set test amplitude (degrees)");
      Serial.println("period=X   - Set test period (seconds)");
      Serial.println("steady[=X] - Run steady-state test (default 15°)");
      Serial.println("sequence   - Run test sequence (0,±10,±20,±30°)");
      Serial.println("sine[=A,F] - Run sine wave test (A=amplitude, F=freq)");
      Serial.println("help       - Show this help");
      Serial.println("===============\n");
    }
  }
}

void handleBLEConnection() {
  // Handle disconnection - restart advertising
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("BLE: Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // Handle new connection
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
}

void runPeriodicTasks(float targetAngle, float actualAngle, float pidOutput)
{
  blinkLed();
  handleSerialCommands();
  handleOrinUartData();
  handleBLEConnection();
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
  Serial.println("\n=== Kart Medulla Started ===");
  Serial.println("Starting with constant 0° (straight)");
  Serial.println("Type 'help' for available commands");
  Serial.println("Type 'sequence' for discrete angle test");
  Serial.println("Type 'sine' for sine wave test");
  Serial.println("===========================\n");
}

void loop()
{
  // 1. Get the desired target angle
  float targetAngle;
  if (steeringTest.isActive()) {
    targetAngle = steeringTest.getTargetAngle();
  } else if (testSequence.isActive()) {
    targetAngle = testSequence.getCurrentTargetAngle();
  } else if (sineTest.isActive()) {
    targetAngle = sineTest.getCurrentTargetAngle();
  } else if (testModeEnabled) {
    switch(testMode) {
      case 1: // Sine wave
        targetAngle = generateSineWaveTargetAngle(testAmplitude, testPeriod);
        break;
      case 2: // Constant angle
        targetAngle = (testAmplitude * PI / 180.0); // Use amplitude as constant angle
        break;
      case 3: // Step function
        targetAngle = ((millis() / (int)(testPeriod * 1000)) % 2 == 0) ? 
                      (testAmplitude * PI / 180.0) : -(testAmplitude * PI / 180.0);
        break;
      default:
        targetAngle = 0.0;
    }
  } else {
    targetAngle = getTargetAngle(); // Use UART or fallback
  }

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
  
  // 5. Update tests if active
  steeringTest.update(actualAngle);
  testSequence.update(actualAngle);
  sineTest.update(actualAngle);
  
  // Re-enable test mode when tests complete
  if (!steeringTest.isActive() && !testSequence.isActive() && !sineTest.isActive() && !testModeEnabled) {
    testModeEnabled = true;
  }
}