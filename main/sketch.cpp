#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>

#include "km_pid.h"
#include "km_rtos.h"
#include "km_sdir.h"
#include "km_act.h"

// ============================================================================
// Pin Configuration
// ============================================================================
const int PIN_THROTTLE_DAC = 26; // GPIO 26 = DAC2 (throttle)
const int PIN_BRAKE_DAC = 25;    // GPIO 25 = DAC1 (brake)
const int PIN_STEERING_PWM = 27; // Steering motor PWM (FUTURE UPDATE TO GPIO32)
const int PIN_STEERING_DIR = 14; // Steering motor direction (FUTURE UPDATE TO GPIO33)
const int PIN_I2C_SDA = 21;      // AS5600 sensor SDA
const int PIN_I2C_SCL = 22;      // AS5600 sensor SCL

// ============================================================================
// System Objects
// ============================================================================
AS5600Sensor steeringSensor; // ENABLED - sensor now connected
PID_Controller steeringPID;
SteeringMotor steeringMotor(PIN_STEERING_PWM, PIN_STEERING_DIR, 0, 5000, 8);
ThrottleMotor throttleMotor(PIN_THROTTLE_DAC);
BrakeMotor brakeMotor(PIN_BRAKE_DAC);

// Joystick deadzone
const int deadzone = 50;

// Array para controlar hasta 4 mandos
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// PID tuning parameters (adjust these for your system) --- CONSTANTS
const float KP = 0.03f; // Proportional gain
const float KI = 0.0f;  // Integral gain
const float KD = 0.0004f; // Derivative gain

// Angle mapping constants
const float MAX_STEERING_ANGLE_DEG = 45.0f; // Maximum steering angle in degrees

// --------------------- Callbacks ---------------------
void onConnectedController(ControllerPtr ctl)
{
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == nullptr)
        {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                           ctl->getModelName(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot)
    {
        Console.println("CALLBACK: Controller connected, but could not find empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl)
{
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++)
    {
        if (myControllers[i] == ctl)
        {
            myControllers[i] = nullptr;
            Console.printf("Controller disconnected! Index=%d\n", i);
            return;
        }
    }
}

// --------------------- Funciones ---------------------
void processGamepad(ControllerPtr ctl)
{
    if (!ctl->isConnected())
        return; // Note: hasData() check removed per AGENTS.md

    // ============================================================================
    // Read AS5600 Steering Sensor (ENABLED - sensor connected)
    // ============================================================================
    float currentAngle = steeringSensor.readAngleDegrees();
    bool sensorAvailable = steeringSensor.isConnected();

    // ============================================================================
    // Throttle and Brake (R2/L2 Triggers) - Use DAC for true analog
    // ============================================================================
    int r2Value = ctl->throttle(); // R2: 0-1023
    int l2Value = ctl->brake();    // L2: 0-1023

    // Convert to 0.0-1.0 range for DAC output
    float throttleValue = (float)r2Value / 1023.0f;
    float brakeValue = (float)l2Value / 1023.0f;

    throttleMotor.setOutput(throttleValue);
    brakeMotor.setOutput(brakeValue);

    // ============================================================================
    // Steering (Left Joystick X-axis) - PID Control
    // ============================================================================
    int axisX = ctl->axisX(); // -511 to 512

    // Map joystick to target angle with deadzone
    float targetAngle = 0.0f;
    if (abs(axisX) > deadzone)
    {
        // Map joystick range to steering angle range
        // LEFT positive convention: axisX < 0 (left) -> positive angle (CCW rotation around Z)
        // axisX: -511 (left) to +512 (right) -> targetAngle: +45° to -45°
        targetAngle = -(float)axisX * MAX_STEERING_ANGLE_DEG / 511.0f;
        targetAngle = constrain(targetAngle, -MAX_STEERING_ANGLE_DEG, MAX_STEERING_ANGLE_DEG);
    }

    // Calculate PID output
    float pidOutput = steeringPID.calculate(targetAngle, currentAngle);

    // Send PID output to steering motor
    steeringMotor.setOutput(pidOutput);

    // Calculate error for monitoring
    float error = targetAngle - currentAngle;

    // ============================================================================
    // Serial Monitor Output - CSV format for easy parsing
    // ============================================================================
    Console.printf("DATA,%d,%d,%d,%d,%d,%d,0x%04X,%d,%d,%.1f,%.1f,%.2f,%.3f,%s\n",
                   ctl->axisX(),                                 // LX
                   ctl->axisY(),                                 // LY
                   ctl->axisRX(),                                // RX
                   ctl->axisRY(),                                // RY
                   l2Value,                                      // L2
                   r2Value,                                      // R2
                   ctl->buttons(),                               // Button state
                   ctl->battery(),                               // Battery level
                   0,                                            // Charging (placeholder)
                   targetAngle,                                  // Target steering angle
                   currentAngle,                                 // Actual steering angle
                   error,                                        // Steering error
                   pidOutput,                                    // PID output
                   sensorAvailable ? "SENSOR_OK" : "NO_SENSOR"); // Sensor status
}

void processControllers()
{
    for (auto ctl : myControllers)
    {
        // Note: hasData() check removed per AGENTS.md (prevents controller timeout)
        if (ctl && ctl->isConnected() && ctl->isGamepad())
        {
            processGamepad(ctl);
        }
    }
}

// --------------------- Setup ---------------------
void setup()
{
    Console.printf("=== Kart Medulla System Initialization ===\n");
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t *addr = BP32.localBdAddress();
    Console.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // ============================================================================
    // Initialize AS5600 Steering Sensor (ENABLED - hardware connected)
    // ============================================================================
    Console.println("\n[1/4] Initializing AS5600 steering sensor...");
    if (!steeringSensor.begin(PIN_I2C_SDA, PIN_I2C_SCL))
    {
        Console.println("WARNING: AS5600 sensor not detected");
    }
    else
    {
        Console.println("AS5600 sensor initialized successfully");
        float initialAngle = steeringSensor.readAngleDegrees();
        Console.printf("Initial steering angle: %.2f degrees\n", initialAngle);
    }

    // ============================================================================
    // Initialize PID Controller
    // ============================================================================
    Console.println("\n[2/4] Initializing PID controller...");
    steeringPID.init(KP, KI, KD);
    steeringPID.setOutputLimits(-1.0f, 1.0f);   // Full motor range
    steeringPID.setIntegralLimits(-0.5f, 0.5f); // Anti-windup protection

    // ============================================================================
    // Initialize Motor Controllers
    // ============================================================================
    Console.println("\n[3/4] Initializing motor controllers...");
    steeringMotor.begin();
    throttleMotor.begin();
    brakeMotor.begin();

    // Set output limits (can be adjusted for safety during testing)
    steeringMotor.setOutputLimit(0.4f); // SAFETY: Limited to 40% for testing - change back to 1.0f when ready
    throttleMotor.setOutputLimit(1.0f); // 100% max
    brakeMotor.setOutputLimit(1.0f);    // 100% max

    // ============================================================================
    // Initialize Bluepad32 Controller Interface
    // ============================================================================
    Console.println("\n[4/4] Initializing Bluepad32...");
    bool startScanning = true;
    BP32.setup(&onConnectedController, &onDisconnectedController, startScanning);

    // Disable virtual devices and BLE service (gamepads only)
    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);

    Console.println("\n=== System ready! Waiting for controller... ===");
    Console.println("CSV Format: DATA,LX,LY,RX,RY,L2,R2,BUTTONS,BATTERY,CHARGING,TARGET,ACTUAL,ERROR,PID,SENSOR_STATUS\n");
}

// --------------------- Loop ---------------------
void loop()
{
    BP32.update();
    processControllers();
    delay(20); // 20ms = 50Hz update rate (good for PID control)
}
