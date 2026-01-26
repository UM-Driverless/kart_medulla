#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "as5600_sensor.h" // ENABLED - sensor now connected
#include "pid_controller.h"
#include "motor_control.h"

// ============================================================================
// Pin Configuration (ESP32-DevKitC V4 USB-C)
// ============================================================================
const int PIN_THROTTLE_DAC = 25; // GPIO 25 (DAC1) - CMD_ACC (H2-9)
const int PIN_BRAKE_DAC = 26;    // GPIO 26 (DAC2) - CMD_BRAKE (H2-10)
const int PIN_STEERING_PWM = 18; // GPIO 18 - CMD_STEER_PWM (H1-11)
const int PIN_STEERING_DIR = 19; // GPIO 19 - CMD_STEER_DIR (H1-12)
const int PIN_I2C_SDA = 21;      // GPIO 21 - STEER_SDA (H1-14)
const int PIN_I2C_SCL = 22;      // GPIO 22 - STEER_SCL (H1-17)
const int PIN_HALL_1 = 17;       // GPIO 17 - MOTOR_HALL_1 (H1-9)
const int PIN_HALL_2 = 33;       // GPIO 33 - MOTOR_HALL_2 (H2-8)
const int PIN_HALL_3 = 16;       // GPIO 16 - MOTOR_HALL_3 (H1-8)
const int PIN_PRESSURE_1 = 36;   // GPIO 36 - PRESSURE_1 (H2-3)
const int PIN_PRESSURE_2 = 39;   // GPIO 39 - PRESSURE_2 (H2-4)
const int PIN_PRESSURE_3 = 34;   // GPIO 34 - PRESSURE_3 (H2-5)
const int PIN_PEDAL_ACC = 35;    // GPIO 35 - PEDAL_ACC (H2-6)
const int PIN_PEDAL_BRAKE = 32;  // GPIO 32 - PEDAL_BRAKE (H2-7)
const int PIN_HYDRAULIC_1 = 27;  // GPIO 27 - HYDRAULIC_1 (H2-11)
const int PIN_HYDRAULIC_2 = 14;  // GPIO 14 - HYDRAULIC_2 (H2-12)
const int PIN_SDC_NOT_EMERGENCY = 13; // GPIO 13 - SDC_NOT_EMERGENCY (H2-15)

// Hall sensor speed calculation (tune these to your drivetrain)
const float HALL_PULSES_PER_MOTOR_REV = 3.0f; // Total rising edges per motor rev across all hall sensors
const float GEAR_REDUCTION = 10.0f;           // Motor revs per wheel rev
const float WHEEL_DIAMETER_M = 0.30f;         // Wheel diameter in meters
const uint32_t HALL_SAMPLE_MS = 100;          // Sample window for frequency calc

// Analog input scaling (tune to your dividers)
const float ADC_REF_V = 3.3f;
const int ADC_MAX_COUNTS = 4095;
const float PRESSURE_DIVIDER_RATIO = 3.0f; // 10k/10k/10k divider => V_in = V_out * 3
const float PEDAL_DIVIDER_RATIO = 2.0f;    // 10k/10k divider => V_in = V_out * 2
const float HYDRAULIC_DIVIDER_RATIO = (2.0f + 3.9f) / 3.9f;

// Commanded analog outputs (post-op-amp)
const float CMD_ACC_MAX_V = 5.0f;
const float CMD_BRAKE_MAX_V = 10.0f;

// Safety task configuration
const uint32_t SAFETY_TASK_MS = 20; // 50Hz safety loop
const bool ENABLE_SDC_SAFETY = false; // Set true once thresholds are validated
const float PRESSURE_MIN_V = 0.0f;
const float PRESSURE_MAX_V = 10.0f;
const float PEDAL_MIN_V = 0.0f;
const float PEDAL_MAX_V = 5.0f;
const float HYDRAULIC_MIN_V = 0.0f;
const float HYDRAULIC_MAX_V = 5.0f;

// ============================================================================
// System Objects
// ============================================================================
AS5600Sensor steeringSensor; // ENABLED - sensor now connected
PIDController steeringPID;
SteeringMotor steeringMotor(PIN_STEERING_PWM, PIN_STEERING_DIR, 0, 5000, 8);
ThrottleMotor throttleMotor(PIN_THROTTLE_DAC);
BrakeMotor brakeMotor(PIN_BRAKE_DAC);

// Joystick deadzone
const int deadzone = 50;

// Array para controlar hasta 4 mandos
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

enum class KartMode
{
    REMOTE,
    AUTONOMOUS,
    MANUAL
};

const KartMode DEFAULT_MODE = KartMode::REMOTE;
KartMode currentMode = DEFAULT_MODE;
bool autonomousWarned = false;

// Hall sensor counters (ISR-safe)
portMUX_TYPE hallMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t hallCounts[3] = {0, 0, 0};
uint32_t lastHallSampleMs = 0;
float hallHz = 0.0f;
float speedMps = 0.0f;
float speedKph = 0.0f;

struct AnalogReadings
{
    uint16_t pressureRaw[3];
    uint16_t pedalRaw[2];
    uint16_t hydraulicRaw[2];
    float pressureV[3];
    float pedalV[2];
    float hydraulicV[2];
};

portMUX_TYPE analogMux = portMUX_INITIALIZER_UNLOCKED;
AnalogReadings analogReadings = {};
volatile bool sdcFaultLatched = false;

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

// --------------------- Hall Sensor ISR ---------------------
void IRAM_ATTR onHall1()
{
    portENTER_CRITICAL_ISR(&hallMux);
    hallCounts[0]++;
    portEXIT_CRITICAL_ISR(&hallMux);
}

void IRAM_ATTR onHall2()
{
    portENTER_CRITICAL_ISR(&hallMux);
    hallCounts[1]++;
    portEXIT_CRITICAL_ISR(&hallMux);
}

void IRAM_ATTR onHall3()
{
    portENTER_CRITICAL_ISR(&hallMux);
    hallCounts[2]++;
    portEXIT_CRITICAL_ISR(&hallMux);
}

float adcToVolts(uint16_t raw)
{
    return (static_cast<float>(raw) * ADC_REF_V) / static_cast<float>(ADC_MAX_COUNTS);
}

bool inRange(float value, float minV, float maxV)
{
    return value >= minV && value <= maxV;
}

void readAnalogSensors(AnalogReadings &out)
{
    out.pressureRaw[0] = analogRead(PIN_PRESSURE_1);
    out.pressureRaw[1] = analogRead(PIN_PRESSURE_2);
    out.pressureRaw[2] = analogRead(PIN_PRESSURE_3);
    out.pedalRaw[0] = analogRead(PIN_PEDAL_ACC);
    out.pedalRaw[1] = analogRead(PIN_PEDAL_BRAKE);
    out.hydraulicRaw[0] = analogRead(PIN_HYDRAULIC_1);
    out.hydraulicRaw[1] = analogRead(PIN_HYDRAULIC_2);

    out.pressureV[0] = adcToVolts(out.pressureRaw[0]) * PRESSURE_DIVIDER_RATIO;
    out.pressureV[1] = adcToVolts(out.pressureRaw[1]) * PRESSURE_DIVIDER_RATIO;
    out.pressureV[2] = adcToVolts(out.pressureRaw[2]) * PRESSURE_DIVIDER_RATIO;
    out.pedalV[0] = adcToVolts(out.pedalRaw[0]) * PEDAL_DIVIDER_RATIO;
    out.pedalV[1] = adcToVolts(out.pedalRaw[1]) * PEDAL_DIVIDER_RATIO;
    out.hydraulicV[0] = adcToVolts(out.hydraulicRaw[0]) * HYDRAULIC_DIVIDER_RATIO;
    out.hydraulicV[1] = adcToVolts(out.hydraulicRaw[1]) * HYDRAULIC_DIVIDER_RATIO;
}

bool checkAnalogFaults(const AnalogReadings &reading)
{
    for (int i = 0; i < 3; i++)
    {
        if (!inRange(reading.pressureV[i], PRESSURE_MIN_V, PRESSURE_MAX_V))
        {
            return true;
        }
    }

    for (int i = 0; i < 2; i++)
    {
        if (!inRange(reading.pedalV[i], PEDAL_MIN_V, PEDAL_MAX_V))
        {
            return true;
        }
        if (!inRange(reading.hydraulicV[i], HYDRAULIC_MIN_V, HYDRAULIC_MAX_V))
        {
            return true;
        }
    }

    return false;
}

void safetyTask(void *arg)
{
    (void)arg;
    for (;;)
    {
        AnalogReadings local = {};
        readAnalogSensors(local);

        if (ENABLE_SDC_SAFETY)
        {
            if (checkAnalogFaults(local))
            {
                sdcFaultLatched = true;
            }
        }
        else
        {
            sdcFaultLatched = true; // Safety disabled => force emergency (MOSFET off)
        }

        digitalWrite(PIN_SDC_NOT_EMERGENCY, sdcFaultLatched ? LOW : HIGH);

        portENTER_CRITICAL(&analogMux);
        analogReadings = local;
        portEXIT_CRITICAL(&analogMux);

        vTaskDelay(pdMS_TO_TICKS(SAFETY_TASK_MS));
    }
}

void updateHallSpeed()
{
    uint32_t now = millis();
    if (lastHallSampleMs == 0)
    {
        lastHallSampleMs = now;
        return;
    }

    uint32_t elapsedMs = now - lastHallSampleMs;
    if (elapsedMs < HALL_SAMPLE_MS)
    {
        return;
    }

    uint32_t c1 = 0;
    uint32_t c2 = 0;
    uint32_t c3 = 0;
    portENTER_CRITICAL(&hallMux);
    c1 = hallCounts[0];
    c2 = hallCounts[1];
    c3 = hallCounts[2];
    hallCounts[0] = 0;
    hallCounts[1] = 0;
    hallCounts[2] = 0;
    portEXIT_CRITICAL(&hallMux);

    lastHallSampleMs = now;
    const uint32_t totalPulses = c1 + c2 + c3;
    const float dt = elapsedMs / 1000.0f;
    if (dt <= 0.0f)
    {
        return;
    }

    hallHz = totalPulses / dt;
    const float motorRps = hallHz / HALL_PULSES_PER_MOTOR_REV;
    const float wheelRps = motorRps / GEAR_REDUCTION;
    speedMps = wheelRps * (PI * WHEEL_DIAMETER_M);
    speedKph = speedMps * 3.6f;
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

    float cmdAccV = throttleValue * CMD_ACC_MAX_V;
    float cmdBrakeV = brakeValue * CMD_BRAKE_MAX_V;

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
    AnalogReadings analogSnapshot = {};
    portENTER_CRITICAL(&analogMux);
    analogSnapshot = analogReadings;
    bool sdcFault = sdcFaultLatched;
    portEXIT_CRITICAL(&analogMux);

    Console.printf("DATA,%d,%d,%d,%d,%d,%d,0x%04X,%d,%d,%.1f,%.1f,%.2f,%.3f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f\n",
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
                   sensorAvailable ? "SENSOR_OK" : "NO_SENSOR",   // Sensor status
                   hallHz,                                       // Hall pulse frequency
                   speedMps,                                     // Speed m/s
                   speedKph,                                     // Speed km/h
                   analogSnapshot.pressureV[0],                  // Pressure 1 (V)
                   analogSnapshot.pressureV[1],                  // Pressure 2 (V)
                   analogSnapshot.pressureV[2],                  // Pressure 3 (V)
                   analogSnapshot.pedalV[0],                     // Pedal acc (V)
                   analogSnapshot.pedalV[1],                     // Pedal brake (V)
                   analogSnapshot.hydraulicV[0],                 // Hydraulic 1 (V)
                   analogSnapshot.hydraulicV[1],                 // Hydraulic 2 (V)
                   sdcFault ? 1 : 0,                             // SDC fault latched
                   cmdAccV,                                      // CMD_ACC output (V)
                   cmdBrakeV);                                   // CMD_BRAKE output (V)
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

void processManual()
{
    steeringMotor.stop();
    throttleMotor.stop();
    brakeMotor.stop();
}

void processAutonomous()
{
    if (!autonomousWarned)
    {
        Console.println("AUTONOMOUS mode active - control logic not implemented yet");
        autonomousWarned = true;
    }
    steeringMotor.stop();
    throttleMotor.stop();
    brakeMotor.stop();
}

void runMode()
{
    switch (currentMode)
    {
    case KartMode::REMOTE:
        throttleMotor.stop();
        brakeMotor.stop();
        steeringMotor.stop();
        processControllers();
        break;
    case KartMode::AUTONOMOUS:
        processAutonomous();
        break;
    case KartMode::MANUAL:
        processManual();
        break;
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

    // ============================================================================
    // Initialize Hall Sensors (speed feedback)
    // ============================================================================
    pinMode(PIN_HALL_1, INPUT);
    pinMode(PIN_HALL_2, INPUT);
    pinMode(PIN_HALL_3, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_1), onHall1, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_2), onHall2, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL_3), onHall3, RISING);

    // ============================================================================
    // Initialize Analog Inputs + SDC MOSFET Control
    // ============================================================================
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_PRESSURE_1, ADC_11db);
    analogSetPinAttenuation(PIN_PRESSURE_2, ADC_11db);
    analogSetPinAttenuation(PIN_PRESSURE_3, ADC_11db);
    analogSetPinAttenuation(PIN_PEDAL_ACC, ADC_11db);
    analogSetPinAttenuation(PIN_PEDAL_BRAKE, ADC_11db);
    analogSetPinAttenuation(PIN_HYDRAULIC_1, ADC_11db); // ADC2: avoid WiFi when sampling
    analogSetPinAttenuation(PIN_HYDRAULIC_2, ADC_11db); // ADC2: avoid WiFi when sampling
    pinMode(PIN_SDC_NOT_EMERGENCY, OUTPUT);
    digitalWrite(PIN_SDC_NOT_EMERGENCY, LOW); // fail-safe until safety task runs

    xTaskCreatePinnedToCore(safetyTask, "safety", 4096, nullptr, 5, nullptr, 1);

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
    Console.println("CSV Format: DATA,LX,LY,RX,RY,L2,R2,BUTTONS,BATTERY,CHARGING,TARGET,ACTUAL,ERROR,PID,SENSOR_STATUS,HALL_HZ,SPEED_MPS,SPEED_KPH,PRESSURE1_V,PRESSURE2_V,PRESSURE3_V,PEDAL_ACC_V,PEDAL_BRAKE_V,HYDRAULIC1_V,HYDRAULIC2_V,SDC_FAULT,CMD_ACC_V,CMD_BRAKE_V\n");
}

// --------------------- Loop ---------------------
void loop()
{
    BP32.update();
    runMode();
    updateHallSpeed();
    delay(20); // 20ms = 50Hz update rate (good for PID control)
}
