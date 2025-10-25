// Kart Medulla - VESC/Little FOCer Control via UART
// Duty Cycle Control Mode - Controls motor with constant duty cycle

#include <Arduino.h>
#include <VescUart.h>

// UART Configuration for Little FOCer
// ESP32 Serial2: RX=GPIO16, TX=GPIO17
// Connect: ESP32 TX (GPIO 17) -> Little FOCer RX (COMM port)
//          ESP32 RX (GPIO 16) -> Little FOCer TX (COMM port)
//          ESP32 GND -> Little FOCer GND
HardwareSerial& VescSerial = Serial2;
VescUart VESC;

// Pin Definitions
#define LED_PIN 2

// Control variables
float dutyCycle = 0.1;  // Default duty cycle (0.1 = 10%)
bool motorRunning = true;

// Timing variables
unsigned long lastPrintTime = 0;
const long printInterval = 100; // Print every 100ms

// Function prototypes
void setupHardware();
void blinkLed();
void handleSerialCommands();
void printDiagnostics();

void setupHardware() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("\n=== Kart Medulla - VESC Duty Cycle Mode ===");

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize UART communication with Little FOCer
  // Using GPIO 16 (RX) and GPIO 17 (TX)
  VescSerial.begin(115200, SERIAL_8N1, 16, 17);

  // Set the serial port for VESC communication
  VESC.setSerialPort(&VescSerial);

  Serial.println("UART initialized - Connecting to Little FOCer...");
  Serial.println("Pins: RX=GPIO16, TX=GPIO17");
  Serial.print("Default duty cycle: ");
  Serial.println(dutyCycle);
  Serial.println("Type 'help' for available commands");
  Serial.println("==============================\n");
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("duty=")) {
      dutyCycle = command.substring(5).toFloat();
      dutyCycle = constrain(dutyCycle, -1.0, 1.0);
      motorRunning = true;
      Serial.print("Duty cycle set to: ");
      Serial.println(dutyCycle);
    }
    else if (command.startsWith("current=")) {
      float current = command.substring(8).toFloat();
      VESC.setCurrent(current);
      motorRunning = false;
      Serial.print("Setting current to: ");
      Serial.print(current);
      Serial.println(" A");
    }
    else if (command.startsWith("rpm=")) {
      float rpm = command.substring(4).toFloat();
      VESC.setRPM(rpm);
      motorRunning = false;
      Serial.print("Setting RPM to: ");
      Serial.println(rpm);
    }
    else if (command == "start") {
      motorRunning = true;
      Serial.print("Motor started with duty cycle: ");
      Serial.println(dutyCycle);
    }
    else if (command == "stop") {
      motorRunning = false;
      VESC.setDuty(0);
      Serial.println("Motor stopped (duty = 0)");
    }
    else if (command == "data") {
      // Request data from VESC
      if (VESC.getVescValues()) {
        Serial.println("\n=== VESC Data ===");
        Serial.print("Duty Cycle: ");
        Serial.print(VESC.data.dutyCycleNow * 100.0);
        Serial.println(" %");
        Serial.print("Motor Current: ");
        Serial.print(VESC.data.avg_motor_current);
        Serial.println(" A");
        Serial.print("Input Current: ");
        Serial.print(VESC.data.avg_input_current);
        Serial.println(" A");
        Serial.print("RPM: ");
        Serial.println(VESC.data.rpm);
        Serial.print("Input Voltage: ");
        Serial.print(VESC.data.inpVoltage);
        Serial.println(" V");
        Serial.print("Rotor Position: ");
        Serial.print(VESC.data.rotor_pos);
        Serial.println(" degrees");
        Serial.print("Tachometer: ");
        Serial.println(VESC.data.tachometer);
        Serial.println("================\n");
      } else {
        Serial.println("Failed to read VESC data");
      }
    }
    else if (command == "help") {
      Serial.println("\n=== VESC Commands ===");
      Serial.println("duty=X     - Set duty cycle (-1.0 to 1.0)");
      Serial.println("current=X  - Set motor current (amps)");
      Serial.println("rpm=X      - Set motor RPM");
      Serial.println("start      - Start motor with current duty cycle");
      Serial.println("stop       - Stop motor (duty = 0)");
      Serial.println("data       - Display VESC telemetry");
      Serial.println("help       - Show this help");
      Serial.println("====================\n");
    }
  }
}

void printDiagnostics() {
  unsigned long currentTime = millis();

  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;

    // Print motor status
    Serial.print(motorRunning ? "[RUNNING] " : "[STOPPED] ");
    Serial.print("Duty: ");
    Serial.print(dutyCycle, 3);
    Serial.print("  ");

    // Try to get VESC data
    if (VESC.getVescValues()) {
      Serial.print("Actual Duty: ");
      Serial.print(VESC.data.dutyCycleNow, 3);
      Serial.print("  Current: ");
      Serial.print(VESC.data.avg_motor_current, 2);
      Serial.print(" A  RPM: ");
      Serial.print(VESC.data.rpm, 0);
      Serial.print("  Voltage: ");
      Serial.print(VESC.data.inpVoltage, 1);
      Serial.println(" V");
    } else {
      Serial.println("  [No VESC data]");
    }
  }
}

void blinkLed() {
  static unsigned long lastLedToggleTime = 0;
  const long ledInterval = 500;

  if (millis() - lastLedToggleTime >= ledInterval) {
    lastLedToggleTime = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void setup() {
  setupHardware();
  delay(100);  // Give VESC time to initialize

  // Set initial duty cycle
  Serial.println("Setting initial duty cycle to 0.1...");
  VESC.setDuty(dutyCycle);
}

void loop() {
  // 1. Handle serial commands
  handleSerialCommands();

  // 2. Send duty cycle to VESC if motor is running
  if (motorRunning) {
    VESC.setDuty(dutyCycle);
  }

  // 3. Print diagnostics
  printDiagnostics();

  // 4. Blink LED to show we're alive
  blinkLed();

  // 5. Small delay to not overwhelm the UART bus
  delay(20);  // 50Hz control loop
}
