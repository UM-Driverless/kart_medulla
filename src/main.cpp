// Kart Medulla - VESC/Little FOCer Control via UART
// This version communicates with Little FOCer Rev3.1 using VESC UART protocol

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

// Test mode variables
bool testModeEnabled = true;
float testAmplitude = 0.0;    // degrees - start at 0 (straight)
float testPeriod = 5.0;       // seconds
int testMode = 2;             // 1=sine, 2=constant, 3=step

// Timing variables
unsigned long lastPrintTime = 0;
const long printInterval = 100; // Print every 100ms

// Function prototypes
void setupHardware();
void blinkLed();
void handleSerialCommands();
void printDiagnostics();
float getTargetAngle();
float generateSineWaveTargetAngle(float amplitude_degrees, float period_seconds);

void setupHardware() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("\n=== Kart Medulla - VESC Mode ===");

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
  Serial.println("Type 'help' for available commands");
  Serial.println("==============================\n");
}

float generateSineWaveTargetAngle(float amplitude_degrees, float period_seconds) {
  return amplitude_degrees * sin(2.0 * PI * (millis() / (period_seconds * 1000.0)));
}

float getTargetAngle() {
  float targetAngle;

  if (testModeEnabled) {
    switch(testMode) {
      case 1: // Sine wave
        targetAngle = generateSineWaveTargetAngle(testAmplitude, testPeriod);
        break;
      case 2: // Constant angle
        targetAngle = testAmplitude;
        break;
      case 3: // Step function
        targetAngle = ((millis() / (int)(testPeriod * 1000)) % 2 == 0) ?
                      testAmplitude : -testAmplitude;
        break;
      default:
        targetAngle = 0.0;
    }
  } else {
    targetAngle = 0.0;
  }

  return targetAngle;
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("pos=")) {
      float angle = command.substring(4).toFloat();
      VESC.setPosition(angle);
      Serial.print("Setting position to: ");
      Serial.print(angle);
      Serial.println(" degrees");
    }
    else if (command.startsWith("current=")) {
      float current = command.substring(8).toFloat();
      VESC.setCurrent(current);
      Serial.print("Setting current to: ");
      Serial.print(current);
      Serial.println(" A");
    }
    else if (command.startsWith("rpm=")) {
      float rpm = command.substring(4).toFloat();
      VESC.setRPM(rpm);
      Serial.print("Setting RPM to: ");
      Serial.println(rpm);
    }
    else if (command.startsWith("duty=")) {
      float duty = command.substring(5).toFloat();
      VESC.setDuty(duty);
      Serial.print("Setting duty cycle to: ");
      Serial.println(duty);
    }
    else if (command == "test") {
      testModeEnabled = !testModeEnabled;
      Serial.print("Test mode: ");
      Serial.println(testModeEnabled ? "ENABLED" : "DISABLED");
    }
    else if (command.startsWith("mode=")) {
      testMode = command.substring(5).toInt();
      Serial.print("Test mode changed to: ");
      Serial.println(testMode == 1 ? "SINE" : (testMode == 2 ? "CONSTANT" : "STEP"));
    }
    else if (command.startsWith("amp=")) {
      testAmplitude = command.substring(4).toFloat();
      Serial.print("Test amplitude: ");
      Serial.print(testAmplitude);
      Serial.println(" degrees");
    }
    else if (command.startsWith("period=")) {
      testPeriod = command.substring(7).toFloat();
      Serial.print("Test period: ");
      Serial.print(testPeriod);
      Serial.println(" seconds");
    }
    else if (command == "stop") {
      VESC.setCurrent(0);
      testModeEnabled = false;
      Serial.println("Motor stopped");
    }
    else if (command == "data") {
      // Request data from VESC
      if (VESC.getVescValues()) {
        Serial.println("\n=== VESC Data ===");
        Serial.print("Rotor Position: ");
        Serial.print(VESC.data.rotor_pos);
        Serial.println(" degrees");
        Serial.print("Motor Current: ");
        Serial.print(VESC.data.avg_motor_current);
        Serial.println(" A");
        Serial.print("Input Current: ");
        Serial.print(VESC.data.avg_input_current);
        Serial.println(" A");
        Serial.print("Duty Cycle: ");
        Serial.print(VESC.data.dutyCycleNow);
        Serial.println(" %");
        Serial.print("RPM: ");
        Serial.println(VESC.data.rpm);
        Serial.print("Input Voltage: ");
        Serial.print(VESC.data.inpVoltage);
        Serial.println(" V");
        Serial.print("Amp Hours: ");
        Serial.print(VESC.data.ampHours);
        Serial.println(" Ah");
        Serial.print("Amp Hours Charged: ");
        Serial.print(VESC.data.ampHoursCharged);
        Serial.println(" Ah");
        Serial.print("Tachometer: ");
        Serial.println(VESC.data.tachometer);
        Serial.print("Tachometer ABS: ");
        Serial.println(VESC.data.tachometerAbs);
        Serial.println("================\n");
      } else {
        Serial.println("Failed to read VESC data");
      }
    }
    else if (command == "help") {
      Serial.println("\n=== VESC Commands ===");
      Serial.println("pos=X      - Set position to X degrees");
      Serial.println("current=X  - Set motor current to X amps");
      Serial.println("rpm=X      - Set motor RPM to X");
      Serial.println("duty=X     - Set duty cycle to X (-1.0 to 1.0)");
      Serial.println("stop       - Stop motor (set current to 0)");
      Serial.println("data       - Request and display VESC data");
      Serial.println("\n=== Test Mode ===");
      Serial.println("test       - Toggle test mode");
      Serial.println("mode=N     - Set test mode (1=sine, 2=constant, 3=step)");
      Serial.println("amp=X      - Set test amplitude (degrees)");
      Serial.println("period=X   - Set test period (seconds)");
      Serial.println("\n=== Info ===");
      Serial.println("help       - Show this help");
      Serial.println("====================\n");
    }
  }
}

void printDiagnostics() {
  unsigned long currentTime = millis();

  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;

    float targetAngle = getTargetAngle();

    // Print test mode status
    if (testModeEnabled) {
      Serial.print("[TEST ");
      if (testMode == 1) Serial.print("SINE");
      else if (testMode == 2) Serial.print("CONST");
      else if (testMode == 3) Serial.print("STEP");
      Serial.print("] ");
    }

    Serial.print("Target: ");
    Serial.print(targetAngle, 2);
    Serial.print("°  ");

    // Try to get VESC data
    if (VESC.getVescValues()) {
      Serial.print("Actual: ");
      Serial.print(VESC.data.rotor_pos, 2);
      Serial.print("°  Error: ");
      Serial.print(targetAngle - VESC.data.rotor_pos, 2);
      Serial.print("°  Current: ");
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
}

void loop() {
  // 1. Handle serial commands
  handleSerialCommands();

  // 2. Get target angle and send to VESC
  if (testModeEnabled) {
    float targetAngle = getTargetAngle();
    VESC.setPosition(targetAngle);
  }

  // 3. Print diagnostics
  printDiagnostics();

  // 4. Blink LED to show we're alive
  blinkLed();

  // 5. Small delay to not overwhelm the UART bus
  delay(20);  // 50Hz control loop
}
