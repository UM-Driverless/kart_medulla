// PS5 Controller Real-Time Value Monitor
#include <Arduino.h>
#include <ps5Controller.h>

void onConnect() {
    Serial.println("\n===========================================");
    Serial.println("  PS5 CONTROLLER CONNECTED!");
    Serial.println("===========================================\n");
    // Set LED to green to indicate connection
    ps5.setLed(0, 255, 0);
    ps5.sendToController();
}

void onDisconnect() {
    Serial.println("\n===========================================");
    Serial.println("  PS5 CONTROLLER DISCONNECTED");
    Serial.println("===========================================\n");
}

void displayControllerData() {
    // Compact CSV format: DATA,LX,LY,RX,RY,L2,R2,BUTTONS,BATTERY,CHARGING,AUDIO,MIC
    Serial.print("DATA,");

    // Analog sticks (-128 to 127)
    Serial.print(ps5.LStickX()); Serial.print(",");
    Serial.print(ps5.LStickY()); Serial.print(",");
    Serial.print(ps5.RStickX()); Serial.print(",");
    Serial.print(ps5.RStickY()); Serial.print(",");

    // Triggers (0-255)
    Serial.print(ps5.L2Value()); Serial.print(",");
    Serial.print(ps5.R2Value()); Serial.print(",");

    // Buttons (pipe-separated list or empty)
    bool firstButton = true;
    if (ps5.Up()) { Serial.print("UP"); firstButton = false; }
    if (ps5.Down()) { if (!firstButton) Serial.print("|"); Serial.print("DOWN"); firstButton = false; }
    if (ps5.Left()) { if (!firstButton) Serial.print("|"); Serial.print("LEFT"); firstButton = false; }
    if (ps5.Right()) { if (!firstButton) Serial.print("|"); Serial.print("RIGHT"); firstButton = false; }
    if (ps5.Triangle()) { if (!firstButton) Serial.print("|"); Serial.print("TRIANGLE"); firstButton = false; }
    if (ps5.Circle()) { if (!firstButton) Serial.print("|"); Serial.print("CIRCLE"); firstButton = false; }
    if (ps5.Cross()) { if (!firstButton) Serial.print("|"); Serial.print("CROSS"); firstButton = false; }
    if (ps5.Square()) { if (!firstButton) Serial.print("|"); Serial.print("SQUARE"); firstButton = false; }
    if (ps5.L1()) { if (!firstButton) Serial.print("|"); Serial.print("L1"); firstButton = false; }
    if (ps5.L2()) { if (!firstButton) Serial.print("|"); Serial.print("L2"); firstButton = false; }
    if (ps5.R1()) { if (!firstButton) Serial.print("|"); Serial.print("R1"); firstButton = false; }
    if (ps5.R2()) { if (!firstButton) Serial.print("|"); Serial.print("R2"); firstButton = false; }
    if (ps5.PSButton()) { if (!firstButton) Serial.print("|"); Serial.print("PS"); firstButton = false; }
    if (ps5.Share()) { if (!firstButton) Serial.print("|"); Serial.print("SHARE"); firstButton = false; }
    if (ps5.Options()) { if (!firstButton) Serial.print("|"); Serial.print("OPTIONS"); firstButton = false; }
    if (ps5.L3()) { if (!firstButton) Serial.print("|"); Serial.print("L3"); firstButton = false; }
    if (ps5.R3()) { if (!firstButton) Serial.print("|"); Serial.print("R3"); firstButton = false; }
    Serial.print(",");

    // Status (battery 0-100, charging 0/1, audio 0/1, mic 0/1)
    Serial.print(ps5.Battery()); Serial.print(",");
    Serial.print(ps5.Charging() ? "1" : "0"); Serial.print(",");
    Serial.print(ps5.Audio() ? "1" : "0"); Serial.print(",");
    Serial.print(ps5.Mic() ? "1" : "0");

    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔═════════════════════════════════════════╗");
    Serial.println("║  PS5 Controller Real-Time Monitor      ║");
    Serial.println("╚═════════════════════════════════════════╝\n");

    ps5.attachOnConnect(onConnect);
    ps5.attachOnDisconnect(onDisconnect);

    Serial.println("Starting PS5 Bluetooth...");
    ps5.begin("AC:36:1B:09:F0:28");

    Serial.println("Controller MAC: AC:36:1B:09:F0:28\n");
    Serial.println("Pairing Instructions:");
    Serial.println("  1. Reset controller (hole on back, 5 sec)");
    Serial.println("  2. Hold SHARE + PS (3-5 sec)");
    Serial.println("  3. Look for RAPID blue flashing");
    Serial.println("\nWaiting for controller...\n");
}

void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastHeartbeat = 0;
    static uint8_t ledMode = 0;
    static bool lastOptionsState = false;

    // Heartbeat every 1 second: HB,uptime_sec,connected
    if (millis() - lastHeartbeat >= 1000) {
        lastHeartbeat = millis();
        Serial.print("HB,");
        Serial.print(millis() / 1000);
        Serial.print(",");
        Serial.println(ps5.isConnected() ? "1" : "0");
    }

    if (ps5.isConnected()) {
        // Update display at 10 Hz (100ms interval) for good responsiveness without overload
        if (millis() - lastUpdate >= 100) {
            lastUpdate = millis();
            displayControllerData();
        }

        // LED color cycling with OPTIONS button
        if (ps5.Options() && !lastOptionsState) {
            ledMode = (ledMode + 1) % 5;
            switch(ledMode) {
                case 0: ps5.setLed(0, 255, 0); break;   // Green
                case 1: ps5.setLed(255, 0, 0); break;   // Red
                case 2: ps5.setLed(0, 0, 255); break;   // Blue
                case 3: ps5.setLed(255, 255, 0); break; // Yellow
                case 4: ps5.setLed(255, 0, 255); break; // Magenta
            }
            ps5.sendToController();
        }
        lastOptionsState = ps5.Options();

    }
    // No extra messages when not connected - heartbeat already shows status

    delay(10);  // Small delay to prevent watchdog issues
}
