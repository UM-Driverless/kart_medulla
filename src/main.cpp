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

void printBar(const char* label, int value, int minVal, int maxVal, int width = 20) {
    // Print a visual bar graph
    Serial.print(label);
    Serial.print(": [");

    int range = maxVal - minVal;
    int pos = map(value, minVal, maxVal, 0, width);

    for (int i = 0; i < width; i++) {
        if (i == width / 2) {
            Serial.print(i == pos ? "|" : "·");
        } else if (i == pos) {
            Serial.print("█");
        } else {
            Serial.print("-");
        }
    }
    Serial.printf("] %4d\n", value);
}

void displayControllerData() {
    Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // Analog Sticks
    Serial.println("ANALOG STICKS:");
    printBar("  L-Stick X", ps5.LStickX(), -128, 127, 30);
    printBar("  L-Stick Y", ps5.LStickY(), -128, 127, 30);
    printBar("  R-Stick X", ps5.RStickX(), -128, 127, 30);
    printBar("  R-Stick Y", ps5.RStickY(), -128, 127, 30);

    // Triggers (0-255)
    Serial.println("\nTRIGGERS:");
    printBar("  L2", ps5.L2Value(), 0, 255, 30);
    printBar("  R2", ps5.R2Value(), 0, 255, 30);

    // D-Pad
    Serial.println("\nD-PAD:");
    Serial.printf("  %-8s %-8s %-8s %-8s\n",
                  ps5.Up() ? "UP" : "---",
                  ps5.Down() ? "DOWN" : "---",
                  ps5.Left() ? "LEFT" : "---",
                  ps5.Right() ? "RIGHT" : "---");

    // Face Buttons
    Serial.println("\nFACE BUTTONS:");
    Serial.printf("  %-10s %-10s %-10s %-10s\n",
                  ps5.Triangle() ? "TRIANGLE" : "---",
                  ps5.Circle() ? "CIRCLE" : "---",
                  ps5.Cross() ? "CROSS" : "---",
                  ps5.Square() ? "SQUARE" : "---");

    // Shoulder Buttons
    Serial.println("\nSHOULDER BUTTONS:");
    Serial.printf("  %-6s %-6s %-6s %-6s\n",
                  ps5.L1() ? "L1" : "---",
                  ps5.L2() ? "L2" : "---",
                  ps5.R1() ? "R1" : "---",
                  ps5.R2() ? "R2" : "---");

    // Special Buttons
    Serial.println("\nSPECIAL:");
    Serial.printf("  %-8s %-8s %-8s %-8s %-8s\n",
                  ps5.PSButton() ? "PS" : "---",
                  ps5.Share() ? "SHARE" : "---",
                  ps5.Options() ? "OPTIONS" : "---",
                  ps5.L3() ? "L3" : "---",
                  ps5.R3() ? "R3" : "---");

    // Status
    Serial.println("\nSTATUS:");
    Serial.printf("  Battery: %d%%  %s  Audio: %s  Mic: %s\n",
                  ps5.Battery(),
                  ps5.Charging() ? "CHARGING" : "NOT CHARGING",
                  ps5.Audio() ? "ON" : "OFF",
                  ps5.Mic() ? "ON" : "OFF");

    Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    Serial.println("Press OPTIONS to cycle LED colors | Press PS to quit\n");
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

    // ALWAYS print heartbeat every 1 second to prove ESP32 is alive
    if (millis() - lastHeartbeat >= 1000) {
        lastHeartbeat = millis();
        Serial.print("💓 [");
        Serial.print(millis() / 1000);
        Serial.print("s] isConnected=");
        Serial.println(ps5.isConnected() ? "YES" : "NO");
        Serial.flush();  // Force output immediately
    }

    if (ps5.isConnected()) {
        // Update display at 20 Hz (50ms interval) for responsiveness
        if (millis() - lastUpdate >= 50) {
            lastUpdate = millis();
            displayControllerData();
            Serial.flush();  // Force output immediately
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

    } else {
        // Heartbeat while waiting
        static unsigned long lastWait = 0;
        if (millis() - lastWait >= 3000) {
            lastWait = millis();
            Serial.println("⏳ Waiting for controller connection...");
            Serial.flush();
        }
    }

    delay(5);  // Reduced delay for faster loop
}
