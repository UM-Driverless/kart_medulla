// PS5 Controller Debug Test
#include <Arduino.h>
#include <ps5Controller.h>

// Event callbacks to see connection progress
void onConnect() {
    Serial.println("\n=================================");
    Serial.println("✓ PS5 CONTROLLER CONNECTED!");
    Serial.println("=================================\n");
}

void onDisconnect() {
    Serial.println("\n=================================");
    Serial.println("✗ PS5 CONTROLLER DISCONNECTED");
    Serial.println("=================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔═══════════════════════════════════╗");
    Serial.println("║  PS5 Controller Connection Test  ║");
    Serial.println("╚═══════════════════════════════════╝\n");

    // Attach event callbacks
    ps5.attachOnConnect(onConnect);
    ps5.attachOnDisconnect(onDisconnect);

    // Start PS5 controller with MAC address
    Serial.println("Starting PS5 library...");
    ps5.begin("AC:36:1B:09:F0:28");

    Serial.println("MAC Address: AC:36:1B:09:F0:28");
    Serial.println("\nWaiting for PS5 controller...");
    Serial.println("Put controller in pairing mode:");
    Serial.println("  → Hold SHARE + PS buttons");
    Serial.println("  → Light should flash blue rapidly");
    Serial.println("\nWatching for L2CAP events...\n");
}

void loop() {
    if (ps5.isConnected()) {
        // Show controller is connected
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink >= 1000) {
            lastBlink = millis();
            Serial.println("✓ Connected - Reading data...");

            // Print some basic data
            Serial.printf("  Left Stick X: %4d\n", ps5.LStickX());
            Serial.printf("  Buttons: %s\n", ps5.Cross() ? "X pressed" : "---");
        }
    } else {
        // Heartbeat while waiting
        static unsigned long lastHeart = 0;
        if (millis() - lastHeart >= 3000) {
            lastHeart = millis();
            Serial.println("⏳ Waiting for controller...");
        }
    }

    delay(10);
}
