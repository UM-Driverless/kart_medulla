#include <Arduino.h>

#define LED_PIN PC13

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, LOW);   // Turn the LED on (PC13 is active low)
  delay(1000);                  // Wait for a second
  digitalWrite(LED_PIN, HIGH);  // Turn the LED off
  delay(1000);                  // Wait for a second
}