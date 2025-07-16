// Simple PWM Test - 50% duty cycle on pin 25

#include <Arduino.h>

#define PWM_PIN 25

void setup()
{
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);
  
  Serial.println("Starting 50% PWM on pin 25");
}

void loop()
{
  analogWrite(PWM_PIN, 128); // 50% duty cycle (128/255 = 0.5)
  delay(1000);
}