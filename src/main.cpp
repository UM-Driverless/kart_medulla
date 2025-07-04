#include <Arduino.h>

#define PWM_PIN PA8      // PWM
#define DIR_PIN PA9      // Sentido de giro
#define LED_BUILTIN PC13 // Onboard LED for Bluepill

int pwm = 0;
bool sentido = LOW;

unsigned long lastMotorChangeTime = 0;
const long motorInterval = 100; // Milliseconds between PWM changes

unsigned long lastLedToggleTime = 0;
const long ledInterval = 500; // Milliseconds between LED toggles

bool ledState = LOW;

void setup()
{
  Serial.begin(115200); // Initialize serial communication
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); // Initial direction

  pinMode(LED_BUILTIN, OUTPUT);   // Set LED pin as output
  digitalWrite(LED_BUILTIN, LOW); // Ensure LED is off initially
}

void loop()
{
  unsigned long currentMillis = millis();

  // Motor control logic (non-blocking)
  if (currentMillis - lastMotorChangeTime >= motorInterval)
  {
    lastMotorChangeTime = currentMillis;

    if (sentido == LOW)
    { // Increasing PWM
      if (pwm < 255)
      {
        pwm += 10;
      }
      else
      { // Reached max PWM, switch internal 'sentido' state to decrease PWM
        sentido = HIGH;
        // No digitalWrite(DIR_PIN) here, as per your request
      }
    }
    else
    { // Decreasing PWM
      if (pwm > 0)
      {
        pwm -= 10;
      }
      else
      { // Reached min PWM, switch internal 'sentido' state to increase PWM AND toggle motor direction
        sentido = LOW;
        digitalWrite(DIR_PIN, !digitalRead(DIR_PIN)); // Toggle motor direction
      }
    }
    analogWrite(PWM_PIN, pwm);
    Serial.print("PWM: ");
    Serial.print(pwm);
    Serial.print(", DIR: ");
    Serial.println(digitalRead(DIR_PIN));
  }

  // LED blinking logic (non-blocking)
  if (currentMillis - lastLedToggleTime >= ledInterval)
  {
    lastLedToggleTime = currentMillis;
    ledState = !ledState; // Toggle LED state
    digitalWrite(LED_BUILTIN, ledState);
  }
}