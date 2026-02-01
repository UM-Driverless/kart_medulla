#include "motor_control.h"

// ============================================================================
// Steering Motor Implementation (PWM + Direction)
// ============================================================================

SteeringMotor::SteeringMotor(int pwm, int dir, int channel, int freq, int resolution)
    : pwmPin(pwm),
      dirPin(dir),
      pwmChannel(channel),
      pwmFreq(freq),
      pwmResolution(resolution),
      outputLimit(1.0f) {
}

void SteeringMotor::begin() {
    // Configure direction pin
    pinMode(dirPin, OUTPUT);
    digitalWrite(dirPin, LOW);

    // Configure PWM using new API (ledcAttach combines setup + attach)
    ledcAttach(pwmPin, pwmFreq, pwmResolution);
    ledcWrite(pwmPin, 0);

    Serial.printf("Steering motor initialized: PWM=%d, DIR=%d, Freq=%dHz\n",
                  pwmPin, dirPin, pwmFreq);
}

void SteeringMotor::setOutput(float pidOutput) {
    // Clamp to limits
    pidOutput = constrain(pidOutput, -outputLimit, outputLimit);

    // Determine direction
    bool direction = (pidOutput >= 0);
    digitalWrite(dirPin, direction ? HIGH : LOW);

    // Calculate PWM value (0-255 for 8-bit resolution)
    int pwmValue = (int)(abs(pidOutput) * ((1 << pwmResolution) - 1));
    pwmValue = constrain(pwmValue, 0, (1 << pwmResolution) - 1);

    // Set PWM (new API uses pin number instead of channel)
    ledcWrite(pwmPin, pwmValue);
}

void SteeringMotor::setOutputLimit(float limit) {
    outputLimit = constrain(limit, 0.0f, 1.0f);
    Serial.printf("Steering motor output limit set to: %.2f\n", outputLimit);
}

void SteeringMotor::stop() {
    ledcWrite(pwmPin, 0);
    digitalWrite(dirPin, LOW);
}

// ============================================================================
// Throttle Motor Implementation (DAC)
// ============================================================================

ThrottleMotor::ThrottleMotor(int dacPin)
    : dacHandle(nullptr), outputLimit(1.0f) {
    if (dacPin == 25) {
        dacChannel = DAC_CHAN_0;  // GPIO 25
    } else if (dacPin == 26) {
        dacChannel = DAC_CHAN_1;  // GPIO 26
    } else {
        Serial.printf("ERROR: Invalid DAC pin %d (must be 25 or 26)\n", dacPin);
        dacChannel = DAC_CHAN_1;  // Default to GPIO 26
    }
}

void ThrottleMotor::begin() {
    // Configure DAC using new oneshot API
    dac_oneshot_config_t dac_cfg = {
        .chan_id = dacChannel
    };

    esp_err_t err = dac_oneshot_new_channel(&dac_cfg, &dacHandle);
    if (err != ESP_OK) {
        Serial.printf("ERROR: Failed to initialize DAC channel (error: 0x%x)\n", err);
        return;
    }

    // Set initial voltage to 0
    dac_oneshot_output_voltage(dacHandle, 0);

    Serial.printf("Throttle motor initialized: DAC Channel=%d (GPIO %d)\n",
                  dacChannel, dacChannel == DAC_CHAN_0 ? 25 : 26);
}

void ThrottleMotor::setOutput(float value) {
    if (dacHandle == nullptr) return;

    // Clamp to limits
    value = constrain(value, 0.0f, outputLimit);

    // Convert to DAC value (0-255)
    uint8_t dacValue = (uint8_t)(value * 255.0f);

    // Set DAC output using oneshot API
    dac_oneshot_output_voltage(dacHandle, dacValue);
}

void ThrottleMotor::setOutputLimit(float limit) {
    outputLimit = constrain(limit, 0.0f, 1.0f);
    Serial.printf("Throttle motor output limit set to: %.2f\n", outputLimit);
}

void ThrottleMotor::stop() {
    if (dacHandle != nullptr) {
        dac_oneshot_output_voltage(dacHandle, 0);
    }
}

// ============================================================================
// Brake Valve Implementation (DAC)
// ============================================================================

BrakeValve::BrakeValve(int dacPin)
    : dacHandle(nullptr), outputLimit(1.0f) {
    if (dacPin == 25) {
        dacChannel = DAC_CHAN_0;  // GPIO 25
    } else if (dacPin == 26) {
        dacChannel = DAC_CHAN_1;  // GPIO 26
    } else {
        Serial.printf("ERROR: Invalid DAC pin %d (must be 25 or 26)\n", dacPin);
        dacChannel = DAC_CHAN_0;  // Default to GPIO 25
    }
}

void BrakeValve::begin() {
    // Configure DAC using new oneshot API
    dac_oneshot_config_t dac_cfg = {
        .chan_id = dacChannel
    };

    esp_err_t err = dac_oneshot_new_channel(&dac_cfg, &dacHandle);
    if (err != ESP_OK) {
        Serial.printf("ERROR: Failed to initialize DAC channel (error: 0x%x)\n", err);
        return;
    }

    // Set initial voltage to 0
    dac_oneshot_output_voltage(dacHandle, 0);

    Serial.printf("Brake valve initialized: DAC Channel=%d (GPIO %d)\n",
                  dacChannel, dacChannel == DAC_CHAN_0 ? 25 : 26);
}

void BrakeValve::setOutput(float value) {
    if (dacHandle == nullptr) return;

    // Clamp to limits
    value = constrain(value, 0.0f, outputLimit);

    // Convert to DAC value (0-255)
    uint8_t dacValue = (uint8_t)(value * 255.0f);

    // Set DAC output using oneshot API
    dac_oneshot_output_voltage(dacHandle, dacValue);
}

void BrakeValve::setOutputLimit(float limit) {
    outputLimit = constrain(limit, 0.0f, 1.0f);
    Serial.printf("Brake valve output limit set to: %.2f\n", outputLimit);
}

void BrakeValve::stop() {
    if (dacHandle != nullptr) {
        dac_oneshot_output_voltage(dacHandle, 0);
    }
}
