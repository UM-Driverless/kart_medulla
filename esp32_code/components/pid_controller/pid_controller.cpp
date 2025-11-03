#include "pid_controller.h"

PIDController::PIDController()
    : kp(0.0f),
      ki(0.0f),
      kd(0.0f),
      integral(0.0f),
      lastError(0.0f),
      lastTime(0),
      outputMin(-1.0f),
      outputMax(1.0f),
      integralMin(-1.0f),
      integralMax(1.0f) {
}

void PIDController::init(float kp_val, float ki_val, float kd_val) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;
    integral = 0.0f;
    lastError = 0.0f;
    lastTime = millis();

    Serial.printf("PID initialized: Kp=%.2f, Ki=%.4f, Kd=%.2f\n", kp, ki, kd);
}

float PIDController::calculate(float setpoint, float measurement) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f;  // Convert to seconds

    // Avoid division by zero
    if (dt <= 0.0f) {
        dt = 0.001f;  // Minimum 1ms
    }

    // Calculate error
    float error = setpoint - measurement;

    // Proportional term
    float pTerm = kp * error;

    // Integral term with anti-windup
    integral += error * dt;
    integral = constrain(integral, integralMin, integralMax);
    float iTerm = ki * integral;

    // Derivative term
    float derivative = (error - lastError) / dt;
    float dTerm = kd * derivative;

    // Calculate total output
    float output = pTerm + iTerm + dTerm;

    // Apply output limits
    output = constrain(output, outputMin, outputMax);

    // Update state
    lastError = error;
    lastTime = currentTime;

    return output;
}

void PIDController::setTunings(float kp_val, float ki_val, float kd_val) {
    kp = kp_val;
    ki = ki_val;
    kd = kd_val;

    Serial.printf("PID tunings updated: Kp=%.2f, Ki=%.4f, Kd=%.2f\n", kp, ki, kd);
}

void PIDController::setOutputLimits(float min, float max) {
    outputMin = min;
    outputMax = max;

    Serial.printf("PID output limits: [%.2f, %.2f]\n", min, max);
}

void PIDController::setIntegralLimits(float min, float max) {
    integralMin = min;
    integralMax = max;

    Serial.printf("PID integral limits: [%.2f, %.2f]\n", min, max);
}

void PIDController::reset() {
    integral = 0.0f;
    lastError = 0.0f;
    lastTime = millis();

    Serial.println("PID controller reset");
}

void PIDController::getTunings(float &kp_val, float &ki_val, float &kd_val) {
    kp_val = kp;
    ki_val = ki;
    kd_val = kd;
}
