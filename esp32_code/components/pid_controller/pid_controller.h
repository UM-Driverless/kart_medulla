#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
private:
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integral;     // Integral accumulator
    float lastError;    // Previous error for derivative
    unsigned long lastTime;  // Last update time

    // Limits
    float outputMin;
    float outputMax;
    float integralMin;  // Anti-windup limits
    float integralMax;

public:
    PIDController();

    // Initialize with gains
    void init(float kp, float ki, float kd);

    // Calculate PID output
    float calculate(float setpoint, float measurement);

    // Set tuning parameters at runtime
    void setTunings(float kp, float ki, float kd);

    // Set output limits
    void setOutputLimits(float min, float max);

    // Set integral limits (anti-windup)
    void setIntegralLimits(float min, float max);

    // Reset controller state
    void reset();

    // Get current gains
    void getTunings(float &kp, float &ki, float &kd);

    // Get integral value (for debugging)
    float getIntegral() const { return integral; }
};

#endif // PID_CONTROLLER_H