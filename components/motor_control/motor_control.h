#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <driver/dac_oneshot.h>

// Steering motor class - uses PWM + Direction
class SteeringMotor {
private:
    int pwmPin;
    int dirPin;
    int pwmChannel;
    int pwmFreq;
    int pwmResolution;
    float outputLimit;

public:
    SteeringMotor(int pwm, int dir, int channel = 0, int freq = 5000, int resolution = 8);

    void begin();
    void setOutput(float pidOutput);  // Input: -1.0 to 1.0
    void setOutputLimit(float limit);  // Limit max output (0.0 to 1.0)
    void stop();
};

// Throttle motor class - uses DAC for true analog output
class ThrottleMotor {
private:
    dac_oneshot_handle_t dacHandle;
    dac_channel_t dacChannel;
    float outputLimit;

public:
    ThrottleMotor(int dacPin);  // GPIO 25 (DAC_CHAN_0) or GPIO 26 (DAC_CHAN_1)

    void begin();
    void setOutput(float value);  // Input: 0.0 to 1.0
    void setOutputLimit(float limit);
    void stop();
};

// Brake valve class - uses DAC for true analog output
class BrakeValve {
private:
    dac_oneshot_handle_t dacHandle;
    dac_channel_t dacChannel;
    float outputLimit;

public:
    BrakeValve(int dacPin);  // GPIO 25 (DAC_CHAN_0) or GPIO 26 (DAC_CHAN_1)

    void begin();
    void setOutput(float value);  // Input: 0.0 to 1.0
    void setOutputLimit(float limit);
    void stop();
};

#endif // MOTOR_CONTROL_H
