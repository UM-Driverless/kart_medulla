#ifndef STEERING_TEST_H
#define STEERING_TEST_H

#include <Arduino.h>

class SteeringTest {
private:
    bool testActive;
    unsigned long testStartTime;
    unsigned long settlingStartTime;
    float targetAngle;  // radians
    float settlingThreshold;  // degrees
    float steadyStateThreshold;  // degrees
    unsigned long maxSettlingTime;  // ms
    bool hasSettled;
    float errorSum;
    int errorCount;
    float maxError;

public:
    SteeringTest();
    void startSteadyStateTest(float targetDegrees);
    bool isActive();
    float getTargetAngle();
    void update(float actualAngle);
};

#endif