#ifndef STEERING_TEST_SEQUENCE_H
#define STEERING_TEST_SEQUENCE_H

#include <Arduino.h>

class SteeringTestSequence {
private:
    bool testActive;
    int currentStep;
    unsigned long stepStartTime;
    unsigned long stepDuration;  // ms per step
    float testAngles[7];
    int numSteps;
    
    // Test results
    float actualAngles[7];
    float errors[7];
    bool stepComplete[7];

public:
    SteeringTestSequence();
    void startSequenceTest();
    bool isActive();
    float getCurrentTargetAngle();
    void update(float actualAngle);
    void printResults();
};

#endif