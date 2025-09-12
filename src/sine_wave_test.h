#ifndef SINE_WAVE_TEST_H
#define SINE_WAVE_TEST_H

#include <Arduino.h>

class SineWaveTest {
private:
    bool testActive;
    unsigned long testStartTime;
    float amplitude;      // degrees
    float frequency;      // Hz
    unsigned long duration; // ms
    
    // Performance metrics
    float maxError;
    float totalError;
    int sampleCount;
    float maxActual;
    float minActual;

public:
    SineWaveTest();
    void startTest(float ampDegrees = 30.0, float freqHz = 0.2, unsigned long durationMs = 20000);
    bool isActive();
    float getCurrentTargetAngle();
    void update(float actualAngle);
    void stopTest();
};

#endif