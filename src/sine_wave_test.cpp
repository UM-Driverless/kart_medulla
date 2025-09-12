#include "sine_wave_test.h"

SineWaveTest::SineWaveTest() {
    testActive = false;
    amplitude = 30.0;
    frequency = 0.2;
    duration = 20000;
}

void SineWaveTest::startTest(float ampDegrees, float freqHz, unsigned long durationMs) {
    Serial.println("\n=== SINE WAVE TEST ===");
    Serial.print("Amplitude: ±");
    Serial.print(ampDegrees);
    Serial.println("°");
    Serial.print("Frequency: ");
    Serial.print(freqHz);
    Serial.println(" Hz");
    Serial.print("Duration: ");
    Serial.print(durationMs / 1000.0);
    Serial.println(" seconds");
    Serial.println("Starting test...\n");
    
    testActive = true;
    testStartTime = millis();
    amplitude = ampDegrees;
    frequency = freqHz;
    duration = durationMs;
    
    // Reset metrics
    maxError = 0.0;
    totalError = 0.0;
    sampleCount = 0;
    maxActual = -180.0;
    minActual = 180.0;
}

bool SineWaveTest::isActive() {
    return testActive;
}

float SineWaveTest::getCurrentTargetAngle() {
    if (!testActive) return 0.0;
    
    unsigned long elapsed = millis() - testStartTime;
    if (elapsed >= duration) {
        stopTest();
        return 0.0;
    }
    
    // Generate sine wave
    float timeSeconds = elapsed / 1000.0;
    float angleRadians = amplitude * sin(2 * PI * frequency * timeSeconds) * PI / 180.0;
    return angleRadians;
}

void SineWaveTest::update(float actualAngle) {
    if (!testActive) return;
    
    unsigned long elapsed = millis() - testStartTime;
    if (elapsed >= duration) {
        stopTest();
        return;
    }
    
    // Calculate target for comparison
    float timeSeconds = elapsed / 1000.0;
    float targetDegrees = amplitude * sin(2 * PI * frequency * timeSeconds);
    float actualDegrees = actualAngle * 180.0 / PI;
    
    // Update metrics
    float error = abs(targetDegrees - actualDegrees);
    totalError += error;
    sampleCount++;
    
    if (error > maxError) {
        maxError = error;
    }
    
    if (actualDegrees > maxActual) {
        maxActual = actualDegrees;
    }
    if (actualDegrees < minActual) {
        minActual = actualDegrees;
    }
}

void SineWaveTest::stopTest() {
    if (!testActive) return;
    
    testActive = false;
    
    // Print results
    Serial.println("\n=== SINE WAVE TEST RESULTS ===");
    Serial.print("Average error: ");
    Serial.print(sampleCount > 0 ? totalError / sampleCount : 0.0, 1);
    Serial.println("°");
    Serial.print("Max error: ");
    Serial.print(maxError, 1);
    Serial.println("°");
    Serial.print("Actual range: ");
    Serial.print(minActual, 1);
    Serial.print("° to ");
    Serial.print(maxActual, 1);
    Serial.println("°");
    Serial.print("Target range: ±");
    Serial.print(amplitude, 1);
    Serial.println("°");
    
    // Pass/Fail
    float avgError = sampleCount > 0 ? totalError / sampleCount : 0.0;
    bool passed = (avgError <= 5.0) && (maxError <= 10.0);
    
    Serial.print("\nResult: ");
    if (passed) {
        Serial.println("*** PASS ***");
    } else {
        Serial.println("*** FAIL ***");
        Serial.println("(Required: avg < 5°, max < 10°)");
    }
    Serial.println("============================\n");
}