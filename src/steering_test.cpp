#include "steering_test.h"

SteeringTest::SteeringTest() {
    testActive = false;
    settlingThreshold = 2.0;
    steadyStateThreshold = 2.0;
    maxSettlingTime = 1000;
}

void SteeringTest::startSteadyStateTest(float targetDegrees) {
    Serial.println("\n=== STEADY-STATE ACCURACY TEST ===");
    Serial.print("Target: ");
    Serial.print(targetDegrees);
    Serial.println(" degrees");
    Serial.print("Settling threshold: ");
    Serial.print(settlingThreshold);
    Serial.println(" degrees");
    Serial.print("Max settling time: ");
    Serial.print(maxSettlingTime);
    Serial.println(" ms");
    Serial.println("Test duration after settling: 3 seconds");
    Serial.println("Starting test...\n");
    
    testActive = true;
    testStartTime = millis();
    settlingStartTime = millis();
    targetAngle = targetDegrees * PI / 180.0;
    hasSettled = false;
    errorSum = 0.0;
    errorCount = 0;
    maxError = 0.0;
}

bool SteeringTest::isActive() {
    return testActive;
}

float SteeringTest::getTargetAngle() {
    return targetAngle;
}

void SteeringTest::update(float actualAngle) {
    if (!testActive) return;
    
    unsigned long currentTime = millis();
    float errorDegrees = abs((targetAngle - actualAngle) * 180.0 / PI);
    
    // Check if we've settled
    if (!hasSettled) {
        if (errorDegrees <= settlingThreshold) {
            hasSettled = true;
            unsigned long settlingTime = currentTime - settlingStartTime;
            Serial.print("SETTLED in ");
            Serial.print(settlingTime);
            Serial.println(" ms");
            Serial.println("Collecting steady-state data...");
            errorSum = 0.0;
            errorCount = 0;
            maxError = 0.0;
        } else if (currentTime - settlingStartTime > maxSettlingTime) {
            Serial.println("TEST FAILED: Did not settle within time limit");
            testActive = false;
            return;
        }
    }
    
    // If settled, collect error data for 3 seconds
    if (hasSettled) {
        errorSum += errorDegrees;
        errorCount++;
        if (errorDegrees > maxError) {
            maxError = errorDegrees;
        }
        
        // After 3 seconds of data collection, report results
        if (currentTime - settlingStartTime > maxSettlingTime + 3000) {
            float avgError = errorSum / errorCount;
            
            Serial.println("\n=== TEST RESULTS ===");
            Serial.print("Average error: ");
            Serial.print(avgError, 2);
            Serial.println(" degrees");
            Serial.print("Max error: ");
            Serial.print(maxError, 2);
            Serial.println(" degrees");
            
            bool passed = (avgError <= steadyStateThreshold) && (maxError <= steadyStateThreshold * 1.5);
            
            Serial.print("Result: ");
            if (passed) {
                Serial.println("*** PASS ***");
            } else {
                Serial.println("*** FAIL ***");
                Serial.print("(Required: avg < ");
                Serial.print(steadyStateThreshold);
                Serial.print("°, max < ");
                Serial.print(steadyStateThreshold * 1.5);
                Serial.println("°)");
            }
            Serial.println("==================\n");
            
            testActive = false;
        }
    }
}