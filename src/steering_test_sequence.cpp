#include "steering_test_sequence.h"

SteeringTestSequence::SteeringTestSequence() {
    testActive = false;
    currentStep = 0;
    stepDuration = 3000;  // 3 seconds per angle
    numSteps = 7;
    
    // Test sequence: 0, +10, -10, +20, -20, +30, -30 degrees
    testAngles[0] = 0.0;
    testAngles[1] = 10.0;
    testAngles[2] = -10.0;
    testAngles[3] = 20.0;
    testAngles[4] = -20.0;
    testAngles[5] = 30.0;
    testAngles[6] = -30.0;
}

void SteeringTestSequence::startSequenceTest() {
    Serial.println("\n=== STEERING TEST SEQUENCE ===");
    Serial.println("Testing angles: 0°, +10°, -10°, +20°, -20°, +30°, -30°");
    Serial.println("Duration: 3 seconds per angle");
    Serial.println("Starting test...\n");
    
    testActive = true;
    currentStep = 0;
    stepStartTime = millis();
    
    // Reset results
    for (int i = 0; i < numSteps; i++) {
        actualAngles[i] = 0.0;
        errors[i] = 0.0;
        stepComplete[i] = false;
    }
    
    Serial.print("Step 1/7: Target = ");
    Serial.print(testAngles[0]);
    Serial.println("°");
}

bool SteeringTestSequence::isActive() {
    return testActive;
}

float SteeringTestSequence::getCurrentTargetAngle() {
    if (!testActive || currentStep >= numSteps) {
        return 0.0;
    }
    // Return angle in radians
    return testAngles[currentStep] * PI / 180.0;
}

void SteeringTestSequence::update(float actualAngle) {
    if (!testActive) return;
    
    unsigned long currentTime = millis();
    float actualDegrees = actualAngle * 180.0 / PI;
    
    // Collect data for current step
    if (!stepComplete[currentStep]) {
        // Average the actual angle over the last second of each step
        if (currentTime - stepStartTime > stepDuration - 1000) {
            actualAngles[currentStep] = actualDegrees;
            errors[currentStep] = testAngles[currentStep] - actualDegrees;
            stepComplete[currentStep] = true;
            
            Serial.print("  Result: Target=");
            Serial.print(testAngles[currentStep], 1);
            Serial.print("°, Actual=");
            Serial.print(actualDegrees, 1);
            Serial.print("°, Error=");
            Serial.print(errors[currentStep], 1);
            Serial.println("°");
        }
    }
    
    // Move to next step
    if (currentTime - stepStartTime >= stepDuration) {
        currentStep++;
        
        if (currentStep >= numSteps) {
            // Test complete
            printResults();
            testActive = false;
        } else {
            // Start next step
            stepStartTime = currentTime;
            Serial.print("\nStep ");
            Serial.print(currentStep + 1);
            Serial.print("/7: Target = ");
            Serial.print(testAngles[currentStep]);
            Serial.println("°");
        }
    }
}

void SteeringTestSequence::printResults() {
    Serial.println("\n=== TEST SEQUENCE RESULTS ===");
    Serial.println("Target | Actual | Error");
    Serial.println("-------|--------|-------");
    
    float totalError = 0.0;
    float maxError = 0.0;
    
    for (int i = 0; i < numSteps; i++) {
        Serial.print(testAngles[i] >= 0 ? " " : "");
        Serial.print(testAngles[i], 1);
        Serial.print("° | ");
        Serial.print(actualAngles[i] >= 0 ? " " : "");
        Serial.print(actualAngles[i], 1);
        Serial.print("° | ");
        Serial.print(errors[i] >= 0 ? " " : "");
        Serial.print(errors[i], 1);
        Serial.println("°");
        
        totalError += abs(errors[i]);
        if (abs(errors[i]) > maxError) {
            maxError = abs(errors[i]);
        }
    }
    
    Serial.println("\nSummary:");
    Serial.print("Average error: ");
    Serial.print(totalError / numSteps, 1);
    Serial.println("°");
    Serial.print("Max error: ");
    Serial.print(maxError, 1);
    Serial.println("°");
    
    // Pass/Fail criteria
    bool passed = (totalError / numSteps <= 3.0) && (maxError <= 5.0);
    Serial.print("\nResult: ");
    if (passed) {
        Serial.println("*** PASS ***");
    } else {
        Serial.println("*** FAIL ***");
        Serial.println("(Required: avg < 3°, max < 5°)");
    }
    Serial.println("============================\n");
}