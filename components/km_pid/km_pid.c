/******************************************************************************
 * @file    KM_PID.c
 * @brief   Implementación de la librería.
 * @author Adrian Navarredonda Arizaleta
 *****************************************************************************/

#include "km_pid.h"

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas

/******************************* MACROS PRIVADAS ******************************/
// Constantes internas, flags de debug
// #define LIBRERIA_DEBUG 1

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/

/******************************* FUNCIONES PÚBLICAS ***************************/

/** @brief Initialize a PID controller. See km_pid.h for full documentation. */
PID_Controller KM_PID_Init(float kp_val, float ki_val, float kd_val) {

    PID_Controller controller;

    controller.kp = kp_val;
    controller.ki = ki_val;
    controller.kd = kd_val;

    controller.integral = 0.0f;
    controller.lastError = 0.0f;
    controller.lastMeasurement = 0.0f;
    controller.primed = false;
    controller.lastTime = esp_timer_get_time();

    return controller;
}

/** @brief Compute PID output for one cycle. See km_pid.h for full documentation. */
float KM_PID_Calculate(PID_Controller *controller, float setpoint, float measurement) {
    unsigned long currentTime = esp_timer_get_time();
    float dt = (currentTime - controller->lastTime) / 1000000.0f;  // Convert to seconds

    // Avoid division by zero
    if (dt <= 0.0f) {
        dt = 0.001f;  // Minimum 1ms
    }

    // Calculate error
    float error = setpoint - measurement;

    // Proportional term
    float pTerm = controller->kp * error;

    // Integral term with anti-windup
    controller->integral += error * dt;

    if (controller->integral < controller->integralMin) {
        controller->integral = controller->integralMin;
    }

    if (controller->integral > controller->integralMax) {
        controller->integral = controller->integralMax;
    }

    float iTerm = controller->ki * controller->integral;

    // Derivative term — ON THE MEASUREMENT, not on the error.
    //
    // error = setpoint - measurement, so d(error)/dt carries d(setpoint)/dt with
    // it. Differentiating the error therefore makes the output respond to the
    // COMMAND changing, not just to the plant moving, and on this vehicle that
    // is large: with kd = 0.10 and the control loop's dt = 2 ms, kd/dt = 50
    // against a kp of 1.20. The steering setpoint arrives from Orin quantized to
    // milliradians, so the smallest possible target change (1 mrad) alone would
    // contribute 0.05 — 10% of the 0.50 steering PWM limit — and a target sweep
    // at 60 deg/s would add a steady kd x rate = 0.105 in the direction of
    // travel, for as long as the kart is turning.
    //
    // -kd x d(measurement)/dt is identical while the setpoint is held and drops
    // both of those. The sign is negative because a rising measurement means the
    // plant is already moving up and the controller should ease off.
    //
    // No derivative on the first cycle after Init or Reset: there is no previous
    // measurement to difference against, and treating the 0 left by Reset as one
    // would differentiate the entire current angle in a single dt. At a steering
    // angle of 1.08 rad that alone is -kd x 1.08 / 0.002 = -54, saturating the
    // output the instant control resumes.
    float dTerm = 0.0f;
    if (controller->primed) {
        float dMeasurement = (measurement - controller->lastMeasurement) / dt;
        dTerm = -controller->kd * dMeasurement;
    }

    // Calculate total output
    float output = pTerm + iTerm + dTerm;

    // Apply output limits
    if (output < controller->outputMin) output = controller->outputMin;
    if (output > controller->outputMax) output = controller->outputMax;

    // Update state
    controller->lastError = error;
    controller->lastMeasurement = measurement;
    controller->primed = true;
    controller->lastTime = currentTime;

    return output;
}

/** @brief Update PID gains at runtime. */
void KM_PID_SetTunings(PID_Controller *controller, float kp, float ki, float kd) {
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
}

/** @brief Set output clamp limits. */
void KM_PID_SetOutputLimits(PID_Controller *controller, float min, float max) {
    controller->outputMin = min;
    controller->outputMax = max;
}

/** @brief Set integral accumulator clamp limits (anti-windup). */
void KM_PID_SetIntegralLimits(PID_Controller *controller, float min, float max) {
    controller->integralMin = min;
    controller->integralMax = max;
}

/** @brief Reset controller state (integral, error, timestamp). */
void KM_PID_Reset(PID_Controller *controller){
    controller->integral = 0.0f;
    controller->lastError = 0.0f;
    controller->lastMeasurement = 0.0f;
    /* Clearing this is what makes the next cycle skip the derivative. Leaving it
     * set would differentiate the new measurement against the 0 above. */
    controller->primed = false;
    controller->lastTime = esp_timer_get_time();
}

/** @brief Read the gains currently in force. See km_pid.h. */
void KM_PID_GetTunings(const PID_Controller *controller, float *kp, float *ki, float *kd) {
    if (controller == NULL) return;
    if (kp != NULL) *kp = controller->kp;
    if (ki != NULL) *ki = controller->ki;
    if (kd != NULL) *kd = controller->kd;
}

/** @brief Return the current integral accumulator (for debugging). */
float KM_PID_GetIntegral(PID_Controller *controller){
    return controller->integral;
}
 
 /******************************* FUNCIONES PRIVADAS ***************************/
 
 /******************************* FIN DE ARCHIVO ********************************/
