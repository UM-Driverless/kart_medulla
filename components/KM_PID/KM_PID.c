/******************************************************************************
 * @file    KM_PID.c
 * @brief   Implementación de la librería.
 * @author Adrian Navarredonda Arizaleta
 *****************************************************************************/

#include "KM_PID.h"

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas

/******************************* MACROS PRIVADAS ******************************/
// Constantes internas, flags de debug
// #define LIBRERIA_DEBUG 1

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/

/******************************* FUNCIONES PÚBLICAS ***************************/
PID_Controller KM_PID_INIT(float kp_val, float ki_val, float kd_val) {

    PID_Controller controller;

    controller.kp = kp_val;
    controller.ki = ki_val;
    controller.kd = kd_val;

    controller.integral = 0.0f;
    controller.lastError = 0.0f;
    controller.lastTime = millis();

    return controller;                                                             
}

float KM_PID_Calculate(PID_Controller controller, float setpoint, float measurement) {
    unsigned long currentTime = millis();
    float dt = (currentTime - controller.lastTime) / 1000.0f;  // Convert to seconds

    // Avoid division by zero
    if (dt <= 0.0f) {
        dt = 0.001f;  // Minimum 1ms
    }

    // Calculate error
    float error = setpoint - measurement;

    // Proportional term
    float pTerm = controller.kp * error;

    // Integral term with anti-windup
    controller.integral += error * dt;

    if (controller.integral < controller.integralMin) {
        controller.integral = controller.integralMin;
    }

    if (controller.integral > controller.integralMax) {
        controller.integral = controller.integralMax;
    }

    float iTerm = controller.ki * controller.integral;

    // Derivative term
    float derivative = (error - controller.lastError) / dt;
    float dTerm = controller.kd * derivative;

    // Calculate total output
    float output = pTerm + iTerm + dTerm;

    // Apply output limits
    if (output < controller.outputMin) output = controller.outputMin;
    if (output > controller.outputMax) output = controller.outputMax;
    

    // Update state
    controller.lastError = error;
    controller.lastTime = currentTime;

    return output;
}

void KM_PID_SetTunings(PID_Controller controller, float kp, float ki, float kd) {
    controller.kp = kp;
    controller.ki = ki;
    controller.kd = kd;
}

void KM_PID_SetOutputLimits(PID_Controller controller, float min, float max) {
    controller.outputMin = min;
    controller.outputMax = max;
}

void KM_PID_SetIntegralLimits(PID_Controller controller, float min, float max) {
    controller.integralMin = min;
    controller.integralMax = max;
}

void KM_PID_Reset(PID_Controller controller){
    controller.integral = 0.0f;
    controller.lastError = 0.0f;
    controller.lastTime = millis();
}

void KM_PID_GetTunings(PID_Controller controller, float kp, float ki, float kd) {
    controller.kp = kp;
    controller.ki = ki;
    controller.kd = kd;
}

// Get integral value (for debugging)
float KM_PID_GetIntegral(PID_Controller controller){
    return controller.integral;
}
 
 /******************************* FUNCIONES PRIVADAS ***************************/
 
 /******************************* FIN DE ARCHIVO ********************************/
