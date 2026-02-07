/******************************************************************************
 * @file    km_pid.h
 * @brief   Interfaz pública de la librería.
 * @author  Adrian Navarredonda Arizaleta
 * @date    25-01-2026
 * @version 1.0
 *****************************************************************************/

#ifndef KM_PID_H
#define KM_PID_H

/******************************* INCLUDES *************************************/
// Includes necesarios para la API pública
#include <stdint.h>
#include "esp_log.h" // Para log
#include "esp_timer.h"

/******************************* DEFINES PÚBLICAS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería

/**
 * @brief Structure that reperesents a PID
 */
typedef struct {
    float kp;               /**< Proportional gain */
    float ki;               /**< Integral gain */
    float kd;               /**< Derivative gain */
    float integral;         /**< Integral accumulator */
    float lastError;        /**< Previous error for derivative */
    uint64_t lastTime;      /**< Last update time */

    // Limits
    float outputMin;        /**< */
    float outputMax;        /**< */
    float integralMin;      /**< Anti-windup limits */
    float integralMax;      /**< */
} PID_Controller;

/******************************* TIPOS PÚBLICOS ********************************/
// Estructuras, enums, typedefs públicos

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

/******************************* FUNCIONES PÚBLICAS ***************************/

// Initialize with gains
PID_Controller KM_PID_INIT(float kp, float ki, float kd);

// Calculate PID output
float KM_PID_Calculate(PID_Controller controller, float setpoint, float measurement);

// Set tuning parameters at runtime
void KM_PID_SetTunings(PID_Controller controller, float kp, float ki, float kd);

// Set output limits
void KM_PID_SetOutputLimits(PID_Controller controller, float min, float max);

// Set integral limits (anti-windup)
void KM_PID_SetIntegralLimits(PID_Controller controller, float min, float max);

// Reset controller state
void KM_PID_Reset(PID_Controller controller);

// Get current gains
void KM_PID_GetTunings(PID_Controller controller, float kp, float ki, float kd);

// Get integral value (for debugging)
float KM_PID_GetIntegral(PID_Controller controller); 
 
#endif // KM_PID_H
