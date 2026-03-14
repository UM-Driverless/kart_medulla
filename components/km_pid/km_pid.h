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
 * @brief Structure that represents a PID controller.
 */
typedef struct {
    float kp;               /**< Proportional gain */
    float ki;               /**< Integral gain */
    float kd;               /**< Derivative gain */
    float integral;         /**< Integral accumulator */
    float lastError;        /**< Previous error for derivative */
    uint64_t lastTime;      /**< Last update timestamp (microseconds) */

    // Limits
    float outputMin;        /**< Minimum output clamp */
    float outputMax;        /**< Maximum output clamp */
    float integralMin;      /**< Anti-windup minimum for integral accumulator */
    float integralMax;      /**< Anti-windup maximum for integral accumulator */
} PID_Controller;

/******************************* TIPOS PÚBLICOS ********************************/
// Estructuras, enums, typedefs públicos

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

/******************************* FUNCIONES PÚBLICAS ***************************/

/**
 * @brief  Create and initialize a PID controller with the given gains.
 * @param  kp  Proportional gain.
 * @param  ki  Integral gain.
 * @param  kd  Derivative gain.
 * @return Initialized PID_Controller struct with zeroed state.
 * @note   Output limits and integral limits are left at 0 after init.
 *         Call KM_PID_SetOutputLimits() and KM_PID_SetIntegralLimits()
 *         before using the controller.
 */
PID_Controller KM_PID_Init(float kp, float ki, float kd);

/**
 * @brief  Compute the PID output for one control cycle.
 * @param  controller   Pointer to the PID controller state.
 * @param  setpoint     Desired target value.
 * @param  measurement  Current measured value.
 * @return Clamped PID output (within outputMin..outputMax).
 * @note   Uses esp_timer_get_time() for dt calculation. The first call
 *         after Init or Reset uses the elapsed time since that call.
 */
float KM_PID_Calculate(PID_Controller *controller, float setpoint, float measurement);

/**
 * @brief  Update PID gains at runtime.
 * @param  controller  Pointer to the PID controller state.
 * @param  kp          New proportional gain.
 * @param  ki          New integral gain.
 * @param  kd          New derivative gain.
 */
void KM_PID_SetTunings(PID_Controller *controller, float kp, float ki, float kd);

/**
 * @brief  Set the output clamp limits.
 * @param  controller  Pointer to the PID controller state.
 * @param  min         Minimum allowed output value.
 * @param  max         Maximum allowed output value.
 */
void KM_PID_SetOutputLimits(PID_Controller *controller, float min, float max);

/**
 * @brief  Set the integral accumulator clamp limits (anti-windup).
 * @param  controller  Pointer to the PID controller state.
 * @param  min         Minimum integral accumulator value.
 * @param  max         Maximum integral accumulator value.
 */
void KM_PID_SetIntegralLimits(PID_Controller *controller, float min, float max);

/**
 * @brief  Reset the controller state (integral, last error, timestamp).
 * @param  controller  Pointer to the PID controller state.
 */
void KM_PID_Reset(PID_Controller *controller);

/**
 * @brief  Overwrite the controller gains from external values.
 * @param  controller  Pointer to the PID controller state.
 * @param  kp          Proportional gain to set.
 * @param  ki          Integral gain to set.
 * @param  kd          Derivative gain to set.
 * @note   This function is identical to KM_PID_SetTunings().
 */
void KM_PID_GetTunings(PID_Controller *controller, float kp, float ki, float kd);

/**
 * @brief  Return the current integral accumulator value.
 * @param  controller  Pointer to the PID controller state.
 * @return Current integral accumulator value.
 * @note   Useful for debugging anti-windup behaviour.
 */
float KM_PID_GetIntegral(PID_Controller *controller);
 
#endif // KM_PID_H
