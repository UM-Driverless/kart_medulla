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
#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h" // Para log
#include "esp_timer.h"

/******************************* DEFINES PÚBLICAS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería

/**
 * @brief Structure that represents a PID controller.
 *
 * @note DERIVATIVE ON MEASUREMENT. The D term differentiates the measurement,
 *       not the error — see KM_PID_Calculate() for the reasoning. `lastError`
 *       is therefore no longer what the derivative is computed from; it is kept
 *       because it is part of the struct's published shape and is useful when
 *       inspecting controller state.
 */
typedef struct {
    float kp;               /**< Proportional gain */
    float ki;               /**< Integral gain */
    float kd;               /**< Derivative gain */
    float integral;         /**< Integral accumulator */
    float lastError;        /**< Previous error (state inspection; not used by the D term) */
    float lastMeasurement;  /**< Previous measurement — this is what the D term differentiates */
    bool  primed;           /**< False until one sample has been seen since Init/Reset */
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
 * @note   The D term is -kd x d(measurement)/dt, NOT kd x d(error)/dt. The two
 *         are identical while the setpoint is constant; they differ whenever
 *         the setpoint moves, and differentiating the error there produces an
 *         output spike caused purely by the command changing. See the function
 *         body for the arithmetic on this vehicle.
 * @note   The first call after Init or Reset contributes NO derivative, because
 *         there is no previous measurement to difference against. Without that,
 *         resuming control would differentiate against a stale or zero sample
 *         and produce a large spurious kick.
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
 * @brief  Reset the controller state (integral, last error and measurement,
 *         primed flag, timestamp).
 * @param  controller  Pointer to the PID controller state.
 * @note   Clears `primed`, so the next KM_PID_Calculate() contributes no
 *         derivative. That is deliberate — see that function's notes.
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
