/******************************************************************************
 * @file    km_act.h
 * @brief   Librería de control de actuadores (acelerador, freno y dirección)
 *
 * DISEÑO:
 * - Todo el hardware (pines, SPI, PWM, etc.) se define en km_gpio
 * - Esta librería SOLO gestiona lógica de control
 *
 * SOPORTE:
 * - ESP32  -> DAC interno para acelerador y freno
 * - ESP32-S3 -> DAC externo MCP4922 (SPI)
 * - Dirección -> PWM + pin de dirección
 *
 * USO:
 *   ACT_Controller accel = KM_ACT_Begin(ACT_ACCEL);
 *   ACT_Controller brake = KM_ACT_Begin(ACT_BRAKE);
 *   ACT_Controller steer = KM_ACT_Begin(ACT_STEER);
 *
 *   KM_ACT_SetOutput(&accel, 0.5f);   // 0–1
 *   KM_ACT_SetOutput(&steer, -1.0f);  // -1–1
 *
 * RANGOS:
 * - Acelerador/Freno: 0.0 → 1.0
 * - Dirección:       -1.0 → 1.0
 *
 * @author  Adrian Navarredonda Arizaleta
 * @date    01-02-2026
 *****************************************************************************/

#ifndef KM_ACT_H
#define KM_ACT_H

#include "km_gpio.h"
#include <stdint.h>
#include <stdbool.h>

/*============================== TIPOS =====================================*/

typedef enum {
    ACT_ACCEL = 0,   /**< Acelerador -> DAC canal A */
    ACT_BRAKE,       /**< Freno      -> DAC canal B */
    ACT_STEER        /**< Dirección  -> PWM + DIR   */
} ACT_Type;

typedef struct {
    ACT_Type type;
    float limit;

    /* PWM (solo dirección) */
    uint8_t pwmChannel;
    uint8_t dirPin;

    /* DAC (solo accel/freno) */
    uint8_t dacChannel;   // 0 = A, 1 = B

    float outputLimit;
    uint32_t lastOutput;
} ACT_Controller;

/*=========================== API PÚBLICA ==================================*/

/**
 * @brief Initializes an actuator controller for the given type.
 *
 * Configures the internal fields of an ACT_Controller (DAC channel, PWM channel,
 * direction pin) based on the actuator type and sets the output limit.
 *
 * @param type  The actuator type (ACT_ACCEL, ACT_BRAKE, or ACT_STEER).
 * @param limit Maximum output limit in normalized units [0.0, 1.0].
 *              Clamped internally to this range.
 * @return A fully initialized ACT_Controller struct with lastOutput = 0.
 *
 * @note The returned struct is stack-allocated; the caller owns the copy.
 * @note Hardware peripherals (DAC, PWM, GPIO) must be initialized separately
 *       via km_gpio before using the controller.
 */
ACT_Controller KM_ACT_Init(ACT_Type type, float limit);

/**
 * @brief Sets the output of an actuator.
 *
 * Converts a normalized floating-point value to the appropriate hardware signal:
 *  - Accelerator/Brake (DAC): value scaled to [0, 255].
 *  - Steering (PWM + direction pin): magnitude scaled to [0, 255], sign sets direction.
 *
 * @param act   Pointer to the actuator controller. Must not be NULL.
 * @param value Desired output in normalized units:
 *              - Accelerator/Brake: [0.0, 1.0] (negative values clamped to 0).
 *              - Steering: [-1.0, 1.0] (sign sets direction, magnitude sets duty).
 *
 * @note The value is clamped to [-outputLimit, +outputLimit] before conversion.
 * @note DAC resolution is 8-bit; for external MCP4922, km_gpio handles upscaling.
 */
void KM_ACT_SetOutput(ACT_Controller *act, float value);

/**
 * @brief Updates the maximum output limit of an actuator.
 *
 * @param act   Pointer to the actuator controller. Must not be NULL.
 * @param limit New output limit in normalized units [0.0, 1.0].
 *              Clamped internally to this range.
 */
void KM_ACT_SetLimit(ACT_Controller *act, float limit);

/**
 * @brief Stops the actuator by setting its output to zero.
 *
 * Writes 0 to the DAC (accelerator/brake) or PWM (steering) and
 * resets lastOutput to 0.
 *
 * @param act Pointer to the actuator controller. Must not be NULL.
 */
void KM_ACT_Stop(ACT_Controller *act);

#endif