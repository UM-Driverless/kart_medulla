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
 * @brief Inicializa un actuador según su tipo
 */
ACT_Controller KM_ACT_Init(ACT_Type type, float limit);

/**
 * @brief Establece la salida del actuador
 *
 * Acelerador/Freno: 0.0 → 1.0  
 * Dirección:       -1.0 → 1.0
 */
void KM_ACT_SetOutput(ACT_Controller *act, float value);

/**
 * @brief Limita la salida máxima del actuador
 */
void KM_ACT_SetLimit(ACT_Controller *act, float limit);

/**
 * @brief Detiene el actuador (salida = 0)
 */
void KM_ACT_Stop(ACT_Controller *act);

#endif