#include "km_act.h"
#include "km_gpio.h"
#include <math.h>

static float clamp(float v, float min, float max)
{
    if (v > max) return max;
    if (v < min) return min;
    return v;
}

ACT_Controller KM_ACT_Init(ACT_Type type, float limit)
{
    ACT_Controller act = {0};

    act.type = type;

    switch (type)
    {
    case ACT_ACCEL:
        act.dacChannel = 0; // Canal A
        act.pwmChannel = 0;
        act.dirPin = 0;
        break;
    case ACT_BRAKE:
        act.dacChannel = 1; // Canal B
        act.pwmChannel = 0;
        act.dirPin = 0;
        break;
    case ACT_STEER:
        act.dacChannel = -1; // Sin DAC
        act.pwmChannel = PIN_STEER_PWM;
        act.dirPin = PIN_STEER_DIR;
        break;
    
    default:
        act.dacChannel = -1;
        act.pwmChannel = 0;
        act.dirPin = 0;
        break;
    }

    float limit_clamp = clamp(limit, 0.0f, 1.0f);
    act.outputLimit = (uint8_t)(limit_clamp * 100);

    act.lastOutput = 0.0f;

    return act;
}

/**
 * @brief Sets the output of a given actuator.
 *
 * This function accepts a normalized floating-point value and converts it
 * to the appropriate hardware signal depending on the actuator type:
 *
 *  - Accelerator / Brake (DAC): value is scaled to [0, 255]
 *  - Steering (PWM + direction pin):
 *        - magnitude is scaled to [0, 255] for PWM duty
 *        - sign determines the digital direction pin
 *
 * @note The input value is clamped to the actuator's outputLimit:
 *       value = clamp(value, -outputLimit, +outputLimit)
 *
 * @param act Pointer to the actuator controller structure.
 * @param value Desired output in normalized units:
 *              - Accelerator / Brake: [0.0 … 1.0] (negative values are clamped to 0)
 *              - Steering: [-1.0 … 1.0] (sign sets direction, magnitude sets PWM duty)
 *
 * @details
 *  - DAC outputs are always written as 8-bit values [0–255].
 *    For internal ESP32 DAC, this is native.
 *    For external MCP4922 (12-bit), km_gpio handles upscaling.
 *  - PWM outputs use an 8-bit duty cycle [0–255], matching the LEDC timer
 *    resolution. Any future PWM resolution change is handled in km_gpio.
 */
void KM_ACT_SetOutput(ACT_Controller *act, float value)
{
    if (!act) return;

    value = clamp(value, -act->outputLimit, act->outputLimit);
    act->lastOutput = value;

    switch (act->type)
    {
        case ACT_ACCEL:
        case ACT_BRAKE:
        {
            float v = clamp(value, 0.0f, act->outputLimit);
            // Depends on resolution of DAC
            KM_GPIO_WriteDAC(act->dacChannel, (uint8_t)(v * 255)); // salida [0-255]
            break;
        }

        case ACT_STEER:
        {
            float mag = clamp(fabsf(value), 0.0f, act->outputLimit);

            uint8_t dir = (value >= 0.0f) ? 1 : 0;

            KM_GPIO_WriteDigital(act->dirPin, dir);
            KM_GPIO_WritePWM(act->pwmChannel, (uint8_t)(mag * 255));
            break;
        }

        default:
            break;
    }
}

void KM_ACT_SetLimit(ACT_Controller *act, float limit)
{
    if (!act) return;

    float limit_clamp = clamp(limit, 0.0f, 1.0f);
    act->outputLimit = (uint8_t)(limit_clamp * 100);
}

void KM_ACT_Stop(ACT_Controller *act)
{
    if (!act) return;

    act->lastOutput = 0.0f;

    switch (act->type)
    {
        case ACT_ACCEL:
        case ACT_BRAKE:
            KM_GPIO_WriteDAC(act->dacChannel, 0);
            break;

        case ACT_STEER:
            KM_GPIO_WritePWM(act->pwmChannel, 0);
            break;

        default:
            break;
    }
}