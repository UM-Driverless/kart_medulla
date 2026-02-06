/******************************************************************************
 * @file    km_act.c
 * @brief   Implementación de la librería.
 *****************************************************************************/

#include "km_act.h"
#include <stdio.h>   // solo si es necesario para debug interno

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas

/******************************* MACROS PRIVADAS ********************************/
// Constantes internas, flags de debug
// #define LIBRERIA_DEBUG 1

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/


/******************************* FUNCIONES PÚBLICAS ***************************/

ACT_Controller KM_ACT_Begin(uint8_t pwmPin, uint8_t dirPin, uint8_t pwmChannel,
                            uint16_t pwmFreq, uint8_t pwmResolution, float outputLimit, 
                            uint8_t dacPin){

    ACT_Controller act_controller;
    int dacChannel;

    // In case of bad input, put default values
    if (outputLimit < -1 || outputLimit > 1) outputLimit = 1;
    if (dacPin == 25) {
        dacChannel = DAC_CHAN_0;  // GPIO 25
    } else if (dacPin == 26) {
        dacChannel = DAC_CHAN_1;  // GPIO 26
    } else {
        Serial.printf("ERROR: Invalid DAC pin %d (must be 25 or 26)\n", dacPin);
        dacChannel = DAC_CHAN_0;  // Default to GPIO 25
    }
    
    act_controller.pwmPin = pwmPin;
    act_controller.dirPin = dirPin;
    act_controller.pwmChannel = pwmChannel;
    act_controller.pwmFreq = pwmFreq;
    act_controller.pwmResolution = pwmResolution;
    act_controller.outputLimit = outputLimit;
    act_controller.dacChannel = dacChannel;
    act_controller.dacPin = dacPin

    // Configure DAC using new oneshot API
    dac_oneshot_config_t dac_cfg = {
        .chan_id = dacChannel
    };

    esp_err_t err = dac_oneshot_new_channel(&dac_cfg, &act_controller.dacHandle);
    if (err != ESP_OK) {
        Serial.printf("ERROR: Failed to initialize DAC channel (error: 0x%x)\n", err);
        return;
    }

    // Set initial voltage to 0
    dac_oneshot_output_voltage(act_controller.dacHandle, 0);

    Serial.printf("Controller initialised: DAC Channel=%d (GPIO %d)\n",
                    dacChannel, dacChannel == DAC_CHAN_0 ? 25 : 26);

    return act_controller;
}

void KM_ACT_SetOutput(ACT_Controller *act_controller, int8_t valor){

    uint8_t pwmValue = 0;

    // Clamp to limits
    valor = constrain(valor, -act_controller->outputLimit, act_controller->outputLimit);

    // Determine direction
    bool direction = (valor >= 0);
    if (act_controller->dirPin != 0)
        digitalWrite(act_controller->dirPin, direction ? HIGH : LOW);

    // Calculate PWM value (0-255 for 8-bit resolution)
    pwmValue = (uint8_t)(abs(valor) * ((1 << act_controller->pwmResolution) - 1));
    pwmValue = constrain(pwmValue, 0, (1 << act_controller->pwmResolution) - 1);

    // Set PWM (new API uses pin number instead of channel)
    ledcWrite(act_controller->pwmPin, pwmValue);

}

void KM_ACT_SetOutputLimit(ACT_Controller *act_controller, int8_t limite){
    act_controller->outputLimit = constrain(limit, 0.0f, 1.0f);
    Serial.printf("Brake motor output limit set to: %.2f\n", act_controller->outputLimit);
}

void KM_ACT_Stop(ACT_Controller *act_controller){
    if (act_controller != NULL && act_controller->dacHandle != NULL) {
        dac_oneshot_output_voltage(act_controller->dacChannel, 0);
    }
}
/******************************* FUNCIONES PRIVADAS ***************************/
/**
 * @brief   Función interna no visible desde fuera
 */
// static void funcion_privada(void);

/******************************* FIN DE ARCHIVO ********************************/
