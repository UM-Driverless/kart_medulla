/******************************************************************************
 * @file    km_cont.h
 * @brief   Interfaz pública de la librería.
 * @author  Adrian Navarredonda Arizaleta
 * @date    6-2-2026
 * @version 1.0
*****************************************************************************/

#ifndef KM_ACT_H
#define KM_ACT_H

/******************************* INCLUDES *************************************/
// Includes necesarios para la API pública
#include <driver/dac_oneshot.h>
#include <stdint.h>
#include "esp_log.h" // Para log

/******************************* DEFINES PÚBLICAS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería


/******************************* TIPOS PÚBLICOS ********************************/
// Estructuras, enums, typedefs públicos
/**
 * @brief Structure that reperesents a controller
 */
typedef struct {
    uint8_t pwmPin;                 /**< Pin de la PWM a la que esta conectada el controlador*/
    uint8_t dirPin;                 /**< Pin para marcar la direccion del actuador*/
    uint8_t pwmChannel;             /**< Canal de la PWM que se usara */
    uint16_t pwmFreq;               /**< Frecuencia de PWM que se usara */
    uint8_t pwmResolution;          /**< Resolucion de la PWM*/
    float outputLimit;              /**< Limitacion de la salida del controlador*/
    uint8_t dacPin;                 /**< Pin donde esta el DAC a usar */
    dac_oneshot_handle_t dacHandle; /**< Manejador del DAC*/
    dac_channel_t dacChannel;       /**< Canal que usara el DAC*/
} ACT_Controller;

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

// extern int ejemplo_variable_publica;

/******************************* FUNCIONES PÚBLICAS ***************************/
/**
 * @brief   Inicializa el controlador del actuador, en caso de que este no tenga
 * una opcion para selecionar la direccion, dejar el campo dirPin a 0.
 * @param   Pin de la PWM a la que esta conectada el controlador
 * @param   Direccion en la que actua un actuador(en caso de no necesitarla poner 0)
 * @param   Canal de la PWM que se usara
 * @param   Frecuencia de PWM que se usara
 * @param   Resolucion de la PWM
 * @param   Limitacion de la salida del controlador
 * @return  Devuelve el struct correspondiente al actuador. 
 * En caso de error devuelve NULL
 */
ACT_Controller KM_ACT_Begin(uint8_t, uint8_t, uint8_t, uint16_t, uint8_t, float, uint8_t);

/**
 * @brief   Establece un valor para el controlador.
 * @param   Controlador del actuador que se quiere establecer output
 * @param   valor deseado que se actue [-1 - 1]
 */
void KM_ACT_SetOutput(ACT_Controller *act_controller, int8_t valor);

/**
 * @brief   Establece un valor limite para el controlador.
 * @param   Controlador del actuador que se quiere limitar
 * @param   valor deseado que se limite [-1 - 1]
 */
void KM_ACT_SetOutputLimit(ACT_Controller *act_controller, int8_t limite);

/**
 * @brief   Establece un valor limite para el controlador.
 * @param   Controlador del actuador que se quere parar
 */
void KM_ACT_Stop(ACT_Controller *act_controller);

#endif /* KM_ACT_H */