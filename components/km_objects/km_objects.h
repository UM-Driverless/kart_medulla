/******************************************************************************
 * @file    km_objects.h
 * @brief   Interfaz pública de la librería.
 * @author  Adrian Navarredonda Arizaleta
 * @date    27-02-2026
 * @version 1.0
 *****************************************************************************/

#ifndef KM_OBJECTS_H
#define KM_OBJECTS_H

/******************************* INCLUDES *************************************/
// Includes necesarios para la API pública
#include <stdint.h>

/******************************* DEFINES PÚBLICAS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería
#define OBJECT_VALUE_ERROR -9223372036854775808

/******************************* TIPOS PÚBLICOS ********************************/
// Estructuras, enums, typedefs públicos

typedef enum
{
TARGET_THROTTLE = 0,    // Target throttle send by ORIN
TARGET_BRAKING,         // Target braking send by ORIN
TARGET_STEERING,        // Target steering angle send by ORIN
MISION_ORIN,            // Mision that is executing the ORIN
MACHINE_STATE_ORIN,     // State inside the state machine of the ORIN
SHUTDOWN_ORIN,          // Status of the shutdown in the ORIN
ACTUAL_SPEED,           // Actual speed of the kart
ACTUAL_THROTTLE,        // Actual value of the throttle
ACTUAL_BRAKING,         // Actual value of the brake
ACTUAL_STEERING,        // Actual value of the steering
MISION_ESP,             // Mision that is executing the ESP
MACHINE_STATE_ESP,      // State inside the state machine of the ESP
ACTUAL_SHUTDOWN,         // Actual state of the SHUTDOWN


KM_OBJ_LAST             // Este debe de ser siempre el ultimo
} km_objects_t;

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

// extern int ejemplo_variable_publica;

/******************************* FUNCIONES PÚBLICAS ***************************/
/**
 * @brief   Establece el valor de un objeto.
 * @param ID del objeto a estrablecer
 * @return  0 si tuvo éxito, otro valor si hubo error
 */
uint8_t KM_OBJ_SetObjectValue(km_objects_t object, int64_t value);


/**
 * @brief   Operación principal de la librería.
 * @param   valor Parámetro de entrada
 * @return  Resultado
 */
int64_t KM_OBJ_GetObjectValue(km_objects_t object);

#endif /* KM_OBJECTS_H */
 