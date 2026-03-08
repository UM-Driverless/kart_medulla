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
#include <math.h>

/******************************* DEFINES PÚBLICAS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería
#define OBJECT_VALUE_ERROR NAN

/******************************* TIPOS PÚBLICOS ********************************/
// Estructuras, enums, typedefs públicos

typedef enum
{
TARGET_THROTTLE = 0,    // Target throttle from ORIN [0-1] (native float)
TARGET_BRAKING,         // Target braking from ORIN [0-1] (native float)
TARGET_STEERING,        // Target steering angle from ORIN in rad (native float)
MISION_ORIN,            // Mision that is executing the ORIN
MACHINE_STATE_ORIN,     // State inside the state machine of the ORIN
SHUTDOWN_ORIN,          // Status of the shutdown in the ORIN 0 or 1

ACTUAL_SPEED,           // Actual speed of the kart in m/s (native float)
ACTUAL_ACCERELATION,    // Actual value of the acceleration of the kart m/s^2 (native float)
ACTUAL_BRAKING,         // Actual value of the brake [0-1] (native float)
ACTUAL_STEERING,        // Actual value of the steering in rad (native float)
MISION_ESP,             // Mision that is executing the ESP
MACHINE_STATE_ESP,      // State inside the state machine of the ESP
ACTUAL_SHUTDOWN,         // Actual state of the SHUTDOWN

CALIBRATE_STEERING_CMD,  // Non-zero = pending center offset to save via NVS

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
uint8_t KM_OBJ_SetObjectValue(km_objects_t object, float value);


/**
 * @brief   Operación principal de la librería.
 * @param   valor Parámetro de entrada
 * @return  Resultado
 */
float KM_OBJ_GetObjectValue(km_objects_t object);

#endif /* KM_OBJECTS_H */
 