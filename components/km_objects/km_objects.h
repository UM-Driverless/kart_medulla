/******************************************************************************
 * @file    km_objects.h
 * @brief   Public interface for the KM_OBJECTS shared-state dictionary.
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
TARGET_THROTTLE = 0,    // Target throttle send by ORIN in [0-1].       x100
TARGET_BRAKING,         // Target braking send by ORIN in [0-1]         x100
TARGET_STEERING,        // Target steering angle send by ORIN in [-1 to 1].   x100
MISION_ORIN,            // Mision that is executing the ORIN
MACHINE_STATE_ORIN,     // State inside the state machine of the ORIN
SHUTDOWN_ORIN,          // Status of the shutdown in the ORIN 0 or 1

ACTUAL_SPEED,           // Actual speed of the kart in m/s x100
ACTUAL_ACCERELATION,    // Actual value of the acceleration of the kart m/s^2 x100 
ACTUAL_BRAKING,         // Actual value of the brake [0-1] interpreted as brake pedal or hydraulic pressure) x100
ACTUAL_STEERING,        // Actual value of the steering in rad x100
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
 * @brief   Sets the value of a shared object.
 *
 * @param   object  Object identifier (member of km_objects_t).
 * @param   value   64-bit signed value to store.
 * @return  1 on success, 0 if the object identifier is out of range.
 */
uint8_t KM_OBJ_SetObjectValue(km_objects_t object, int64_t value);

/**
 * @brief   Gets the current value of a shared object.
 *
 * @param   object  Object identifier (member of km_objects_t).
 * @return  The stored 64-bit value, or OBJECT_VALUE_ERROR if the identifier
 *          is out of range.
 */
int64_t KM_OBJ_GetObjectValue(km_objects_t object);

#endif /* KM_OBJECTS_H */
 