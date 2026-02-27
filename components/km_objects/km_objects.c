/******************************************************************************
 * @file    km_objects.c
 * @brief   Implementación de la librería.
 *****************************************************************************/

#include "km_objects.h"
#include <stdio.h>   // solo si es necesario para debug interno

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas

/******************************* MACROS PRIVADAS ********************************/
// Constantes internas, flags de debug

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)
static int64_t km_objects_values[KM_OBJ_LAST];

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/


/******************************* FUNCIONES PÚBLICAS ***************************/
/**
 * @brief   Implementación de la función pública declarada en el header
 */
uint8_t KM_OBJ_SetObjectValue(km_objects_t object, int64_t value){
    if (object >= KM_OBJ_LAST) return 0; // error: objeto inválido
    km_objects_values[object] = value;
    return 1; // éxito
}

int64_t KM_OBJ_GetObjectValue(km_objects_t object){
    if (object >= KM_OBJ_LAST) return OBJECT_VALUE_ERROR; // Valor de error
    return km_objects_values[object];
}

/******************************* FUNCIONES PRIVADAS ***************************/
/**
 * @brief   Función interna no visible desde fuera
 */
// static void funcion_privada(void);

/******************************* FIN DE ARCHIVO ********************************/
 