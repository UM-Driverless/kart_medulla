/******************************************************************************
 * @file    KM_RTOS_H.h
 * @brief   Interfaz pública de la librería.
 * @author  Adrian Navarredonda Arizaleta
 * @date    24-01-2026
 * @version 1.0
 *****************************************************************************/

#ifndef KM_RTOS_H
#define KM_RTOS_H

/******************************* INCLUDES *************************************/
// Includes necesarios para la API pública
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/******************************* DEFINES PUBLICOS *****************************/
// Constantes, flags o configuraciones visibles desde fuera de la librería
#define RTOS_MAX_TASKS          10  // Número máximo de tareas que puede manejar la librería

/**
 * @brief Logical task function type (RTOS-agnostic)
 *
 * This function is executed periodically by the RTOS wrapper.
 * It must NOT contain an infinite loop or call vTaskDelay().
 *
 * @param context User-defined context pointer
 */
typedef void (*KM_RTOS_TaskFunction_t)(void *context);

/**
 * @brief Structure that reperesents a task in FreeRTOS
 */
typedef struct {
    TaskHandle_t handle;          /**< FreeRTOS task handle */
    const char *name;             /**< Task name */

    KM_RTOS_TaskFunction_t taskFn;     /**< Logical task function */
    void *context;                /**< User context pointer */

    uint32_t period_ms;            /**< Execution period in milliseconds */
    uint16_t stackSize;            /**< Stack size in words */
    UBaseType_t priority;          /**< Task priority */

    uint8_t active;                /**< Task active flag */
} RTOS_Task;

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

// extern int ejemplo_variable_publica;

/**
 * @brief Array that will contain all the task created
 */
extern RTOS_Task tasks[RTOS_MAX_TASKS];

/******************************* FUNCIONES PÚBLICAS ***************************/
// Inicializa la librería (colas, semáforos, estructura de tareas)
void KM_RTOS_Init(void);

// Destruye la librería, eliminando todas las tareas
void KM_RTOS_Destroy(void);

// Crear una nueva tarea
uint8_t KM_RTOS_CreateTask(RTOS_Task task);

// Destruir una tarea existente
uint8_t KM_RTOS_DeleteTask(TaskHandle_t handle);

// Suspender una tarea
uint8_t KM_RTOS_SuspendTask(TaskHandle_t handle);

// Reanudar una tarea
uint8_t KM_RTOS_ResumeTask(TaskHandle_t handle);

// Reiniciar una tarea (destruir + volver a crear)
uint8_t KM_RTOS_RestartTask(TaskHandle_t handle);

// Cambiar prioridad
uint8_t KM_RTOS_ChangePriority(TaskHandle_t handle, uint8_t newPriority);

#endif /* NOMBRE_LIBRERIA_H */
 