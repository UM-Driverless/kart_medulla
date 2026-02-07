/**
 * @file KM_RTOS.c
 * @brief Implementation of a lib
 *
 * Este archivo contiene la implementación de las funciones para:
 * - Inicializar la librería
 * - Crear, destruir, suspender y reiniciar tareas
 * - Gestionar internamente el array de tareas
 *
 * Contiene tanto funciones públicas (API) como funciones privadas (helpers).
 *
 * @author Adrian Navarredonda Arizaleta
 * @date 24-01-2026
 */
 
/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas
#include "km_rtos.h"

#include <string.h>

/******************************* DEFINES PRIVADAS *****************************/
// Constantes internas
#define RTOS_DEFAULT_STACK_SIZE 1024// Default size stack
#define RTOS_DEFAULT_PRIORITY   1   // Default priority for each task
#define RTOS_DEFAULT_PERIOD_MS 50   // Default time for executing each task

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)
RTOS_Task tasks[RTOS_MAX_TASKS];

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/
int8_t KM_RTOS_FindTask(TaskHandle_t handle);
static void KM_RTOS_TaskWrapper(void *params);

/******************************* FUNCIONES PÚBLICAS ***************************/

// Inicia las estructuras de la liberia
void KM_RTOS_Init(void){
    memset(tasks, 0, sizeof(tasks));
}

// Destruye todas las tareas
void KM_RTOS_Destroy(void){
    for (int i = 0; i < RTOS_MAX_TASKS; i++) {
        if (tasks[i].name != NULL) {
            vTaskDelete(tasks[i].handle);
        }
    }

    memset(tasks, 0, sizeof(tasks));
}

// Crear una nueva tarea,  devuelve 0 en caso de error, 1 en caso correcto
uint8_t KM_RTOS_CreateTask(RTOS_Task task){

    // Buscar hueco libre en array y comprobar que no exista ya esa tarea
    for (uint8_t i = 0; i < RTOS_MAX_TASKS; i++) {
        // Hueco ocupado
        if (tasks[i].handle != NULL) continue;
        
        tasks[i] = task;
        tasks[i].active = 1;

        if (tasks[i].priority == 0) tasks[i].priority = RTOS_DEFAULT_PRIORITY;
        if (tasks[i].stackSize == 0) tasks[i].stackSize = RTOS_DEFAULT_STACK_SIZE;
        if (tasks[i].period_ms == 0) tasks[i].period_ms = RTOS_DEFAULT_PERIOD_MS;

        BaseType_t result = xTaskCreate(
            KM_RTOS_TaskWrapper,  // Wrapper
            task.name,
            task.stackSize,
            &tasks[i],            // Pointer to this task
            task.priority,
            &tasks[i].handle
        );

        if (result != pdPASS) {
            memset(&tasks[i], 0, sizeof(RTOS_Task));
            return 0; // Error creating task
        }

        return 1; // Success
    }

    return 0; // Fail, no space in array
}

// Destruir una tarea existente, devuelve 0 en caso de error, 1 en caso correcto
uint8_t KM_RTOS_DeleteTask(TaskHandle_t handle){
    // Search for task in array

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return 0;

    if (tasks[index].handle == handle) {
        vTaskDelete(handle);
        memset(&tasks[index], 0, sizeof(RTOS_Task));
        return 1;
    }

    // NO se ha encontrado la tarea
    return 0;
}

// Suspender una tarea, devuelve 0 en caso de error, 1 en caso correcto
uint8_t KM_RTOS_SuspendTask(TaskHandle_t handle) {

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return 0;

    if (tasks[index].handle == handle && tasks[index].active) {
        vTaskSuspend(handle);
        tasks[index].active = 0;
        return 1;
    }

    // No se ha encontrado la tarea o ya estaba suspendida
    return 0;
}

// Reanudar una tarea, devuelve 0 en caso de error, 1 en caso correcto
uint8_t KM_RTOS_ResumeTask(TaskHandle_t handle){

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return 0;

    if (tasks[index].handle == handle && !tasks[index].active) {
        vTaskResume(handle);
        tasks[index].active = 1;

        return 1;
    }

    // No se ha encontrado la tarea o ya estaba activa
    return 0;

}

// Reiniciar una tarea (destruir + volver a crear), devuelve 0 en caso de error, 1 en caso correcto
uint8_t KM_RTOS_RestartTask(TaskHandle_t handle){

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return 0;

    RTOS_Task copy = tasks[index];

    // Destruir tarea
    if(!KM_RTOS_DeleteTask(handle)) return 0;

    // Crear tarea
    if(!KM_RTOS_CreateTask(copy)) return 0;

    // Reiniciado correctamente
    return 1;
}

// Cambiar prioridad, devuelve 0 en caso de error, 1 en caso correcto
uint8_t KM_RTOS_ChangePriority(TaskHandle_t handle, uint8_t newPriority){

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return 0;

    vTaskPrioritySet(tasks[index].handle, newPriority);

    // Actualizar informacion sobre tarea
    tasks[index].priority = newPriority;

    return 1;

}

/******************************* FUNCIONES PRIVADAS ***************************/

// Funcion para buscar una tarea en funcion de su handle, si no se encuentra devuelve -1
int8_t KM_RTOS_FindTask(TaskHandle_t handle) {

    for (int8_t i = 0; i < RTOS_MAX_TASKS; i++) {
        if (tasks[i].handle == handle) {
            return i;
        }
    }

    return -1;
}

/**
 * @brief RTOS wrapper that calls the logical task function periodically
 *
 * @param params Pointer to RTOS_Task
 */
static void KM_RTOS_TaskWrapper(void *params) {
    RTOS_Task *task = (RTOS_Task *)params;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    if (!task) vTaskDelete(NULL);

    while (1) {
        if (task->active && task->taskFn != NULL) {
            task->taskFn(task->context);  // Call the logical function
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(task->period_ms));
    }
}

/******************************* FIN DE ARCHIVO ********************************/
 