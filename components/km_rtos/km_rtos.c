/**
 * @file km_rtos.c
 * @brief Implementation of the KM_RTOS task management library.
 *
 * This source file contains the implementation of the KM_RTOS library, which provides
 * a lightweight abstraction for managing FreeRTOS tasks in a structured and safe manner.
 *
 * The library handles:
 *  - Initialization and cleanup of the task management subsystem.
 *  - Creation, deletion, suspension, resumption, and restart of tasks.
 *  - Periodic task execution using a task wrapper to enforce timing.
 *  - Internal management of the static task array, ensuring consistency and safe access.
 *
 * Both public API functions and private helper functions are implemented in this file.
 *
 * Usage:
 *  - Call KM_RTOS_Init() before creating tasks.
 *  - Use the provided KM_RTOS_* functions to manipulate tasks safely.
 *
 * Author: Adrian Navarredonda Arizaleta
 * Date: 24-01-2026
 * Version: 1.0
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
static RTOS_Task tasks[RTOS_MAX_TASKS];

/******************************* DECLARACION FUNCIONES PRIVADAS ***************/
int8_t KM_RTOS_FindTask(TaskHandle_t handle);
static void KM_RTOS_TaskWrapper(void *params);

/******************************* FUNCIONES PÚBLICAS ***************************/

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

RTOS_Task KM_COMS_CreateTask(char *name, KM_RTOS_TaskFunction_t taskFn, void *context,
        uint32_t period_ms, uint16_t stackSize, UBaseType_t priority, uint8_t active){

    RTOS_Task task;

    task.name = name;
    task.taskFn = taskFn;
    task.context = context;
    task.period_ms = period_ms;
    task.stackSize = stackSize;
    task.priority = priority;
    task.active = active;

    return task;
}

esp_err_t KM_RTOS_AddTask(RTOS_Task task){

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
            return ESP_FAIL; // Error creating task
        }

        return ESP_OK; // Success
    }

    return ESP_FAIL; // Fail, no space in array
}

// Destruir una tarea existente, devuelve 0 en caso de error, 1 en caso correcto
esp_err_t KM_RTOS_DeleteTask(TaskHandle_t handle){
    // Search for task in array

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return ESP_FAIL;

    if (tasks[index].handle == handle) {
        vTaskDelete(handle);
        memset(&tasks[index], 0, sizeof(RTOS_Task));
        return ESP_OK;
    }

    // NO se ha encontrado la tarea
    return ESP_FAIL;
}

// Suspender una tarea, devuelve 0 en caso de error, 1 en caso correcto
esp_err_t KM_RTOS_SuspendTask(TaskHandle_t handle) {

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return ESP_FAIL;

    if (tasks[index].handle == handle && tasks[index].active) {
        vTaskSuspend(handle);
        tasks[index].active = 0;
        return ESP_OK;
    }

    // No se ha encontrado la tarea o ya estaba suspendida
    return ESP_FAIL;
}

// Reanudar una tarea, devuelve 0 en caso de error, 1 en caso correcto
esp_err_t KM_RTOS_ResumeTask(TaskHandle_t handle){

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return 0;

    if (tasks[index].handle == handle && !tasks[index].active) {
        vTaskResume(handle);
        tasks[index].active = 1;

        return ESP_OK;
    }

    // No se ha encontrado la tarea o ya estaba activa
    return ESP_FAIL;
}

// Reiniciar una tarea (destruir + volver a crear), devuelve 0 en caso de error, 1 en caso correcto
esp_err_t KM_RTOS_RestartTask(TaskHandle_t handle){

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return ESP_FAIL;

    RTOS_Task copy = tasks[index];

    // Destruir tarea
    if(KM_RTOS_DeleteTask(handle) == ESP_FAIL) return ESP_FAIL;

    // Crear tarea
    if(KM_RTOS_AddTask(copy) == ESP_FAIL) return ESP_FAIL;

    // Reiniciado correctamente
    return ESP_OK;
}

// Cambiar prioridad, devuelve 0 en caso de error, 1 en caso correcto
esp_err_t KM_RTOS_ChangePriority(TaskHandle_t handle, uint8_t newPriority){

    int8_t index = KM_RTOS_FindTask(handle);
    if (index == -1) return ESP_FAIL;

    vTaskPrioritySet(tasks[index].handle, newPriority);

    // Actualizar informacion sobre tarea
    tasks[index].priority = newPriority;

    return ESP_OK;
}

/******************************* FUNCIONES PRIVADAS ***************************/

/**
 * @brief Finds the index of a task in the internal task array by its FreeRTOS handle.
 *
 * This function searches the internal `tasks` array for a task that matches
 * the given FreeRTOS `TaskHandle_t`. It is used to identify tasks for monitoring,
 * activation/deactivation, or debugging purposes.
 *
 * @param handle The FreeRTOS task handle to search for.
 * @return The index (0 to RTOS_MAX_TASKS-1) of the task in the internal array if found.
 * @return -1 if the task handle was not found.
 *
 * @note The function iterates over a static array of size `RTOS_MAX_TASKS`.
 * @note Does not modify the task array or the tasks themselves.
 */
int8_t KM_RTOS_FindTask(TaskHandle_t handle) {

    for (int8_t i = 0; i < RTOS_MAX_TASKS; i++) {
        if (tasks[i].handle == handle) {
            return i;
        }
    }

    return -1;
}

/**
 * @brief FreeRTOS wrapper function for executing a periodic task.
 *
 * This function serves as a generic wrapper for all RTOS tasks created through
 * the `KM_RTOS_CreateTask` system. It manages periodic execution and ensures that
 * the user-defined task function is called at the configured interval.
 *
 * Steps performed by the function:
 * 1. Casts the `params` pointer to an `RTOS_Task` structure.
 * 2. Retrieves the current tick count to use as a reference for periodic delays.
 * 3. Continuously loops:
 *    - Checks if the task is active and has a valid function pointer.
 *    - Calls the task's logical function, passing the user-defined context.
 *    - Delays until the next scheduled execution using `vTaskDelayUntil` to maintain a fixed period.
 *
 * @param params Pointer to the `RTOS_Task` structure representing this task.
 *
 * @note This function is intended to be passed to `xTaskCreate` as the FreeRTOS task function.
 * @note Handles the periodic execution of tasks based on `task->period_ms`.
 * @note If `task` is NULL, the wrapper deletes itself immediately.
 * @note Task execution is controlled by the `active` flag in the `RTOS_Task` structure.
 */
static void KM_RTOS_TaskWrapper(void *params) {
    RTOS_Task *task = (RTOS_Task *)params;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    if (!task || task->taskFn) vTaskDelete(NULL);

    while (1) {
        if (task->active && task->taskFn != NULL) {
            task->taskFn(task->context);  // Call the logical function
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(task->period_ms));
    }
}

/******************************* FIN DE ARCHIVO ********************************/
 