/******************************************************************************
 * @file    Km_rtos.h
 * @brief   Public API for the KM_RTOS task management library.
 *
 * This header provides the public interface for the KM_RTOS library, which 
 * offers a structured and safe way to manage FreeRTOS tasks. Through this API, 
 * users can create, delete, suspend, resume, restart tasks, and change task 
 * priorities without directly manipulating internal data structures.
 *
 * The library ensures:
 *  - Controlled access to a static internal task array.
 *  - Consistent task creation with default parameters if needed.
 *  - Safe periodic execution of tasks via the KM_RTOS_TaskWrapper.
 *  - Simplified FreeRTOS task management while maintaining thread safety.
 *
 * Usage:
 *  1. Call KM_RTOS_Init() before creating any tasks.
 *  2. Use KM_RTOS_CreateTask / KM_RTOS_AddTask to define and add tasks.
 *  3. Manage tasks using KM_RTOS_DeleteTask, KM_RTOS_SuspendTask, KM_RTOS_ResumeTask, 
 *     KM_RTOS_RestartTask, and KM_RTOS_ChangePriority.
 *
 * Author: Adrian Navarredonda Arizaleta
 * Date: 24-01-2026
 * Version: 1.0
 *****************************************************************************/

#ifndef KM_RTOS_H
#define KM_RTOS_H

/******************************* INCLUDES *************************************/
// Includes necesarios para la API pública
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include "esp_log.h" // Para log

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
    TaskHandle_t handle;            /**< FreeRTOS task handle */
    const char *name;               /**< Task name */

    KM_RTOS_TaskFunction_t taskFn;  /**< Logical task function */
    void *context;                  /**< User context pointer */

    uint32_t period_ms;             /**< Execution period in milliseconds */
    uint16_t stackSize;             /**< Stack size in words */
    UBaseType_t priority;           /**< Task priority */

    uint8_t active;                 /**< Task active flag */
} RTOS_Task;

/******************************* VARIABLES PÚBLICAS ***************************/
// Variables globales visibles (si realmente se necesitan)

// extern int ejemplo_variable_publica;

/******************************* FUNCIONES PÚBLICAS ***************************/
/**
 * @brief Initializes the RTOS task system.
 *
 * This function resets the internal `tasks` array by zeroing all entries,
 * effectively marking all task slots as free and inactive. It should be
 * called once at system startup before creating or managing any tasks.
 *
 * @note This function does not create any FreeRTOS tasks; it only prepares
 *       the internal task array for use.
 */
void KM_RTOS_Init(void);

/**
 * @brief Destroys all tasks managed by the RTOS task system and resets the task array.
 *
 * This function iterates through the internal `tasks` array and deletes all active
 * FreeRTOS tasks using `vTaskDelete`. After deleting tasks, it clears the `tasks`
 * array to mark all slots as free and inactive.
 *
 * The function wraps the entire operation in a critical section (`taskENTER_CRITICAL` /
 * `taskEXIT_CRITICAL`) to prevent the scheduler from switching tasks while the array
 * is being modified. This ensures thread-safety during the destruction process.
 *
 * @note All tasks created through `KM_RTOS_CreateTask` will be stopped.
 * @note After calling this function, the internal task array is fully reset and ready for re-initialization.
 */
void KM_RTOS_Destroy(void);

/**
 * @brief Creates and initializes an RTOS_Task structure.
 *
 * This function fills a `RTOS_Task` structure with the provided parameters,
 * preparing it to be added to the task scheduler or created with `KM_RTOS_CreateTask`.
 *
 * @param name       Name of the task (used by FreeRTOS for debugging/logging)
 * @param taskFn     Function pointer to the task implementation (of type KM_RTOS_TaskFunction_t)
 * @param context    Pointer to a user-defined context or data structure passed to the task
 * @param period_ms  Task period in milliseconds (used for periodic scheduling)
 * @param stackSize  Stack size for the FreeRTOS task
 * @param priority   FreeRTOS task priority
 * @param active     1 to mark the task as active, 0 to mark as inactive
 *
 * @return A fully-initialized `RTOS_Task` structure ready to be added to the scheduler.
 *
 * @note This function does not create the FreeRTOS task itself; it only prepares the
 *       task structure for use with `KM_RTOS_CreateTask`.
 */
RTOS_Task KM_COMS_CreateTask(char *name, KM_RTOS_TaskFunction_t taskFn, void *context,
        uint32_t period_ms, uint16_t stackSize, UBaseType_t priority, uint8_t active);

/**
 * @brief Creates a new FreeRTOS task and stores it in the internal task array.
 *
 * This function searches for a free slot in the internal `tasks` array and attempts
 * to create a new FreeRTOS task using the provided `RTOS_Task` configuration. It ensures
 * that the task does not already exist and fills in default values for priority, stack size,
 * and period if they are not specified.
 *
 * Steps performed by the function:
 * 1. Iterate through the `tasks` array to find an empty slot (`handle == NULL`).
 * 2. Assign the provided `task` to the slot and mark it as active.
 * 3. Fill default parameters if `priority`, `stackSize`, or `period_ms` are zero.
 * 4. Call `xTaskCreate` with `KM_RTOS_TaskWrapper` as the task function, passing
 *    the task structure pointer as the argument.
 * 5. On success, store the FreeRTOS task handle in the task structure.
 * 6. On failure, clear the slot and return 0.
 *
 * @param task The RTOS_Task structure containing task configuration
 *             (name, priority, stack size, period, etc.)
 * @return 1 if the task was successfully created
 * @return 0 if task creation failed or there was no free slot in the array
 *
 * @note This function manages tasks in a static array of size `RTOS_MAX_TASKS`.
 * @note Task execution is wrapped by `KM_RTOS_TaskWrapper`.
 * @note Default values are applied for any missing parameters:
 *       - priority: `RTOS_DEFAULT_PRIORITY`
 *       - stack size: `RTOS_DEFAULT_STACK_SIZE`
 *       - period_ms: `RTOS_DEFAULT_PERIOD_MS`
 */
esp_err_t KM_RTOS_AddTask(RTOS_Task task);

/**
 * @brief Deletes a specific FreeRTOS task and clears its slot in the internal task array.
 *
 * This function searches the internal `tasks` array for a task that matches the provided
 * FreeRTOS task handle. If found, the task is deleted using `vTaskDelete` and the corresponding
 * slot in the array is cleared with `memset`, marking it as free.
 *
 * @param handle The FreeRTOS task handle to delete.
 * @return ESP_OK if the task was successfully deleted.
 * @return ESP_FAIL if the task handle was not found in the internal array.
 *
 * @note After this function, the task slot is fully reset and can be reused for new tasks.
 * @note Thread-safd calling this while other tasks may concurrently access the `tasks` array.
 */
esp_err_t KM_RTOS_DeleteTask(TaskHandle_t handle);

/**
 * @brief Suspends a specific FreeRTOS task and marks it as inactive in the internal task array.
 *
 * This function searches the internal `tasks` array for the task corresponding to the
 * provided FreeRTOS handle. If found and the task is currently active, it calls `vTaskSuspend`
 * to pause the task's execution and sets its `active` flag to 0.
 *
 * @param handle The FreeRTOS task handle to suspend.
 * @return ESP_OK if the task was successfully suspended.
 * @return ESP_FAIL if the task handle was not found or the task was already suspended.
 *
 * @note Only affects the specified task; other tasks and array slots remain untouched.
 * @note Safe to call under normal operation, as it modifies only the targeted task entry.
 */
esp_err_t KM_RTOS_SuspendTask(TaskHandle_t handle);

/**
 * @brief Resumes a specific FreeRTOS task and marks it as active in the internal task array.
 *
 * This function searches the internal `tasks` array for the task corresponding to the
 * provided FreeRTOS handle. If found and the task is currently suspended, it calls `vTaskResume`
 * to restart the task's execution and sets its `active` flag to 1.
 *
 * @param handle The FreeRTOS task handle to resume.
 * @return ESP_OK if the task was successfully resumed.
 * @return ESP_FAIL if the task handle was not found or the task was already active.
 *
 * @note Only affects the specified task; other tasks and array slots remain untouched.
 * @note Safe to call under normal operation, as it modifies only the targeted task entry.
 */
esp_err_t KM_RTOS_ResumeTask(TaskHandle_t handle);

/**
 * @brief Restarts a specific FreeRTOS task by deleting and recreating it.
 *
 * This function searches the internal `tasks` array for the task matching the provided
 * handle. If found, it performs the following steps:
 * 1. Makes a copy of the task structure to preserve its configuration.
 * 2. Deletes the task using `KM_RTOS_DeleteTask`.
 * 3. Re-adds the task using `KM_RTOS_AddTask` with the preserved configuration.
 *
 * @param handle The FreeRTOS task handle to restart.
 * @return ESP_OK if the task was successfully restarted.
 * @return ESP_FAIL if the task handle was not found, deletion failed, or creation failed.
 *
 * @note This operation temporarily stops the task and replaces it with a new instance.
 */
esp_err_t KM_RTOS_RestartTask(TaskHandle_t handle);

/**
 * @brief Changes the priority of a specific FreeRTOS task and updates the internal task array.
 *
 * This function searches the internal `tasks` array for the task corresponding to the
 * provided handle. If found, it calls `vTaskPrioritySet` to update the task's FreeRTOS
 * priority and updates the `priority` field in the internal `tasks` array.
 *
 * @param handle      The FreeRTOS task handle whose priority should be changed.
 * @param newPriority The new priority value to assign to the task.
 * @return ESP_OK if the priority was successfully updated.
 * @return ESP_FAIL if the task handle was not found in the internal array.
 *
 * @note Only affects the specified task; other tasks remain unchanged.
 * @note Safe under normal operation, as it modifies only the targeted task entry.
 * @note The change is immediate from the scheduler's perspective.
 */
esp_err_t KM_RTOS_ChangePriority(TaskHandle_t handle, uint8_t newPriority);

#endif /* NOMBRE_LIBRERIA_H */
 