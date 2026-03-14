/* Fake FreeRTOS.h for native unit tests */
#ifndef FAKE_FREERTOS_H
#define FAKE_FREERTOS_H

#include <stdint.h>

typedef void * SemaphoreHandle_t;
typedef void * TaskHandle_t;
typedef int BaseType_t;

#define pdTRUE  1
#define pdFALSE 0
#define portTICK_PERIOD_MS 1
#define portMAX_DELAY 0xFFFFFFFF

#define pdMS_TO_TICKS(ms) ((uint32_t)(ms))

/* Mutex stubs — always succeed */
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, uint32_t t) { (void)s;(void)t; return pdTRUE; }
static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { (void)s; return pdTRUE; }

static inline uint32_t xTaskGetTickCount(void) { return 0; }
#define vTaskDelay(x) (void)(x)

#endif
