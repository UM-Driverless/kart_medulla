/* Fake freertos/task.h for native unit tests */
#ifndef FAKE_FREERTOS_TASK_H
#define FAKE_FREERTOS_TASK_H

#include "FreeRTOS.h"

#define xTaskCreate(fn, name, stack, param, prio, handle) (void)0
#define uxTaskGetStackHighWaterMark(t) ((uint32_t)1024)

#endif
