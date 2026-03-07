#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>

#include "km_act.h"
#include "km_coms.h"
#include "km_gamc.h"
#include "km_pid.h"
#include "km_rtos.h"
#include "km_sdir.h"
#include "km_sta.h"
#include "km_gpio.h"
#include "km_objects.h"

static const char *TAG = "TEST_C";

// Motor stays off until Orin sends first steering target
volatile bool g_steering_target_received = false;

#define MAX_ERROR_COUNT_SDIR 10

typedef struct {
    sensor_struct *sdir;
    ACT_Controller *dir_act;
    ACT_Controller *throttle_act;
    ACT_Controller *brake_act;
    PID_Controller *dir_pid;
} control_context_t;

// 20 Hz — UART RX + parse
void comms_task(void *ctx) {
    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();
}

// 10 Hz — full control loop with feedback
void control_task(void *ctx) {
    control_context_t *c = (control_context_t *)ctx;

    // Send feedback FIRST
    float actual_rad = (float)KM_OBJ_GetObjectValue(ACTUAL_STEERING) / 1000.0f;
    int16_t actual_i16 = (int16_t)(actual_rad * 1000.0f);
    uint8_t fb[2] = {(uint8_t)(actual_i16 >> 8), (uint8_t)(actual_i16 & 0xFF)};
    KM_COMS_SendMsg(ESP_ACT_STEERING, fb, 2);

    float target_rad = (float)KM_OBJ_GetObjectValue(TARGET_STEERING) / 1000.0f;

    // Throttle + brake
    float thr = (float)KM_OBJ_GetObjectValue(TARGET_THROTTLE) / 255.0f;
    float brk = (float)KM_OBJ_GetObjectValue(TARGET_BRAKING) / 255.0f;
    KM_ACT_SetOutput(c->throttle_act, thr);
    KM_ACT_SetOutput(c->brake_act, brk);

    // Read sensor
    float new_rad = KM_SDIR_ReadAngleRadians(c->sdir);
    KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(new_rad * 1000));

    // PID only after first target from Orin
    if (g_steering_target_received) {
        float pid_out = KM_PID_Calculate(c->dir_pid, target_rad, new_rad);
        KM_ACT_SetOutput(c->dir_act, pid_out);
    }
}

// 1 Hz — heartbeat
void heartbeat_task(void *ctx) {
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    KM_COMS_SendMsg(ESP_HEARTBEAT, payload, sizeof(payload));
}

void system_init(void) {
    if (KM_GPIO_Init() != ESP_OK)
        ESP_LOGE(TAG, "GPIO init error");

    KM_RTOS_Init();

    if (KM_COMS_Init(UART_NUM_0) != ESP_OK)
        ESP_LOGE(TAG, "COMS init error");

    sensor_struct sdir = KM_SDIR_Init(MAX_ERROR_COUNT_SDIR);
    KM_SDIR_Begin(&sdir, GPIO_NUM_21, GPIO_NUM_22);
    KM_SDIR_setCenterOffset(&sdir, 0);

    float init_rad = KM_SDIR_ReadAngleRadians(&sdir);
    if (sdir.errorCount == 0) {
        KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(init_rad * 1000));
        KM_OBJ_SetObjectValue(TARGET_STEERING, (int64_t)(init_rad * 1000));
    }

    ACT_Controller dir_act = KM_ACT_Init(ACT_STEER, 0.08);
    ACT_Controller throttle_act = KM_ACT_Init(ACT_ACCEL, 1.0);
    ACT_Controller brake_act = KM_ACT_Init(ACT_BRAKE, 1.0);
    KM_ACT_SetLimit(&dir_act, 0.08);
    KM_ACT_SetLimit(&throttle_act, 1.0);
    KM_ACT_SetLimit(&brake_act, 1.0);
    KM_ACT_Stop(&dir_act);

    PID_Controller dir_pid = KM_PID_Init(2.0f, 0.0f, 0.005f);
    KM_PID_SetOutputLimits(&dir_pid, -1.0f, 1.0f);
    KM_PID_SetIntegralLimits(&dir_pid, -10.0f, 10.0f);

    static control_context_t ctrl_ctx;
    static sensor_struct sdir_static;
    static ACT_Controller dir_act_static, throttle_act_static, brake_act_static;
    static PID_Controller dir_pid_static;

    sdir_static = sdir;
    dir_act_static = dir_act;
    throttle_act_static = throttle_act;
    brake_act_static = brake_act;
    dir_pid_static = dir_pid;

    ctrl_ctx.sdir = &sdir_static;
    ctrl_ctx.dir_act = &dir_act_static;
    ctrl_ctx.throttle_act = &throttle_act_static;
    ctrl_ctx.brake_act = &brake_act_static;
    ctrl_ctx.dir_pid = &dir_pid_static;

    // All three tasks
    RTOS_Task t1 = KM_COMS_CreateTask("coms", comms_task, NULL, 50, 4096, 5, 1);
    RTOS_Task t0 = KM_COMS_CreateTask("ctrl", control_task, &ctrl_ctx, 100, 4096, 5, 1);
    RTOS_Task t2 = KM_COMS_CreateTask("hb", heartbeat_task, NULL, 1000, 2048, 3, 1);
    KM_RTOS_AddTask(t1);
    KM_RTOS_AddTask(t0);
    KM_RTOS_AddTask(t2);
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);
    system_init();
}
