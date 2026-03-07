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

static const char *TAG = "TEST_B";

volatile bool g_steering_target_received = false;
static bool g_logged_first_target = false;

#define MAX_ERROR_COUNT_SDIR 10

typedef struct {
    sensor_struct *sdir;
    ACT_Controller *dir_act;
    PID_Controller *dir_pid;
} control_context_t;

// 20 Hz — UART RX + parse
void comms_task(void *ctx) {
    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();
}

// 10 Hz — read sensor, run PID only after first target received
void control_task(void *ctx) {
    control_context_t *c = (control_context_t *)ctx;

    float target_rad = (float)KM_OBJ_GetObjectValue(TARGET_STEERING) / 1000.0f;
    float new_rad = KM_SDIR_ReadAngleRadians(c->sdir);
    KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(new_rad * 1000));

    if (g_steering_target_received) {
        if (!g_logged_first_target) {
            // Log the moment the guard triggers (helps identify UART noise)
            printf("!!! GUARD TRIGGERED: target=%.3f actual=%.3f\n", target_rad, new_rad);
            g_logged_first_target = true;
        }
        float pid_out = KM_PID_Calculate(c->dir_pid, target_rad, new_rad);
        KM_ACT_SetOutput(c->dir_act, pid_out);
    }
}

void system_init(void) {
    if (KM_GPIO_Init() != ESP_OK)
        printf("GPIO init error\n");

    KM_RTOS_Init();

    // Enable comms — this takes over UART0 for binary protocol
    if (KM_COMS_Init(UART_NUM_0) != ESP_OK)
        printf("COMS init error\n");

    sensor_struct sdir = KM_SDIR_Init(MAX_ERROR_COUNT_SDIR);
    KM_SDIR_Begin(&sdir, GPIO_NUM_21, GPIO_NUM_22);
    KM_SDIR_setCenterOffset(&sdir, 0);

    float init_rad = KM_SDIR_ReadAngleRadians(&sdir);
    if (sdir.errorCount == 0) {
        KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(init_rad * 1000));
        KM_OBJ_SetObjectValue(TARGET_STEERING, (int64_t)(init_rad * 1000));
    }

    ACT_Controller dir_act = KM_ACT_Init(ACT_STEER, 0.08);
    KM_ACT_SetLimit(&dir_act, 0.08);
    KM_ACT_Stop(&dir_act);

    PID_Controller dir_pid = KM_PID_Init(2.0f, 0.0f, 0.005f);
    KM_PID_SetOutputLimits(&dir_pid, -1.0f, 1.0f);
    KM_PID_SetIntegralLimits(&dir_pid, -10.0f, 10.0f);

    static control_context_t ctrl_ctx;
    static sensor_struct sdir_static;
    static ACT_Controller dir_act_static;
    static PID_Controller dir_pid_static;

    sdir_static = sdir;
    dir_act_static = dir_act;
    dir_pid_static = dir_pid;

    ctrl_ctx.sdir = &sdir_static;
    ctrl_ctx.dir_act = &dir_act_static;
    ctrl_ctx.dir_pid = &dir_pid_static;

    // Comms task at 20 Hz
    RTOS_Task t1 = KM_COMS_CreateTask("coms", comms_task, NULL, 50, 4096, 5, 1);
    KM_RTOS_AddTask(t1);

    // Control task at 10 Hz
    RTOS_Task t0 = KM_COMS_CreateTask("ctrl", control_task, &ctrl_ctx, 100, 4096, 5, 1);
    KM_RTOS_AddTask(t0);
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);
    system_init();
}
