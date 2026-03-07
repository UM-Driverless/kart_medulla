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

static const char *TAG = "TEST_D";

// Required by km_coms.c extern reference
volatile bool g_steering_target_received = false;

#define MAX_ERROR_COUNT_SDIR 10

typedef struct {
    sensor_struct *sdir;
    ACT_Controller *dir_act;
    PID_Controller *dir_pid;
    int tick_count;
    bool offset_applied;
} control_context_t;

// 10 Hz — PID direction verification
void control_task(void *ctx) {
    control_context_t *c = (control_context_t *)ctx;
    c->tick_count++;

    float target_rad = (float)KM_OBJ_GetObjectValue(TARGET_STEERING) / 1000.0f;
    float new_rad = KM_SDIR_ReadAngleRadians(c->sdir);
    KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(new_rad * 1000));

    // After 5 seconds (50 ticks at 10 Hz), offset target by +0.1 rad
    if (c->tick_count == 50 && !c->offset_applied) {
        float offset_target = new_rad + 0.1f;
        KM_OBJ_SetObjectValue(TARGET_STEERING, (int64_t)(offset_target * 1000));
        printf(">>> OFFSET APPLIED: new target=%.3f (actual was %.3f)\n", offset_target, new_rad);
        c->offset_applied = true;
    }

    float error = target_rad - new_rad;
    float pid_out = KM_PID_Calculate(c->dir_pid, target_rad, new_rad);
    KM_ACT_SetOutput(c->dir_act, pid_out);

    // Log direction pin state for verification
    int dir_pin = (pid_out >= 0.0f) ? 1 : 0;
    printf("T=%.3f A=%.3f E=%.4f PID=%.4f DIR=%d tick=%d\n",
           target_rad, new_rad, error, pid_out, dir_pin, c->tick_count);
}

void system_init(void) {
    if (KM_GPIO_Init() != ESP_OK)
        printf("GPIO init error\n");

    KM_RTOS_Init();
    // NO KM_COMS_Init — keep UART0 for printf

    sensor_struct sdir = KM_SDIR_Init(MAX_ERROR_COUNT_SDIR);
    KM_SDIR_Begin(&sdir, GPIO_NUM_21, GPIO_NUM_22);
    KM_SDIR_setCenterOffset(&sdir, 0);

    float init_rad = KM_SDIR_ReadAngleRadians(&sdir);
    if (sdir.errorCount == 0) {
        KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(init_rad * 1000));
        KM_OBJ_SetObjectValue(TARGET_STEERING, (int64_t)(init_rad * 1000));
        printf("AS5600 OK — init=%.3f rad\n", init_rad);
    } else {
        printf("AS5600 NOT responding\n");
    }

    ACT_Controller dir_act = KM_ACT_Init(ACT_STEER, 0.15);
    KM_ACT_SetLimit(&dir_act, 0.15);
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
    ctrl_ctx.tick_count = 0;
    ctrl_ctx.offset_applied = false;

    RTOS_Task t0 = KM_COMS_CreateTask("ctrl", control_task, &ctrl_ctx, 100, 4096, 5, 1);
    KM_RTOS_AddTask(t0);

    printf("TEST D: PID direction check.\n");
    printf("First 5s: target==actual, motor still.\n");
    printf("After 5s: target += 0.1 rad, motor should move TOWARD target.\n");
    printf("If it moves AWAY -> negate PID output or swap DIR pin.\n");
}

void app_main(void) {
    printf("=== TEST D: PID direction check ===\n");
    system_init();
}
