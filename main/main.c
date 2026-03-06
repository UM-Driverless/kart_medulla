#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"

#include <math.h>
#include <stdarg.h>

// Librerias propias
#include "km_act.h"
#include "km_coms.h"
#include "km_gamc.h"
#include "km_pid.h"
#include "km_rtos.h"
#include "km_sdir.h"
#include "km_sta.h"
#include "km_gpio.h"
#include "km_objects.h"

static const char *TAG = "MAIN";

#define MAX_ERROR_COUNT_SDIR 10

// Context struct shared by control task
typedef struct {
    sensor_struct *sdir;
    ACT_Controller *dir_act;
    ACT_Controller *throttle_act;
    ACT_Controller *brake_act;
    PID_Controller *dir_pid;
} control_context_t;

// ===========================
// FreeRTOS task functions
// ===========================

// 20 Hz — UART RX + parse incoming messages
void comms_task(void *ctx) {
    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();
}

// 10 Hz — read sensor, run PID, drive actuators, send feedback
void control_task(void *ctx) {
    control_context_t *c = (control_context_t *)ctx;

    // Send feedback FIRST (use last known value) so frames arrive even if I2C blocks
    float actual_rad = (float)KM_OBJ_GetObjectValue(ACTUAL_STEERING) / 1000.0f;
    int16_t actual_i16 = (int16_t)(actual_rad * 1000.0f);
    uint8_t fb[2] = {(uint8_t)(actual_i16 >> 8), (uint8_t)(actual_i16 & 0xFF)};
    KM_COMS_SendMsg(ESP_ACT_STEERING, fb, 2);

    // Target from Orin: int16 radians × 1000
    float target_rad = (float)KM_OBJ_GetObjectValue(TARGET_STEERING) / 1000.0f;

    // Throttle + brake: unchanged (effort commands, 0-255)
    float thr = (float)KM_OBJ_GetObjectValue(TARGET_THROTTLE) / 255.0f;
    float brk = (float)KM_OBJ_GetObjectValue(TARGET_BRAKING) / 255.0f;
    KM_ACT_SetOutput(c->throttle_act, thr);
    KM_ACT_SetOutput(c->brake_act, brk);

    // Read sensor AFTER sending — if I2C hangs, at least feedback/actuators ran
    float new_rad = KM_SDIR_ReadAngleRadians(c->sdir);
    KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(new_rad * 1000));

    // PID in radians
    float pid_out = KM_PID_Calculate(c->dir_pid, target_rad, new_rad);
    KM_ACT_SetOutput(c->dir_act, pid_out);
}

// 1 Hz — heartbeat to Orin
void heartbeat_task(void *ctx) {
    uint8_t payload[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    KM_COMS_SendMsg(ESP_HEARTBEAT, payload, sizeof(payload));
}

void system_init(void) {

    // Initialize hardware
    if(KM_GPIO_Init() != ESP_OK)
        ESP_LOGE(TAG, "Error inicializando libreria gpio\n");

    // Initialise tasks
    KM_RTOS_Init();

    // Initialise comunications on UART0 (USB to Orin)
    if (KM_COMS_Init(UART_NUM_0) != ESP_OK)
        ESP_LOGE(TAG, "Error inicializando libreria de comunicaciones");
    
    sensor_struct sdir = KM_SDIR_Init(MAX_ERROR_COUNT_SDIR);
    KM_SDIR_Begin(&sdir, GPIO_NUM_21, GPIO_NUM_22);

    // Test AS5600 connectivity and seed initial angle
    float init_rad = KM_SDIR_ReadAngleRadians(&sdir);
    if (sdir.errorCount == 0) {
        KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(init_rad * 1000));
        ESP_LOGI(TAG, "AS5600 connected — %.3f rad", init_rad);
    } else {
        ESP_LOGW(TAG, "AS5600 NOT responding — steering feedback will be stale");
    }

    // ------------------------------------------------------
    // Initialize Motor controllers
    ACT_Controller dir_act = KM_ACT_Init(ACT_STEER, 0.4);
    ACT_Controller throttle_act = KM_ACT_Init(ACT_ACCEL, 1.0);
    ACT_Controller brake_act = KM_ACT_Init(ACT_BRAKE, 1.0);

    KM_ACT_SetLimit(&dir_act, 0.4);
    KM_ACT_SetLimit(&throttle_act, 1.0);
    KM_ACT_SetLimit(&brake_act, 1.0);

    // Initialise PID for steering
    float kp = 0.03;
    float ki = 0.0;
    float kd = 0.0004;
    PID_Controller dir_pid = KM_PID_Init(kp, ki, kd);
    KM_PID_SetOutputLimits(&dir_pid, -1.0f, 1.0f);
    KM_PID_SetIntegralLimits(&dir_pid, -10.0f, 10.0f);

    // Build control context (static so it outlives system_init)
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

    // Register FreeRTOS tasks
    RTOS_Task t1 = KM_COMS_CreateTask("comms", comms_task, NULL, 10, 2048, 2, 1);
    RTOS_Task t2 = KM_COMS_CreateTask("control", control_task, &ctrl_ctx, 10, 4096, 1, 1);
    RTOS_Task t3 = KM_COMS_CreateTask("heartbeat", heartbeat_task, NULL, 1000, 2048, 1, 1);

    KM_RTOS_AddTask(t1);
    KM_RTOS_AddTask(t2);
    KM_RTOS_AddTask(t3);

    ESP_LOGI(TAG, "All tasks registered — scheduler running");
    // system_init returns, FreeRTOS scheduler keeps tasks alive
}

void app_main(void) {
    /* UART2 removed — GPIO17/16 reserved for hall sensors on PCB.
       All logs go to UART0 (USB) at 115200. */
    esp_log_level_set("*", ESP_LOG_NONE);  /* Logs disabled — UART0 shared with binary protocol */
    ESP_LOGI(TAG, "ESP32 starting...");
    system_init();
}
