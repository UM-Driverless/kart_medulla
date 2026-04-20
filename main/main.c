/******************************************************************************
 * @file    main.c
 * @brief   ESP32 application entry point and FreeRTOS task definitions for
 *          the kart_medulla firmware.
 *
 * @details Brings up all hardware peripherals, initializes controllers and
 *          sensors, and registers periodic FreeRTOS tasks for communications,
 *          control, heartbeat, and health monitoring.
 *****************************************************************************/

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/dac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

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
#define COMMS_WATCHDOG_MS    1000  // Zero outputs if no command for this long
#define MISSION_MANUAL       0     // Mission ID 0 = manual (no electronic actuation)

/**
 * @brief Context shared between the control and health tasks.
 */
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

/**
 * @brief   Communications task — receives and processes UART messages from Orin.
 *
 * @details Runs at 20 Hz via KM_RTOS periodic wrapper. Reads incoming bytes from
 *          UART0 and parses complete binary-encoded messages into shared objects.
 *
 * @param   ctx  Unused (NULL).
 */
void comms_task(void *ctx) {
    km_coms_ReceiveMsg();
    KM_COMS_ProccessMsgs();
}

/**
 * @brief   Control task — steering PID, actuator output, and sensor feedback.
 *
 * @details Runs at 10 Hz (100 ms period). On each cycle:
 *          1. Sends steering feedback (angle + raw encoder) to Orin FIRST, so
 *             frames arrive even if the subsequent I2C read blocks.
 *          2. Applies throttle and brake actuator outputs from Orin targets.
 *          3. Reads the AS5600 steering angle via I2C.
 *          4. Runs the steering PID controller and sets the motor output.
 *
 * @param   ctx  Pointer to a control_context_t with sensor, actuator, and PID references.
 */
void control_task(void *ctx) {
    control_context_t *c = (control_context_t *)ctx;

    // Send feedback FIRST (use last known value) so frames arrive even if I2C blocks
    static float last_pid_out = 0.0f;
    int32_t fb[3] = {
        (int32_t)KM_OBJ_GetObjectValue(ACTUAL_STEERING),  // angle_rad x 1000
        (int32_t)c->sdir->lastRawValue,                    // raw encoder
        (int32_t)(last_pid_out * 1000)                     // PID output (PWM duty) x 1000
    };
    KM_COMS_SendMsg(ESP_ACT_STEERING, fb, 3);

    // Read sensor — AS5600 already positive=left, matches our convention
    float new_rad = KM_SDIR_ReadAngleRadians(c->sdir);
    KM_OBJ_SetObjectValue(ACTUAL_STEERING, (int64_t)(new_rad * 1000));

    // --- Safety: comms watchdog + manual mode ---
    TickType_t last_cmd = KM_COMS_GetLastCmdTick();
    TickType_t now = xTaskGetTickCount();
    int mission = (int)KM_OBJ_GetObjectValue(MISION_ORIN);
    int comms_stale = (last_cmd == 0) || ((now - last_cmd) > pdMS_TO_TICKS(COMMS_WATCHDOG_MS));

    if (comms_stale || mission == MISSION_MANUAL) {
        // No commands received recently OR manual mode → zero all outputs
        KM_ACT_Stop(c->throttle_act);
        KM_ACT_Stop(c->brake_act);
        KM_ACT_Stop(c->dir_act);
        KM_PID_Reset(c->dir_pid);
        last_pid_out = 0.0f;
        return;
    }

    // Steering mode: 0=PID (default), 1=direct PWM
    int steer_mode = (int)KM_OBJ_GetObjectValue(STEER_MODE);

    // Target from Orin: interpretation depends on mode
    float target_raw = (float)KM_OBJ_GetObjectValue(TARGET_STEERING) / 1000.0f;

    // Throttle + brake: int32 effort (0-255 range from Orin)
    float thr = (float)KM_OBJ_GetObjectValue(TARGET_THROTTLE) / 255.0f;
    float brk = (float)KM_OBJ_GetObjectValue(TARGET_BRAKING) / 255.0f;
    KM_ACT_SetOutput(c->throttle_act, thr);
    KM_ACT_SetOutput(c->brake_act, brk);

    float steer_out;
    if (steer_mode == 1) {
        // Direct PWM mode: target_raw is PWM value [-1.0, 1.0]
        steer_out = target_raw;
        // Reset PID integral so it doesn't wind up while inactive
        KM_PID_Reset(c->dir_pid);
    } else {
        // PID mode: target_raw is angle in radians
        steer_out = KM_PID_Calculate(c->dir_pid, target_raw, new_rad);
    }
    KM_ACT_SetOutput(c->dir_act, steer_out);
    last_pid_out = steer_out;
}

/**
 * @brief   Heartbeat task — sends a periodic alive signal to Orin.
 *
 * @details Runs at 1 Hz. Sends a single int32 payload containing the ESP32
 *          uptime in milliseconds as an ESP_HEARTBEAT message over UART.
 *
 * @param   ctx  Unused (NULL).
 */
void heartbeat_task(void *ctx) {
    int32_t payload[1] = {(int32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS)};
    KM_COMS_SendMsg(ESP_HEARTBEAT, payload, 1);
}

/**
 * @brief   Health monitoring task — checks magnet, I2C, and heap status.
 *
 * @details Runs at 1 Hz in its own FreeRTOS task (not via KM_RTOS wrapper).
 *          Reports a 4-element int32 payload [flags, agc, heap_kb, i2c_errors]
 *          to Orin as an ESP_HEALTH_STATUS message.
 *
 *          Flag bits:
 *          - bit 0 (HEALTH_FLAG_MAGNET_OK): AGC in valid range [20, 235].
 *          - bit 1 (HEALTH_FLAG_I2C_OK): AS5600 I2C read succeeded.
 *          - bit 2 (HEALTH_FLAG_HEAP_OK): Free heap >= 4 KB.
 *
 * @param   ctx  Pointer to a control_context_t (needs sdir for AS5600 access).
 *
 * @note    This task contains its own infinite loop with vTaskDelay; it is NOT
 *          intended for use with KM_RTOS_TaskWrapper.
 */
#define HEALTH_FLAG_MAGNET_OK (1 << 0)
#define HEALTH_FLAG_I2C_OK    (1 << 1)
#define HEALTH_FLAG_HEAP_OK   (1 << 2)
#define HEALTH_HEAP_MIN_BYTES 4096
#define AGC_MIN 20   // below = magnet too strong
#define AGC_MAX 235  // above = magnet too weak

void health_task(void *ctx) {
    control_context_t *c = (control_context_t *)ctx;

    while (1) {
        int32_t flags = 0;

        // AGC is the reliable magnet strength indicator (0=too strong, 255=too weak)
        uint8_t as_status = 0, agc = 0;
        int8_t i2c_ok = KM_SDIR_ReadStatusAGC(c->sdir, &as_status, &agc);

        if (i2c_ok) flags |= HEALTH_FLAG_I2C_OK;
        if (i2c_ok && agc >= AGC_MIN && agc <= AGC_MAX) flags |= HEALTH_FLAG_MAGNET_OK;

        // Heap check
        uint32_t free_heap = esp_get_free_heap_size();
        if (free_heap >= HEALTH_HEAP_MIN_BYTES) flags |= HEALTH_FLAG_HEAP_OK;
        uint16_t heap_kb = (uint16_t)(free_heap / 1024);

        // Payload: [flags, agc, heap_kb, i2c_errors] — 4 int32 values
        int32_t payload[4] = {flags, (int32_t)agc, (int32_t)heap_kb, (int32_t)c->sdir->errorCount};
        KM_COMS_SendMsg(ESP_HEALTH_STATUS, payload, 4);

        // Log warnings for critical issues
        if (i2c_ok && agc < AGC_MIN)
            ESP_LOGW(TAG, "HEALTH: magnet too strong (AGC=%d)", agc);
        if (i2c_ok && agc > AGC_MAX)
            ESP_LOGW(TAG, "HEALTH: magnet too weak (AGC=%d)", agc);
        if (!i2c_ok)
            ESP_LOGW(TAG, "HEALTH: I2C read failed (err=%d)", c->sdir->errorCount);
        if (!(flags & HEALTH_FLAG_HEAP_OK))
            ESP_LOGW(TAG, "HEALTH: low heap! %lu bytes free", (unsigned long)free_heap);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief   Initializes all subsystems and registers FreeRTOS tasks.
 *
 * @details Performs the full system bring-up sequence:
 *          1. GPIO peripherals (ADC, DAC, PWM, I2C, direction pin).
 *          2. RTOS task manager.
 *          3. UART communications to Orin.
 *          4. AS5600 steering encoder (I2C).
 *          5. Actuator controllers (steering, throttle, brake) with output limits.
 *          6. Steering PID controller.
 *          7. Registers periodic tasks: comms (20 Hz), control (10 Hz),
 *             heartbeat (1 Hz), and health monitoring (1 Hz).
 *
 * @note    All controller/sensor structs are copied to file-scope statics so
 *          they outlive this function. The FreeRTOS scheduler keeps tasks alive
 *          after system_init returns.
 */
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
    // *** STEERING PWM LIMIT — keep low during testing to protect gears ***
    // Increase gradually once PID is tuned. 1.0 = full power.
    ACT_Controller dir_act = KM_ACT_Init(ACT_STEER, 0.40);
    ACT_Controller throttle_act = KM_ACT_Init(ACT_ACCEL, 1.0);
    ACT_Controller brake_act = KM_ACT_Init(ACT_BRAKE, 1.0);

    KM_ACT_SetLimit(&dir_act, 0.50);
    KM_ACT_SetLimit(&throttle_act, 1.0);
    KM_ACT_SetLimit(&brake_act, 1.0);

    // TEMPORARY TEST: raw ESP-IDF DAC write to GPIO 25 (DAC_CHAN_0)
    // Bypasses all our abstraction. Should produce ~1.65V.
    dac_output_enable(DAC_CHAN_0);  // GPIO 25
    dac_output_voltage(DAC_CHAN_0, 128);  // 128/255 * 3.3V ≈ 1.65V

    // Initialise PID for steering
    float kp = 1.50;
    float ki = 0.0;
    float kd = 0.02;
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

    // NOTE: UART2 log redirect removed — it caused crashes and UART0 protocol noise.
    // Logs go to UART0 at 115200. SerialDriver ignores non-0xAA bytes (SOF filtering).

    // Register FreeRTOS tasks
    RTOS_Task t1 = KM_COMS_CreateTask("comms", comms_task, NULL, 10, 4096, 2, 1);
    RTOS_Task t2 = KM_COMS_CreateTask("control", control_task, &ctrl_ctx, 2, 4096, 1, 1);
    RTOS_Task t3 = KM_COMS_CreateTask("heartbeat", heartbeat_task, NULL, 1000, 2048, 1, 1);

    KM_RTOS_AddTask(t1);
    KM_RTOS_AddTask(t2);
    KM_RTOS_AddTask(t3);

    ESP_LOGI(TAG, "All tasks registered — scheduler running");

    // Launch health monitoring task (1 Hz, checks magnet/I2C/heap)
    xTaskCreate(health_task, "health", 4096, &ctrl_ctx, 1, NULL);

    // system_init returns, FreeRTOS scheduler keeps tasks alive
}

/**
 * @brief   Application entry point (called by ESP-IDF after boot).
 *
 * @details Initializes NVS flash (required by ESP-IDF internals such as
 *          WiFi and Bluetooth stacks), sets the global log level to INFO,
 *          and calls system_init() to bring up all subsystems and FreeRTOS tasks.
 *
 * @note    If NVS partition is corrupt or has a version mismatch, the flash
 *          is erased and re-initialized automatically.
 */
void app_main(void) {
    // Init NVS (needed by ESP-IDF internals: WiFi, BT stacks, etc.)
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Log init message before disabling logs — UART0 is shared with binary protocol
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "ESP32 starting...");

    system_init();

    // Disable all logging on UART0 to prevent ASCII text from corrupting
    // binary protocol frames. Without this, ESP_LOG output interleaves with
    // protocol bytes and causes CRC mismatches on the Orin side.
    esp_log_level_set("*", ESP_LOG_NONE);
}
