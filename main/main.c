#include "nvs_flash.h"          // Obligatorio para almacenar parámetros BT/WiFi
#include "esp_system.h"         // Funciones básicas de ESP32
#include "esp_log.h"            // Logging

#include "esp_bt.h"             // Core BT
// #include "esp_bt_main.h"        // Funciones de inicio BT
// #include "esp_bt_device.h"      // Información del dispositivo
// #include "esp_gap_ble_api.h"    // GAP BLE (escaneo, publicidad)
// #include "esp_gatts_api.h"      // Servidor GATT
// #include "esp_gatt_common_api.h"// Funciones GATT comunes

#include <btstack_port_esp32.h>           // Puerto BTstack para ESP32
#include <btstack_run_loop.h>             // Loop de BTstack
#include <btstack_stdio_esp32.h>          // Consola BTstack
#include <uni.h>                           // Core Bluepad32 (unijoysticle)
// #include <platform/esp32/uni_platform_esp32.h> // Adaptación a ESP32

#include "esp_timer.h"       // Temporizadores de alta resolución
#include "esp_wifi.h"        // Para WiFi
#include "esp_event.h"       // Event loop de ESP-IDF
#include "esp_netif.h"       // Configuración de interfaces de red
#include "esp_sntp.h"        // Para sincronizar tiempo
#include "esp_system.h"      // Información de sistema

// Librerias propias
#include "km_act.h"
#include "km_coms.h"
#include "km_gamc.h"
#include "km_pid.h"
#include "km_rtos.h"
#include "km_sdir.h"
#include "km_sta.h"
#include "km_gpio.h"

static const char *TAG = "MAIN";

// ===========================
// Funciones auxiliares de Bluetooth
// ===========================
// void init_bluetooth(void) {
//     esp_err_t ret;

//     // Inicializar NVS (necesario para BT/WiFi)
//     ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // Inicializar controlador BT
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
//     ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BTDM)); // BLE + Classic

//     // Inicializar Bluedroid
//     ESP_ERROR_CHECK(esp_bluedroid_init());
//     ESP_ERROR_CHECK(esp_bluedroid_enable());

//     // Inicializar consola BTstack
//     btstack_stdio_init();

//     // Configurar plataforma ESP32 para Bluepad32
//     uni_platform_set_custom(get_esp32_platform());

//     // Inicializar Bluepad32
//     uni_init(0, NULL);

//     ESP_LOGI(TAG, "Bluetooth y Bluepad32 inicializados");
// }

#define MAX_ERROR_COUNT_SDIR 10

void system_init(void) {

    // Initialize hardware
    if(KM_GPIO_Init() != ESP_OK)
        ESP_LOGE(TAG, "Error inicializando libreria gpio\n");

    // Initialize I2C
    if (KM_GPIO_I2CInit() != ESP_OK)
        ESP_LOGE(TAG, "Error inicializando libreria i2c\n");

    // Initialise tasks
    KM_RTOS_Init();

    // Initialise comunications
    if (KM_COMS_Init(UART_NUM_0) != ESP_OK)
        ESP_LOGE(TAG, "Error inicializando libreria de comunicaciones");
    
    // KM_SDIR_Init(MAX_ERROR_COUNT_SDIR);

    // KM_ACT_Begin();
    // KM_PID_Init();

    uint8_t payload[31];  // 255 - 4 = 251
    uint8_t len = 31;     // LEN solo cuenta los bytes del payload
    for (size_t i = 0; i < 31; i++) {
        payload[i] = i;
    }
    
    // while (true) {
    //     ESP_LOGE(TAG, "Antes de enviar mensaje");
    //     KM_COMS_SendMsg(ESP_HEARTBEAT, payload, len);
    //     ESP_LOGE(TAG, "Despues de enviar mensaje");
    //     sys_delay_ms(100);
    // }
    km_coms_msg msg;
    while (true) {
        km_coms_ReceiveMsg();
        ESP_LOGI(TAG, "Despues de receive msg");
    //     KM_COMS_ProccessMsgs();

    //     if(xQueueReceive(km_coms_queue, &msg, 0) == pdPASS)
    //         ESP_LOGI(TAG, "El mensaje recibido es de tipo: %d, tiene logitud: %d, y CRC: %d", msg.type, msg.len, msg.crc);
        //sys_delay_ms(1000);
        ESP_LOGI(TAG, "Despues de delay");
    }
}

// ===========================
// app_main - punto de entrada
// ===========================
void app_main(void) {

    // Sin trazas
    esp_log_level_set("*", ESP_LOG_NONE);

    debug_led_init();


    ESP_LOGI(TAG, "ESP iniciando...");

    // Inicializa Bluetooth, Bluepad32, NVS, etc.
    // init_bluetooth();

    // Inicia todas las librerias que se necesitan
    system_init();

    // Creacion de toda las tareas de FreeRTOS
    // Tareas de libreria de comunicaciones
    // Tareas de libreria de maquina de estado

    // Ejecutar loop de BTstack (bloquea dentro de app_main, pero otras tareas siguen)
    //btstack_run_loop_execute();
}

// ESP_LOGI(TAG, "Mensaje: %d", valor); → Información normal

// ESP_LOGW(TAG, "Mensaje: %s", cadena); → Advertencia

// ESP_LOGE(TAG, "Mensaje: %s", cadena); → Error

// ESP_LOGD(TAG, "Mensaje debug"); → Debug (solo si está habilitado)
