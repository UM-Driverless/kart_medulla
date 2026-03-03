set(srcs
        "main.c")
        # "sketch.cpp")

set(requires
    "bluepad32" "btstack"
    "bt" "nvs_flash" "esp_wifi" "esp_event" "esp_timer" "esp_system" "esp_netif"
    "km_act" "km_coms" "km_gamc" "km_gpio" "km_objects" "km_pid" "km_rtos" "km_sdir" "km_sta")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
