set(srcs
        "main.c"
        "sketch.cpp")

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "km_rtos" "km_pid" "km_sdir" "km_act")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
