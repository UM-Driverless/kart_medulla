set(srcs
        "main.c")
        # "sketch.cpp")

set(requires
    "km_act" "km_coms" "km_gamc" "km_gpio" "km_objects" "km_pid" "km_rtos" "km_sdir" "km_sta")

idf_component_register(SRCS "test_main.c" "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
