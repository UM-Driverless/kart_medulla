idf_component_register(
    SRCS "km_coms.c"
    INCLUDE_DIRS "."
    REQUIRES "km_gpio" km_debug_led driver
)