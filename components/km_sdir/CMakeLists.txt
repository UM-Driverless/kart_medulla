idf_component_register(
    SRCS "km_sdir.c" "km_sdir_pwm.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)
