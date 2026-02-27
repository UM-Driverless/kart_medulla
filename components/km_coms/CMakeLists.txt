idf_component_register(
    SRCS "km_coms.c"
    INCLUDE_DIRS "."           # si tus headers están en la misma carpeta
    REQUIRES km_objects km_gpio
)