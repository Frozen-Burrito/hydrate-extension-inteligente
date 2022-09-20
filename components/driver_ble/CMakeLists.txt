set(SOURCES "driver_ble.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include"
    REQUIRES bt esp_common hydrate_common
)
