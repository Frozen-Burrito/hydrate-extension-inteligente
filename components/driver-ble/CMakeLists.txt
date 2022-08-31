set(SOURCES "driver_ble.c")

idf_component_register(SRCS ${SOURCES}
                        INCLUDE_DIRS "include"
                        REQUIRES freertos bt esp_common)
