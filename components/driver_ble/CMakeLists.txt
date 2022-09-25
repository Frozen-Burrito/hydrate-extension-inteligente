set(SOURCES "driver_ble.c" "gatt_server.c" "ble_driver.c" "ble_gap.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include" "services"
    REQUIRES bt esp_common hydrate_common
)
