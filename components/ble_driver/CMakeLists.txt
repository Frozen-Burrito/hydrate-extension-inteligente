set(SOURCES "gatt_server.c" "ble_driver.c" "ble_gap.c" "ble_common.c" "services/ble_service_battery.c" "services/ble_service_device_time.c" "services/ble_service_hydration.c")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include" "services/include"
    REQUIRES bt esp_common hydrate_common
)
