#ifndef _BLE_DRIVER_H_
#define _BLE_DRIVER_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_bt.h>
#include <esp_bt_main.h>

#include "hydrate_common.h"
#include "ble_common.h"

#include "ble_gap.h"
#include "gatt_server.h"

#include "ble_service_battery.h"
#include "ble_service_device_time.h"
#include "ble_service_hydration.h"

/**
 * @brief Inicializa el driver BLE con sus perfiles GAP y GATT, para luego
 * comenzar el advertising.
 */ 
esp_err_t ble_driver_init(const char* device_name);

esp_err_t ble_driver_sleep();

esp_err_t ble_driver_shutdown(void);

esp_err_t ble_synchronize_hydration_record(const hydration_record_t* record, const uint32_t sync_timeout_ms);

/**
 * @brief Esperar a que el status de la conexion BLE sea igual a status.
 * 
 * Si ms_to_wait es cero, el valor retornado por esta funcion sera el
 * estado actual del driver BLE, independientemente de si es el status
 * esperado.
 * 
 * Si ms_to_wait es mayor a cero y ble_wait_for_state retornó porque
 * supero este timeout, el valor retornado es el estado del driver BLE
 * en el momento que el timepo del bloqueo expiró.
 */
ble_status_t ble_wait_for_state(const ble_status_t status, const bool match_exact_state, const uint32_t ms_to_wait); 

#endif /* _BLE_DRIVER_H_ */
