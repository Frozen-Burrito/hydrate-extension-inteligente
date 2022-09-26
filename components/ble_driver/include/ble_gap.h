#ifndef _BLE_GAP_H_
#define _BLE_GAP_H_

#include <stdbool.h>
#include <esp_err.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "ble_common.h"

extern const uint16_t ADV_CONFIG_FLAG;
extern const uint16_t SCAN_RESPONSE_CONFIG_FLAG;

esp_err_t ble_gap_set_adv_data(const char* device_name);

esp_err_t ble_gap_start_adv(void);

esp_err_t ble_gap_init(EventGroupHandle_t xBleStateEventGroup);

esp_err_t ble_gap_shutdown(void);

#endif /* _BLE_GAP_H_ */
