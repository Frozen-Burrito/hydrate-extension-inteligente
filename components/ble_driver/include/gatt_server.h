#ifndef _GATT_SERVER_H_
#define _GATT_SERVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include <esp_err.h>

#include "ble_gap.h"
#include "ble_common.h"

#include "ble_service_battery.h"
#include "ble_service_device_time.h"
#include "ble_service_hydration.h"

esp_err_t ble_gatt_server_init(const char* device_name, EventGroupHandle_t bleStatusEventGroup);
esp_err_t ble_gatt_server_shutdown(void);

esp_err_t ble_gatt_server_disconnect(void);

esp_err_t ble_gatt_server_indicate(uint16_t attribute_handle, uint16_t value_len, uint8_t* value, bool need_confirm);

#endif /* _GATT_SERVER_H_ */