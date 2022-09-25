#ifndef _GATT_SERVER_H_
#define _GATT_SERVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_err.h>

esp_err_t ble_gatt_server_init();
esp_err_t ble_gatt_server_shutdown();

esp_err_t ble_gatt_server_indicate(uint16_t attribute_handle, uint16_t value_len, uint8_t* value, bool need_confirm);

#endif /* _GATT_SERVER_H_ */