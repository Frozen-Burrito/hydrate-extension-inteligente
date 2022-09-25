#ifndef _BLE_GAP_H_
#define _BLE_GAP_H_

#include <stdbool.h>
#include <esp_err.h>

extern const uint16_t ADV_CONFIG_FLAG = (1 << 0);
extern const uint16_t SCAN_RESPONSE_CONFIG_FLAG = (1 << 1);

esp_err_t ble_gap_set_adv_data(const char* device_name);

esp_err_t ble_gap_start_adv(void);

esp_err_t ble_gap_init(void);

esp_err_t ble_gap_shutdown(void);

#endif /* _BLE_GAP_H_ */
