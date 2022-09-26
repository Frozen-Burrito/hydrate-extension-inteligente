#ifndef _BLE_SERVICE_DEVICE_TIME_H_
#define _BLE_SERVICE_DEVICE_TIME_H_

#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>

#include "ble_common.h"
#include "hydrate_common.h"

/* Atributos GATT del servicio de fecha del dispositivo. */
enum {
    DEV_TIME_IDX,    /* Índice del servicio de fecha del dispositivo */

    DEV_TIME_IDX_DEVICE_TIME_CHAR, /* Declaración de la característica de fecha del dispositivo. */
    DEV_TIME_IDX_DEVICE_TIME_VAL,  /* Índice del valor de la característica de fecha del dispositivo. */
    DEV_TIME_IDX_DEVICE_TIME_DESCR,  /* Índice del valor de la característica de fecha del dispositivo. */

    DEV_TIME_SVC_ATTRIBUTE_COUNT
};

uint16_t device_time_handle_table[DEV_TIME_SVC_ATTRIBUTE_COUNT];

extern const uint16_t device_time_service_uuid;

const esp_gatts_attr_db_t device_time_svc_attr_table[DEV_TIME_SVC_ATTRIBUTE_COUNT];

uint16_t get_attribute_by_dev_time_handle(uint16_t attribute_handle);

esp_err_t device_time_svc_set_value(int64_t local_device_time_seconds);

void handle_device_time_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response);
esp_gatt_status_t handle_device_time_svc_write_evt(uint16_t attribute_index, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

#endif /* _BLE_SERVICE_DEVICE_TIME_H_ */
