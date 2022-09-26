#ifndef _BLE_SERVICE_BATTERY_H_
#define _BLE_SERVICE_BATTERY_H_

#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>

#include "ble_common.h"
#include "hydrate_common.h"

/* Atributos GATT del servicio de batería. */
enum {
    BATTERY_SVC_IDX,    /* Índice del servicio de batería. */

    BAT_IDX_REMAINING_CHARGE_CHAR, /* Declaración de la característica de batería restante. */
    BAT_IDX_REMAINING_CHARGE_VAL,  /* Índice del valor de la característica de batería restante. */
    BAT_IDX_REMAINING_CHARGE_DESCR,  /* Índice del descriptor de la característica de batería restante. */

    BATTERY_SVC_ATTRIBUTE_COUNT
};

uint16_t battery_handle_table[BATTERY_SVC_ATTRIBUTE_COUNT];

extern const uint16_t battery_service_uuid;

const esp_gatts_attr_db_t battery_svc_attr_table[BATTERY_SVC_ATTRIBUTE_COUNT];

uint16_t get_attribute_by_battery_handle(uint16_t attribute_handle);

esp_err_t battery_svc_set_value(uint8_t remaining_battery_percent);

void handle_battery_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response);

#endif /* _BLE_SERVICE_BATTERY_H_ */
