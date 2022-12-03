#ifndef _BLE_SERVICE_HYDRATION_H_
#define _BLE_SERVICE_HYDRATION_H_

#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "ble_common.h"
#include "hydrate_common.h"

/* Atributos GATT del servicio de hidratacion perfil. */
enum {
    HYDRATION_SVC_IDX,    /* Índice del servicio de hidratación. */

    HYDR_IDX_MILILITERS_CHAR, /* Índice de la característica de cantidad de agua en ml. */
    HYDR_IDX_MILILITERS_VAL,  /* Índice del valor de la característica de cantidad de agua en ml. */
    HYDR_IDX_MILILITERS_DESCR,  /* Índice del valor de la característica de cantidad de agua en ml. */

    HYDR_IDX_TEMP_CHAR,     /* Índice de la característica de temperatura aproximada. */
    HYDR_IDX_TEMP_VAL,      /* Índice del valor de la característica de temperatura aprox. en C. */
    HYDR_IDX_TEMP_DESCR,      /* Índice del valor de la característica de temperatura aprox. en C. */

    HYDR_IDX_TIMESTAMP_CHAR,    /* Índice del valor de la característica de fecha (timestamp). */
    HYDR_IDX_TIMESTAMP_VAL,     
    HYDR_IDX_TIMESTAMP_DESCR,     

    HYDR_IDX_HAS_NEW_RECORDS_CHAR,
    HYDR_IDX_HAS_NEW_RECORDS_VAL,
    HYDR_IDX_HAS_NEW_RECORDS_DESCR,
    HYDR_IDX_HAS_NEW_RECORDS_NTF_CFG,

    HYDR_SVC_ATTRIBUTE_COUNT
};

uint16_t hydration_handle_table[HYDR_SVC_ATTRIBUTE_COUNT];

extern const uint8_t hydration_service_uuid[ESP_UUID_LEN_128];

const esp_gatts_attr_db_t hydration_svc_attr_table[HYDR_SVC_ATTRIBUTE_COUNT];

uint16_t get_attribute_by_hydration_handle(uint16_t attribute_handle);

esp_err_t hydration_svc_set_ble_state_events(EventGroupHandle_t xBleStateEventGroup);

esp_err_t hydration_svc_set_value(const hydration_record_t* hydration_record);

void handle_hydration_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response);
esp_gatt_status_t handle_hydration_svc_write_evt(uint16_t attribute_index, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

#endif /* _BLE_SERVICE_HYDRATION_H_ */
