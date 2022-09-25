#ifndef _BLE_SERVICE_HYDRATION_H_
#define _BLE_SERVICE_HYDRATION_H_

#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>

/* Atributos GATT del servicio de hidratacion perfil. */
enum {
    HYDRATION_SVC_IDX,    /* Índice del servicio de hidratación. */

    HYDR_IDX_CHAR_MILILITERS, /* Índice de la característica de cantidad de agua en ml. */
    HYDR_IDX_VAL_MILILITERS,  /* Índice del valor de la característica de cantidad de agua en ml. */

    HYDR_IDX_CHAR_TEMP,     /* Índice de la característica de temperatura aproximada. */
    HYDR_IDX_VAL_TEMP,      /* Índice del valor de la característica de temperatura aprox. en C. */

    HYDR_IDX_CHAR_DATE,    /* Índice del valor de la característica de fecha (timestamp). */
    HYDR_IDX_VAL_DATE,     

    HYDR_IDX_CHAR_NUM_NEW_RECORDS,
    HYDR_IDX_VAL_NUM_NEW_RECORDS,
    HYDR_IDX_CHAR_NUM_NEW_RECORDS_NTF_CFG,

    HYDR_SVC_ATTRIBUTE_COUNT
};

uint16_t hydration_handle_table[HYDR_SVC_ATTRIBUTE_COUNT];

extern const uint8_t hydration_service_uuid[ESP_UUID_LEN_128] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xf5, 0x19, 0x00, 0x00,
};

extern const uint32_t AMOUNT_ML_GATTS_CHAR_UUID = 0x0faf892c;
extern const uint16_t TEMP_GATTS_CHAR_UUID = 0x2a6e;
extern const uint16_t DATE_GATTS_CHAR_UUID = 0x0fff;
extern const uint32_t PENDING_RECORD_COUNT_GATTS_CHAR_UUID = 0x0faf892f;

extern const esp_gatts_attr_db_t hydration_svc_attr_table[HYDR_SVC_ATTRIBUTE_COUNT];

#endif /* _BLE_SERVICE_HYDRATION_H_ */
