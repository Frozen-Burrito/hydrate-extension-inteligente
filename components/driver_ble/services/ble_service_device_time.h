#ifndef _BLE_SERVICE_DEVICE_TIME_H_
#define _BLE_SERVICE_DEVICE_TIME_H_

#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>

/* Atributos GATT del servicio de fecha del dispositivo. */
enum {
    DEV_TIME_IDX,    /* Índice del servicio de fecha del dispositivo */

    DEV_TIME_IDX_CHAR_DEVICE_TIME, /* Declaración de la característica de fecha del dispositivo. */
    DEV_TIME_IDX_VAL_DEVICE_TIME,  /* Índice del valor de la característica de fecha del dispositivo. */

    DEV_TIME_ATTRIBUTE_COUNT
};

uint16_t device_time_handle_table[DEV_TIME_ATTRIBUTE_COUNT];

extern const uint8_t device_time_service_uuid[ESP_UUID_LEN_16] = {
    0x47, 0x18
};

extern const uint16_t DEVICE_TIME_CHAR_UUID = 0x2b90;

extern const esp_gatts_attr_db_t device_time_svc_attr_table[DEV_TIME_ATTRIBUTE_COUNT];

#endif /* _BLE_SERVICE_DEVICE_TIME_H_ */
