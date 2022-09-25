#ifndef _BLE_SERVICE_BATTERY_H_
#define _BLE_SERVICE_BATTERY_H_

#include <esp_gatt_defs.h>
#include <esp_gatts_api.h>

/* Atributos GATT del servicio de batería. */
enum {
    BATTERY_SVC_IDX,    /* Índice del servicio de información del dispositivo */

    BAT_IDX_CHAR_LEVEL, /* Declaración de la característica de fecha del dispositivo. */
    BAT_IDX_VAL_LEVEL,  /* Índice del valor de la característica de fecha del dispositivo. */

    BATTERY_SVC_ATTRIBUTE_COUNT
};

uint16_t battery_handle_table[BATTERY_SVC_ATTRIBUTE_COUNT];

extern uint8_t battery_service_uuid[ESP_UUID_LEN_16] = {
    0x0f, 0x18
};

extern const uint16_t BATTERY_LVL_GATTS_CHAR_UUID = 0x2a19;

extern const esp_gatts_attr_db_t battery_svc_attr_table[BATTERY_SVC_ATTRIBUTE_COUNT];

#endif /* _BLE_SERVICE_BATTERY_H_ */
