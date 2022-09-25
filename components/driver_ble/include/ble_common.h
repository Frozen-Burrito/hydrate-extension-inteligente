#ifndef _BLE_COMMON_H_
#define _BLE_COMMON_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_gatts_api.h>

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

typedef enum {
    UNKNOWN,
    INACTIVE,
    INITIALIZING,
    ADVERTISING,
    PAIRING,
    PAIRED,
    SHUTTING_DOWN,
} ble_status_t;

const uint16_t primary_service_uuid        = ESP_GATT_UUID_PRI_SERVICE;
const uint16_t secondary_service_uuid      = ESP_GATT_UUID_SEC_SERVICE;
const uint16_t char_client_config_uuid     = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
const uint16_t char_declare_uuid           = ESP_GATT_UUID_CHAR_DECLARE;
const uint16_t char_description            = ESP_GATT_UUID_CHAR_DESCRIPTION;

const uint8_t char_property_read              = ESP_GATT_CHAR_PROP_BIT_READ;
const uint8_t char_property_write             = ESP_GATT_CHAR_PROP_BIT_WRITE;
const uint8_t char_property_read_write        = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
const uint8_t char_property_notify            = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
const uint8_t char_property_read_notify       = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
const uint8_t char_property_write_notify      = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
const uint8_t char_property_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

const EventBits_t INACTIVE_BIT = ( 1 << 0 );
const EventBits_t INITIALIZING_BIT = ( 1 << 1 );
const EventBits_t ADVERTISING_BIT = ( 1 << 2 );
const EventBits_t PAIRING_BIT = ( 1 << 3 );
const EventBits_t PAIRED_BIT = ( 1 << 4 );
const EventBits_t RECORD_SYNCHRONIZED_BIT = ( 1 << 5 );
const EventBits_t SHUTTING_DOWN_BIT = ( 1 << 6 );

const EventBits_t ALL_BITS = (
    INACTIVE | INITIALIZING_BIT | ADVERTISING_BIT | PAIRING_BIT | PAIRED_BIT | 
    RECORD_SYNCHRONIZED_BIT | SHUTTING_DOWN_BIT
);

#endif /* _BLE_COMMON_H_ */