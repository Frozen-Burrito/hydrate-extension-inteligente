#include "ble_common.h"

const uint16_t primary_service_uuid        = ESP_GATT_UUID_PRI_SERVICE;
const uint16_t secondary_service_uuid      = ESP_GATT_UUID_SEC_SERVICE;
const uint16_t char_client_config_uuid     = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
const uint16_t char_declare_uuid           = ESP_GATT_UUID_CHAR_DECLARE;
const uint16_t char_descriptor_uuid        = ESP_GATT_UUID_CHAR_DESCRIPTION;

const uint8_t char_property_read              = ESP_GATT_CHAR_PROP_BIT_READ;
const uint8_t char_property_write             = ESP_GATT_CHAR_PROP_BIT_WRITE;
const uint8_t char_property_read_write        = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
const uint8_t char_property_notify            = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
const uint8_t char_property_read_notify       = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
const uint8_t char_property_write_notify      = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
const uint8_t char_property_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

const EventBits_t INACTIVE_BIT = ( 1 << 0 );
const EventBits_t INITIALIZING_BIT = ( 1 << 1 );
const EventBits_t INITIALIZED_BIT = ( 1 << 2 );
const EventBits_t ADVERTISING_BIT = ( 1 << 3 );
const EventBits_t PAIRING_BIT = ( 1 << 4 );
const EventBits_t PAIRED_BIT = ( 1 << 5 );
const EventBits_t RECORD_SYNCHRONIZED_BIT = ( 1 << 6 );
const EventBits_t SHUTTING_DOWN_BIT = ( 1 << 7 );
const EventBits_t SHUT_DOWN_BIT = ( 1 << 8 );

const EventBits_t ALL_BITS = (
    INACTIVE | INITIALIZING_BIT | INITIALIZED_BIT | ADVERTISING_BIT | 
    PAIRING_BIT | PAIRED_BIT | RECORD_SYNCHRONIZED_BIT | 
    SHUTTING_DOWN_BIT | SHUT_DOWN_BIT
);

const uint16_t INDICATE_NOTIFY_DISABLED = 0x0000;
const uint16_t NOTIFY_ENABLED = 0x0001;
const uint16_t INDICATE_ENABLED = 0x0002;

bool uuid128_equals(const uint8_t first_uuid[], const uint8_t second_uuid[]) 
{
    for (size_t i = 0; i < ESP_UUID_LEN_128; ++i) 
    {
        if (first_uuid[i] != second_uuid[i])
        {
            return false;
        }
    }

    return true;
}