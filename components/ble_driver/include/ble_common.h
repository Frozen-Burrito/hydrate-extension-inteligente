#ifndef _BLE_COMMON_H_
#define _BLE_COMMON_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_gatts_api.h>
#include <esp_err.h>

#define MAX_GATTS_CHAR_LEN_BYTES 500
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

typedef enum {
    UNKNOWN,
    INACTIVE,
    INITIALIZING,
    INITIALIZED,
    ENABLED,
    ADVERTISING,
    PAIRING,
    PAIRED,
    SHUTTING_DOWN,
    SHUT_DOWN
} ble_status_t;

extern const uint16_t primary_service_uuid;
extern const uint16_t secondary_service_uuid;
extern const uint16_t char_client_config_uuid;
extern const uint16_t char_declare_uuid;
extern const uint16_t char_descriptor_uuid;

extern const uint8_t char_property_read;
extern const uint8_t char_property_write;
extern const uint8_t char_property_read_write;
extern const uint8_t char_property_notify;
extern const uint8_t char_property_read_notify;
extern const uint8_t char_property_write_notify;
extern const uint8_t char_property_read_write_notify;

extern const EventBits_t INACTIVE_BIT;
extern const EventBits_t INITIALIZING_BIT;
extern const EventBits_t INITIALIZED_BIT;
extern const EventBits_t ENABLED_BIT;
extern const EventBits_t ADVERTISING_BIT;
extern const EventBits_t PAIRING_BIT;
extern const EventBits_t PAIRED_BIT;
extern const EventBits_t RECORD_SYNCHRONIZED_BIT;
extern const EventBits_t SHUTTING_DOWN_BIT;
extern const EventBits_t SHUT_DOWN_BIT;

extern const EventBits_t ALL_BITS;

extern const uint16_t INDICATE_NOTIFY_DISABLED;
extern const uint16_t NOTIFY_ENABLED;
extern const uint16_t INDICATE_ENABLED;

bool uuid128_equals(const uint8_t first_uuid[], const uint8_t second_uuid[]);

#endif /* _BLE_COMMON_H_ */
