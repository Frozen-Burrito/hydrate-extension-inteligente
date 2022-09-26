#include "ble_service_battery.h"

#include <string.h>
#include <esp_log.h>
#include <esp_err.h>

static const char* TAG = "BLE_BATTERY_SVC";

const uint16_t battery_service_uuid = 0x180f;

static const uint16_t BATTERY_LVL_GATTS_CHAR_UUID = 0x2a19;

static const uint8_t BATTERY_SVC_CHAR_LEVEL_DESCR[] = "Nivel de Bateria";

typedef struct {
    uint8_t remaining_charge[sizeof(uint8_t)];
} battery_svc_data_buffer_t;

static battery_svc_data_buffer_t battery_svc_data = {};

const esp_gatts_attr_db_t battery_svc_attr_table[BATTERY_SVC_ATTRIBUTE_COUNT] = {
    // Declaración del servicio de nivel de batería.
    [BATTERY_SVC_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_16, sizeof(battery_service_uuid), (uint8_t *)&battery_service_uuid}},

    /* Declaración de característica para el nivel de batería. */
    [BAT_IDX_REMAINING_CHARGE_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read}},

    /* Valor de la característica del nivel de batería. */
    [BAT_IDX_REMAINING_CHARGE_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&BATTERY_LVL_GATTS_CHAR_UUID, ESP_GATT_PERM_READ,
      MAX_GATTS_CHAR_LEN_BYTES, sizeof(battery_svc_data.remaining_charge), (uint8_t *) battery_svc_data.remaining_charge}},

    /* Declaracion del descriptor de la caracteristica de nivel de batería.*/
    [BAT_IDX_REMAINING_CHARGE_DESCR] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&char_descriptor_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, 0, NULL}},
};

uint16_t get_attribute_by_battery_handle(uint16_t attribute_handle) 
{
    uint16_t attribute_index = BATTERY_SVC_ATTRIBUTE_COUNT;

    for (uint16_t index = 0; index < BATTERY_SVC_ATTRIBUTE_COUNT; ++index) 
    {
        if (battery_handle_table[index] == attribute_handle) 
        {
            attribute_index = index;
            break;
        }
    }

    return attribute_index;
}

esp_err_t battery_svc_set_value(uint8_t remaining_battery_percent)
{
    esp_err_t status = ESP_OK; 

    battery_svc_data.remaining_charge[0] = remaining_battery_percent & 0xFF;

    ESP_LOG_BUFFER_HEX(TAG, battery_svc_data.remaining_charge, sizeof(battery_svc_data.remaining_charge));
    
    return status;
}

void handle_battery_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response)
{
    switch (attribute_index) 
    {
        case BAT_IDX_REMAINING_CHARGE_VAL:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, battery_svc_data.remaining_charge, sizeof(battery_svc_data.remaining_charge));
            response->attr_value.len = sizeof(battery_svc_data.remaining_charge);
            break;
        case BAT_IDX_REMAINING_CHARGE_DESCR:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, BATTERY_SVC_CHAR_LEVEL_DESCR, sizeof(BATTERY_SVC_CHAR_LEVEL_DESCR));
            response->attr_value.len = sizeof(BATTERY_SVC_CHAR_LEVEL_DESCR);
            break;
        default:
            ESP_LOGW(TAG, "Indice de atributo no soportado para el servicio de bateria (%d)", attribute_index);
            break;
    }
}

