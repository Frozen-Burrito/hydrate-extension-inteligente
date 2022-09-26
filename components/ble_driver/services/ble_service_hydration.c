#include "ble_service_hydration.h"

#include <string.h>
#include <esp_log.h>

static const char* TAG = "BLE_HYDR_SVC";

const uint8_t hydration_service_uuid[ESP_UUID_LEN_128] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xf5, 0x19, 0x00, 0x00,
};

static const uint32_t AMOUNT_ML_GATTS_CHAR_UUID = 0x0faf892c;
static const uint16_t TEMP_GATTS_CHAR_UUID = 0x2a6e;
static const uint16_t TIMESTAMP_GATTS_CHAR_UUID = 0x0fff;
static const uint32_t PENDING_RECORD_COUNT_GATTS_CHAR_UUID = 0x0faf892f;

static const uint8_t HYDRATION_CHAR_ML_DESCR[] = "Mililitros ingeridos";
static const uint8_t HYDRATION_CHAR_TEMP_DESCR[] = "Temperatura ambiente";
static const uint8_t HYDRATION_CHAR_TIMESTAMP_DESCR[] = "Timestamp";
static const uint8_t HYDRATION_CHAR_HAS_NEW_DESCR[] = "Tiene un nuevo registro?";

static uint8_t hydration_char_pending_ccc[2] = { 0x00, 0x00 };

static EventGroupHandle_t xBleStateEvents = NULL;

static hydration_record_t current_hydration_record;
static uint8_t has_pending_record = 0x00;

typedef struct {
    uint8_t water_amount[sizeof(current_hydration_record.water_amount)];
    uint8_t temperature[sizeof(current_hydration_record.temperature)];
    uint8_t timestamp[sizeof(current_hydration_record.timestamp)];
    uint8_t has_pending[sizeof(has_pending_record)];
} hydr_svc_data_buffer_t;

static hydr_svc_data_buffer_t hydr_svc_data = {};

/* Descripción completa de las características de los servicios del perfil BLE. */
const esp_gatts_attr_db_t hydration_svc_attr_table[HYDR_SVC_ATTRIBUTE_COUNT] = {
    // Declaracion del servicio de hidratacion.
    [HYDRATION_SVC_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*) &primary_service_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_128, sizeof(hydration_service_uuid), (uint8_t*) &hydration_service_uuid}},

    /* Declaracion de la caracteristica de cantidad de consumo, en mililitros. */
    [HYDR_IDX_MILILITERS_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_32, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read}},

    /* Configurar el valor de la caracteristica de mililitros. */
    [HYDR_IDX_MILILITERS_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_32, (uint8_t *)&AMOUNT_ML_GATTS_CHAR_UUID, ESP_GATT_PERM_READ,
      MAX_GATTS_CHAR_LEN_BYTES, 0, (uint8_t *) NULL}},

    /* Declaracion del descriptor de la caracteristica de cantidad de consumo. */
    [HYDR_IDX_MILILITERS_DESCR] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&char_descriptor_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, 0, NULL}},
    
    /* Declaracion de la caracteristica de temperatura. */
    [HYDR_IDX_TEMP_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read}},

    /* Configurar el valor de la caracteristica de temperatura. */
    [HYDR_IDX_TEMP_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&TEMP_GATTS_CHAR_UUID, ESP_GATT_PERM_READ,
      MAX_GATTS_CHAR_LEN_BYTES, 0, NULL}},

    /* Declaracion del descriptor de la caracteristica de temperatura*/
    [HYDR_IDX_TEMP_DESCR] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&char_descriptor_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, 0, NULL}},
    
    /* Declaracion de la caracteristica de fecha. */
    [HYDR_IDX_TIMESTAMP_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read}},

    /* Configurar el valor de la caracteristica de fecha. */
    [HYDR_IDX_TIMESTAMP_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&TIMESTAMP_GATTS_CHAR_UUID, ESP_GATT_PERM_READ,
      MAX_GATTS_CHAR_LEN_BYTES, 0, NULL}},

    /* Declaracion del descriptor de la caracteristica de fecha. */
    [HYDR_IDX_TIMESTAMP_DESCR] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&char_descriptor_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, 0, NULL}},
    
    /* Declaracion de la caracteristica de registros nuevos. */
    [HYDR_IDX_HAS_NEW_RECORDS_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_32, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read_write_notify}},

    /* Configurar el valor de la caracteristica de registros nuevos. */
    [HYDR_IDX_HAS_NEW_RECORDS_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_32, (uint8_t *)&PENDING_RECORD_COUNT_GATTS_CHAR_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      MAX_GATTS_CHAR_LEN_BYTES, 0, NULL}},

    /* Declaracion del descriptor de la caracteristica de registros pendientes. */
    [HYDR_IDX_HAS_NEW_RECORDS_DESCR] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&char_descriptor_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, 0, NULL}},

    // Client Characteristic Configuration Descriptor (CCCD) de registros nuevos.
    [HYDR_IDX_HAS_NEW_RECORDS_NTF_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(hydration_char_pending_ccc), (uint8_t *)hydration_char_pending_ccc}},
};

uint16_t get_attribute_by_hydration_handle(uint16_t attribute_handle) 
{
    uint16_t attribute_index = HYDR_SVC_ATTRIBUTE_COUNT;

    for (uint16_t index = 0; index < HYDR_SVC_ATTRIBUTE_COUNT; ++index) 
    {
        if (hydration_handle_table[index] == attribute_handle) 
        {
            attribute_index = index;
            break;
        }
    }

    return attribute_index;
}

esp_err_t hydration_svc_set_ble_state_events(EventGroupHandle_t xBleStateEventGroup)
{
    if (NULL == xBleStateEventGroup) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    xBleStateEvents = xBleStateEventGroup;

    return ESP_OK;
}

esp_err_t hydration_svc_set_value(const hydration_record_t* hydration_record)
{
    for (int i = 0; i < sizeof(uint16_t); ++i) {
        hydr_svc_data.water_amount[i] = (hydration_record->water_amount >> 8 * i) & 0xFF;
    }

    int16_t scaledTemperature = hydration_record->temperature * 100;

    for (int i = 0; i < sizeof(int16_t); ++i) {
        hydr_svc_data.temperature[i] = (scaledTemperature >> 8 * i) & 0xFF;
    }

    for (int i = 0; i < sizeof(int64_t); ++i) {
        hydr_svc_data.timestamp[i] = (hydration_record->timestamp >> 8 * i) & 0xFF;
    }

    hydr_svc_data.has_pending[0] = 0x01;

    ESP_LOG_BUFFER_HEX(TAG, hydr_svc_data.water_amount, sizeof(hydr_svc_data.water_amount));
    ESP_LOG_BUFFER_HEX(TAG, hydr_svc_data.temperature, sizeof(hydr_svc_data.temperature));
    ESP_LOG_BUFFER_HEX(TAG, hydr_svc_data.timestamp, sizeof(hydr_svc_data.timestamp));

    return ESP_OK;
}

void handle_hydration_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response)
{
    switch (attribute_index) 
    {
        case HYDR_IDX_MILILITERS_VAL:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, hydr_svc_data.water_amount, sizeof(hydr_svc_data.water_amount));
            response->attr_value.len = sizeof(hydr_svc_data.water_amount);
            break;
        case HYDR_IDX_MILILITERS_DESCR:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, HYDRATION_CHAR_ML_DESCR, sizeof(HYDRATION_CHAR_ML_DESCR));
            response->attr_value.len = sizeof(HYDRATION_CHAR_ML_DESCR);
            break;
        case HYDR_IDX_TEMP_VAL:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, hydr_svc_data.temperature, sizeof(hydr_svc_data.temperature));
            response->attr_value.len = sizeof(hydr_svc_data.temperature);
            break;
        case HYDR_IDX_TEMP_DESCR:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, HYDRATION_CHAR_TEMP_DESCR, sizeof(HYDRATION_CHAR_TEMP_DESCR));
            response->attr_value.len = sizeof(HYDRATION_CHAR_TEMP_DESCR);
            break;
        case HYDR_IDX_TIMESTAMP_VAL:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, hydr_svc_data.timestamp, sizeof(hydr_svc_data.timestamp));
            response->attr_value.len = sizeof(hydr_svc_data.timestamp);
            break;
        case HYDR_IDX_TIMESTAMP_DESCR:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, HYDRATION_CHAR_TIMESTAMP_DESCR, sizeof(HYDRATION_CHAR_TIMESTAMP_DESCR));
            response->attr_value.len = sizeof(HYDRATION_CHAR_TIMESTAMP_DESCR);
            break;
        case HYDR_IDX_HAS_NEW_RECORDS_VAL:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, &hydr_svc_data.has_pending, sizeof(hydr_svc_data.has_pending));
            response->attr_value.len = sizeof(hydr_svc_data.has_pending);
            break;
        case HYDR_IDX_HAS_NEW_RECORDS_DESCR:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, HYDRATION_CHAR_HAS_NEW_DESCR, sizeof(HYDRATION_CHAR_HAS_NEW_DESCR));
            response->attr_value.len = sizeof(HYDRATION_CHAR_HAS_NEW_DESCR);
            break;
        default:
            ESP_LOGW(TAG, "Indice de atributo no soportado para el servicio de hidratacion (%d)", attribute_index);
            break;
    }
}

esp_gatt_status_t handle_hydration_svc_write_evt(uint16_t attribute_index, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) 
{
    esp_gatt_status_t status = ESP_GATT_WRITE_NOT_PERMIT;
    size_t value_length = param->write.len;
    uint8_t* value = param->write.value; 

    switch (attribute_index) 
    {
        case HYDR_IDX_HAS_NEW_RECORDS_VAL:
            ESP_LOGI(TAG, "WRITE en HYDR_IDX_HAS_NEW_RECORDS_VAL, len = %d, value = %d", value_length, value[0]);
            if (value_length == 1) {
                hydr_svc_data.has_pending[0] = value[0];
                if (hydr_svc_data.has_pending[0] == 0x00) {
                    status = ESP_GATT_OK;
                    xEventGroupSetBits(xBleStateEvents, RECORD_SYNCHRONIZED_BIT);
                    ESP_LOGI(TAG, "Confirmado: registro fue obtenido por dispositivo periferico");
                }
            }
            break;
        case HYDR_IDX_HAS_NEW_RECORDS_NTF_CFG:
            if (value_length == 2) {
                // Configurar notificaciones para HYDR_IDX_HAS_NEW_RECORDS_VAL.
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];

                if (NOTIFY_ENABLED == descr_value) {
                    status = ESP_GATT_OK;
                    ESP_LOGI(TAG, "'Notify' activado para HYDR_IDX_HAS_NEW_RECORDS_NTF_CFG");
                    esp_ble_gatts_send_indicate(
                        gatts_if, 
                        param->write.conn_id, 
                        hydration_handle_table[HYDR_IDX_HAS_NEW_RECORDS_VAL],
                        sizeof(hydr_svc_data.has_pending), 
                        hydr_svc_data.has_pending, 
                        false
                    );
                } else if (INDICATE_ENABLED == descr_value) 
                {
                    status = ESP_GATT_OK;
                    ESP_LOGI(TAG, "'Indicate' activado para HYDR_IDX_HAS_NEW_RECORDS_NTF_CFG");

                    esp_ble_gatts_send_indicate(
                        gatts_if, 
                        param->write.conn_id, 
                        hydration_handle_table[HYDR_IDX_HAS_NEW_RECORDS_VAL], 
                        sizeof(hydr_svc_data.has_pending), 
                        hydr_svc_data.has_pending, 
                        true
                    );
                } else if (INDICATE_NOTIFY_DISABLED == descr_value)
                {
                    status = ESP_GATT_OK;
                    ESP_LOGI(TAG, "Notify/Indicate desactivado para HYDR_IDX_HAS_NEW_RECORDS_NTF_CFG");
                } else 
                {
                    ESP_LOGW(TAG, "Valor de descriptor desconocido");
                    ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
                }
            }
            break;
        default:
            ESP_LOGW(TAG, "El atributo con indice (%d) del servicio de hidratacion no soporta escritura", attribute_index);
            break;

    }

    return status;
}
