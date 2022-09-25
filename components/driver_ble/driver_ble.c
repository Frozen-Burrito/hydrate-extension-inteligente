#include <stdio.h>
#include <string.h>
#include "driver_ble.h"

static const char* TAG = "BLE";

static uint8_t pending_records_count = 0;

static uint8_t hydration_service_uuid[ESP_UUID_LEN_128] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xf5, 0x19, 0x00, 0x00,
};
static uint8_t battery_service_uuid[ESP_UUID_LEN_16] = {
    0x0f, 0x18
};

/* Descripción completa de las características de los servicios del perfil BLE. */
static const esp_gatts_attr_db_t hydration_svc_attr_table[HYDR_IDX_NB] = {
    // Declaracion del servicio de hidratacion.
    [HYDRATION_SVC_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_128, sizeof(hydration_service_uuid), (uint8_t *)&hydration_service_uuid}},

    /* Declaracion de la caracteristica de cantidad de consumo, en mililitros. */
    [HYDR_IDX_CHAR_MILILITERS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_32, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read}},

    /* Configurar el valor de la caracteristica de mililitros. */
    [HYDR_IDX_VAL_MILILITERS] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_32, (uint8_t *)&AMOUNT_ML_GATTS_CHAR_UUID, ESP_GATT_PERM_READ,
      MAX_GATTS_CHAR_LEN_BYTES, sizeof(current_record.water_amount), (uint8_t *) current_record.water_amount}},
    
    /* Declaracion de la caracteristica de temperatura. */
    [HYDR_IDX_CHAR_TEMP] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read}},

    /* Configurar el valor de la caracteristica de temperatura. */
    [HYDR_IDX_VAL_TEMP] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&TEMP_GATTS_CHAR_UUID, ESP_GATT_PERM_READ,
      MAX_GATTS_CHAR_LEN_BYTES, sizeof(current_record.temperature), (uint8_t *) current_record.temperature}},
    
    /* Declaracion de la caracteristica de fecha. */
    [HYDR_IDX_CHAR_DATE] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read}},

    /* Configurar el valor de la caracteristica de fecha. */
    [HYDR_IDX_VAL_DATE] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&DATE_GATTS_CHAR_UUID, ESP_GATT_PERM_READ,
      MAX_GATTS_CHAR_LEN_BYTES, sizeof(current_record.timestamp), (uint8_t *) current_record.timestamp}},
    
    /* Declaracion de la caracteristica de registros nuevos. */
    [HYDR_IDX_CHAR_NUM_NEW_RECORDS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_32, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read_write_notify}},

    /* Configurar el valor de la caracteristica de registros nuevos. */
    [HYDR_IDX_VAL_NUM_NEW_RECORDS] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_32, (uint8_t *)&PENDING_RECORD_COUNT_GATTS_CHAR_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      MAX_GATTS_CHAR_LEN_BYTES, sizeof(pending_records_count), (uint8_t *) &pending_records_count}},

    // Client Characteristic Configuration Descriptor (CCCD) de registros nuevos.
    [HYDR_IDX_CHAR_NUM_NEW_RECORDS_NTF_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(hydration_char_pending_ccc), (uint8_t *)hydration_char_pending_ccc}},
};

static const esp_gatts_attr_db_t battery_svc_attr_table[BAT_IDX_NB] = {
    // Declaración del servicio de nivel de batería.
    [BATTERY_SVC_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_16, sizeof(battery_service_uuid), (uint8_t *)&battery_service_uuid}},

    /* Declaración de característica para el nivel de batería. */
    [BAT_IDX_CHAR_LEVEL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read}},

    /* Valor de la característica del nivel de batería. */
    [BAT_IDX_VAL_LEVEL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&BATTERY_LVL_GATTS_CHAR_UUID, ESP_GATT_PERM_READ,
      MAX_GATTS_CHAR_LEN_BYTES, sizeof(current_record.battery_level), (uint8_t *) current_record.battery_level}},
};

static void prepare_write_event(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param);

static void exec_write_event(prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param);

static void handle_read_event(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

static uint16_t get_attribute_by_hydration_handle(uint16_t attribute_handle);

static uint16_t get_attribute_by_battery_handle(uint16_t attribute_handle);

static void handle_hydration_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response);

static void handle_battery_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response);

static void gatts_exec_read(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_read_env, esp_ble_gatts_cb_param_t* param, uint8_t *p_rsp_v, uint16_t v_len);

static esp_gatt_status_t handle_hydration_svc_write_evt(uint16_t attribute_index, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

static uint16_t get_attribute_by_hydration_handle(uint16_t attribute_handle) 
{
    uint16_t attribute_index = HYDR_IDX_NB;

    for (uint16_t index = 0; index < HYDR_IDX_NB; ++index) 
    {
        if (hydration_handle_table[index] == attribute_handle) 
        {
            attribute_index = index;
            break;
        }
    }

    return attribute_index;
}

static uint16_t get_attribute_by_battery_handle(uint16_t attribute_handle) 
{
    uint16_t attribute_index = BAT_IDX_NB;

    for (uint16_t index = 0; index < BAT_IDX_NB; ++index) 
    {
        if (battery_handle_table[index] == attribute_handle) 
        {
            attribute_index = index;
            break;
        }
    }

    return attribute_index;
}

static void handle_hydration_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response)
{
    switch (attribute_index) 
    {
        case HYDR_IDX_VAL_MILILITERS:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, current_record.water_amount, sizeof(current_record.water_amount));
            response->attr_value.len = sizeof(current_record.water_amount);
            break;
        case HYDR_IDX_VAL_TEMP:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, current_record.temperature, sizeof(current_record.temperature));
            response->attr_value.len = sizeof(current_record.temperature);
            break;
        case HYDR_IDX_VAL_DATE:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, current_record.timestamp, sizeof(current_record.timestamp));
            response->attr_value.len = sizeof(current_record.timestamp);
            break;
        case HYDR_IDX_VAL_NUM_NEW_RECORDS:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, &pending_records_count, sizeof(pending_records_count));
            response->attr_value.len = sizeof(pending_records_count);
            break;
        default:
            ESP_LOGW(TAG, "Indice de atributo no soportado para el servicio de hidratacion (%d)", attribute_index);
            break;
    }
}

static void handle_battery_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response)
{
    switch (attribute_index) 
    {
        case BAT_IDX_VAL_LEVEL:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, current_record.battery_level, sizeof(current_record.battery_level));
            response->attr_value.len = sizeof(current_record.battery_level);
            break;
        default:
            ESP_LOGW(TAG, "Indice de atributo no soportado para el servicio de bateria (%d)", attribute_index);
            break;
    }
}

static esp_gatt_status_t handle_hydration_svc_write_evt(uint16_t attribute_index, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param) 
{
    esp_gatt_status_t status = ESP_GATT_WRITE_NOT_PERMIT;
    size_t value_length = param->write.len;
    uint8_t* value = param->write.value; 

    switch (attribute_index) 
    {
        case HYDR_IDX_VAL_NUM_NEW_RECORDS:
            if (value_length == 1) {
                pending_records_count = value[0];
                if (pending_records_count == 0) {
                    status = ESP_GATT_OK;
                    xEventGroupSetBits(xBleConnectionStatus, RECORD_SYNCHRONIZED_BIT);
                    ESP_LOGI(TAG, "Confirmado: registro fue obtenido por dispositivo periferico");
                }
            }
            break;
        case HYDR_IDX_CHAR_NUM_NEW_RECORDS_NTF_CFG:
            if (value_length == 2) {
                // Configurar notificaciones para HYDR_IDX_CHAR_NUM_NEW_RECORDS.
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];

                if (descr_value == 0x0001) {
                    status = ESP_GATT_OK;
                    ESP_LOGI(TAG, "'Notify' activado para HYDR_IDX_CHAR_NUM_NEW_RECORDS_NTF_CFG");
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, 
                        hydration_handle_table[HYDR_IDX_VAL_NUM_NEW_RECORDS], sizeof(pending_records_count), &pending_records_count, false);
                } else if (descr_value == 0x0002) 
                {
                    status = ESP_GATT_OK;
                    ESP_LOGI(TAG, "'Indicate' activado para HYDR_IDX_CHAR_NUM_NEW_RECORDS_NTF_CFG");

                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, 
                        hydration_handle_table[HYDR_IDX_VAL_NUM_NEW_RECORDS], sizeof(pending_records_count), &pending_records_count, true);
                } else if (descr_value == 0x0000)
                {
                    status = ESP_GATT_OK;
                    ESP_LOGI(TAG, "Notify/Indicate desactivado para HYDR_IDX_CHAR_NUM_NEW_RECORDS_NTF_CFG");
                } else 
                {
                    ESP_LOGW(TAG, "Valor de descriptor desconocido");
                    esp_log_buffer_hex(TAG, param->write.value, param->write.len);
                }
            }
            break;
        default:
            ESP_LOGW(TAG, "Indice el atributo con indice (%d) del servicio de hidratacion no soporta escritura", attribute_index);
            break;

    }

    return status;
}
