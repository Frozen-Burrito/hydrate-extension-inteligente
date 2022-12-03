#include "ble_service_device_time.h"

#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <esp_log.h>

static const char* TAG = "BLE_DEV_TIME_SVC";

const uint16_t device_time_service_uuid = 0x1847;

static const uint16_t DEVICE_TIME_CHAR_UUID = 0x2b90;

static const uint8_t DEVICE_TIME_SVC_CHAR_TIME_DESCR[] = "Fecha actual";

typedef struct {
    uint8_t device_time[sizeof(int64_t)];
} device_time_svc_data_buffer_t;

static device_time_svc_data_buffer_t device_time_svc_data = {};

const esp_gatts_attr_db_t device_time_svc_attr_table[DEV_TIME_SVC_ATTRIBUTE_COUNT] = {
    // Declaración del servicio de nivel de batería.
    [DEV_TIME_IDX] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_16, sizeof(device_time_service_uuid), (uint8_t *)&device_time_service_uuid}},

    /* Declaración de característica para el nivel de batería. */
    [DEV_TIME_IDX_DEVICE_TIME_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&char_declare_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_property_read_write}},

    /* Valor de la característica del nivel de batería. */
    [DEV_TIME_IDX_DEVICE_TIME_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&DEVICE_TIME_CHAR_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      MAX_GATTS_CHAR_LEN_BYTES, 0, NULL}},

    /* Declaracion del descriptor de la caracteristica de cantidad de consumo. */
    [DEV_TIME_IDX_DEVICE_TIME_DESCR] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&char_descriptor_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, 0, NULL}},
};

uint16_t get_attribute_by_dev_time_handle(uint16_t attribute_handle)
{
    uint16_t attribute_index = DEV_TIME_SVC_ATTRIBUTE_COUNT;

    for (uint16_t index = 0; index < DEV_TIME_SVC_ATTRIBUTE_COUNT; ++index) 
    {
        if (device_time_handle_table[index] == attribute_handle) 
        {
            attribute_index = index;
            break;
        }
    }

    return attribute_index;
}

esp_err_t device_time_svc_set_value(int64_t local_device_time_seconds)
{
    esp_err_t status = ESP_OK; 

    for (int i = 0; i < sizeof(int64_t); ++i) {
        device_time_svc_data.device_time[i] = (local_device_time_seconds >> 8 * i) & 0xFF;
    }
    
    return status;
}

void handle_device_time_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response)
{
    switch (attribute_index) 
    {
        case DEV_TIME_IDX_DEVICE_TIME_VAL:
            device_time_svc_set_value((int64_t) time(NULL));
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, device_time_svc_data.device_time, sizeof(device_time_svc_data.device_time));
            response->attr_value.len = sizeof(device_time_svc_data.device_time);
            break;
        case DEV_TIME_IDX_DEVICE_TIME_DESCR:
            memset(response->attr_value.value, 0, sizeof(response->attr_value.value));
            memcpy(response->attr_value.value, DEVICE_TIME_SVC_CHAR_TIME_DESCR, sizeof(DEVICE_TIME_SVC_CHAR_TIME_DESCR));
            response->attr_value.len = sizeof(DEVICE_TIME_SVC_CHAR_TIME_DESCR);
            break;
        default:
            ESP_LOGW(TAG, "Indice de atributo no soportado por el servicio de fecha del dispositivo (%d)", attribute_index);
            break;
    }
}

esp_gatt_status_t handle_device_time_svc_write_evt(uint16_t attribute_index, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    esp_gatt_status_t status = ESP_GATT_WRITE_NOT_PERMIT;
    size_t value_length = param->write.len;
    uint8_t* value = param->write.value; 

    ESP_LOGI(TAG, "Handling device time WRITE");
    ESP_LOG_BUFFER_HEX(TAG, value, value_length);

    switch (attribute_index) 
    {
        case DEV_TIME_IDX_DEVICE_TIME_VAL:
            if (value_length == sizeof(int64_t)) {
                int64_t seconds_since_epoch = 0;

                for (size_t i = 0; i < sizeof(int64_t); ++i) 
                {
                    device_time_svc_data.device_time[i] = value[i];
                    seconds_since_epoch |= (value[i] << 8 * i);
                }

                struct timeval now;
                
                now.tv_sec = seconds_since_epoch;

                int set_time_result = settimeofday(&now, NULL);

                if (set_time_result == 0) 
                {
                    time_t current_time;
                    struct tm timeinfo;

                    time(&current_time);
                    localtime_r(&current_time, &timeinfo);
                    char str_time_buf[64];

                    strftime(str_time_buf, sizeof(str_time_buf), "%c", &timeinfo);

                    ESP_LOGI(TAG, "Tiempo del sistema ajustado: %s", str_time_buf);
                } else
                {
                    ESP_LOGW(
                        TAG, 
                        "Error al invocar settimeofday(), con segundos = %lld (errno = %d)", 
                        seconds_since_epoch,
                        set_time_result
                    );
                }
            } else 
            {
                ESP_LOGW(TAG, "El tiempo del sistema iba a ser ajustado, pero value_length != sizeof(int64_t)");
            }
            break;
        default:
            ESP_LOGW(TAG, "El atributo con indice (%d) del servicio de fecha del dispositivo no soporta escritura", attribute_index);
            break;
    }

    return status;
}

