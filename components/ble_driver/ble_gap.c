#include "ble_gap.h"

#include <esp_log.h>

#include <esp_bt.h>
#include <esp_bt_defs.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_gatt_defs.h>
#include <esp_bt_device.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>

static const char* TAG = "BLE_GAP";

static EventGroupHandle_t xBleStateEvents = NULL;

const uint16_t ADV_CONFIG_FLAG = (1 << 0);
const uint16_t SCAN_RESPONSE_CONFIG_FLAG = (1 << 1);

/* Conversion: N * 1.25 ms. Range: 0x0006 (7.5 ms) to 0x0C80 (400ms)*/
#define ADV_INT_MIN 80   /* 100 ms */
#define ADV_INT_MAX 160  /* 200 ms */

static uint8_t adv_config_done = 0x00;

static uint8_t advertised_svc_uuid[ESP_UUID_LEN_128] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xf5, 0x19, 0x00, 0x00,
};

static esp_ble_adv_params_t advertise_params = {
    .adv_int_min = ADV_INT_MIN,
    .adv_int_max = ADV_INT_MAX,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t advertise_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = ADV_INT_MIN,
    .max_interval        = ADV_INT_MAX,
    .appearance          = 0x00,
    .manufacturer_len    = 0, 
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(advertised_svc_uuid),
    .p_service_uuid      = advertised_svc_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_response_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = ADV_INT_MIN,
    .max_interval        = ADV_INT_MAX,
    .appearance          = 0x00,
    .manufacturer_len    = 0, 
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(advertised_svc_uuid),
    .p_service_uuid      = advertised_svc_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

esp_err_t ble_gap_init(EventGroupHandle_t xBleStateEventGroup)
{
    esp_err_t status = ESP_OK;

    xBleStateEvents = xBleStateEventGroup; 

    if (NULL == xBleStateEvents) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    status = esp_ble_gap_register_callback(gap_event_handler);

    if (ESP_OK != status)
    {
        ESP_LOGE(
            TAG, 
            "No se pudo registrar el hanlder de eventos GAP (%s)", 
            esp_err_to_name(status)
        );
        return status;
    }

    return status;
}

esp_err_t ble_gap_set_adv_data(const char* device_name)
{
    esp_err_t status = ESP_OK;

    if (NULL == device_name)
    {
        device_name = CONFIG_BLE_DEVICE_NAME;
    }

    status = esp_ble_gap_set_device_name(device_name);
            
    if (ESP_OK != status) {
        ESP_LOGE(TAG, "Error al configurar el nombre del dispositivo: (%s)", esp_err_to_name(status));
        return status;
    }

    // Configurar datos para advertising.
    status = esp_ble_gap_config_adv_data(&advertise_data);

    if (ESP_OK != status){
        ESP_LOGE(TAG, "Error al configurar datos de advertising: (%s)", esp_err_to_name(status));
        return status;
    }

    adv_config_done |= ADV_CONFIG_FLAG;

    // Configurar los datos para la respuesta de advertising.
    status = esp_ble_gap_config_adv_data(&scan_response_data);

    if (ESP_OK != status){
        ESP_LOGE(TAG, "Error al confgiurar respuesta a scan: (%s)", esp_err_to_name(status));
        return status;
    }

    adv_config_done |= SCAN_RESPONSE_CONFIG_FLAG;

    return status;
}

esp_err_t ble_gap_start_adv(void)
{
    esp_err_t status = ESP_OK;

    status = esp_ble_gap_start_advertising(&advertise_params);

    return status;
}

esp_err_t ble_gap_shutdown(void)
{
    esp_err_t status = ESP_OK;

    status = esp_ble_gap_stop_advertising();

    return status;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:

            adv_config_done &= (~ADV_CONFIG_FLAG);

            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&advertise_params);
            }

            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:

            adv_config_done &= (~SCAN_RESPONSE_CONFIG_FLAG);

            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&advertise_params);
            }

            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            // Revisar si el advertising comenzó con éxito.
            if (ESP_BT_STATUS_SUCCESS != param->adv_start_cmpl.status)
            {
                ESP_LOGE(TAG, "No se pudo comenzar el advertising");
            } else 
            {
                xEventGroupClearBits(xBleStateEvents, ALL_BITS);
                xEventGroupSetBits(xBleStateEvents, ADVERTISING_BIT);
                ESP_LOGI(TAG, "Advertising comenzado");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            // Revisar si el advertising fue detenido exitosamente.
            if (ESP_BT_STATUS_SUCCESS != param->adv_stop_cmpl.status)
            {
                ESP_LOGE(TAG, "Error deteniendo el advertising");
            } else 
            {
                xEventGroupClearBits(xBleStateEvents, ALL_BITS);
                xEventGroupSetBits(xBleStateEvents, INACTIVE_BIT);
                ESP_LOGI(TAG, "Advertising detenido");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(
                TAG, 
                "Parámetros de conexión actualizados:\nstatus = %d\nmin_int = %d\nmax_int = %d\nconn_int = %d\nlatency = %d\ntimeout = %d",
                param->update_conn_params.status,
                param->update_conn_params.min_int,
                param->update_conn_params.max_int,
                param->update_conn_params.conn_int,
                param->update_conn_params.latency,
                param->update_conn_params.timeout
            );

            xEventGroupClearBits(xBleStateEvents, ALL_BITS);
            xEventGroupSetBits(xBleStateEvents, PAIRED_BIT);
            break;
        
        default:
            break;
    }
}
