#include "ble_driver.h"

#include "ble_gap.h"
#include "gatt_server.h"

static const char* TAG = "BLE_DRIVER";

typedef struct {
    uint8_t water_amount[sizeof(uint16_t)];
    uint8_t temperature[sizeof(int16_t)];
    uint8_t battery_level[sizeof(uint8_t)];
    uint8_t timestamp[sizeof(int64_t)];
} record_buffer_t;

static record_buffer_t current_record = {};

static EventGroupHandle_t xBleConnectionStatus = NULL;

static uint8_t is_record_available = 0;

esp_err_t ble_driver_init(void)
{
    esp_err_t status = ESP_OK;

    ESP_LOGI(TAG, "Inicializando driver BLE");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    if (ESP_OK == status) 
    {
        xBleConnectionStatus = xEventGroupCreate();

        if (NULL == xBleConnectionStatus)
        {
            status = ESP_ERR_NO_MEM;
            ESP_LOGE(TAG, "El grupo de eventos para el estado de BLE no pudo ser creado. Asegura que haya memoria disponible en heap.");
            return status;
        }

        xEventGroupClearBits(xBleConnectionStatus, ALL_BITS);
        xEventGroupSetBits(xBleConnectionStatus, INITIALIZING_BIT);
    }
    
    if (ESP_OK == status) 
    {
        esp_bt_controller_config_t ble_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

        status = esp_bt_controller_init(&ble_config);

        if (ESP_OK != status)
        {
            ESP_LOGE(TAG, "Error al inicializar controlador BLE con configuracion por default");
        }
    }

    if (ESP_OK == status) 
    {
        status = esp_bt_controller_enable(ESP_BT_MODE_BLE);

        if (ESP_OK != status)
        {
            ESP_LOGE(TAG, "El controlador BLE no pudo ser activado");
        }
    }

    if (ESP_OK == status) 
    {
        status = esp_bluedroid_init();

        if (ESP_OK != status)
        {
            ESP_LOGE(TAG, "Error al inicializar bluedroid");
        }
    }

    if (ESP_OK == status) 
    {
        status = esp_bluedroid_enable();

        if (ESP_OK != status)
        {
            ESP_LOGE(TAG, "Error al activar bluedroid");
        }
    }

    if (ESP_OK == status) 
    {
        status = ble_gap_init();
    }

    if (ESP_OK == status) 
    {
        status = ble_gatt_server_init();
    }

    if (ESP_OK == status) 
    {
        ESP_LOGD(TAG, "Driver BLE inicializado");
    }

    return status;
}

esp_err_t ble_driver_sleep()
{
    esp_err_t status = ESP_OK;
}

esp_err_t ble_driver_shutdown(void)
{
    esp_err_t status = ESP_OK;

    if (NULL != xBleConnectionStatus)
    {
        // Notificar que BLE está siendo desactivado.
        xEventGroupClearBits(xBleConnectionStatus, ALL_BITS);
        xEventGroupSetBits(xBleConnectionStatus, SHUTTING_DOWN_BIT);
    }

    status = ble_gatt_server_shutdown(void);

    if (ESP_OK != status) 
    {
        return status;
    }

    status = ble_gap_shutdown(void);

    if (ESP_OK != status) 
    {
        return status;
    }

    status = esp_bluedroid_disable();

    if (ESP_OK != status) 
    {
        ESP_LOGW(TAG, "Error al desactivar bluedroid (%s)", esp_err_to_name(status));
        return status;
    }

    status = esp_bluedroid_deinit();

    if (ESP_OK != status) 
    {
        ESP_LOGW(TAG, "Error al des-inicializar bluedroid (%s)", esp_err_to_name(status));
        return status;
    }

    status = esp_bt_controller_disable();

    if (ESP_OK != status) 
    {
        ESP_LOGW(TAG, "Error al desactivar el controlador BT (%s)", esp_err_to_name(status));
        return status;
    }

    status = esp_bt_controller_deinit();

    if (ESP_OK != status) 
    {
        ESP_LOGW(TAG, "Error al des-inicializar el controlador BT (%s)", esp_err_to_name(status));
        return status;
    }

    status = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    if (ESP_OK != status) 
    {
        ESP_LOGW(TAG, "Error al liberar la memoria de BLE (%s)", esp_err_to_name(status));
        return status;
    }

    return ESP_OK;
}

esp_err_t ble_synchronize_hydration_record(const hydration_record_t* record, const uint32_t sync_timeout_ms)
{
    for (int i = 0; i < sizeof(uint16_t); ++i) {
        current_record.water_amount[i] = (record->water_amount >> 8 * i) & 0xFF;
    }

    int16_t scaledTemperature = record->temperature * 100;

    for (int i = 0; i < sizeof(int16_t); ++i) {
        current_record.temperature[i] = (scaledTemperature >> 8 * i) & 0xFF;
    }

    current_record.battery_level[0] = (record->battery_level) & 0xFF;

    for (int i = 0; i < sizeof(int64_t); ++i) {
        current_record.timestamp[i] = (record->timestamp >> 8 * i) & 0xFF;
    }

    ESP_LOGI(
        TAG, 
        "Registro convertido a buffers para ser sincronizado (water_amount, temperature, battery_level, timestamp):"
    );

    ESP_LOG_BUFFER_HEX(TAG, current_record.water_amount, sizeof(current_record.water_amount));
    ESP_LOG_BUFFER_HEX(TAG, current_record.temperature, sizeof(current_record.temperature));
    ESP_LOG_BUFFER_HEX(TAG, current_record.battery_level, sizeof(current_record.battery_level));
    ESP_LOG_BUFFER_HEX(TAG, current_record.timestamp, sizeof(current_record.timestamp));

    xEventGroupClearBits(xBleConnectionStatus, RECORD_SYNCHRONIZED_BIT);

    is_record_available = 1;

    //TODO: usar gatt_server
    esp_err_t notify_status = esp_ble_gatts_send_indicate(
        gatt_profile.gatt_if,
        gatt_profile.conn_id,
        hydration_handle_table[HYDR_IDX_VAL_NUM_NEW_RECORDS],
        sizeof(is_record_available),
        &is_record_available,
        false
    );

    if (ESP_OK == notify_status) 
    {
        // Esperar a recibir un WRITE_EVT en is_record_available, o a timeout.
        EventBits_t bleStatusBits = xEventGroupWaitBits(
            xBleConnectionStatus,
            RECORD_SYNCHRONIZED_BIT,
            pdFALSE,
            pdTRUE,
            pdMS_TO_TICKS(sync_timeout_ms)
        );

        if ((bleStatusBits & RECORD_SYNCHRONIZED_BIT) && is_record_available == 0) 
        {
            xEventGroupClearBits(xBleConnectionStatus, RECORD_SYNCHRONIZED_BIT);
            return ESP_OK;
        } else 
        {
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_FAIL;
}

ble_status_t ble_wait_for_state(const ble_status_t status, const bool match_exact_state, const uint32_t ms_to_wait) 
{
    // Determinar los bits que deben ser esperados.
    EventBits_t bitsToWaitFor = ble_status_to_event_bits(status);

    // Esperar a que los bits tengan el valor esperado, o el tiempo
    // de bloqueo haga timeout.
    EventBits_t resultBits = xEventGroupWaitBits(
        xBleConnectionStatus,
        bitsToWaitFor,
        pdFALSE,
        match_exact_state,
        pdMS_TO_TICKS(ms_to_wait)
    );

    ESP_LOGD(TAG, "Bits de xBleConnectionStatus: %#X", resultBits);

    ble_status_t result_status = event_bits_to_status(resultBits);

    ESP_LOGI(TAG, "Estado de BLE: %s", ble_status_to_string(result_status));

    return result_status;
}

/* UTILIDADES */
static EventBits_t ble_status_to_event_bits(const ble_status_t status) 
{
    EventBits_t bits = 0;

    switch (status) {
        case INACTIVE:
            bits |= INACTIVE;
            break;
        case INITIALIZING:
            bits |= INITIALIZING_BIT;
            break;
        case ADVERTISING:
            bits |= ADVERTISING_BIT;
            break;
        case PAIRING:
            bits |= PAIRING_BIT;
            break;
        case PAIRED:    
            bits |= PAIRED_BIT;
            break;
        case UNKNOWN:
            bits = 0;
            break;
    }

    return bits;
}

static const char* ble_status_to_string(const ble_status_t status) 
{
    switch (status) {
        case INACTIVE: return "Inactive";
        case INITIALIZING: return "Initializing";
        case ADVERTISING: return "Advertising";
        case PAIRING: return "Pairing";
        case PAIRED: return "Paired";
        case UNKNOWN: return "Unknown";
    }

    return "Unknown";
}

static ble_status_t event_bits_to_status(const EventBits_t bits) 
{
    ble_status_t status = UNKNOWN;

    if (bits & INACTIVE) {
        status = INACTIVE;
    } else if (bits & INITIALIZING_BIT) {
        status = INITIALIZING;
    } else if (bits & ADVERTISING_BIT) {
        status = ADVERTISING;
    } else if (bits & PAIRING_BIT) {
        status = PAIRING;
    } else if (bits & PAIRED_BIT) {
        status = PAIRED;
    } 

    return status;
}
