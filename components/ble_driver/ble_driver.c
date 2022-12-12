#include "ble_driver.h"

static const char* TAG = "BLE_DRIVER";

static EventGroupHandle_t xBleStateEvents = NULL;

static uint8_t is_record_available = 0;

static EventBits_t ble_status_to_event_bits(const ble_status_t status);
static ble_status_t event_bits_to_status(const EventBits_t bits);

static const char* ble_status_to_string(const ble_status_t status);

esp_err_t ble_driver_init(void)
{
    esp_err_t status = ESP_OK;

    ESP_LOGI(TAG, "Inicializando driver BLE");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    if (ESP_OK == status) 
    {
        xBleStateEvents = xEventGroupCreate();

        if (NULL == xBleStateEvents)
        {
            status = ESP_ERR_NO_MEM;
            ESP_LOGE(TAG, "El grupo de eventos para el estado de BLE no pudo ser creado. Asegura que haya memoria disponible en heap.");
            return status;
        }

        xEventGroupClearBits(xBleStateEvents, ALL_BITS);
        xEventGroupSetBits(xBleStateEvents, INITIALIZING_BIT);
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

    if (ESP_OK == status && NULL != xBleStateEvents) 
    {
        xEventGroupClearBits(xBleStateEvents, ALL_BITS);
        xEventGroupSetBits(xBleStateEvents, INITIALIZED_BIT);
    }

    if (ESP_OK == status) 
    {
        ESP_LOGD(TAG, "Driver BLE inicializado");
    }

    return status;
}

esp_err_t ble_driver_enable(const char* device_name)
{
    esp_err_t status = ESP_OK;

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
        status = ble_gatt_server_init(device_name, xBleStateEvents);
    }

    if (ESP_OK == status && NULL != xBleStateEvents)
    {
        xEventGroupClearBits(xBleStateEvents, ALL_BITS);
        xEventGroupSetBits(xBleStateEvents, ENABLED_BIT);
    }

    return status;
}

esp_err_t ble_driver_start_advertising(void)
{
    esp_err_t status = ESP_OK;

    if (ESP_OK == status) 
    {
        status = ble_gap_init(xBleStateEvents);

        if (ESP_OK != status)
        {
            ESP_LOGE(TAG, "Error al activar BLE GAP");
        }
    }

    return status;
}

esp_err_t ble_disconnect(void)
{
    esp_err_t status = ESP_OK;

    if (ESP_OK == status)
    {
        status = ble_gatt_server_disconnect();
    }

    return status;
}

esp_err_t ble_stop_advertising(void)
{
    esp_err_t status = ESP_OK;

    if (ESP_OK == status)
    {
        status = ble_gap_shutdown();
    } else 
    {
        ESP_LOGE(TAG, "BLE GAP stop advertising error (%s)", esp_err_to_name(status));
    }

    if (ESP_OK == status && NULL != xBleStateEvents)
    {
        // Notificar que el advertising ha sido detenido.
        xEventGroupClearBits(xBleStateEvents, ALL_BITS);
        xEventGroupSetBits(xBleStateEvents, ENABLED_BIT);
    }

    return status;
}

esp_err_t ble_driver_shutdown(void)
{
    esp_err_t status = ESP_OK;

    if (NULL != xBleStateEvents)
    {
        // Notificar que BLE está siendo desactivado.
        xEventGroupClearBits(xBleStateEvents, ALL_BITS);
        xEventGroupSetBits(xBleStateEvents, SHUTTING_DOWN_BIT);
    }

    status = ble_gatt_server_shutdown();

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

    if (NULL != xBleStateEvents)
    {
        // Notificar que BLE ha sido desactivado.
        xEventGroupClearBits(xBleStateEvents, ALL_BITS);
        xEventGroupSetBits(xBleStateEvents, INITIALIZED_BIT);
    }

    return status;
}

esp_err_t ble_driver_deinit(void)
{
    esp_err_t status = ESP_OK;

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

    if (NULL != xBleStateEvents)
    {
        xEventGroupClearBits(xBleStateEvents, ALL_BITS);
        xEventGroupSetBits(xBleStateEvents, INACTIVE_BIT);
    }

    return status;
}

esp_err_t ble_driver_sleep(void)
{
    esp_err_t status = ESP_OK;

    if (ESP_OK == status) 
    {
        status = esp_bt_sleep_enable();
    }

    return status;
}

esp_err_t ble_sync_battery_charge(uint8_t remaining_battery_charge)
{
    esp_err_t status = ESP_OK; 

    if (ESP_OK == status) 
    {
        status = battery_svc_set_value(remaining_battery_charge);
    }
    
    return status;
}

esp_err_t ble_synchronize_hydration_record(const hydration_record_t* record, const uint32_t sync_timeout_ms)
{
    esp_err_t status = ESP_OK;

    if (NULL == xBleStateEvents)
    {
        status = ESP_FAIL;
    }

    if (NULL == record) 
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status) 
    {
        status = hydration_svc_set_value(record);
    }

    if (ESP_OK == status) 
    {
        status = battery_svc_set_value(record->battery_level);
    }

    if (ESP_OK == status)
    {
        ESP_LOGI(
            TAG, 
            "Registro convertido a buffers para ser sincronizado (water_amount, temperature, battery_level, timestamp):"
        );

        xEventGroupClearBits(xBleStateEvents, RECORD_SYNCHRONIZED_BIT);

        is_record_available = 0x01;

        status = ble_gatt_server_indicate(
            hydration_handle_table[HYDR_IDX_HAS_NEW_RECORDS_VAL],
            sizeof(is_record_available),
            &is_record_available,
            false
        );
    }

    if (ESP_OK == status) 
    {
        // Esperar a recibir un WRITE_EVT en is_record_available, o a timeout.
        EventBits_t bleStatusBits = xEventGroupWaitBits(
            xBleStateEvents,
            RECORD_SYNCHRONIZED_BIT,
            pdFALSE,
            pdTRUE,
            pdMS_TO_TICKS(sync_timeout_ms)
        );

        if (bleStatusBits & RECORD_SYNCHRONIZED_BIT) 
        {
            xEventGroupClearBits(xBleStateEvents, RECORD_SYNCHRONIZED_BIT);
        } else 
        {
            status = ESP_ERR_TIMEOUT;
        }
    }


    return status;
}

ble_status_t ble_wait_for_state(const ble_status_t status, const bool match_exact_state, const uint32_t ms_to_wait) 
{
    if (NULL == xBleStateEvents)
    {
        return INACTIVE;
    }

    // Determinar los bits que deben ser esperados.
    EventBits_t bitsToWaitFor = ble_status_to_event_bits(status);

    // Esperar a que los bits tengan el valor esperado, o el tiempo
    // de bloqueo haga timeout.
    EventBits_t resultBits = xEventGroupWaitBits(
        xBleStateEvents,
        bitsToWaitFor,
        pdFALSE,
        match_exact_state,
        pdMS_TO_TICKS(ms_to_wait)
    );

    ESP_LOGD(TAG, "Bits de xBleStateEvents: %#X", resultBits);

    ble_status_t result_status = event_bits_to_status(resultBits);

    ESP_LOGD(TAG, "Estado de BLE: %s", ble_status_to_string(result_status));

    return result_status;
}

/* UTILIDADES */
static EventBits_t ble_status_to_event_bits(const ble_status_t status) 
{
    EventBits_t bits = 0;

    switch (status) {
        case INACTIVE:
            bits |= INACTIVE_BIT;
            break;
        case INITIALIZING:
            bits |= INITIALIZING_BIT;
            break;
        case INITIALIZED:
            bits |= INITIALIZED_BIT;
            break;
        case ENABLED:
            bits |= ENABLED_BIT;
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
        case SHUTTING_DOWN:    
            bits |= SHUTTING_DOWN_BIT;
            break;
        case SHUT_DOWN:
            bits |= SHUT_DOWN_BIT;
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
        case INITIALIZED: return "Initialized";
        case ENABLED: return "Enabled";
        case ADVERTISING: return "Advertising";
        case PAIRING: return "Pairing";
        case PAIRED: return "Paired";
        case SHUTTING_DOWN: return "Shutting down";
        case SHUT_DOWN: return "Shut down";
        case UNKNOWN: return "Unknown";
    }

    return "Unknown";
}

static ble_status_t event_bits_to_status(const EventBits_t bits) 
{
    ble_status_t status = UNKNOWN;

    if (bits & INACTIVE_BIT) {
        status = INACTIVE;
    } else if (bits & INITIALIZING_BIT) {
        status = INITIALIZING;
    } else if (bits & INITIALIZED_BIT) {
        status = INITIALIZED;
    } else if (bits & ENABLED_BIT) {
        status = ENABLED;
    } else if (bits & ADVERTISING_BIT) {
        status = ADVERTISING;
    } else if (bits & PAIRING_BIT) {
        status = PAIRING;
    } else if (bits & PAIRED_BIT) {
        status = PAIRED;
    } else if (bits & SHUTTING_DOWN_BIT) {
        status = SHUTTING_DOWN;
    } else if (bits & SHUT_DOWN_BIT) {
        status = SHUT_DOWN;
    }     

    return status;
}
