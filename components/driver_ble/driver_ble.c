#include <stdio.h>
#include <string.h>
#include "driver_ble.h"

static const char* TAG = "BLE";

#define NUMBER_OF_PROFILES 1
#define APP_PROFILE_IDX 0
#define HYDRATE_APP_ID 0xF0

#define INST_SVC_ID 0
#define MAX_GATTS_CHAR_LEN_BYTES 500
#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

typedef struct {
    uint8_t water_amount[sizeof(uint16_t)];
    uint8_t temperature[sizeof(int16_t)];
    uint8_t battery_level[sizeof(uint8_t)];
    uint8_t timestamp[sizeof(int64_t)];
} record_buffer_t;

static uint8_t pending_records_count = 0;

static record_buffer_t current_record = {};

static EventGroupHandle_t xBleConnectionStatus = NULL;

static const EventBits_t INACTIVE_BIT = ( 1 << 0 );
static const EventBits_t INITIALIZING_BIT = ( 1 << 1 );
static const EventBits_t ADVERTISING_BIT = ( 1 << 2 );
static const EventBits_t PAIRING_BIT = ( 1 << 3 );
static const EventBits_t PAIRED_BIT = ( 1 << 4 );
static const EventBits_t RECORD_SYNCHRONIZED_BIT = ( 1 << 5 );
static const EventBits_t ALL_BITS = INACTIVE | INITIALIZING_BIT | ADVERTISING_BIT | PAIRING_BIT | PAIRED_BIT | RECORD_SYNCHRONIZED_BIT;

static const uint16_t ADV_CONFIG_FLAG = (1 << 0);
static const uint16_t SCAN_RESPONSE_CONFIG_FLAG = (1 << 1);

static uint8_t adv_config_done = 0x00;

static uint16_t gatt_mtu = 23;

static uint8_t hydration_service_uuid[ESP_UUID_LEN_128] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xf5, 0x19, 0x00, 0x00,
};
static uint8_t battery_service_uuid[ESP_UUID_LEN_16] = {
    0x0f, 0x18
};

static uint16_t hydration_handle_table[HYDR_IDX_NB];
static uint16_t battery_handle_table[BAT_IDX_NB];

static const char device_name[16] = "Hydrate-0000";

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct {
    uint8_t* buffer;
    size_t length;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;
static prepare_type_env_t prepare_read_buf;

typedef struct {
    uint16_t conn_id;
    esp_gatt_if_t gatt_if;
} gatts_profile_inst_t;

static gatts_profile_inst_t gatt_profile;

static const uint32_t AMOUNT_ML_GATTS_CHAR_UUID = 0x0faf892c;
static const uint16_t TEMP_GATTS_CHAR_UUID = 0x2a6e;
static const uint16_t DATE_GATTS_CHAR_UUID = 0x0fff;
static const uint32_t PENDING_RECORD_COUNT_GATTS_CHAR_UUID = 0x0faf892f;
static const uint16_t BATTERY_LVL_GATTS_CHAR_UUID = 0x2a19;

static esp_ble_adv_params_t advertise_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t advertise_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, 
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(hydration_service_uuid),
    .p_service_uuid      = hydration_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_response_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, 
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(hydration_service_uuid),
    .p_service_uuid      = hydration_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static const uint16_t primary_service_uuid        = ESP_GATT_UUID_PRI_SERVICE;
// static const uint16_t secondary_service_uuid     = ESP_GATT_UUID_SEC_SERVICE;
static const uint16_t char_declare_uuid           = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t char_client_config_uuid     = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_property_read           = ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_property_write          = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_property_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t hydration_char_pending_ccc[2] = {0x00, 0x00};

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

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static EventBits_t ble_status_to_event_bits(const ble_status_t status);

static ble_status_t event_bits_to_status(const EventBits_t bits);

static const char* ble_status_to_string(const ble_status_t status);

static void prepare_write_event(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param);

static void exec_write_event(prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param);

static void handle_read_event(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

static uint16_t get_attribute_by_hydration_handle(uint16_t attribute_handle);

static uint16_t get_attribute_by_battery_handle(uint16_t attribute_handle);

static void handle_hydration_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response);

static void handle_battery_svc_read_evt(uint16_t attribute_index, esp_ble_gatts_cb_param_t* param, esp_gatt_rsp_t* response);

static void gatts_exec_read(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_read_env, esp_ble_gatts_cb_param_t* param, uint8_t *p_rsp_v, uint16_t v_len);

static esp_gatt_status_t handle_hydration_svc_write_evt(uint16_t attribute_index, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

static struct gatts_profile_inst hydration_profile_tab[NUMBER_OF_PROFILES] = {
    [APP_PROFILE_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE
    },
};

// Definiciones para funciones, tanto publicas como estaticas.
esp_err_t ble_driver_init()
{
    esp_err_t status = ESP_OK;

    if (status == ESP_OK) 
    {
        xBleConnectionStatus = xEventGroupCreate();

        if (NULL == xBleConnectionStatus)
        {
            status = ESP_ERR_NO_MEM;
            ESP_LOGE(TAG, "El grupo de eventos para el estado de BLE no pudo ser creado. Asegura que haya memoria disponible en heap.");
        }
    }
    
    xEventGroupClearBits(xBleConnectionStatus, ALL_BITS);
    xEventGroupSetBits(xBleConnectionStatus, INITIALIZING_BIT);

    if (status == ESP_OK) 
    {
        esp_bt_controller_config_t ble_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

        status = esp_bt_controller_init(&ble_config);

        if (status != ESP_OK)
        {
            ESP_LOGE(TAG, "Error al inicializar controlador BLE con configuracion por default");
        }
    }

    if (status == ESP_OK) 
    {
        status = esp_bt_controller_enable(ESP_BT_MODE_BLE);

        if (status != ESP_OK)
        {
            ESP_LOGE(TAG, "El controlador BLE no pudo ser activado");
        }
    }

    if (status == ESP_OK) 
    {
        status = esp_bluedroid_init();

        if (status != ESP_OK)
        {
            ESP_LOGE(TAG, "Error al inicializar bluedroid");
        }
    }

    if (status == ESP_OK) 
    {
        status = esp_bluedroid_enable();

        if (status != ESP_OK)
        {
            ESP_LOGE(TAG, "Error al activar bluedroid");
        }
    }

    if (status == ESP_OK) 
    {
        status = esp_ble_gatts_register_callback(gatts_event_handler);

        if (status != ESP_OK)
        {
            ESP_LOGE(
                TAG, 
                "No se pudo registrar el hanlder de eventos GATT (%s)", 
                esp_err_to_name(status)
            );
        }
    }

    if (status == ESP_OK) 
    {
        status = esp_ble_gap_register_callback(gap_event_handler);

        if (status != ESP_OK)
        {
            ESP_LOGE(
                TAG, 
                "No se pudo registrar el hanlder de eventos GAP (%s)", 
                esp_err_to_name(status)
            );
        }
    }

    if (status == ESP_OK) 
    {
        status = esp_ble_gatts_app_register(HYDRATE_APP_ID);

        if (status != ESP_OK)
        {
            ESP_LOGE(
                TAG, 
                "No se pudo registrar la app GATTS (%s)", 
                esp_err_to_name(status)
            );
        }
    }

    if (status == ESP_OK) 
    {
        esp_err_t set_mtu_result = esp_ble_gatt_set_local_mtu(500);

        if (set_mtu_result != ESP_OK)
        {
            ESP_LOGE(
                TAG, 
                "No se pudo configurar el MTU local (%s)", 
                esp_err_to_name(set_mtu_result)
            );
        }
    }

    if (ESP_OK == status) 
    {
        ESP_LOGD(TAG, "Driver BLE inicializado");
    }

    return status;
}

esp_err_t ble_synchronize_hydration_record(const hydration_record_t* record, const uint32_t sync_timeout_ms)
{
    for (int i = 0; i < sizeof(uint16_t); ++i) {
        current_record.water_amount[i] = (record->water_amount >> 8 * i) & 0xFF;
    }

    for (int i = 0; i < sizeof(int16_t); ++i) {
        current_record.temperature[i] = (record->temperature >> 8 * i) & 0xFF;
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

    pending_records_count = 1;

    esp_err_t notify_status = esp_ble_gatts_send_indicate(
        gatt_profile.gatt_if,
        gatt_profile.conn_id,
        hydration_handle_table[HYDR_IDX_VAL_NUM_NEW_RECORDS],
        sizeof(pending_records_count),
        &pending_records_count,
        false
    );

    if (ESP_OK == notify_status) 
    {
        // Esperar a recibir un WRITE_EVT en pending_records_count, o a timeout.
        EventBits_t bleStatusBits = xEventGroupWaitBits(
            xBleConnectionStatus,
            RECORD_SYNCHRONIZED_BIT,
            pdFALSE,
            pdTRUE,
            pdMS_TO_TICKS(sync_timeout_ms)
        );

        if ((bleStatusBits & RECORD_SYNCHRONIZED_BIT) && pending_records_count == 0) 
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

esp_err_t ble_get_pending_records_count(uint8_t* out_pending_records_count) {

    *out_pending_records_count = pending_records_count;
    return ESP_OK;
}

esp_err_t ble_set_pending_records_count(const uint8_t num_records_pending_sync, bool need_confirm)
{
    esp_err_t notify_status = ESP_OK;

    return notify_status;
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

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // Si el tipo de evento es ESP_GATTS_REG_EVT, almacenar el gatts_if de cada perfil.
    if (event == ESP_GATTS_REG_EVT) 
    {
        if (param->reg.status == ESP_GATT_OK) 
        {
            hydration_profile_tab[APP_PROFILE_IDX].gatts_if = gatts_if;

        } else 
        {
            ESP_LOGE(TAG, "No se pudo registrar el perfil, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        for (int i = 0; i < NUMBER_OF_PROFILES; i++) 
        {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == hydration_profile_tab[i].gatts_if) 
            {
                if (hydration_profile_tab[i].gatts_cb) 
                {
                    hydration_profile_tab[i].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
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
                xEventGroupClearBits(xBleConnectionStatus, ALL_BITS);
                xEventGroupSetBits(xBleConnectionStatus, ADVERTISING_BIT);
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
                xEventGroupClearBits(xBleConnectionStatus, ALL_BITS);
                xEventGroupSetBits(xBleConnectionStatus, INACTIVE_BIT);
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

            xEventGroupClearBits(xBleConnectionStatus, ALL_BITS);
            xEventGroupSetBits(xBleConnectionStatus, PAIRED_BIT);
            break;
        
        default:
            break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) 
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
        {
            gatt_profile.gatt_if = gatts_if;
            esp_err_t adv_status = esp_ble_gap_set_device_name(device_name);
            
            if (ESP_OK != adv_status) {
                ESP_LOGE(TAG, "Error al configurar el nombre del dispositivo: (%s)", esp_err_to_name(adv_status));
            }

            // Configurar datos para advertising.
            adv_status = esp_ble_gap_config_adv_data(&advertise_data);
            if (ESP_OK != adv_status){
                ESP_LOGE(TAG, "Error al configurar datos de advertising: (%s)", esp_err_to_name(adv_status));
            }

            adv_config_done |= ADV_CONFIG_FLAG;

            // Configurar los datos para la respuesta de advertising.
            adv_status = esp_ble_gap_config_adv_data(&scan_response_data);
            if (ESP_OK != adv_status){
                ESP_LOGE(TAG, "Error al confgiurar respuesta a scan: (%s)", esp_err_to_name(adv_status));
            }

            adv_config_done |= SCAN_RESPONSE_CONFIG_FLAG;

            // Crear la tabla GATT de atributos para el servicio de hidratación.
            esp_err_t create_attr_status = esp_ble_gatts_create_attr_tab(hydration_svc_attr_table, gatts_if, HYDR_IDX_NB, INST_SVC_ID);
            
            if (ESP_OK != adv_status){
                ESP_LOGE(
                    TAG, 
                    "Error al crear la tabla de atributos para el servicio de hidratacion (%s)", 
                    esp_err_to_name(create_attr_status)
                );
            }

            // Crear la tabla GATT de atributos para el servicio de batería.
            create_attr_status = esp_ble_gatts_create_attr_tab(battery_svc_attr_table, gatts_if, BAT_IDX_NB, INST_SVC_ID);
            
            if (ESP_OK != adv_status){
                ESP_LOGE(
                    TAG, 
                    "Error al crear la tabla de atributos para el servicio de bateria (%s)", 
                    esp_err_to_name(create_attr_status)
                );
            }
        }
       	    break;
               
        case ESP_GATTS_READ_EVT:
            handle_read_event(gatts_if, param);
       	    break;

        case ESP_GATTS_WRITE_EVT: 
            if (!param->write.is_prep) {
                ESP_LOGI(TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);

                uint16_t attribute_idx = get_attribute_by_hydration_handle(param->write.handle);

                if (attribute_idx < HYDR_IDX_NB) {
                    ESP_LOGI(TAG, "Hydration service WRITE");
                    handle_hydration_svc_write_evt(attribute_idx, gatts_if, param);
                    return;
                }

                if (param->write.need_rsp) 
                {
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            } else 
            {
                prepare_write_event(gatts_if, &prepare_write_env, param);
            }
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            exec_write_event(&prepare_write_env, param);
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            gatt_mtu = param->mtu.mtu;
            break;

        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle = %d", param->conf.status, param->conf.handle);
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle = %d", param->start.status, param->start.service_handle);
            break;

        case ESP_GATTS_CONNECT_EVT:
            gatt_profile.conn_id = param->connect.conn_id;
            gatt_mtu = 23;

            ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms

            //Enviar los parámetros de conección actualizados al dispositivo emparejado.
            esp_ble_gap_update_conn_params(&conn_params);

            xEventGroupClearBits(xBleConnectionStatus, ALL_BITS);
            xEventGroupSetBits(xBleConnectionStatus, PAIRED_BIT);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            xEventGroupClearBits(xBleConnectionStatus, ALL_BITS);
            xEventGroupSetBits(xBleConnectionStatus, INACTIVE_BIT);

            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = %#X", param->disconnect.reason);
            esp_ble_gap_start_advertising(&advertise_params);
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        {
            if (ESP_GATT_OK == param->add_attr_tab.status)
            {
                switch (param->add_attr_tab.num_handle) 
                {
                case HYDR_IDX_NB:
                    ESP_LOGI(TAG, "Tabla GATT para HYDR_SVC creada, handle = %d", param->add_attr_tab.num_handle);
                    memcpy(hydration_handle_table, param->add_attr_tab.handles, sizeof(hydration_handle_table));
                    esp_ble_gatts_start_service(hydration_handle_table[HYDRATION_SVC_IDX]);
                    break;
                case BAT_IDX_NB:
                    ESP_LOGI(TAG, "Tabla GATT para BAT_SVC creada, handle = %d", param->add_attr_tab.num_handle);
                    memcpy(battery_handle_table, param->add_attr_tab.handles, sizeof(battery_handle_table));
                    esp_ble_gatts_start_service(battery_handle_table[BATTERY_SVC_IDX]);
                    break;
                default: 
                    // num_handle no coincide con ninguno de los servicios. Esto es 
                    // un problema.
                    ESP_LOGW(
                        TAG, 
                        "Tabla de atributos GATT creada anormalmente, num_handle (%d) \
                        deberia ser igual a %d (servicio de hidratacion) o a %d (servicio de bateria), pero no lo es", 
                        param->add_attr_tab.num_handle, 
                        HYDR_IDX_NB,
                        BAT_IDX_NB
                    );
                    break;
                }
            } else 
            {
                ESP_LOGE(TAG, "Error al crear la tabla de atributos GATT (status = %#X)", param->add_attr_tab.status);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

static void prepare_write_event(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param)
{
    ESP_LOGI(TAG, "Prepare write EVT, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_OK;

    if (prepare_write_env->buffer == NULL) 
    {
        prepare_write_env->buffer = (uint8_t*) malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->length = 0;

        if (prepare_write_env->buffer == NULL) 
        {
            ESP_LOGE(TAG, "Gatt server prepare write no mem (%s)", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else 
    {
        if (param->write.offset > PREPARE_BUF_MAX_SIZE) 
        {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) 
        {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    
    if (param->write.need_rsp) 
    {
        esp_gatt_rsp_t* gatt_response = (esp_gatt_rsp_t*) malloc(sizeof(esp_gatt_rsp_t));

        if (NULL != gatt_response) 
        {
            gatt_response->attr_value.len = param->write.len;
            gatt_response->attr_value.handle = param->write.handle;
            gatt_response->attr_value.offset = param->write.offset;
            gatt_response->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;

            memcpy(gatt_response->attr_value.value, param->write.value, param->write.len);

            esp_err_t response_status = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_response);

            if (ESP_OK != response_status) 
            {
                ESP_LOGE(TAG, "Error enviando respuesta a evento de escritura (%s)", esp_err_to_name(response_status));
            }

            free(gatt_response);
            gatt_response = NULL;
        } else 
        {
            ESP_LOGE(TAG, "Error asignando memoria a la respuesta GATT (en %s)", __func__);
        }
    }

    if (ESP_OK == status) 
    {
        memcpy(
            prepare_write_env->buffer + param->write.offset, 
            param->write.value,
            param->write.len
        );

        prepare_write_env->length += param->write.len;
    }
}

static void exec_write_event(prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && NULL != prepare_write_env->buffer)
    {
        esp_log_buffer_hex(TAG, prepare_write_env->buffer, prepare_write_env->length);
    } else 
    {
        ESP_LOGI(TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }

    if (NULL != prepare_write_env->buffer) 
    {
        free(prepare_write_env->buffer);
        prepare_write_env->buffer = NULL;
    }

    prepare_write_env->length = 0;
}

static void gatts_exec_read(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_read_env, esp_ble_gatts_cb_param_t* param, uint8_t *p_rsp_v, uint16_t v_len)
{
    if (!param->read.need_rsp) {
        return;
    }

    uint16_t value_len = gatt_mtu - 1;
    if ((v_len - param->read.offset) < (gatt_mtu - 1))
    {
        value_len = v_len - param->read.offset;
    } else {
        ESP_LOGI(TAG, "TODO: manejar long reads");
    }

    esp_gatt_rsp_t response;
    memset(&response, 0, sizeof(esp_gatt_rsp_t));
    response.attr_value.handle = param->read.handle;
    response.attr_value.len = value_len;
    response.attr_value.offset = param->read.offset;
    memcpy(response.attr_value.value, &p_rsp_v[param->read.offset], value_len);

    esp_err_t response_status = esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &response);

    if (ESP_OK == response_status) {
        ESP_LOGI(TAG, "Respuesta enviada para evento de READ");
    }
}

static void handle_read_event(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    if (!param->read.is_long)
    {
        int attributeIdx;
        esp_gatt_rsp_t response;
        response.attr_value.len = 0;

        attributeIdx = get_attribute_by_hydration_handle(param->read.handle);

        if (attributeIdx < HYDR_IDX_NB)
        {
            ESP_LOGI(TAG, "Hydration service READ, attribute index = %d", attributeIdx);
            handle_hydration_svc_read_evt(attributeIdx, param, &response);
        }

        attributeIdx = get_attribute_by_battery_handle(param->read.handle);

        if (attributeIdx < BAT_IDX_NB)
        {
            ESP_LOGI(TAG, "Battery service READ, attribute index = %d", attributeIdx);
            handle_battery_svc_read_evt(attributeIdx, param, &response);
        }

        gatts_exec_read(gatts_if, &prepare_read_buf, param, response.attr_value.value, response.attr_value.len);
    }
}

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
