#include "driver_ble.h"

static const char* TAG = "DRIVER-BLE";

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // Si el tipo de evento es ESP_GATTS_REG_EVT, almacenar el gatts_if de cada perfil.
    if (event == ESP_GATTS_REG_EVT) 
    {
        if (param->reg.status == ESP_GATT_OK) 
        {
            tab_perfil_hidratacion[IDX_APP_PERFIL].gatts_if = gatts_if;

        } else 
        {
            ESP_LOGE(TAG, "No se pudo registrar el perfil, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        for (int i = 0; i < NUM_PERFILES; i++) 
        {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == tab_perfil_hidratacion[i].gatts_if) 
            {
                if (tab_perfil_hidratacion[i].gatts_cb) 
                {
                    tab_perfil_hidratacion[i].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:

            config_adv_lista &= (~ADV_CONFIG_FLAG);

            if (config_adv_lista == 0) {
                esp_ble_gap_start_advertising(&params_descubrimiento);
            }

            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:

            config_adv_lista &= (~SCAN_RSP_CONFIG_FLAG);

            if (config_adv_lista == 0) {
                esp_ble_gap_start_advertising(&params_descubrimiento);
            }

            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            // Revisar si el advertising comenzó con éxito.
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "No se pudo comenzar el advertising");
            } else 
            {
                ESP_LOGI(TAG, "Advertising comenzado");
                // estado_dispositivo = ANUNCIANDO;
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            // Revisar si el advertising fue detenido exitosamente.
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                ESP_LOGE(TAG, "Error deteniendo el advertising");
            } else 
            {
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

            // estado_dispositivo = EMPAREJADO;
            break;
        
        default:
            break;
    }
}

esp_err_t init_driver_ble(const char* nombre, uint16_t uuid_app)
{
    esp_err_t status = ESP_OK;
    // estado_dispositivo = INACTIVO;

    esp_bt_controller_config_t config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    strncpy(nombre_dispositivo, nombre, 16);
    nombre_dispositivo[16 - 1] = '\0';

    status = esp_bt_controller_init(&config);

    if (status != ESP_OK)
    {
        ESP_LOGE(TAG, "Error en la inicialización del controlador con configuración por defecto.");
        return status;
    }

    status = esp_bt_controller_enable(ESP_BT_MODE_BLE);

    if (status != ESP_OK)
    {
        ESP_LOGE(TAG, "El controlador no pudo ser activado");
        return status;
    }

    status = esp_bluedroid_init();

    if (status != ESP_OK)
    {
        ESP_LOGE(TAG, "La inicialización de Bluedroid falló.");
        return status;
    }

    status = esp_bluedroid_enable();

    if (status != ESP_OK)
    {
        ESP_LOGE(TAG, "La activación de Bluedroid falló.");
        return status;
    }

    status = esp_ble_gatts_register_callback(gatts_event_handler);

    if (status != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo registrar el receptor de calbacks GATT: %x", status);
        return status;
    }

    status = esp_ble_gap_register_callback(gap_event_handler);

    if (status != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo registrar el receptor de calbacks GAP, err: %x", status);
        return status;
    }

    status = esp_ble_gatts_app_register(uuid_app);

    if (status != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo registrar la app, err: %x", status);
        return status;
    }

    esp_err_t local_mtu_result = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_result) 
    {
        ESP_LOGE(TAG, "No se pudo configurar el nivel MTU local, err: %x", local_mtu_result);
    }

    if (ESP_OK == status) 
    {
        // estado_dispositivo = INICIALIZADO;
    }

    return status;
}

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
        {
            esp_err_t adv_status = ESP_OK;
            adv_status = esp_ble_gap_set_device_name(nombre_dispositivo);
            
            if (ESP_OK != adv_status) {
                ESP_LOGE(TAG, "No se pudo configurar el nombre del dispositivo, err: %x", adv_status);
            }

            // Config adv data.
            adv_status = esp_ble_gap_config_adv_data(&datos_descubrimiento);
            if (adv_status){
                ESP_LOGE(TAG, "Fallo en configuración de adv, err: %x", adv_status);
            }

            config_adv_lista |= ADV_CONFIG_FLAG;

            // Config scan response data
            adv_status = esp_ble_gap_config_adv_data(&datos_resp_scan);
            if (adv_status){
                ESP_LOGE(TAG, "Fallo en configuración de respuesta a scan, err: %x", adv_status);
            }

            config_adv_lista |= SCAN_RSP_CONFIG_FLAG;

            // Create the attribute table
            esp_err_t create_attr_status = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HIDR_IDX_NB + BAT_IDX_NB -1, ID_INST_SVC);
            if (create_attr_status){
                ESP_LOGE(TAG, "No se pudo crear la tabla de atributos, err: %x", create_attr_status);
            }

            // estado_dispositivo = EMPAREJANDO;
        }
       	    break;
               
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_READ_EVT");
       	    break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;

        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;

        case ESP_GATTS_CONNECT_EVT:
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

            // estado_dispositivo = EMPAREJADO;
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            // estado_dispositivo = DESCONECTADO;

            ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&params_descubrimiento);
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        {
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                ESP_LOGE(TAG, "Fallo en creación de tabla de atributos, err: 0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HIDR_IDX_NB + BAT_IDX_NB -1){
                ESP_LOGE(TAG, "create attribute table abnormally, num_handle (%d) \
                        no es igual a la HIDR_IDX_NB + BAT_IDX_NB -1(%d)", param->add_attr_tab.num_handle, HIDR_IDX_NB + BAT_IDX_NB -1);
            }
            else {
                ESP_LOGI(TAG, "Tabla de atributos creada, handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(tabla_handles, param->add_attr_tab.handles, sizeof(tabla_handles));

                esp_ble_gatts_start_service(tabla_handles[IDX_SVC_HIDRATACION]);
                esp_ble_gatts_start_service(tabla_handles[IDX_SVC_BATERIA]);
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
        case ESP_GATTS_WRITE_EVT: // Not sure about these last two
        case ESP_GATTS_EXEC_WRITE_EVT:
        default:
            break;
    }
}