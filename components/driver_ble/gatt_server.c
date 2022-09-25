#include "gatt_server.h"

#include <esp_bt.h>

#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>

#include "services/ble_service_battery.h"
#include "services/ble_service_device_time.h"
#include "services/ble_service_hydration.h"

static const char* TAG = "GATT_SERVER";

static const char device_name[16] = "Hydrate-0000";

#define NUMBER_OF_PROFILES 1
#define APP_PROFILE_IDX 0
#define HYDRATE_APP_ID 0xF0

#define SVC_INST_ID 0
#define MAX_GATTS_CHAR_LEN_BYTES 500
#define PREPARE_BUF_MAX_SIZE 1024

static uint16_t gatt_mtu = 23;

typedef struct {
    uint8_t* buffer;
    size_t length;
    uint16_t handle;
} prepare_char_access_t;

static prepare_char_access_t prepare_write_buf;
static prepare_char_access_t prepare_read_buf;

typedef struct {
    uint16_t conn_id;
    esp_gatt_if_t gatt_if;
    bool is_connected;
} gatts_profile_inst_t;

static gatts_profile_inst_t profile;

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

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst hydrate_profile_tab[NUMBER_OF_PROFILES] = {
    [APP_PROFILE_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE
    },
};

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static void prepare_write_event(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param);
static void exec_write_event(prepare_type_env_t* prepare_write_env, esp_ble_gatts_cb_param_t* param);

static void handle_read_event(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);
static void gatts_exec_read(esp_gatt_if_t gatts_if, prepare_type_env_t* prepare_read_env, esp_ble_gatts_cb_param_t* param, uint8_t *p_rsp_v, uint16_t v_len);

esp_err_t ble_gatt_server_init()
{
    esp_err_t status = ESP_OK;

    if (ESP_OK == status) 
    {
        status = esp_ble_gatts_register_callback(gatts_event_handler);

        if (ESP_OK != status)
        {
            ESP_LOGE(
                TAG, 
                "Error al registrar el hanlder de eventos GATT (%s)", 
                esp_err_to_name(status)
            );
        }
    }

    if (ESP_OK == status) 
    {
        status = esp_ble_gatts_app_register(HYDRATE_APP_ID);

        if (ESP_OK != status)
        {
            ESP_LOGE(TAG, "Error al registrar la app GATT (%s)", esp_err_to_name(status));
        }
    }

    if (ESP_OK == status) 
    {
        esp_err_t set_mtu_result = esp_ble_gatt_set_local_mtu(500);

        if (ESP_OK != status)
        {
            ESP_LOGE(
                TAG, 
                "Error al configurar el MTU local (%s)", 
                esp_err_to_name(set_mtu_result)
            );
        }
    }

    return status;
}

esp_err_t ble_gatt_server_shutdown()
{
    esp_err_t status = ESP_OK;
    esp_bt_controller_status_t bt_controller_status = esp_bt_controller_get_status();

    if (ESP_BT_CONTROLLER_STATUS_ENABLED != bt_controller_status) 
    {
        ESP_LOGW(TAG, "Intento para desactivar el servidor GATT cuando BT_status != enabled");
        return ESP_FAIL;
    }

    status = esp_ble_gatts_app_unregister();    

    if (ESP_OK != status) 
    {
        ESP_LOGE(TAG, "La app GATTS no pudo ser de-registrada (%s)", esp_err_to_name(status));
        return status;
    }

    return status;
}

esp_err_t ble_gatt_server_indicate(uint16_t attribute_handle, uint16_t value_len, uint8_t* value, bool need_confirm)
{
    esp_err_t status = ESP_OK;

    if (profile.is_connected) 
    {
        status = esp_ble_gatts_send_indicate(
            profile.gatts_if, 
            profile.conn_id, 
            attribute_handle, 
            value_len, 
            value, 
            need_confirm
        );
    } else 
    {
        status = ESP_FAIL;
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

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) 
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
        {
            gatt_profile.gatt_if = gatts_if;
            // TODO: ble_gap_set_adv_data(device_name);

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

#endif /* _GATT_SERVER_H_ */