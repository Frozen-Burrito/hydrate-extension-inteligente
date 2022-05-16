#ifndef _DRIVER_BLE_H_
#define _DRIVER_BLE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <esp_bt.h> 
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>

#include <esp_system.h> 
#include <esp_log.h> 

#define NUM_PERFILES            1
#define IDX_APP_PERFIL          0
#define ID_INST_SVC             0
#define LONGITUD_MAX_CHAR_GATTS 500
#define SIZE_DECLARACION_CHAR (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

typedef enum {
  INACTIVO,
  INICIALIZADO,
  ANUNCIANDO,
  EMPAREJANDO,
  EMPAREJADO,
  DESCONECTADO
} estado_ble_t;

estado_ble_t estado_dispositivo = EMPAREJADO;

/* Atributos GATT del servicio de hidratacion perfil. */
enum {
    IDX_SVC_HIDRATACION,    /* Índice del servicio de hidratación. */

    HIDR_IDX_CHAR_MILILITROS, /* Índice de la característica de cantidad de agua en ml. */
    HIDR_IDX_VAL_MILILITROS,  /* Índice del valor de la característica de cantidad de agua en ml. */

    HIDR_IDX_CHAR_TEMP,     /* Índice de la característica de temperatura aproximada. */
    HIDR_IDX_VAL_TEMP,      /* Índice del valor de la característica de temperatura aprox. en C. */

    HIDR_IDX_CHAR_FECHA,    /* Índice del valor de la característica de fecha (timestamp). */
    HIDR_IDX_VAL_FECHA,     

    HIDR_IDX_CHAR_REGISTROS_NUEVOS,
    HIDR_IDX_VAL_REGISTROS_NUEVOS,
    HIDR_IDX_CHAR_REGISTROS_NUEVOS_NTF_CFG,

    HIDR_IDX_NB
};

enum {
    IDX_SVC_BATERIA,        /* Índice del servicio de batería. */

    BAT_IDX_CHAR_NIVEL,
    BAT_IDX_VAL_NIVEL,
    BAT_IDX_CHAR_NIVEL_NTF_CFG,

    BAT_IDX_NB,
};

static uint8_t config_adv_lista = 0;

static char nombre_dispositivo[16] = "Hydrate-0000";

uint16_t tabla_handles[HIDR_IDX_NB + BAT_IDX_NB -1];

static uint8_t uuid_servicio_hidr[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xf5, 0x19, 0x00, 0x00,
};

static uint8_t uuid_servicio_bat[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x0f, 0x18, 0x00, 0x00,
};

static const uint16_t GATTS_SERV_UUID_BATERIA       = 0x180F;
static const uint32_t GATTS_CHAR_UUID_CANT_ML       = 0x0faf892c;
static const uint16_t GATTS_CHAR_UUID_TEMP_AMB      = 0x2a6e;
static const uint16_t GATTS_CHAR_UUID_FECHA         = 0x0fff;
static const uint32_t GATTS_CHAR_UUID_NUEVOS_REG    = 0x0faf892f;
static const uint16_t GATTS_CHAR_UUID_BATERIA       = 0x2a19;

static esp_ble_adv_params_t params_descubrimiento = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t datos_descubrimiento = {
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
    .service_uuid_len    = sizeof(uuid_servicio_hidr),
    .p_service_uuid      = uuid_servicio_hidr,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t datos_resp_scan = {
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
    .service_uuid_len    = sizeof(uuid_servicio_hidr),
    .p_service_uuid      = uuid_servicio_hidr,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_WS         = 0x181a;
static const uint16_t GATTS_CHAR_UUID_AMB_TEMP      = 0x2a6e;
static const uint16_t GATTS_CHAR_UUID_AMB_HUM       = 0x2a6f;
// static const uint16_t GATTS_CHAR_UUID_HEAT_INDEX    = 0x2a7a;

static const uint16_t uuid_servicio_primario       = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t uuid_servicio_secundario     = ESP_GATT_UUID_SEC_SERVICE;
static const uint16_t uuid_declaracion_char        = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t uuid_config_cliente_char     = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t propiedad_char_leer           = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t propiedad_char_escribir       = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t propiedad_char_leer_escribir_notif = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t ccc_char_idx_nuevos_reg[2]    = {0x00, 0x00};
static const uint8_t ccc_char_idx_nivel_bat[2]    = {0x00, 0x00};
//TODO: Cambiar esto
static const uint8_t valor_char[4]                 = {0x11, 0x22, 0x33, 0x44};

struct inst_perfil_gatts {
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

esp_err_t init_driver_ble(const char* nombre, uint16_t uuid_app);

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct inst_perfil_gatts tab_perfil_hidratacion[NUM_PERFILES] = {
    [IDX_APP_PERFIL] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE
    },
};

/* Descripción completa de las características de los servicios del perfil BLE. */
static const esp_gatts_attr_db_t gatt_db[HIDR_IDX_NB + BAT_IDX_NB] =
{
    // Declaracion del servicio de hidratacion.
    [IDX_SVC_HIDRATACION] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&uuid_servicio_primario, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(uuid_servicio_hidr), (uint8_t *)&uuid_servicio_hidr}},

    /* Declaracion de la caracteristica de mililitros. */
    [HIDR_IDX_CHAR_MILILITROS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_declaracion_char, ESP_GATT_PERM_READ,
      SIZE_DECLARACION_CHAR, SIZE_DECLARACION_CHAR, (uint8_t *)&propiedad_char_leer}},

    /* Configurar el valor de la caracteristica de mililitros. */
    [HIDR_IDX_VAL_MILILITROS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_32, (uint8_t *)&GATTS_CHAR_UUID_CANT_ML, ESP_GATT_PERM_READ,
      LONGITUD_MAX_CHAR_GATTS, sizeof(valor_char), (uint8_t *)valor_char}},
    
    /* Declaracion de la caracteristica de temperatura. */
    [HIDR_IDX_CHAR_TEMP] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_declaracion_char, ESP_GATT_PERM_READ,
      SIZE_DECLARACION_CHAR, SIZE_DECLARACION_CHAR, (uint8_t *)&propiedad_char_leer}},

    /* Configurar el valor de la caracteristica de temperatura. */
    [HIDR_IDX_VAL_TEMP] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_AMB_TEMP, ESP_GATT_PERM_READ,
      LONGITUD_MAX_CHAR_GATTS, sizeof(valor_char), (uint8_t *)valor_char}},
    
    /* Declaracion de la caracteristica de fecha. */
    [HIDR_IDX_CHAR_FECHA] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_declaracion_char, ESP_GATT_PERM_READ,
      SIZE_DECLARACION_CHAR, SIZE_DECLARACION_CHAR, (uint8_t *)&propiedad_char_leer}},

    /* Configurar el valor de la caracteristica de fecha. */
    [HIDR_IDX_VAL_FECHA] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_FECHA, ESP_GATT_PERM_READ,
      LONGITUD_MAX_CHAR_GATTS, sizeof(valor_char), (uint8_t *)valor_char}},
    
    /* Declaracion de la caracteristica de registros nuevos. */
    [HIDR_IDX_CHAR_REGISTROS_NUEVOS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_declaracion_char, ESP_GATT_PERM_READ,
      SIZE_DECLARACION_CHAR, SIZE_DECLARACION_CHAR, (uint8_t *)&propiedad_char_leer}},

    /* Configurar el valor de la caracteristica de registros nuevos. */
    [HIDR_IDX_VAL_REGISTROS_NUEVOS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_32, (uint8_t *)&GATTS_CHAR_UUID_NUEVOS_REG, ESP_GATT_PERM_READ,
      LONGITUD_MAX_CHAR_GATTS, sizeof(valor_char), (uint8_t *)valor_char}},

    // Client Characteristic Configuration Descriptor (CCCD) de registros nuevos.
    [HIDR_IDX_CHAR_REGISTROS_NUEVOS_NTF_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_config_cliente_char, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(ccc_char_idx_nuevos_reg), (uint8_t *)ccc_char_idx_nuevos_reg}},

    // Declaracion del servicio de bateria.
    [IDX_SVC_BATERIA] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_servicio_secundario, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERV_UUID_BATERIA), (uint8_t *)&GATTS_SERV_UUID_BATERIA}},

    /* Characteristic Declaration */
    [BAT_IDX_CHAR_NIVEL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_declaracion_char, ESP_GATT_PERM_READ,
      SIZE_DECLARACION_CHAR, SIZE_DECLARACION_CHAR, (uint8_t *)&propiedad_char_leer}},

    /* Characteristic Value */
    [BAT_IDX_VAL_NIVEL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_BATERIA, ESP_GATT_PERM_READ,
      LONGITUD_MAX_CHAR_GATTS, sizeof(valor_char), (uint8_t *)valor_char}},
    
    // Client Characteristic Configuration Descriptor (CCCD) de registros nuevos.
    [BAT_IDX_CHAR_NIVEL_NTF_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&uuid_config_cliente_char, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(ccc_char_idx_nivel_bat), (uint8_t *)ccc_char_idx_nivel_bat}},
};

#endif /* _DRIVER_BLE_H_ */