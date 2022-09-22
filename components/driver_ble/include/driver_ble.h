#ifndef _DRIVER_BLE_H_
#define _DRIVER_BLE_H_

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_bt.h>

#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>

#include "hydrate_common.h"

typedef enum {
    UNKNOWN,
    INACTIVE,
    INITIALIZING,
    ADVERTISING,
    PAIRING,
    PAIRED
} ble_status_t;

/**
 * @brief Inicializa el driver BLE con sus perfiles GAP y GATT, para luego
 * comenzar el advertising.
 */ 
esp_err_t ble_driver_init();

esp_err_t ble_synchronize_hydration_record(const hydration_record_t* record, const uint32_t sync_timeout_ms);

/**
 * @brief Obtiene el valor de la característica con el número de 
 * registros pendientes de sincronización.
 * 
 * Este valor puede ser modificado por el dispositivo cliente a 
 * través del perfil GATT.
 */ 
esp_err_t ble_get_pending_records_count(uint8_t* out_pending_records_count);

esp_err_t ble_set_pending_records_count(const uint8_t pending_records_count, bool need_confirm);

/**
 * @brief Esperar a que el status de la conexion BLE sea igual a status.
 * 
 * Si ms_to_wait es cero, el valor retornado por esta funcion sera el
 * estado actual del driver BLE, independientemente de si es el status
 * esperado.
 * 
 * Si ms_to_wait es mayor a cero y ble_wait_for_state retornó porque
 * supero este timeout, el valor retornado es el estado del driver BLE
 * en el momento que el timepo del bloqueo expiró.
 */
ble_status_t ble_wait_for_state(const ble_status_t status, const bool match_exact_state, const uint32_t ms_to_wait); 

/* Atributos GATT del servicio de hidratacion perfil. */
enum {
    HYDRATION_SVC_IDX,    /* Índice del servicio de hidratación. */

    HYDR_IDX_CHAR_MILILITERS, /* Índice de la característica de cantidad de agua en ml. */
    HYDR_IDX_VAL_MILILITERS,  /* Índice del valor de la característica de cantidad de agua en ml. */

    HYDR_IDX_CHAR_TEMP,     /* Índice de la característica de temperatura aproximada. */
    HYDR_IDX_VAL_TEMP,      /* Índice del valor de la característica de temperatura aprox. en C. */

    HYDR_IDX_CHAR_DATE,    /* Índice del valor de la característica de fecha (timestamp). */
    HYDR_IDX_VAL_DATE,     

    HYDR_IDX_CHAR_NUM_NEW_RECORDS,
    HYDR_IDX_VAL_NUM_NEW_RECORDS,
    HYDR_IDX_CHAR_NUM_NEW_RECORDS_NTF_CFG,

    HYDR_IDX_NB
};

enum {
    BATTERY_SVC_IDX,        /* Índice del servicio de batería. */

    BAT_IDX_CHAR_LEVEL,
    BAT_IDX_VAL_LEVEL,

    BAT_IDX_NB,
};

#endif // _DRIVER_BLE_H_