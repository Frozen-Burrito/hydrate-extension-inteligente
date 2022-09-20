#ifndef _STORAGE_H_
#define _STORAGE_H_

#include <freertos/FreeRTOS.h>
#include <nvs_flash.h>
#include <nvs.h>

#include "hydrate_common.h"

#define MAX_STORED_RECORD_COUNT 5

esp_err_t storage_init(void);

esp_err_t storage_open(nvs_handle_t* out_handle);

esp_err_t get_stored_count(const nvs_handle_t handle, int16_t* out_count);

esp_err_t store_hydration_record(const nvs_handle_t handle, const int16_t recordIndex, const hydration_record_t* record);

/**
 * @brief Obtiene los datos de un registro de hidratación, no modifica la cuenta
 * de registros almacenados.
 */
esp_err_t get_hydration_record(const nvs_handle_t handle, const int16_t index, hydration_record_t* out_record);

/**
 * @brief Obtiene los datos de un registro de hidratación y modifica la cuenta
 * de registros almacenados.
 */ 
esp_err_t retrieve_hydration_record(const nvs_handle_t handle, const int16_t index, hydration_record_t* out_record);

/**
 * @brief Cierra un handle de NVS.
 */
void storage_close(const nvs_handle_t handle);

#endif
