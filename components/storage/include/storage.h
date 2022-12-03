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

/**
 * @brief Almacena un nuevo registro de hidratacion en NVS. 
 * 
 * Si ya se alcanzó el límite de registros almacenados en NVS, record reemplazará el 
 * registro de hidratación más viejo encontrado en NVS.  
*/
esp_err_t store_latest_hydration_record(const nvs_handle_t handle, const hydration_record_t* const record);

/**
 * @brief Recupera el registro más viejo almacenado en NVS.
 * 
 * Si no hay ningún registro almacenado en NVS, esta función retorna ESP_FAIL y out_record
 * no es modificado.
*/
esp_err_t retrieve_oldest_hydration_record(const nvs_handle_t handle, hydration_record_t* const out_record);

/**
 * @brief Confirma que el registro fue recuperado con éxito y puede ser sobreescrito 
 * por el almacenamiento en NVS.
*/
esp_err_t commit_retrieval(const nvs_handle_t handle);

/**
 * @brief Cierra un handle de NVS.
 */
void storage_close(const nvs_handle_t handle);

#endif
