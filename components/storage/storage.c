#include <stdio.h>
#include <string.h>
#include <esp_log.h>

#include "storage.h"

static const char* TAG = "STORAGE";
static const char* recordsNvsNamespace = CONFIG_HYDRATION_RECORD_NVS_NAMESPACE;

// Nombres de llaves NVS (la longitud de una llave debe ser < NVS_KEY_NAME_MAX_SIZE-1)
static const char* recordCountKey = "record_count";

// Llaves de NVS para los datos de un registro de hidratacion. 
static const uint16_t recordFieldCount = 4;
static const char* keyWaterAmount = "wtr_amount_%d";
static const char* keyTemperature = "temperature_%d";
static const char* keyBatteryLvl  = "battery_lvl_%d";
static const char* keyTimestamp   = "timestamp_%d";

esp_err_t storage_init(void) 
{
    // Intentar inicializar NVS.
    esp_err_t storage_err = nvs_flash_init();

    if (storage_err == ESP_ERR_NVS_NO_FREE_PAGES || storage_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // La partición de NVS fue truncada y debe ser borrada.
        ESP_ERROR_CHECK(nvs_flash_erase());
        // Intentar inicializar otra vez.
        storage_err = nvs_flash_init();
    }

    return storage_err;
}

esp_err_t storage_open(nvs_handle_t* out_handle) 
{
    ESP_LOGD(TAG, "Abriendo una handle para NVS");

    esp_err_t openResult = nvs_open(recordsNvsNamespace, NVS_READWRITE, out_handle);

    if (openResult != ESP_OK) {
        ESP_LOGE(TAG, "Error abriendo handle para NVS (%s)", esp_err_to_name(openResult));
    } else {
        ESP_LOGD(TAG, "Handle para NVS abierta con exito");
    }

    return openResult;
}

void storage_close(const nvs_handle_t handle)
{
    ESP_LOGD(TAG, "Cerrando handle de NVS");
    nvs_close(handle);
}

static esp_err_t set_stored_count(const nvs_handle_t handle, int16_t count)
{
    esp_err_t result = ESP_OK;

    // Restringir el rango de count.
    if (count < 0) {
        count = 0;
    } else if (count > MAX_STORED_RECORD_COUNT) {
        count = MAX_STORED_RECORD_COUNT;
    }

    result = nvs_set_i16(handle, recordCountKey, count);

    return result;
}

esp_err_t get_stored_count(const nvs_handle_t handle, int16_t* out_count)
{
    esp_err_t result = ESP_OK;

    // Esto produce ESP_ERR_NVS_NOT_FOUND cuando recordCountKey no ha sido
    // creada en NVS.
    result = nvs_get_i16(handle, recordCountKey, out_count);

    if (ESP_ERR_NVS_NOT_FOUND == result) {
        result = set_stored_count(handle, 0);
        *out_count = 0;
    }

    // Determinar si el número de registros obtenido de NVS está 
    // dentro del rango válido. 
    if (ESP_OK == result && *out_count > MAX_STORED_RECORD_COUNT) {
        // Correjir el valor para la cuenta de registros almacenados.
        ESP_LOGW(TAG, "El número de registros almacenados en NVS está fuera de rango (0 <= out_count <= MAX_STORED_RECORD_COUNT)");
        result = set_stored_count(handle, MAX_STORED_RECORD_COUNT);
        *out_count = MAX_STORED_RECORD_COUNT;
    }

    return result;
}

esp_err_t store_hydration_record(const nvs_handle_t handle, const int16_t recordIndex, const hydration_record_t* record)
{
    int32_t adjustedIndex = recordIndex % MAX_STORED_RECORD_COUNT;
    
    ESP_LOGD(
        TAG, 
        "Guardando registro %d { water: %i, temp: %i, bat: %i, time: %lld} en nvs...",
        recordIndex, record->water_amount, record->temperature, record->battery_level, record->timestamp
    );

    // Write record to nvs.
    esp_err_t write_status = ESP_OK;
    int16_t writeCount = 0;
    BaseType_t overwriteOnFull = pdTRUE;

    // Obtener el número de registros almacenados en NVS.
    int16_t recordsInStorage = 0;
    write_status = get_stored_count(handle, &recordsInStorage);

    BaseType_t continueWrite = (recordsInStorage < MAX_STORED_RECORD_COUNT || overwriteOnFull);

    if (ESP_OK == write_status && !continueWrite) {
        ESP_LOGW(TAG, "El almacenamiento esta lleno y overwriteOnFull es pdFALSE; Guardar cancelado");
        return write_status;
    }

    ESP_LOGD(TAG, "Status de NVS: %s", esp_err_to_name(write_status));

    if (ESP_OK == write_status) {
        char keyWaterAmountIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyWaterAmountIdx, NVS_KEY_NAME_MAX_SIZE, keyWaterAmount, adjustedIndex);
        ESP_LOGI(TAG, "Sample key name: %s", keyWaterAmountIdx);

        write_status = nvs_set_u16(handle, keyWaterAmountIdx, record->water_amount);
        writeCount++;
    }

    if (ESP_OK == write_status) {
        char keyTemperatureIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyTemperatureIdx, NVS_KEY_NAME_MAX_SIZE, keyTemperature, adjustedIndex);
        ESP_LOGI(TAG, "Sample key name: %s", keyTemperatureIdx);
        write_status = nvs_set_i16(handle, keyTemperatureIdx, record->temperature);
        writeCount++;
    }

    if (ESP_OK == write_status) {
        char keyBatteryLvlIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyBatteryLvlIdx, NVS_KEY_NAME_MAX_SIZE, keyBatteryLvl, adjustedIndex);
        ESP_LOGI(TAG, "Sample key name: %s", keyBatteryLvlIdx);
        write_status = nvs_set_u8(handle, keyBatteryLvlIdx, record->battery_level);
        writeCount++;
    }

    if (ESP_OK == write_status) {
        char keyTimestampIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyTimestampIdx, NVS_KEY_NAME_MAX_SIZE, keyTimestamp, adjustedIndex);
        ESP_LOGI(TAG, "Sample key name: %s", keyTimestampIdx);
        write_status = nvs_set_i64(handle, keyTimestampIdx, record->timestamp);
        writeCount++;
    }

    // Log the resulting write status.
    if (ESP_OK == write_status && writeCount == recordFieldCount) {
        // Incrementar el contador de registros guardados.
        ++recordsInStorage;
        set_stored_count(handle, recordsInStorage);

        ESP_LOGD(TAG, "Completo: %i/%i", writeCount, recordFieldCount);

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. 
        ESP_LOGD(TAG, "Haciendo commit de NVS...");

        write_status = nvs_commit(handle);

    } else {
        ESP_LOGW(
            TAG, 
            "Error de escritura en NVS (%i/%i): %s", 
            writeCount, 
            recordFieldCount, 
            esp_err_to_name(write_status)
        );
    }

    // Log the resulting commit status.
    if (ESP_OK == write_status) {
        ESP_LOGD(TAG, "Commit a NVS completado: %i/%i", writeCount, recordFieldCount);
    } else {
        ESP_LOGW(
            TAG, 
            "Error al hacer commit de NVS (%i/%i): %s", 
            writeCount, 
            recordFieldCount,
            esp_err_to_name(write_status)
        );
    }

    return write_status;
}

esp_err_t get_hydration_record(const nvs_handle_t handle, const int16_t index, hydration_record_t* out_record)
{
    esp_err_t readStatus = ESP_OK;
    int16_t readCount = 0;

    int16_t adjustedIndex = index % MAX_STORED_RECORD_COUNT;

    if (ESP_OK == readStatus) {
        char keyWaterAmountIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyWaterAmountIdx, NVS_KEY_NAME_MAX_SIZE, keyWaterAmount, adjustedIndex);

        uint16_t waterAmount = 0;
        readStatus = nvs_get_u16(handle, keyWaterAmountIdx, &waterAmount);

        out_record->water_amount = waterAmount;
        readCount++;
    }

    if (ESP_OK == readStatus) {
        char keyTemperatureIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyTemperatureIdx, NVS_KEY_NAME_MAX_SIZE, keyTemperature, adjustedIndex);

        int16_t temperature = 0;
        readStatus = nvs_get_i16(handle, keyTemperatureIdx, &temperature);

        out_record->temperature = temperature;
        readCount++;
    }

    if (ESP_OK == readStatus) {
        char keyBatteryLvlIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyBatteryLvlIdx, NVS_KEY_NAME_MAX_SIZE, keyBatteryLvl, adjustedIndex);

        uint8_t batLvl = 0;
        readStatus = nvs_get_u8(handle, keyBatteryLvlIdx, &batLvl);

        out_record->battery_level = batLvl;
        readCount++;
    }

    if (ESP_OK == readStatus) {
        char keyTimestampIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyTimestampIdx, NVS_KEY_NAME_MAX_SIZE, keyTimestamp, adjustedIndex);

        int64_t timestamp = 0;
        readStatus = nvs_get_i64(handle, keyTimestampIdx, &timestamp);

        out_record->timestamp = timestamp;
        readCount++;
    }

    // Log the resulting write status.
    if (ESP_OK == readStatus) {
        ESP_LOGD(TAG, "Completado: %i/%i", readCount, recordFieldCount);
    } else {
        ESP_LOGW(
            TAG, 
            "Error de lectura de NVS (%i/%i): %s",
            readCount, 
            recordFieldCount,
            esp_err_to_name(readStatus)
        );
    }

    return readStatus;
}

esp_err_t retrieve_hydration_record(const nvs_handle_t handle, const int16_t index, hydration_record_t* out_record)
{
    esp_err_t readStatus = ESP_OK;
    int16_t numOfStoredRecords = 0;

    if (ESP_OK == readStatus) {
        // Obtener el número de registros almacenados.
        readStatus = get_stored_count(handle, &numOfStoredRecords);
    }

    if (ESP_OK == readStatus && numOfStoredRecords > 0) {
        // Obtener los datos del registro.
        readStatus = get_hydration_record(handle, index, out_record);
    }
    
    if (ESP_OK == readStatus) {
        // Reducir la cuenta de registros almacenados.
        --numOfStoredRecords;
        set_stored_count(handle, numOfStoredRecords);
    }

    return readStatus;
}
