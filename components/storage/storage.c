#include <stdio.h>
#include <string.h>
#include <esp_log.h>

#include "storage.h"

static const char* TAG = "STORAGE";
static const char* recordsNvsNamespace = CONFIG_HYDRATION_RECORD_NVS_NAMESPACE;

// Nombres de llaves NVS (la longitud de una llave debe ser < NVS_KEY_NAME_MAX_SIZE-1)
static const char* recordCountKey = "record_count";
static const char* queueTailKey = "rec_queue_head"; 

static const int16_t defaultStoredRecordCount = 0;
static const uint32_t defaultQueueTailIndex = 0;

// Llaves de NVS para los datos de un registro de hidratacion. 
static const uint16_t recordFieldCount = 4;
static const char* keyWaterAmount = "wtr_amount_%d";
static const char* keyTemperature = "temperature_%d";
static const char* keyBatteryLvl  = "battery_lvl_%d";
static const char* keyTimestamp   = "timestamp_%d";

static esp_err_t get_storage_queue_tail(const nvs_handle_t handle, uint32_t* out_tail_idx);
static esp_err_t set_storage_queue_tail(const nvs_handle_t handle, const uint32_t tail_idx);
static void incr_storage_queue_tail(uint32_t* out_tail_idx);

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

static esp_err_t get_storage_queue_tail(const nvs_handle_t handle, uint32_t* out_tail_idx)
{
    esp_err_t status = ESP_FAIL;

    status = nvs_get_u32(handle, queueTailKey, out_tail_idx);

    if (ESP_ERR_NVS_NOT_FOUND == status) {
        status = set_storage_queue_tail(handle, defaultQueueTailIndex);
        *out_tail_idx = defaultQueueTailIndex;
    }

    return status;
}

static esp_err_t set_storage_queue_tail(const nvs_handle_t handle, const uint32_t tail_idx)
{
    esp_err_t status = ESP_OK;

    if (tail_idx >= MAX_STORED_RECORD_COUNT)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status) 
    {
        status = nvs_set_u32(handle, queueTailKey, tail_idx);
    }

    return status;
}

static void incr_storage_queue_tail(uint32_t* out_tail_idx)
{
    *out_tail_idx = (*out_tail_idx + 1) % MAX_STORED_RECORD_COUNT;

    ESP_LOGI(TAG, "Queue head index at = %d", *out_tail_idx);
}

esp_err_t commit_retrieval(const nvs_handle_t handle)
{
    esp_err_t status = ESP_OK;
    int16_t numOfStoredRecords = defaultStoredRecordCount;

    if (ESP_OK == status) {
        status = get_stored_count(handle, &numOfStoredRecords);
    }
    
    if (ESP_OK == status) {
        // Reducir la cuenta de registros almacenados.
        --numOfStoredRecords;
        status = set_stored_count(handle, numOfStoredRecords);
    }

    return status;
}

esp_err_t store_latest_hydration_record(const nvs_handle_t handle, const hydration_record_t* const record)
{
    esp_err_t status = ESP_OK;
    int16_t writeCount = 0;
    BaseType_t overwriteOnFull = pdTRUE;

    // Obtener el número de registros almacenados en NVS.
    int16_t recordsInStorage = 0;

    if (ESP_OK == status) 
    {
        status = get_stored_count(handle, &recordsInStorage);
    }

    BaseType_t continueWrite = (recordsInStorage < MAX_STORED_RECORD_COUNT || overwriteOnFull);

    if (ESP_OK == status && !continueWrite) {
        ESP_LOGW(TAG, "El almacenamiento esta lleno y overwriteOnFull es pdFALSE; Guardar cancelado");
        status = ESP_ERR_NVS_NOT_ENOUGH_SPACE;
    }

    uint32_t queueHeadIdx = defaultQueueTailIndex;

    if (ESP_OK == status) 
    {
        status = get_storage_queue_tail(handle, &queueHeadIdx);
        ESP_LOGI(TAG, "Almacenando registro en posicion %u", queueHeadIdx);
    }

    ESP_LOGD(
        TAG, 
        "Guardando registro [%d] { water: %i, temp: %i, bat: %i, time: %lld} en nvs...",
        queueHeadIdx, record->water_amount, record->temperature, record->battery_level, record->timestamp
    );

    if (ESP_OK == status) {
        char keyWaterAmountIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyWaterAmountIdx, NVS_KEY_NAME_MAX_SIZE, keyWaterAmount, queueHeadIdx);
        status = nvs_set_u16(handle, keyWaterAmountIdx, record->water_amount);
        ++writeCount;
    }

    if (ESP_OK == status) {
        char keyTemperatureIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyTemperatureIdx, NVS_KEY_NAME_MAX_SIZE, keyTemperature, queueHeadIdx);
        status = nvs_set_i16(handle, keyTemperatureIdx, record->temperature);
        ++writeCount;
    }

    if (ESP_OK == status) {
        char keyBatteryLvlIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyBatteryLvlIdx, NVS_KEY_NAME_MAX_SIZE, keyBatteryLvl, queueHeadIdx);
        status = nvs_set_u8(handle, keyBatteryLvlIdx, record->battery_level);
        ++writeCount;
    }

    if (ESP_OK == status) {
        char keyTimestampIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyTimestampIdx, NVS_KEY_NAME_MAX_SIZE, keyTimestamp, queueHeadIdx);
        status = nvs_set_i64(handle, keyTimestampIdx, record->timestamp);
        ++writeCount;
    }

    if (writeCount != recordFieldCount)  
    {
        status = ESP_FAIL;
    }

    if (ESP_OK == status) 
    {
        incr_storage_queue_tail(&queueHeadIdx);

        status = set_storage_queue_tail(handle, queueHeadIdx);
    }

    if (ESP_OK == status) 
    {
        // Incrementar el contador de registros guardados.
        ++recordsInStorage;
        status = set_stored_count(handle, recordsInStorage);
    }

    // Enviar a logs el resultado de almacenamiento en NVS.
    if (ESP_OK == status) {
        ESP_LOGD(TAG, "Completo: [%i/%i]", writeCount, recordFieldCount);

        status = nvs_commit(handle);

    } else {
        ESP_LOGW(
            TAG, 
            "Error de escritura en NVS [%i/%i] (%s)", 
            writeCount, 
            recordFieldCount, 
            esp_err_to_name(status)
        );
    }

    // Enviar a logs el resultado de nvs_commit.
    if (ESP_OK == status) {
        ESP_LOGD(TAG, "Commit a NVS completado: [%i/%i]", writeCount, recordFieldCount);
    } else {
        ESP_LOGW(
            TAG, 
            "Error al hacer commit de NVS [%i/%i] (%s)", 
            writeCount, 
            recordFieldCount,
            esp_err_to_name(status)
        );
    }

    return status;
}

esp_err_t retrieve_oldest_hydration_record(const nvs_handle_t handle, hydration_record_t* const out_record)
{
    esp_err_t status = ESP_OK;
    int16_t readCount = 0;

    uint32_t queueTailIdx = defaultQueueTailIndex;

    if (NULL == out_record)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status) 
    {
        status = get_storage_queue_tail(handle, &queueTailIdx);
    }

    int16_t recordsInStorage = defaultStoredRecordCount;

    if (ESP_OK == status) 
    {
        status = get_stored_count(handle, &recordsInStorage);

        if (ESP_OK == status && recordsInStorage == 0) 
        {
            ESP_LOGW(TAG, "No records stored yet");
            return ESP_FAIL;
        }
    }

    uint32_t indexOfRecord = 0;

    if (ESP_OK == status) 
    {
        // noAlmacenados = 5 - 4 = 1;
        // almacenados = 4
        // tail = 0
        // 1 = (tail + noAlmacenados) % MAX) 
        uint32_t unusedStorageSlots = MAX_STORED_RECORD_COUNT - (uint32_t) recordsInStorage;
        indexOfRecord = (queueTailIdx + unusedStorageSlots) % MAX_STORED_RECORD_COUNT;
        ESP_LOGI(TAG, "Recuperando registro en posicion %u", indexOfRecord);
    }

    if (ESP_OK == status) {
        char keyWaterAmountIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyWaterAmountIdx, NVS_KEY_NAME_MAX_SIZE, keyWaterAmount, indexOfRecord);

        uint16_t waterAmount = 0;
        status = nvs_get_u16(handle, keyWaterAmountIdx, &waterAmount);

        out_record->water_amount = waterAmount;
        readCount++;
    }

    if (ESP_OK == status) {
        char keyTemperatureIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyTemperatureIdx, NVS_KEY_NAME_MAX_SIZE, keyTemperature, indexOfRecord);

        int16_t temperature = 0;
        status = nvs_get_i16(handle, keyTemperatureIdx, &temperature);

        out_record->temperature = temperature;
        readCount++;
    }

    if (ESP_OK == status) {
        char keyBatteryLvlIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyBatteryLvlIdx, NVS_KEY_NAME_MAX_SIZE, keyBatteryLvl, indexOfRecord);

        uint8_t batLvl = 0;
        status = nvs_get_u8(handle, keyBatteryLvlIdx, &batLvl);

        out_record->battery_level = batLvl;
        readCount++;
    }

    if (ESP_OK == status) {
        char keyTimestampIdx[NVS_KEY_NAME_MAX_SIZE];
        snprintf(keyTimestampIdx, NVS_KEY_NAME_MAX_SIZE, keyTimestamp, indexOfRecord);

        int64_t timestamp = 0;
        status = nvs_get_i64(handle, keyTimestampIdx, &timestamp);

        out_record->timestamp = timestamp;
        readCount++;
    }

    // Enviar a logs el resultado del read de NVS.
    if (ESP_OK == status) {
        ESP_LOGD(TAG, "Completado [%i/%i]", readCount, recordFieldCount);
    } else {
        ESP_LOGW(
            TAG, 
            "Error de lectura de NVS [%i/%i] (%s)",
            readCount, 
            recordFieldCount,
            esp_err_to_name(status)
        );
    }

    return status;
}
