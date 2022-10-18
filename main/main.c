#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>
#include <freertos/timers.h>

#include <esp_system.h>
#include <esp_log.h>
#include <driver/gpio.h>

// Componentes personalizados
#include "hydrate_common.h"
#include "storage.h"
#include "ble_driver.h"
#include "ble_common.h"
#include "hx711.h"
#include "mpu6050_sensor.h"
#include "battery_monitor.h"
#include "power-manager.h"

static const char* TAG = "MAIN";

#define STARTUP_DELAY_MS 3000

#define MAX_RECORD_QUEUE_LEN 5
#define MAX_SYNC_QUEUE_LEN 5
#define BATTERY_LVL_QUEUE_SIZE 1

#define NUM_SENSOR_MEASURES_PER_SECOND 4
#define MAX_SENSOR_DATA_BUF_SECONDS 5
#define MAX_SENSOR_DATA_BUF_LEN MAX_SENSOR_DATA_BUF_SECONDS * NUM_SENSOR_MEASURES_PER_SECOND
#define MAX_SENSOR_DATA_QUEUE_LEN 10

#define RECORD_RECEIVE_TIMEOUT_MS 500
#define NUM_BATTERY_CHARGE_SAMPLES 5

static QueueHandle_t xStorageQueue = NULL;
static QueueHandle_t xSyncQueue = NULL;
static QueueHandle_t xSensorDataQueue = NULL;
static QueueHandle_t xBatteryLevelQueue = NULL;

static const TickType_t power_management_period_ticks = pdMS_TO_TICKS(60 * 1000);

static esp_err_t battery_monitor_status = ESP_FAIL;

static esp_err_t send_record_to_sync(const hydration_record_t* hydrationRecord, bool* sentToStorage)
{
    if (NULL == hydrationRecord || NULL == sentToStorage) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    static const TickType_t genWaitForSpaceTimeoutMs = pdMS_TO_TICKS(50);

    // Si xBleConnectionStatus(CONNECTED_BIT) esta set, enviar el registro
    // directamente a la queue de sincronziacion BLE. Si no es asi, 
    // enviar el registro a la queue de almacenamiento.
    ble_status_t connectionStatus = ble_wait_for_state(PAIRED, true, 0);

    char strRegistroHidratacion[100] = {};
    hydration_record_to_string(strRegistroHidratacion, hydrationRecord);

    ESP_LOGI(TAG, "%s", strRegistroHidratacion);

    esp_err_t status = ESP_OK;
        
    if (PAIRED == connectionStatus) {
        // Intentar enviar el registro de hidratación generado a ser  
        // sincronizado o almacenado. Si la queue está full, esperar por
        // genWaitForSpaceTimeoutMs ms a que se libere espacio. 
        BaseType_t queueSendResult = xQueueSendToBack( 
            xSyncQueue, 
            (void*) hydrationRecord,
            genWaitForSpaceTimeoutMs
        );

        if (queueSendResult == errQUEUE_FULL) {
            status = ESP_ERR_NO_MEM;
        }
    } else {
        // Intentar enviar el registro de hidratación generado a ser  
        // sincronizado o almacenado. Si la queue está full, esperar por
        // genWaitForSpaceTimeoutMs ms a que se libere espacio. 
        BaseType_t queueSendResult = xQueueSendToBack( 
            xStorageQueue, 
            (void*) hydrationRecord,
            genWaitForSpaceTimeoutMs
        );

        if (queueSendResult == errQUEUE_FULL) {
            status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == status)
    {
        *sentToStorage = !(PAIRED == connectionStatus);
    }

    return status;
}

static void battery_monitor_timer_callback(TimerHandle_t xTimer) 
{
    battery_measurement_t battery_measurement = {};

    if (ESP_OK != battery_monitor_status) 
    {
        // Si el sensor de batería no está en un estado correcto, intentar
        // re-inicializarlo.
        battery_monitor_status = battery_monitor_init();
    }

    esp_err_t measure_status = multi_sample_battery_level(&battery_measurement, NUM_BATTERY_CHARGE_SAMPLES);

    if (ESP_OK == measure_status) 
    {
        xQueueOverwrite(xBatteryLevelQueue, &battery_measurement);

        ESP_LOGI(TAG, "Carga restante de la bateria: %d%%", battery_measurement.remaining_charge);
    }
} 

static void power_management_timer_callback(TimerHandle_t xTimer) 
{
    ESP_LOGI(TAG, "Preparing for deep sleep");

    begin_deep_sleep_when_ready();
}

static void storage_task(void* pvParameters) 
{
    // Abrir y obtener un handle para almacenamiento.
    nvs_handle_t storage_handle;

    esp_err_t nvs_status = storage_open(&storage_handle);

    if (xStorageQueue == NULL || nvs_status != ESP_OK)  {
        // Terminar inmediatamente este task, no debe seguir.
        ESP_LOGE(TAG, "NVS initialization error (%s)", esp_err_to_name(nvs_status));
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "NVS inicializado");

    int16_t indexOfLastStoredRecord = 0;
    hydration_record_t xReceivedRecord = { 0, 0, 0, 0 };

    while (true) 
    {
        int32_t recordsInStorageQueue = uxQueueMessagesWaiting(xStorageQueue);

        // Revisar el estado de la conexión al dispositivo cliente.
        ble_status_t connectionStatus = ble_wait_for_state(PAIRED, true, 0);

        // Determinar si hay registros pendientes para ser almacenados.
        BaseType_t hasRecordsPendingStorage = recordsInStorageQueue > 0;

        // Si el dispositivo está emparejado o en proceso de emparejamiento, 
        // enviar los registros pendientes de almacenamiento directamente para 
        // ser sincronizados y recuperar los registros almacenados.
        BaseType_t shouldSync = PAIRED == connectionStatus || PAIRING == connectionStatus;

        ESP_LOGD(TAG, "La tarea de almacenamiento debe sincronizar: %s", shouldSync ? "verdadero" : "falso");

        // Determinar si debe enviar los registros almacenados para 
        // sincronizarlos.
        if (shouldSync) {

            ESP_LOGD(TAG, "Numero de registros en queue de almacenamiento durante sincronizacion: %d", recordsInStorageQueue);

            // Si por casualidad hay registros que quedaron en la queue
            // para ser almacenados, enviarlos directamente a la queue de 
            // sincronización.
            for (int16_t i = 0; i < recordsInStorageQueue; ++i) {
                // Recibir un registro debería ser casi instantáneo, porque 
                // recordsInStorageQueue > 0.
                BaseType_t recordReceived = xQueueReceive(
                    xStorageQueue, 
                    &( xReceivedRecord ), 
                    (TickType_t) 0 
                );

                if (recordReceived) {
                    // Si el registro fue recuperado correctamente, enviarlo a xSyncQueue 
                    // para que sea sincronizado.
                    BaseType_t sendResult = xQueueSendToBack(
                        xSyncQueue, 
                        (void*) &( xReceivedRecord ), 
                        (TickType_t) 0
                    );

                    if (sendResult) {
                        ESP_LOGI(TAG, "Registro que iba a ser almacenado enviado directamente a sincronizacion");
                    } else {
                        ESP_LOGE(TAG, "El registro no pudo ser enviado para sincronizacion");
                    }
                }
            }

            int16_t numOfStoredRecords = 0;

            nvs_status = get_stored_count(storage_handle, &numOfStoredRecords);

            ESP_LOGI(TAG, "Numero de registros almacenados: %d", numOfStoredRecords);

            if (ESP_OK == nvs_status) {
                // Obtener y enviar cada registro almacenado para que sea sincronizado.
                for (int16_t i = 0; i < numOfStoredRecords; ++i) {

                    hydration_record_t xRecordToSync = { 0, 0, 0, 0 };

                    int16_t recordIndex = (i + indexOfLastStoredRecord + 1) % MAX_STORED_RECORD_COUNT;

                    // Hay uno o más registros por sincronizar en NVS. 
                    nvs_status = retrieve_hydration_record(storage_handle, recordIndex, &xRecordToSync);

                    if (ESP_OK == nvs_status) {
                    
                        ESP_LOGI(
                            TAG, 
                            "Registro obtenido de NVS: (indice %d) { water: %i, temp: %i, bat: %i, time: %lld}",
                            i, xRecordToSync.water_amount, xRecordToSync.temperature, 
                            xRecordToSync.battery_level, xRecordToSync.timestamp
                        );

                        // El registro pudo ser recuperado de NVS con éxito. Enviarlo
                        // a xSyncQueue para que sea sincronizado.
                        BaseType_t sendResult = xQueueSendToBack(
                            xSyncQueue, 
                            (void*) &xRecordToSync, 
                            (TickType_t) 0
                        );

                        if (sendResult == errQUEUE_FULL) {
                            ESP_LOGE(
                                TAG, 
                                "El registro fue recuperado de NVS, pero no pudo ser enviado (errQUEUE_FULL)"
                            );
                        } else {
                            ESP_LOGD(TAG, "Registro recuperado de NVS y enviado para ser sincronizado");
                        }

                    } else {
                        // Hubo un error al intentar obtener el registro de hidratacion
                        // almacenado.
                        ESP_LOGE(TAG, "Error al recuperar registro desde NVS (%s)", esp_err_to_name(nvs_status)); 
                    }
                }
            } else {
                // Hubo un error al intentar obtener la cuenta de registros
                // almacenados.
                ESP_LOGE(TAG, "Error obteniendo el numero de registros almacenados (%s)", esp_err_to_name(nvs_status));
            }

        } else if (hasRecordsPendingStorage) {
            // No debe sincronizar los registros. Almacenar todos los registros
            // que estén pendientes en xStorageQueue.
            for (int16_t i = 0; i < recordsInStorageQueue; ++i) {
                // Esperar a recibir un nuevo registro para almacenar.
                // Cuando ocurra esto, activar el modo de "storage".
                BaseType_t queueHadItems = xQueuePeek(
                    xStorageQueue, 
                    &( xReceivedRecord ), 
                    // Recibir un elemento debería ser casi instantáneo, porque hasRecordsPendingStorage es pdTRUE.
                    (TickType_t) 0 
                );

                BaseType_t recordWasStored = pdFALSE;

                if (queueHadItems) {
                    // Ajustar el índice para que sea un valor en (0, MAX_STORED_RECORD_COUNT) 
                    // y tome en cuenta el índice último registro almacenado.
                    int16_t recordIndex = (i + indexOfLastStoredRecord + 1) % MAX_STORED_RECORD_COUNT;

                    nvs_status = store_hydration_record(storage_handle, recordIndex, &xReceivedRecord);

                    recordWasStored = ESP_OK == nvs_status;

                    if (recordWasStored) {
                        if (i == (recordsInStorageQueue - 1)) {
                            // Cuando el último registro es almacenado, actualizar
                            // el offset para los siguientes registros.
                            indexOfLastStoredRecord = recordIndex;
                        }

                        ESP_LOGI(TAG, "Registro almacenado en NVS, con indice = %i", recordIndex);
                    } else {
                        // Por algún motivo, el almacenamiento en NVS produjo un error.
                        ESP_LOGE(TAG, "Error al guardar registro en NVS (%s)", esp_err_to_name(nvs_status));
                    }
                } else {
                    ESP_LOGE(TAG, "Esperaba obtener un registro para almacenar, pero la queue estaba vacia");
                }

                if (recordWasStored) {
                    BaseType_t recordRemovedFromQueue = xQueueReceive(
                        xStorageQueue,
                        &( xReceivedRecord ), 
                        (TickType_t) 0
                    );

                    if (!recordRemovedFromQueue) {
                        ESP_LOGE(TAG, "El registro fue almacenado, pero no fue removido de xStorageQueue (%s)", esp_err_to_name(nvs_status));
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(RECORD_RECEIVE_TIMEOUT_MS));
    }

    // Close handle for NVS.
    storage_close(storage_handle);

    vTaskDelete(NULL);
}

static void record_generator_task(void* pvParameters) 
{
    int32_t numRecordsSentForSync = 0;
    int32_t numRecordsSentForStorage = 0; 

    static const TickType_t waitForReadingsTimeoutMs = pdMS_TO_TICKS(10000);

    static sensor_measures_t readingsBuffer[MAX_SENSOR_DATA_BUF_LEN] = {};
    size_t indexOfLatestSensorReadings = 0;
    bool has_reached_required_sample_count = false;

    while (true) 
    {
        BaseType_t measurementsWereReceived = xQueueReceive(
            xSensorDataQueue,
            &readingsBuffer[indexOfLatestSensorReadings],
            waitForReadingsTimeoutMs
        );

        measurementsWereReceived = pdPASS;

        if (measurementsWereReceived) 
        {
            hydration_record_t hydrationRecord = {};
            
            // Algoritmo para determinar si las mediciones en readingsBuffer
            // son indicativas de consumo de agua.
            bool was_hydration_recorded = has_reached_required_sample_count && hydration_record_from_measures(readingsBuffer, MAX_SENSOR_DATA_BUF_LEN, &hydrationRecord);

            ESP_LOGI(TAG, "Do measures represent hydration: %s", (was_hydration_recorded ? "yes" : "no" ));

            if (was_hydration_recorded) 
            {
                // Obtener la carga restante de la batería, para asociarla
                // con el registro de hidratación.
                battery_measurement_t battery_measurement = {};

                BaseType_t batteryDataAvailable = xQueuePeek(
                    xBatteryLevelQueue,
                    &battery_measurement,
                    (TickType_t) 0
                );

                if (batteryDataAvailable == pdPASS) 
                {
                    hydrationRecord.battery_level = battery_measurement.remaining_charge; 
                }

                bool wasRecordStored = false;

                esp_err_t status = send_record_to_sync(&hydrationRecord, &wasRecordStored);

                char* targetMsg = "sincronizacion";

                if (ESP_OK == status) 
                {            
                    if (wasRecordStored) 
                    {
                        targetMsg = "almacenamiento";
                        ++numRecordsSentForStorage;
                    } else 
                    {
                        ++numRecordsSentForSync;
                    }
                }

                int32_t totalRecordsSent = numRecordsSentForStorage + numRecordsSentForSync;

                if (ESP_OK == status) 
                {
                    ESP_LOGI(
                        TAG, 
                        "Nuevo registro de hidratacion enviado para %s, total %d", 
                        targetMsg, 
                        totalRecordsSent
                    );
                } else {
                    ESP_LOGW(
                        TAG, 
                        "Un nuevo registro %d no pudo ser enviado para %s (errQUEUE_FULL)",
                        totalRecordsSent,
                        targetMsg
                    );
                }
            }

            if (indexOfLatestSensorReadings >= MAX_SENSOR_DATA_BUF_LEN - 1)
            {
                has_reached_required_sample_count = true;
            }

            ++indexOfLatestSensorReadings;
            indexOfLatestSensorReadings = (indexOfLatestSensorReadings >= MAX_SENSOR_DATA_BUF_LEN)
                ? indexOfLatestSensorReadings % MAX_SENSOR_DATA_BUF_LEN
                : indexOfLatestSensorReadings;

        } else {
            ESP_LOGW(TAG, "No hay mediciones de sensores para generar registros de hidratacion");
        }
    }

    vTaskDelete(NULL);
}

static void communication_task(void* pvParameters)
{
    // El watchdog para evitar sincronizar registros infinitamente.
    static const uint16_t maxRecordsToSync = MAX_SYNC_QUEUE_LEN + MAX_STORED_RECORD_COUNT;
    static const uint16_t MAX_SYNC_ATTEMPTS = 5;
    static const TickType_t SYNC_FAILED_BACKOFF_DELAY = pdMS_TO_TICKS(5000);

    static const uint32_t waitForPairedStatusTimeoutMS = 5000;
    static const uint32_t recordReadTimeoutMS = 10 * 1000; 

    hydration_record_t xRecordToSync = { 0, 0, 0, 0 };

    // Intentar inicializar el driver BLE.
    esp_err_t ble_error = ble_driver_init("Hydrate-0001");

    if (NULL == xSyncQueue || ble_error != ESP_OK)  {
        // Error fatal, terminar inmediatamente este task.
        ESP_LOGE(TAG, "Error de inicializacion del driver de BLE (%s)", esp_err_to_name(ble_error));
        vTaskDelete(NULL);
    }

    while (true) 
    {
        ble_status_t status = ble_wait_for_state(PAIRED, true, waitForPairedStatusTimeoutMS);

        bool isPaired = PAIRED == status;

        if (isPaired) {

            int16_t synchronizedRecordsCount = 0;
            int16_t syncAttemptsForCurrentRecord = 0;

            int32_t totalRecordsPendingSync = uxQueueMessagesWaiting(xSyncQueue);

            BaseType_t hasRemainingRecordsToSync = totalRecordsPendingSync > 0;

            // Iterar mientras la extensión esté PAIRED, queden registros de hidratación 
            // por sincronizar, los intentos de sincronización no superen a MAX_SYNC_ATTEMPTS
            //  y la cuenta total de registros sincronizados no supere a maxRecordsToSync.      
            while (isPaired && hasRemainingRecordsToSync && 
                   syncAttemptsForCurrentRecord < MAX_SYNC_ATTEMPTS && synchronizedRecordsCount < maxRecordsToSync
            ) {
                // Obtener el siguiente registro por sincronizar, sin removerlo 
                // de la queue (un registro es removido hasta que haya sincronizado
                // con éxito).
                // Esta operación debería ser prácticamente instantánea, porque 
                // hasRecordsToSync es pdTRUE y esta es la única task que puede 
                // recibir elementos de xSyncQueue.
                BaseType_t wasRecordReceived = xQueuePeek(
                    xSyncQueue, 
                    &( xRecordToSync ), 
                    pdMS_TO_TICKS(1000) //TODO: determinar si este timeout es necesario.
                );

                if (wasRecordReceived) {
                    // El registro de hidratación fue recibido con éxito desde
                    // xSyncQueue. Es posible sincronizarlo a través de BLE.

                    // El modificar el perfil GATT BLE con los datos del
                    // registro de hidratación. Bloquear esta task por un 
                    // momento, hasta que el cliente confirme que obtuvo el registro o
                    // timeout expire. 
                    esp_err_t sync_status = ble_synchronize_hydration_record(&xRecordToSync, recordReadTimeoutMS);

                    // Determinar si el registro pudo ser sincronizado, según sync_status:
                    if (ESP_OK == sync_status) {
                        // El registro de hidratación fue recibido por el
                        // cliente con éxito. Indicarlo con una variable "bandera":
                        ++synchronizedRecordsCount;
                        syncAttemptsForCurrentRecord = 0;

                        // Si el registro pudo ser sincronizado y recibido por
                        // el cliente, removerlo de xSyncQueue para señalizar
                        // que ya no es necesario mantenerlo.
                        xQueueReceive(
                            xSyncQueue,
                            &( xRecordToSync ), 
                            pdMS_TO_TICKS(1000) //TODO: determinar si este timeout es necesario.
                        );

                        ESP_LOGI(TAG, "Registro obtenido por el dispositivo periferico, restantes = %i", (totalRecordsPendingSync - synchronizedRecordsCount));

                    } else if (ESP_ERR_TIMEOUT == sync_status) {
                        // Si el registro no fue obtenido por el dispositivo periférico
                        // (el timeout expiró sin confirmación), incrementar la cuenta de intentos.
                        ++syncAttemptsForCurrentRecord;
                        ESP_LOGW(
                            TAG, 
                            "Registro sincronizado, pero no fue obtenido por dispositivo periferico, restantes = %i",
                            (totalRecordsPendingSync - synchronizedRecordsCount)
                        );
                    } else {
                        ESP_LOGW(
                            TAG, 
                            "Error al sincronizar registro con BLE (%s)",
                            esp_err_to_name(sync_status)
                        );
                    }

                    // Revisar si el dispotivo periférico todavía está emparejado.
                    ble_status_t status = ble_wait_for_state(PAIRED, true, 0);
                    isPaired = PAIRED == status;

                    hasRemainingRecordsToSync = synchronizedRecordsCount < totalRecordsPendingSync;
                }
            }

            if (syncAttemptsForCurrentRecord >= MAX_SYNC_ATTEMPTS) {
                vTaskDelay(SYNC_FAILED_BACKOFF_DELAY);
                syncAttemptsForCurrentRecord = 0;
            }
        }

        // Esperar un momento antes de volver a revisar el estado de 
        // la conexión, evitar acaparar todo el tiempo del scheduler 
        // en revisar la conexion.
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    vTaskDelete(NULL);
}

static void read_sensors_measurements_task(void* pvParameters) 
{
    static TickType_t measurementIntervalMs = pdMS_TO_TICKS(1000 / NUM_SENSOR_MEASURES_PER_SECOND);
    static const TickType_t batteryMeasurePeriodMs = pdMS_TO_TICKS(10 * 1000);

    static hx711_t hx711_sensor = {
        .data_out = CONFIG_HX711_DATA_OUT_GPIO,
        .pd_sck = CONFIG_HX711_PD_SCK_GPIO,
        .gain = HX711_GAIN_A_64
    };

    esp_err_t hx711_init_status = ESP_OK;
    esp_err_t mpu6050_init_status = ESP_OK;
    esp_err_t hx711_read_status = ESP_OK;
    esp_err_t mpu6050_read_status = ESP_OK;

    // Inicializar los sensores.
    hx711_init_status = hx711_init(&hx711_sensor);
    mpu6050_init_status = mpu6050_init(true);
    battery_monitor_status = battery_monitor_init();

    ESP_LOGI(TAG, "HX711 sensor status (%s)", esp_err_to_name(hx711_init_status));
    ESP_LOGI(TAG, "MPU6050 sensor status (%s)", esp_err_to_name(mpu6050_init_status));
    ESP_LOGI(TAG, "Battery monitor status (%s)", esp_err_to_name(battery_monitor_status));

    if (ESP_OK == mpu6050_init_status)
    {
        uint8_t mpu6050_deviceid;
        mpu6050_get_i2c_id(&mpu6050_deviceid);
        ESP_LOGI(TAG, "MPU6050 sensor device ID = %2x", mpu6050_deviceid);
    }

    TimerHandle_t xBatteryLevelTimer = xTimerCreate(
        "battery_level_timer",
        batteryMeasurePeriodMs,
        pdTRUE,
        NULL,
        battery_monitor_timer_callback
    );

    // Comenzar el timer periodico para obtener mediciones del
    // nivel de bateria.
    xTimerStart(xBatteryLevelTimer, (TickType_t) 0);

    while (true) {
        // Obtener lecturas de los sensores.
        sensor_measures_t sensor_data = {};

        if (ESP_OK == hx711_init_status && ESP_OK == mpu6050_init_status) 
        {
            start_measurement_period(&sensor_data);
        } 

        if (ESP_OK == hx711_init_status) 
        {
            hx711_read_status = hx711_read_average(
                &hx711_sensor, 
                CONFIG_HX711_AVG_SAMPLES_COUNT, 
                &sensor_data.raw_weight_data
            );

            if (ESP_OK == hx711_read_status)
            {
                uint16_t volume_ml = hx711_volume_ml_from_measurement(&sensor_data.raw_weight_data);
                sensor_data.volume_ml = volume_ml;
            }
            if (ESP_ERR_TIMEOUT == hx711_read_status) 
            {
                ESP_LOGW(TAG, "HX711 data read timeout");
            } else if (ESP_OK != hx711_read_status) 
            {
                ESP_LOGW(TAG, "HX711 read error (%s)", esp_err_to_name(hx711_read_status));
            }
        } 

        if (ESP_OK == mpu6050_init_status) 
        {
            // Obtener mediciones del MPU6050.
            mpu6050_read_status = mpu6050_get_measurements(&sensor_data);

            if (ESP_OK != mpu6050_read_status) 
            {
                ESP_LOGW(TAG, "Error obtaining data from MPU6050 (%s)", esp_err_to_name(mpu6050_read_status));
            }
        }

        if (ESP_OK == hx711_init_status && ESP_OK == mpu6050_init_status) 
        {
            end_measurement_period(&sensor_data);
        } 

        if (ESP_OK == hx711_read_status && ESP_OK == mpu6050_read_status && NULL != xSensorDataQueue) 
        {
            ESP_LOGI(
                TAG, 
                "Lecturas de sensores: { raw_weight: %i, volume: %uml, accel: (%.2f, %.2f, %.2f), gyro: (%.2f, %.2f, %.2f), temp: %.2f, interval:%lld-%lld ms}", 
                sensor_data.raw_weight_data,
                sensor_data.volume_ml,
                sensor_data.acceleration.x, sensor_data.acceleration.y, sensor_data.acceleration.z, 
                sensor_data.gyroscope.x, sensor_data.gyroscope.y, sensor_data.gyroscope.z, 
                sensor_data.temperature,
                sensor_data.start_time_ms,
                sensor_data.end_time_ms
            );

            BaseType_t dataWasSent = xQueueSendToBack(
                xSensorDataQueue,
                &sensor_data,
                (TickType_t) 0
            );

            if (!dataWasSent) {
                ESP_LOGW(TAG, "Las lecturas de los sensores no pudieron ser enviadas");
            }
        } else {
            ESP_LOGW(TAG, "Something went wrong while getting sensor readings (hx711 %s) (mpu6050 %s) (xSensorDataQueue != NULL ? %d)", esp_err_to_name(hx711_read_status), esp_err_to_name(mpu6050_read_status), (xSensorDataQueue != NULL));
        }

        vTaskDelay(measurementIntervalMs);
    }

    // Liberar recursos
    xTimerStop(xBatteryLevelTimer, (TickType_t) 0);

    hx711_set_power(&hx711_sensor, true);
    mpu6050_free_resources(true);

    vTaskDelete(NULL);
}

static void setup_power_management(void) 
{
    after_wakeup();

    xTimerCreate(
        "power_management_timer",
        power_management_period_ticks,
        pdTRUE,
        NULL,
        power_management_timer_callback
    );
}

static esp_err_t setup(void) 
{
    // Para fines de prueba, mostrar la memoria heap disponible.
    ESP_LOGI(TAG, "Size minimo de heap libre (antes de setup()): %u bytes", esp_get_minimum_free_heap_size());

    ESP_LOGD(TAG, "Configurando power management");
    setup_power_management();

    ESP_LOGD(TAG, "Inicializando NVS");

    //NOTA: storage_init() está aquí porque si lo dejaba com oparte de 
    // una task y otra task la interrumpía, la inicialización produce
    // un LoadProhibited error. No estoy seguro sobre las implicaciones 
    // que tiene el preempt con storage_init.
    esp_err_t nvs_status = storage_init();
    
    if (ESP_OK != nvs_status)
    {
        ESP_LOGW(
            TAG, 
            "NVS no pudo ser inicializado"
        );
        return nvs_status;
    }

    // Inicializar los objetos de FreeRTOS usados para controlar 
    // los procesos de almacenar y obtener registros de hidratación. 
    ESP_LOGD(TAG, "Creando objetos de FreeRTOS");
    xStorageQueue = xQueueCreate( MAX_RECORD_QUEUE_LEN, sizeof(hydration_record_t) );
    xSyncQueue = xQueueCreate( MAX_SYNC_QUEUE_LEN, sizeof(hydration_record_t) );
    xSensorDataQueue = xQueueCreate( MAX_SENSOR_DATA_QUEUE_LEN, sizeof(sensor_measures_t) );
    xBatteryLevelQueue = xQueueCreate( BATTERY_LVL_QUEUE_SIZE, sizeof(battery_measurement_t) );

    if (NULL == xStorageQueue || NULL == xSyncQueue || NULL == xSensorDataQueue || NULL == xBatteryLevelQueue) 
    {
        ESP_LOGW(
            TAG, 
            "Uno o mas objetos de FreeRTOS no pudo ser creado. Asegura que haya memoria disponible en heap."
        );
        return ESP_ERR_NO_MEM;
    }

    // Configurar todos los pines de GPIO usados.
    ESP_LOGD(TAG, "Configurando I/O");

    // Para fines de prueba, mostrar la memoria heap disponible.
    ESP_LOGI(TAG, "Size minimo de heap libre (despues de setup()): %u bytes", esp_get_minimum_free_heap_size());

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t setup_status = setup();

    if (ESP_OK == setup_status) 
    {
        // Dar tiempo para reaccionar si el módulo se reinicia.
        vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

        // Crear tasks para que sean ejecutadas.
        xTaskCreatePinnedToCore(storage_task, "storage task", 2048, NULL, 3, NULL, APP_CPU_NUM);
        xTaskCreatePinnedToCore(record_generator_task, "generator task", 2048, NULL, 3, NULL, APP_CPU_NUM);
        xTaskCreatePinnedToCore(communication_task, "communication and sync task", 4096, NULL, 4, NULL, APP_CPU_NUM);
        xTaskCreatePinnedToCore(read_sensors_measurements_task, "read sensors task", 2048, NULL, 3, NULL, APP_CPU_NUM);

    } else {
        ESP_LOGE(
            TAG, 
            "Error al inicializar app usando setup() (%s)", 
            esp_err_to_name(setup_status)
        );
    }
}
