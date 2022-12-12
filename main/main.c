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
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/i2c.h>

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

#define STARTUP_DELAY_MS 1000

#define MAX_RECORD_QUEUE_LEN 5
#define MAX_SYNC_QUEUE_LEN 10
#define BATTERY_LVL_QUEUE_SIZE 1

#define NUM_SENSOR_MEASURES_PER_SECOND 5
#define MAX_SENSOR_DATA_BUF_SECONDS 5
#define MAX_SENSOR_DATA_BUF_LEN MAX_SENSOR_DATA_BUF_SECONDS * NUM_SENSOR_MEASURES_PER_SECOND
#define MAX_HX711_DATA_QUEUE_LEN 1

#define RECORD_RECEIVE_TIMEOUT_MS 500
#define NUM_BATTERY_CHARGE_SAMPLES 5

#define NO_RECENT_HYDRATION_BIT (0x01 << 0) 
#define BLE_INACTIVE_BIT (0x01 << 1) 
#define LOW_BAT_BIT (0x01 << 2)

#define MPU_INTERRUPT_BIT (0x01 << 0)
#define HX711_INTERRUPT_BIT (0x01 << 1)
#define DEEP_SLEEP_BEGIN_BIT (0x01 << 2)

#define I2C_SCL_GPIO CONFIG_I2C_SCL_IO
#define I2C_SDA_GPIO CONFIG_I2C_SDA_IO

static QueueHandle_t xStorageQueue = NULL;
static QueueHandle_t xSyncQueue = NULL;
static QueueHandle_t xHx711DataQueue = NULL;
static QueueHandle_t xBatteryLevelQueue = NULL;
/* Handles para tasks */
static TaskHandle_t xCommunicationTask = NULL;
static TaskHandle_t xStorageTask = NULL;
static TaskHandle_t xHydrationInferenceTask = NULL; 

static EventGroupHandle_t xDeepSleepConditionEvents = NULL;

static mpu_handle_t xMpuSensor = NULL;

TimerHandle_t power_mgmt_timer_handle = NULL;
static const TickType_t power_management_period_ticks = pdMS_TO_TICKS(CONFIG_PWR_MANAGER_EXECUTE_PERIOD_MS);
static const TickType_t device_shutdown_timeout_ticks = pdMS_TO_TICKS(3 * 1000);

static esp_err_t battery_monitor_status = ESP_FAIL;

typedef struct {
    BaseType_t useLazyBleEnable;
} communication_task_param_t;

/* Tasks de FreeRTOS */
static void storage_task(void* pvParameters);
static void hydration_inference_task(void* pvParameters);
static void communication_task(void* pvParameters);

/* Callbacks de timers */
static void battery_monitor_callback(TimerHandle_t xTimer);
static void power_management_callback(TimerHandle_t xTimer);

/* Funciones */
static esp_err_t send_record_to_sync(const hydration_record_t* const hydrationRecord);
static esp_err_t sync_records_from_storage(const nvs_handle_t storageHandle);
static esp_err_t sync_next_record_with_ble();
static esp_err_t transfer_hydration_records_from_queue(const QueueHandle_t source, const QueueHandle_t target, int32_t max_count, const TickType_t xTicksToWait);
static esp_err_t store_pending_records(const nvs_handle_t storageHandle);
static void get_comm_task_params(communication_task_param_t* const out_params, void* const pvParameters);

static esp_err_t hx711_setup(void);
static void IRAM_ATTR hx711_data_isr_handler(void* arg);

static esp_err_t i2c_setup(void);
static esp_err_t mpu_setup(void);
static void IRAM_ATTR mpu_data_rdy_isr(void* arg);

static mpu6050_config_t mpu_config = {
    .i2c_port_num = I2C_NUM_0,
    .address = CONFIG_MPU_I2C_ADDRESS,
    .enabled_interrupts = MPU_INT_DATA_RDY_BIT,
    .mpu_int = CONFIG_MPU_INT_GPIO,
    .isr_handler = mpu_data_rdy_isr,
    .min_read_interval_us = CONFIG_MPU_DATA_SAMPLE_INTERVAL_MS
};

static void log_sensor_measurements(const sensor_measures_t* const measurements);

static void init_power_management(BaseType_t* const wakeupFromEXT);
static BaseType_t are_deep_sleep_conditions_met(void);
static BaseType_t is_notify_for_deep_sleep_start(uint32_t notifyValue);

static void init_battery_monitoring(void);

static esp_err_t ble_init(void);
static esp_err_t ble_enable(const char* const device_name);

static esp_err_t app_startup(communication_task_param_t* const outCommunicationTaskParams);

/* Main app entry point */
void app_main(void)
{
    ESP_LOGI(TAG, "Memoria heap disponible = %u bytes", esp_get_minimum_free_heap_size());
    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

    static communication_task_param_t comm_task_params = {};
    esp_err_t setup_status = app_startup(&comm_task_params);

    if (ESP_OK == setup_status) 
    {      
        xTaskCreatePinnedToCore(storage_task, "storage_task", 2048, NULL, 3, &xStorageTask, APP_CPU_NUM);
        xTaskCreatePinnedToCore(hydration_inference_task, "hydr_infer_task", 4096, NULL, 3, &xHydrationInferenceTask, APP_CPU_NUM);
        xTaskCreatePinnedToCore(communication_task, "comm_sync_task", 4096, (void*) &comm_task_params, 4, &xCommunicationTask, APP_CPU_NUM);

    } else if (ESP_ERR_NO_MEM == setup_status) {
        ESP_LOGW(
            TAG, 
            "Uno o mas objetos de FreeRTOS no pudo ser creado. Asegura que haya memoria disponible en heap."
        );
    } else {
        ESP_LOGE(
            TAG, 
            "Error al inicializar app usando app_startup() (%s)", 
            esp_err_to_name(setup_status)
        );
    }

    ESP_LOGI(TAG, "Memoria heap disponible = %u bytes", esp_get_minimum_free_heap_size());
}

static void storage_task(void* pvParameters) 
{
    static const char* taskTag = "storage_task";
    // Abrir y obtener un handle para almacenamiento.
    nvs_handle_t storageHandle;

    esp_err_t nvs_status = storage_open(&storageHandle);

    if (xStorageQueue == NULL || nvs_status != ESP_OK)  {
        // Terminar inmediatamente este task, no debe seguir.
        ESP_LOGE(TAG, "NVS initialization error (%s)", esp_err_to_name(nvs_status));
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Storage inicializado");

    esp_err_t storageRegisterPwrManagerStatus = add_module_to_notify_before_deep_sleep(taskTag);

    if (ESP_OK != storageRegisterPwrManagerStatus)
    {
        ESP_LOGW(TAG, "Unable to register the storage task for energy saver notifications");
    }

    BaseType_t taskWasNotified = pdFALSE;
    uint32_t notify_value = 0;

    while (true) 
    {
        taskWasNotified = xTaskNotifyWait((uint32_t) 0, (uint32_t) 0, &notify_value, (TickType_t) 0);

        if (taskWasNotified)
        {
            BaseType_t aboutToEnterSleep = is_notify_for_deep_sleep_start(notify_value);

            if (aboutToEnterSleep)
            {
                // Close handle for NVS.
                storage_close(storageHandle);

                set_module_ready_for_deep_sleep(taskTag, true);
                break;
            }
        }

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
        if (shouldSync) 
        {
            ESP_LOGI(TAG, "Transfiriendo registros de hidratacion de xStorageQueue a xSyncQueue");
            esp_err_t queueTransferStatus = transfer_hydration_records_from_queue(xStorageQueue, xSyncQueue, MAX_RECORD_QUEUE_LEN, pdMS_TO_TICKS(50));

            if (ESP_OK != queueTransferStatus) 
            {
                ESP_LOGE(TAG, "Error al transferir elmentos de xStorageQueue a xSyncQueue (%s)", esp_err_to_name(queueTransferStatus));
            }

            esp_err_t retrieveForSyncStatus = sync_records_from_storage(storageHandle); 

            if (ESP_OK != retrieveForSyncStatus) 
            {
                ESP_LOGE(TAG, "Error al obtener registros desde NVS para sync (%s)", esp_err_to_name(retrieveForSyncStatus));
            }
        } else if (hasRecordsPendingStorage) {
            // No debe sincronizar los registros. Almacenar todos los registros
            // que estén pendientes en xStorageQueue.
            esp_err_t storageStatus = store_pending_records(storageHandle);

            if (ESP_OK != storageStatus) {
                ESP_LOGE(TAG, "Error al almacenar registros en NVS (%s)", esp_err_to_name(storageStatus));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(RECORD_RECEIVE_TIMEOUT_MS));
    }

    vTaskDelete(NULL);
}

static void hydration_inference_task(void* pvParameters) 
{
    static const char* taskTag = "hydr_inf";

    static const TickType_t waitForDataTimeoutTicks = pdMS_TO_TICKS(1000);
    static const TickType_t dataReadIntervalTicks = pdMS_TO_TICKS(200);

    static sensor_measures_t readingsBuffer[MAX_SENSOR_DATA_BUF_LEN] = {};
    size_t indexOfLatestSensorReadings = 0;

    static int64_t maxPeriodWithNoHydrationUs = 30000000LL;
    int64_t timestampOfLastHydrationUs = esp_timer_get_time();

    esp_err_t pwrManagerAddStatus = add_module_to_notify_before_deep_sleep(taskTag);

    if (ESP_OK != pwrManagerAddStatus)
    {
        ESP_LOGW(TAG, "Unable to register hydration inference task for power manager notifications");
    }

    while (true) 
    {
        uint32_t notifyValue = 0L;
        BaseType_t taskWasNotified = xTaskNotifyWait(
            (uint32_t) 0,
            (uint32_t) MPU_INTERRUPT_BIT | HX711_INTERRUPT_BIT,
            &notifyValue,
            waitForDataTimeoutTicks 
        );

        BaseType_t aboutToEnterSleep = is_notify_for_deep_sleep_start(notifyValue);

        if (aboutToEnterSleep)
        {
            if (NULL != xMpuSensor)
            {
                esp_err_t disableMpuInterruptsStatus = mpu6050_disable_interrupts(xMpuSensor, MPU_INT_DATA_RDY_BIT);

                if (ESP_OK != disableMpuInterruptsStatus)
                {
                    ESP_LOGW(TAG, "Error desactivar interrupts de MPU (%s)", esp_err_to_name(disableMpuInterruptsStatus));
                }

                esp_err_t motionEnableStatus = mpu6050_enable_motion_detect(xMpuSensor, &mpu_config);

                if (ESP_OK != motionEnableStatus)
                {
                    ESP_LOGW(TAG, "Error enabling motion detection (%s)", esp_err_to_name(motionEnableStatus));
                }
            }

            esp_err_t shutdownConfirmStatus = set_module_ready_for_deep_sleep(taskTag, true);

            if (ESP_OK != shutdownConfirmStatus)
            {
                ESP_LOGW(TAG, "Could not notify that hydr inference task is ready for sleep");
            }

            break;
        }

        BaseType_t mpuWasInterrupted = (taskWasNotified && (notifyValue & MPU_INTERRUPT_BIT) == MPU_INTERRUPT_BIT);
        BaseType_t mpuHasData = pdFALSE;

        if (mpuWasInterrupted && NULL != xMpuSensor)
        {
            uint8_t interrupt_status = 0x00;
            mpu6050_read_interrupt_status(xMpuSensor, &interrupt_status);

            ESP_LOGD(TAG, "MPU6050 interrupt status %0X", interrupt_status);

            if (mpu6050_is_intr_for_data(interrupt_status))
            {
                mpu6050_measures_t mpuData = {};
                esp_err_t read_status = mpu6050_get_measurements(xMpuSensor, &mpuData);

                mpuHasData = ESP_OK == read_status;

                if (mpuHasData)
                {
                    readingsBuffer[indexOfLatestSensorReadings].accel_gyro_measurements = mpuData;
                }
            }

            if (mpu6050_is_intr_for_motion(interrupt_status))
            {
                ESP_LOGD(TAG, "MPU detected motion");
            }
        }

        BaseType_t hx711_data_available = xQueueReceive(
            xHx711DataQueue,
            &(readingsBuffer[indexOfLatestSensorReadings].weight_measurements),
            (TickType_t) 0
        );

        bool sensor_measurements_available = mpuHasData == pdTRUE;

        if (sensor_measurements_available) 
        {
            ESP_LOGI(TAG, "Measurements index = %d", indexOfLatestSensorReadings);
            record_measurements_timestamp(&readingsBuffer[indexOfLatestSensorReadings]);

            // sensor_measures_t latest_sensor_data = readingsBuffer[indexOfLatestSensorReadings];

            // log_sensor_measurements(&latest_sensor_data);

            hydration_record_t hydrationRecord = {};
            
            // Algoritmo para determinar si las mediciones en readingsBuffer
            // son indicativas de consumo de agua.
            bool was_hydration_recorded = hydration_record_from_measures(readingsBuffer, MAX_SENSOR_DATA_BUF_LEN, &hydrationRecord);
            // bool was_hydration_recorded = !auxTestRecordCreated;

            if (was_hydration_recorded) 
            {
                ESP_LOGI(TAG, "Do measures represent hydration? yes");

                timestampOfLastHydrationUs = esp_timer_get_time();
                
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
                
                esp_err_t sendRecordStatus = send_record_to_sync(&hydrationRecord);

                if (ESP_OK != sendRecordStatus) 
                {            
                    ESP_LOGW(TAG, "Error sending hydration record to sync (%s)", esp_err_to_name(sendRecordStatus));
                }
            } else
            {
                EventBits_t sleepConditionStatus = xEventGroupWaitBits(
                    xDeepSleepConditionEvents, 
                    NO_RECENT_HYDRATION_BIT,
                    pdFALSE,
                    pdTRUE,
                    (TickType_t) 0
                );

                BaseType_t hasHydrationInactivityPeriodExpired = esp_timer_get_time() > (timestampOfLastHydrationUs + maxPeriodWithNoHydrationUs);
                BaseType_t shouldNotifyNoRecentHydration = ((sleepConditionStatus & NO_RECENT_HYDRATION_BIT) != NO_RECENT_HYDRATION_BIT);

                if (hasHydrationInactivityPeriodExpired && shouldNotifyNoRecentHydration) 
                {
                    ESP_LOGI(TAG, "%lld + %lld > %lld", timestampOfLastHydrationUs, maxPeriodWithNoHydrationUs, esp_timer_get_time());

                    ESP_LOGI(TAG, "Pasaron mas de %lld us sin un registro de hidratacion", maxPeriodWithNoHydrationUs);
                    EventBits_t resultBits = xEventGroupSetBits(xDeepSleepConditionEvents, NO_RECENT_HYDRATION_BIT);

                    if ((resultBits & NO_RECENT_HYDRATION_BIT) != NO_RECENT_HYDRATION_BIT)
                    {
                        ESP_LOGW(TAG, "El bit NO_RECENT_HYDRATION_BIT de eventos de deep sleep no fue set");
                    }
                }
            }
            
            indexOfLatestSensorReadings = (indexOfLatestSensorReadings + 1) % MAX_SENSOR_DATA_BUF_LEN;
        }

        vTaskDelay(dataReadIntervalTicks);
    }

    vTaskDelete(NULL);
}

static void communication_task(void* pvParameters)
{
    communication_task_param_t* params = &((communication_task_param_t) {});
    get_comm_task_params(params, pvParameters);

    static const char* taskTag = "communication_task";

    static int32_t advAttemptCount = 0;
    int64_t advStartedTimestampUs = 0;
    int64_t advEndTimestampUs = 0;
    int64_t advRestartTimestampUs = 0;

    static const uint32_t taskDelayTicks = pdMS_TO_TICKS(500);

    // El valor "watchdog" para evitar sincronizar registros infinitamente.
    static const uint16_t maxRecordsToSync = MAX_SYNC_QUEUE_LEN + MAX_STORED_RECORD_COUNT;
    static const uint16_t MAX_SYNC_ATTEMPTS = 3;
    static const TickType_t SYNC_FAILED_BACKOFF_DELAY = pdMS_TO_TICKS(5000);

    esp_err_t ble_init_error = ESP_FAIL;

    esp_err_t bleRegisterPwrManagerStatus = add_module_to_notify_before_deep_sleep(taskTag);

    if (ESP_OK != bleRegisterPwrManagerStatus)
    {
        ESP_LOGW(TAG, "Unable to register the communication task for energy saver notifications");
    }

    BaseType_t taskWasNotified = pdFALSE;
    uint32_t notify_value = 0;
    BaseType_t aboutToEnterSleep = pdFALSE;
    esp_err_t bleShutdownStatus = ESP_OK;

    static ble_status_t ble_driver_status = INACTIVE;
    static BaseType_t did_status_change = pdFALSE;

    while (true) 
    {
        taskWasNotified = xTaskNotifyWait((uint32_t) 0, (uint32_t) 0, &notify_value, (TickType_t) 0);

        if (taskWasNotified)
        {
            aboutToEnterSleep = is_notify_for_deep_sleep_start(notify_value);
        }

        ble_status_t previous_ble_status = ble_driver_status;
        ble_driver_status = ble_wait_for_state(PAIRED, true, (TickType_t) 0);
        did_status_change = previous_ble_status != ble_driver_status;

        switch (ble_driver_status)
        {
        case PAIRED:
        {
            if (taskWasNotified && aboutToEnterSleep)
            {
                bleShutdownStatus = ble_disconnect();

                if (ESP_OK == bleShutdownStatus)
                {
                    continue;
                } else 
                {
                    ESP_LOGW(TAG, "Error al desconectar BLE (%s)", esp_err_to_name(bleShutdownStatus));
                }
            }

            int16_t synchronizedRecordsCount = 0;
            int16_t syncAttemptsForCurrentRecord = 0;

            int32_t totalRecordsPendingSync = uxQueueMessagesWaiting(xSyncQueue);

            BaseType_t hasRemainingRecordsToSync = totalRecordsPendingSync > 0;

            // Iterar mientras la extensión esté PAIRED, queden registros de hidratación 
            // por sincronizar, los intentos de sincronización no superen a MAX_SYNC_ATTEMPTS
            //  y la cuenta total de registros sincronizados no supere a maxRecordsToSync.      
            while (PAIRED == ble_driver_status && hasRemainingRecordsToSync && 
                syncAttemptsForCurrentRecord < MAX_SYNC_ATTEMPTS && synchronizedRecordsCount < maxRecordsToSync
            ) {
                esp_err_t sync_status = sync_next_record_with_ble();

                if (ESP_OK == sync_status)
                {
                    // El registro de hidratación fue recibido por el
                    // cliente con éxito. Indicarlo con una variable "bandera":
                    ++synchronizedRecordsCount;
                    syncAttemptsForCurrentRecord = 0;

                    ESP_LOGI(TAG, "Registro obtenido por el dispositivo periferico, restantes = %i", 
                                (totalRecordsPendingSync - synchronizedRecordsCount));
                } else if (ESP_ERR_TIMEOUT == sync_status)
                {
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
                ble_driver_status = ble_wait_for_state(PAIRED, true, 0);

                hasRemainingRecordsToSync = synchronizedRecordsCount < totalRecordsPendingSync;
            }

            if (syncAttemptsForCurrentRecord >= MAX_SYNC_ATTEMPTS) {
                vTaskDelay(SYNC_FAILED_BACKOFF_DELAY);
                syncAttemptsForCurrentRecord = 0;
            }
        }
            break;
        case ADVERTISING:
        {
            if (did_status_change)
            {
                advStartedTimestampUs = esp_timer_get_time();
                ++advAttemptCount;

                if (advAttemptCount == 1)
                {
                    advEndTimestampUs = advStartedTimestampUs + (CONFIG_BLE_FIRST_ADV_DURATION_MS * 1000);
                } else 
                {
                    advEndTimestampUs = advStartedTimestampUs + (CONFIG_BLE_ADDITIONAL_ADV_DURATION_MS * 1000);
                }
            }

            if (taskWasNotified && aboutToEnterSleep && ESP_OK == bleShutdownStatus)
            {
                bleShutdownStatus = ble_stop_advertising();

                if (ESP_OK != bleShutdownStatus)
                {
                    ESP_LOGW(TAG, "Error al detener advertising BLE (%s)", esp_err_to_name(bleShutdownStatus));
                }
            }

            BaseType_t advAttemptLimitReached = did_status_change && advAttemptCount > (1 + CONFIG_BLE_ADDITIONAL_ADV_ATTEMPT_COUNT);

            if (advAttemptLimitReached)
            {
                ESP_LOGI(TAG, "BLE advertise attempt limit reached");
                if (NULL != xDeepSleepConditionEvents)
                {
                    xEventGroupSetBits(xDeepSleepConditionEvents, BLE_INACTIVE_BIT);
                }
            } else if (esp_timer_get_time() > advEndTimestampUs)
            {
                ESP_LOGI(TAG, "Timeout de advertising BLE alcanzado: %lld > %lld", esp_timer_get_time(), advEndTimestampUs);
                ESP_LOGI(TAG, "Intentos de advertising BLE = %d/%d", advAttemptCount, CONFIG_BLE_ADDITIONAL_ADV_ATTEMPT_COUNT + 1);
                // Ya pasó más de la duración esperada del advertising, pero 
                // no el limite de intentos. Entrar en light sleep.
                // esp_err_t ble_shutdown_status = ble_driver_shutdown();
                esp_err_t ble_shutdown_status = ESP_OK;
                advRestartTimestampUs = advEndTimestampUs + (CONFIG_BLE_ADVERTISING_SLEEP_INTERVAL_MS * 1000LL);

                if (ESP_OK == ble_shutdown_status)
                {
                    ESP_LOGI(TAG, "Transfiriendo registros de hidratacion de xSyncQueue a xStorageQueue");
                    ble_shutdown_status = transfer_hydration_records_from_queue(
                        xSyncQueue, 
                        xStorageQueue,
                        MAX_RECORD_QUEUE_LEN, 
                        pdMS_TO_TICKS(50)
                    );
                }
                
                if (ESP_OK == ble_shutdown_status) 
                {
                    ble_shutdown_status = ble_stop_advertising();
                }

                if (ESP_OK != ble_shutdown_status)
                {
                    ESP_LOGW(TAG, "Error al detener advertising BLE (%s)", esp_err_to_name(ble_shutdown_status));
                }

                if (ESP_OK == ble_shutdown_status)
                {
                    ble_shutdown_status = ble_driver_shutdown();
                }

                if (ESP_OK == ble_shutdown_status)
                {
                    ESP_LOGI(TAG, "Starting light sleep on BLE advertising timeout");
                    // esp_err_t light_sleep_status = enter_light_sleep(CONFIG_BLE_ADVERTISING_SLEEP_INTERVAL_MS * 1000ULL);

                    // if (ESP_OK != light_sleep_status)
                    // {
                    //     ESP_LOGW(TAG, "Error al intentar entrar en light sleep (%s)", esp_err_to_name(light_sleep_status));
                    // }
                }
            }
        } 
            break;
        case INITIALIZED:
        {
            UBaseType_t numOfRecordsPendingSync = uxQueueMessagesWaiting(xSyncQueue);
            BaseType_t isBleEnableFromHydration = params->useLazyBleEnable && numOfRecordsPendingSync > 0;

            BaseType_t canRetryAdv = advAttemptCount > 0 && esp_timer_get_time() >= advRestartTimestampUs;
            
            if (isBleEnableFromHydration || canRetryAdv)
            {
                esp_err_t ble_enable_status = ble_enable(CONFIG_BLE_DEVICE_NAME);

                if (ESP_OK == ble_enable_status)
                {
                    ESP_LOGI(TAG, "Advertising BLE activado, intento = %d", advAttemptCount);
                } else 
                {
                    ESP_LOGE(TAG, "Error al activar advertising (%s)", esp_err_to_name(ble_enable_status));
                }
            }
        }
            break;
        case INACTIVE:
            ble_init_error = ble_init();

            if (NULL == xSyncQueue || ble_init_error != ESP_OK)  {
                ESP_LOGE(TAG, "Error de inicializacion del driver de BLE (%s)", esp_err_to_name(ble_init_error));
            }
            break;
        default:
            break;
        }

        if (taskWasNotified && aboutToEnterSleep && ESP_OK == bleShutdownStatus)
        {
            if (ESP_OK == ble_init_error)
            {
                if (ESP_OK == bleShutdownStatus)
                {
                    bleShutdownStatus = ble_driver_shutdown();
                }

                if (ESP_OK == bleShutdownStatus)
                {
                    bleShutdownStatus = ble_driver_deinit();
                }

                if (ESP_OK != bleShutdownStatus)
                {
                    ESP_LOGW(TAG, "Error al de-inicializar BLE (%s)", esp_err_to_name(bleShutdownStatus));
                }
            }

            bleShutdownStatus = set_module_ready_for_deep_sleep(taskTag, true);

            if (ESP_OK == bleShutdownStatus)
            {
                // Terminar la ejecución de la task, ya que va a entrar en 
                // deep sleep.
                break;
            }
        }

        // Esperar un momento antes de volver a revisar el estado de 
        // la conexión, evitar acaparar todo el tiempo del scheduler 
        // en revisar la conexion.
        vTaskDelay(taskDelayTicks);
    }

    vTaskDelete(NULL);
}

static esp_err_t hx711_setup(void)
{
    static hx711_t hx711_sensor = {
        .data_out = CONFIG_HX711_DATA_OUT_GPIO,
        .pd_sck = CONFIG_HX711_PD_SCK_GPIO,
        .gain = HX711_GAIN_A_64,
        .interrupt_on_data = pdTRUE,
        .isr_handler = hx711_data_isr_handler,
        .min_read_interval_ms = 200
    };

    esp_err_t hx711_init_status = hx711_init(&hx711_sensor);

    if (ESP_OK != hx711_init_status)
    {
        ESP_LOGW(TAG, "HX711 sensor status (%s)", esp_err_to_name(hx711_init_status));
    }

    return hx711_init_status;
}

static void IRAM_ATTR hx711_data_isr_handler(void* arg)
{
    hx711_t* hx711Sensor = (hx711_t*) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (NULL != hx711Sensor)
    {
        gpio_intr_disable(hx711Sensor->data_out);
        hx711_measures_t hx711_measurement = {};
        esp_err_t read_status = hx711_get_measurements(hx711Sensor, &hx711_measurement, 0);

        if (ESP_OK == read_status && NULL != xHx711DataQueue)
        {
            xQueueOverwriteFromISR(xHx711DataQueue, &hx711_measurement, &xHigherPriorityTaskWoken);
        }     
    }

    if (xHigherPriorityTaskWoken) 
    {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t i2c_setup(void)
{
    i2c_config_t i2c_bus_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_I2C_MASTER_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };

    esp_err_t i2c_init_status = ESP_OK; 

    if (ESP_OK == i2c_init_status) 
    {
        i2c_init_status = i2c_param_config(I2C_NUM_0, &i2c_bus_config);

        if (i2c_init_status != ESP_OK) 
        {
            ESP_LOGW(TAG, "I2C initialization error (%s)", esp_err_to_name(i2c_init_status));
        }
    }

    if (ESP_OK == i2c_init_status) 
    {
        i2c_init_status = i2c_driver_install(I2C_NUM_0, i2c_bus_config.mode, 0, 0, 0);

        if (i2c_init_status != ESP_OK) 
        {
            ESP_LOGW(TAG, "I2C initialization error (%s)", esp_err_to_name(i2c_init_status));
        }
    }

    return i2c_init_status;
}

static esp_err_t mpu_setup(void)
{
    esp_err_t mpu_init_status = mpu6050_init(&mpu_config, &xMpuSensor);

    if (ESP_OK != mpu_init_status || xMpuSensor == NULL)
    {
        ESP_LOGW(TAG, "MPU6050 init error, status (%s)", esp_err_to_name(mpu_init_status));
    }

    if (ESP_OK == mpu_init_status)
    {
        mpu_init_status = mpu6050_enable_interrupts(xMpuSensor, &mpu_config);
    }

    return mpu_init_status;
}

static void IRAM_ATTR mpu_data_rdy_isr(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (NULL != xHydrationInferenceTask)
    {
        xTaskNotifyFromISR(
            xHydrationInferenceTask, 
            MPU_INTERRUPT_BIT, 
            eSetBits, 
            &xHigherPriorityTaskWoken
        );
    }

    if (xHigherPriorityTaskWoken) 
    {
        portYIELD_FROM_ISR();
    }
}

static void init_battery_monitoring(void)
{
    battery_monitor_status = battery_monitor_init();

    ESP_LOGI(TAG, "Battery monitor status (%s)", esp_err_to_name(battery_monitor_status));

    TimerHandle_t xBatteryMonitorTimer = xTimerCreate(
        "battery_level_timer",
        pdMS_TO_TICKS(CONFIG_BATTERY_MEASURE_INTERVAL_MS),
        pdTRUE,
        NULL,
        battery_monitor_callback
    );

    // Comenzar el timer periodico para obtener mediciones del
    // nivel de bateria.
    xTimerStart(xBatteryMonitorTimer, (TickType_t) 0);
}

static void battery_monitor_callback(TimerHandle_t xTimer) 
{
    battery_measurement_t battery_measurement = {};

    if (ESP_OK != battery_monitor_status) 
    {
        // Si el sensor de batería no está listo para hacer un sample, intentar
        // re-inicializarlo.
        battery_monitor_status = battery_monitor_init();
    }

    esp_err_t measure_status = multi_sample_battery_level(&battery_measurement, NUM_BATTERY_CHARGE_SAMPLES);

    if (ESP_OK == measure_status) 
    {
        xQueueOverwrite(xBatteryLevelQueue, &battery_measurement);

        ESP_LOGI(TAG, "Carga restante de la bateria: %d%%", battery_measurement.remaining_charge);

        if (NULL != xDeepSleepConditionEvents)
        {
            if (is_battery_low(&battery_measurement))
            {
                ESP_LOGI(TAG, "Bateria baja");
                xEventGroupSetBits(xDeepSleepConditionEvents, LOW_BAT_BIT);
            } else 
            {
                xEventGroupClearBits(xDeepSleepConditionEvents, LOW_BAT_BIT);
            }
        }
    }
} 

static void init_power_management(BaseType_t* const wakeupFromEXT) 
{
    after_wakeup(wakeupFromEXT);
    set_max_deep_sleep_duration(((int64_t) CONFIG_MAX_DEEP_SLEEP_DURATION_MS) * 1000);

    esp_err_t wakeupSourceStatus = setup_wakeup_sources((gpio_num_t) CONFIG_MPU_INT_GPIO);

    if (ESP_OK != wakeupSourceStatus)
    {
        ESP_LOGW(TAG, "Error al configurar fuentes de activacion de deep sleep (%s)", esp_err_to_name(wakeupSourceStatus));
    }

    if (CONFIG_LIGHT_SLEEP_ENABLED)
    {
        esp_err_t lightSleepStatus = setup_light_sleep(true);

        if (ESP_OK != lightSleepStatus)
        {
            ESP_LOGW(TAG, "Error en setup de light sleep (%s)", esp_err_to_name(lightSleepStatus));
        }
    }

    power_mgmt_timer_handle = xTimerCreate(
        "power_management_timer",
        power_management_period_ticks,
        pdTRUE,
        NULL,
        power_management_callback
    );

    if (NULL != power_mgmt_timer_handle)
    {
        xTimerStart(power_mgmt_timer_handle, pdMS_TO_TICKS(100));

    } else {
        ESP_LOGW(TAG, "Error al crear timer para manejo de energia. Revisa que exista suficiente memoria en heap.");
    }
}

static void power_management_callback(TimerHandle_t xTimer) 
{
    if (NULL == xDeepSleepConditionEvents)
    {
        ESP_LOGW(TAG, "El event group para manejo de energia es NULL");
        return;
    }

    ESP_LOGI(TAG, "Power manager is running");
 
    BaseType_t shouldEnterSleep = are_deep_sleep_conditions_met();  

    ESP_LOGI(TAG, "Should enter deep sleep? %s", (shouldEnterSleep) ? "yes" : "no");

    if (CONFIG_DEEP_SLEEP_ENABLED && shouldEnterSleep)
    {
        const int32_t sleepEnterAttemptCount = increment_sleep_attempt_count();
        BaseType_t hasReachedRetryLimit = sleepEnterAttemptCount >= CONFIG_MAX_DEEP_SLEEP_ATTEMPTS;

        BaseType_t areTasksReadyForDeepSleep = pdFALSE;
        esp_err_t sleep_check_status = is_ready_for_deep_sleep(&areTasksReadyForDeepSleep);

        if (ESP_OK == sleep_check_status)
        {
            ESP_LOGI(TAG, "Tasks ready for sleep? %s, Retry limit reached? %s",
                     (areTasksReadyForDeepSleep ? "yes" : "no"), (hasReachedRetryLimit ? "yes" : "no"));

            if (areTasksReadyForDeepSleep || hasReachedRetryLimit) 
            {
                enter_deep_sleep();
            } else 
            {
                BaseType_t changePeriodResult = xTimerChangePeriod(
                    power_mgmt_timer_handle, 
                    device_shutdown_timeout_ticks, 
                    (TickType_t) 0
                );

                if (pdPASS == changePeriodResult)
                {
                    ESP_LOGI(TAG, "Deep sleep enter attempt count: %d", sleepEnterAttemptCount);

                    // Notificar a tasks que el chip va a iniciar deep sleep.
                    if (NULL != xStorageTask) xTaskNotify(xStorageTask, DEEP_SLEEP_BEGIN_BIT, eSetBits);
                    if (NULL != xCommunicationTask) xTaskNotify(xCommunicationTask, DEEP_SLEEP_BEGIN_BIT, eSetBits);
                    if (NULL != xHydrationInferenceTask) xTaskNotify(xHydrationInferenceTask, DEEP_SLEEP_BEGIN_BIT, eSetBits);
                } else 
                {
                    ESP_LOGW(TAG, "Error al configurar timer de manejo de potencia en modo 'timeout'.");
                }
            }
        } else 
        {
            ESP_LOGW(TAG, "Error al revisar si las tasks estan listas para deep sleep (%s)", esp_err_to_name(sleep_check_status));
        }
    }
}

static esp_err_t app_startup(communication_task_param_t* const outCommunicationTaskParams) 
{
    esp_err_t setup_status = ESP_OK;

    if (ESP_OK == setup_status)
    {
        ESP_LOGD(TAG, "Configurando power management");
        BaseType_t wakeupFromEXT1 = pdFALSE;
        init_power_management(&wakeupFromEXT1);

        if (NULL != outCommunicationTaskParams)
        {
            outCommunicationTaskParams->useLazyBleEnable = !wakeupFromEXT1;
        }
    }

    if (ESP_OK == setup_status)
    {
        // Inicializar los objetos de FreeRTOS usados para controlar 
        // los procesos de almacenar y obtener registros de hidratación. 
        xStorageQueue = xQueueCreate( MAX_RECORD_QUEUE_LEN, sizeof(hydration_record_t) );
        xSyncQueue = xQueueCreate( MAX_SYNC_QUEUE_LEN, sizeof(hydration_record_t) );
        xHx711DataQueue = xQueueCreate( MAX_HX711_DATA_QUEUE_LEN, sizeof(hx711_measures_t) );
        xBatteryLevelQueue = xQueueCreate( BATTERY_LVL_QUEUE_SIZE, sizeof(battery_measurement_t) );

        if (NULL == xStorageQueue || NULL == xSyncQueue || 
            NULL == xHx711DataQueue|| NULL == xBatteryLevelQueue) 
        {
            setup_status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == setup_status)
    {
        xDeepSleepConditionEvents = xEventGroupCreate();

        if (NULL == xDeepSleepConditionEvents)
        {
            setup_status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == setup_status)
    {
        ESP_LOGD(TAG, "Configurando monitor de bateria");
        init_battery_monitoring();

        if (ESP_OK == setup_status)
        {
            // Agregar el nivel inicial de la bateria, en vez de esperar 
            // a que el timer ejecute el callback.
            battery_monitor_callback(NULL);
        }
    }

    if (ESP_OK == setup_status)
    {
        ESP_LOGD(TAG, "Inicializando NVS");
        //NOTA: storage_init() está aquí porque si lo dejaba como parte de 
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
            setup_status = nvs_status;
        }
    }

    if (ESP_OK == setup_status)
    {
        setup_status = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    }

    if (ESP_OK == setup_status)
    {
        setup_status = i2c_setup();
    }

    if (ESP_OK == setup_status)
    {
        setup_status = mpu_setup();
    }

    if (ESP_OK == setup_status) 
    {
        setup_status = hx711_setup();
    }

    return setup_status;
}

static BaseType_t are_deep_sleep_conditions_met(void)
{
    EventBits_t deepSleepConditions = xEventGroupGetBits(xDeepSleepConditionEvents);
    EventBits_t inactiveBits = NO_RECENT_HYDRATION_BIT | BLE_INACTIVE_BIT;

    BaseType_t isBatteryLow = ((deepSleepConditions & LOW_BAT_BIT) == LOW_BAT_BIT);
    
    if (pdPASS == isBatteryLow)
    {
        ESP_LOGW(TAG, "Battery low");
    }

    if ((deepSleepConditions & inactiveBits) == inactiveBits)
    {
        ESP_LOGW(TAG, "BLE and hydration inference inactivity");
    }

    BaseType_t shouldEnterSleep = isBatteryLow || (deepSleepConditions & inactiveBits) == inactiveBits; 

    return shouldEnterSleep;
}

static BaseType_t is_notify_for_deep_sleep_start(uint32_t notifyValue)
{
    return (BaseType_t) ((notifyValue & DEEP_SLEEP_BEGIN_BIT) == DEEP_SLEEP_BEGIN_BIT);    
}

static esp_err_t send_record_to_sync(const hydration_record_t* const hydrationRecord)
{
    if (NULL == hydrationRecord) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    static int32_t generatedHydrationRecordCount = 0;

    static const TickType_t waitForSpaceTimeoutTicks = pdMS_TO_TICKS(50);

    char strRegistroHidratacion[100] = {};
    hydration_record_to_string(strRegistroHidratacion, hydrationRecord);

    ESP_LOGI(TAG, "%s", strRegistroHidratacion);

    esp_err_t status = ESP_OK;
        
    if (ESP_OK == status) {
        // Intentar enviar el registro de hidratación generado a ser  
        // sincronizado o almacenado. Si la queue está full, esperar por
        // waitForSpaceTimeoutTicks a que se libere espacio. 
        BaseType_t queueSendResult = xQueueSendToBack( 
            xSyncQueue, 
            (void*) hydrationRecord,
            waitForSpaceTimeoutTicks
        );

        if (queueSendResult == errQUEUE_FULL) {
            status = ESP_ERR_NO_MEM;
        }
    } 

    if (ESP_OK == status)
    {
        ++generatedHydrationRecordCount;

        ESP_LOGI(
            TAG, 
            "Nuevo registro de hidratacion enviado para sincronizacion, total %d", 
            generatedHydrationRecordCount
        );
    } else 
    {
        ESP_LOGW(
            TAG, 
            "Un nuevo registro %d no pudo ser enviado (errQUEUE_FULL)",
            generatedHydrationRecordCount
        );
    }

    return status;
}

static esp_err_t sync_records_from_storage(const nvs_handle_t storageHandle) 
{
    esp_err_t status = ESP_OK;
    int16_t numOfStoredRecords = 0;

    status = get_stored_count(storageHandle, &numOfStoredRecords);

    ESP_LOGI(TAG, "%d registros almacenados en NVS", numOfStoredRecords);

    if (ESP_OK == status) {
        // Obtener y enviar cada registro almacenado para que sea sincronizado.
        for (int16_t i = 0; i < numOfStoredRecords; ++i) {

            hydration_record_t xRecordToSync = { 0, 0, 0, 0 };

            // Hay uno o más registros por sincronizar en NVS. 
            status = retrieve_oldest_hydration_record(storageHandle, &xRecordToSync);

            if (ESP_OK == status) {
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

                if (sendResult == ESP_OK) {
                    status = commit_retrieval(storageHandle);

                    if (status != ESP_OK) {
                        ESP_LOGW(TAG, "Registro obtenido de NVS, pero error en commit (%s)", esp_err_to_name(status));
                    }
                } else {
                    ESP_LOGE(
                        TAG, 
                        "El registro fue recuperado de NVS, pero no pudo ser enviado (errQUEUE_FULL)"
                    );
                }
            } else {
                // Hubo un error al intentar obtener el registro de hidratacion
                // almacenado.
                ESP_LOGE(TAG, "Error al recuperar registro desde NVS (%s)", esp_err_to_name(status)); 
            }
        }
    } else {
        // Hubo un error al intentar obtener la cuenta de registros
        // almacenados.
        ESP_LOGE(TAG, "Error obteniendo el numero de registros almacenados (%s)", esp_err_to_name(status));
    }

    return status;
}

static esp_err_t transfer_hydration_records_from_queue(const QueueHandle_t source, const QueueHandle_t target, int32_t max_count, const TickType_t xTicksToWait)
{
    esp_err_t status = ESP_OK;

    if (ESP_OK == status && (NULL == source || NULL == target))
    {
        status = ESP_ERR_INVALID_ARG;
    }

    int32_t sourceQueueItemCount = 0;
    
    if (ESP_OK == status)
    {
        sourceQueueItemCount = uxQueueMessagesWaiting(source);
        ESP_LOGI(TAG, "La queue source contiene %d elementos, para transferir a target queue", sourceQueueItemCount);

        if (sourceQueueItemCount == 0)
        {
            return status;
        } else if (sourceQueueItemCount > max_count)
        {
            // Asegurar que transfiere hasta max_count elementos entre 
            // las queues.
            sourceQueueItemCount = max_count;
        }
    }

    if (ESP_OK == status)
    {
        hydration_record_t xReceivedRecord = { 0, 0, 0, 0 };

        for (int16_t i = 0; i < sourceQueueItemCount; ++i) 
        {
            // Recibir un elemento de source queue debería ser casi 
            // instantáneo, ya que sourceQueueItemCount > 0.
            BaseType_t recordReceived = xQueueReceive(
                source, 
                &(xReceivedRecord), 
                (TickType_t) 0 
            );

            if (recordReceived) {
                // Si el registro fue recuperado correctamente, enviarlo a xSyncQueue 
                // para que sea sincronizado.
                BaseType_t sendResult = xQueueSendToBack(
                    target, 
                    ((void*) &(xReceivedRecord)), 
                    xTicksToWait
                );

                if (pdPASS == sendResult) 
                {
                    ESP_LOGI(TAG, "Elemento transferido entre dos queues");
                } else {
                    status = ESP_FAIL;
                    ESP_LOGE(TAG, "El elemento no pudo ser transferido entre dos queues (espERR_QUEUE_FULL)");
                }
            } else {
                status = ESP_FAIL;
                ESP_LOGE(
                    TAG, 
                    "Esperaba encontrar %d elemento en la queue fuente, pero no fue posible obtenerlos", 
                    sourceQueueItemCount
                );
            }
        }
    }

    return status;
}

static esp_err_t store_pending_records(const nvs_handle_t storageHandle)
{
    esp_err_t status = ESP_OK;
    hydration_record_t xReceivedRecord = { 0, 0, 0, 0 };

    int32_t recordsInStorageQueue = uxQueueMessagesWaiting(xStorageQueue);

    for (int16_t i = 0; i < recordsInStorageQueue; ++i) 
    {
        BaseType_t queueHadItems = xQueuePeek(
            xStorageQueue, 
            &( xReceivedRecord ), 
            // Recibir un elemento debería ser casi instantáneo, porque hasRecordsPendingStorage es pdTRUE.
            (TickType_t) 0 
        );

        BaseType_t recordWasStored = pdFALSE;

        if (queueHadItems) 
        {
            status = store_latest_hydration_record(storageHandle, &xReceivedRecord);

            recordWasStored = ESP_OK == status;

            if (recordWasStored) 
            {
                ESP_LOGI(TAG, "Registro almacenado en NVS");
            } else {
                // Por algún motivo, el almacenamiento en NVS produjo un error.
                ESP_LOGE(TAG, "Error al guardar registro en NVS (%s)", esp_err_to_name(status));
            }
        } else {
            ESP_LOGE(TAG, "Esperaba obtener un registro para almacenar, pero la queue estaba vacia");
            status = ESP_FAIL;
        }

        if (recordWasStored) 
        {
            BaseType_t recordRemovedFromQueue = xQueueReceive(
                xStorageQueue,
                &( xReceivedRecord ), 
                (TickType_t) 0
            );

            if (!recordRemovedFromQueue) {
                ESP_LOGE(
                    TAG, 
                    "El registro fue almacenado, pero no pudo ser removido de xStorageQueue (%s)", 
                    esp_err_to_name(status)
                );
            }
        }
    }

    return status;
}

static void get_comm_task_params(communication_task_param_t* const out_params, void* const pvParameters)
{
    if (NULL != out_params)
    {
        if (NULL != pvParameters)
        {
            out_params->useLazyBleEnable = ((communication_task_param_t*) pvParameters)->useLazyBleEnable;
        } else 
        {
            out_params->useLazyBleEnable = pdFALSE;
        }
    }
}

static esp_err_t ble_init(void)
{
    esp_err_t status = ESP_OK;
    ble_status_t driver_status = ble_wait_for_state(INACTIVE, true, (TickType_t) 0);

    if (INACTIVE == driver_status)
    {
        if (ESP_OK == status)
        {
            status = ble_driver_init();
            ESP_LOGI(TAG, "BLE init result (%s)", esp_err_to_name(status));
        }
    }

    return status;
}

static esp_err_t ble_enable(const char* const device_name)
{
    esp_err_t status = ESP_OK;

    if (ESP_OK == status)
    {
        status = ble_driver_enable(device_name);
        ESP_LOGI(TAG, "BLE driver enable result (%s)", esp_err_to_name(status));
    }

    if (ESP_OK == status)
    {
        status = ble_driver_start_advertising();
        ESP_LOGI(TAG, "BLE advertising start result (%s)", esp_err_to_name(status));
    }

    return status;
}

static esp_err_t sync_next_record_with_ble()
{
    static const uint32_t recordReadTimeoutMS = 10 * 1000; 

    esp_err_t status = ESP_FAIL;
    hydration_record_t xRecordToSync = { 0, 0, 0, 0 };
    // Obtener el siguiente registro por sincronizar, sin removerlo 
    // de la queue (un registro es removido hasta que haya sincronizado
    // con éxito).
    // Esta operación debería ser prácticamente instantánea, porque 
    // hasRecordsToSync es pdTRUE y esta es la única task que puede 
    // recibir elementos de xSyncQueue.
    BaseType_t wasRecordReceived = xQueuePeek(
        xSyncQueue, 
        &xRecordToSync, 
        (TickType_t) 0
    );

    if (wasRecordReceived) 
    {
        status = ble_synchronize_hydration_record(&xRecordToSync, recordReadTimeoutMS);

        if (ESP_OK == status) 
        {
            BaseType_t wasRecordRemoved = xQueueReceive(
                xSyncQueue,
                &xRecordToSync, 
                (TickType_t) 0
            );

            if (wasRecordRemoved == pdFALSE) 
            {
                status = ESP_FAIL;
            }

        } 
    }

    return status;
}

static void log_sensor_measurements(const sensor_measures_t* const measurements)
{
    if (NULL != measurements)
    {
        ESP_LOGI(TAG, "Timestamp de datos de sensores: %lld", measurements->timestamp_ms);

        ESP_LOGI(
            TAG, 
            "Lecturas de HX711: { raw_weight: %d, volume_ml: %u, timestamp: %lld }", 
            measurements->weight_measurements.raw_weight, measurements->weight_measurements.volume_ml,
            measurements->weight_measurements.timestamp_ms
        );

        ESP_LOGI(
            TAG, 
            "Lecturas de MPU6050: { accel: (%.2f, %.2f, %.2f), gyro: (%.2f, %.2f, %.2f), temp: %.2f }", 
            measurements->accel_gyro_measurements.acceleration.x, measurements->accel_gyro_measurements.acceleration.y, measurements->accel_gyro_measurements.acceleration.z, 
            measurements->accel_gyro_measurements.gyroscope.x, measurements->accel_gyro_measurements.gyroscope.y, measurements->accel_gyro_measurements.gyroscope.z, 
            measurements->accel_gyro_measurements.temperature
        );
    }
}
