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
#define LATEST_HYDR_TIMESTAMP_QUEUE_SIZE 1
#define BLE_ADV_DURATION_QUEUE_SIZE 1

#define NUM_SENSOR_MEASURES_PER_SECOND 5
#define MAX_SENSOR_DATA_BUF_SECONDS 5
#define MAX_SENSOR_DATA_BUF_LEN MAX_SENSOR_DATA_BUF_SECONDS * NUM_SENSOR_MEASURES_PER_SECOND
#define MAX_MPU6050_DATA_QUEUE_LEN 10
#define MAX_HX711_DATA_QUEUE_LEN 1

#define RECORD_RECEIVE_TIMEOUT_MS 500
#define NUM_BATTERY_CHARGE_SAMPLES 5

#define NO_RECENT_HYDRATION_BIT (1u << 0) 
#define BLE_INACTIVE_BIT (1u << 1) 
#define LOW_BAT_BIT (1u < 2)

#define I2C_SCL_GPIO CONFIG_I2C_SCL_IO
#define I2C_SDA_GPIO CONFIG_I2C_SDA_IO

static QueueHandle_t xStorageQueue = NULL;
static QueueHandle_t xSyncQueue = NULL;
static QueueHandle_t xMpu6050DataQueue = NULL;
static QueueHandle_t xHx711DataQueue = NULL;
static QueueHandle_t xBatteryLevelQueue = NULL;
/* Queues para ahorro de energia */
static QueueHandle_t xLatestHydrationTimestampQueue = NULL;
static QueueHandle_t xBleAdvertisingStartTime = NULL;
/* Handles para tasks */
static TaskHandle_t xCommunicationTask = NULL;
static TaskHandle_t xStorageTask = NULL;
static TaskHandle_t xHydrationInferenceTask = NULL; 

static EventGroupHandle_t xDeepSleepConditionEvents = NULL;

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
static esp_err_t send_record_to_sync(const hydration_record_t* hydrationRecord, bool* sentToStorage);
static esp_err_t sync_records_from_storage(const nvs_handle_t storageHandle);
static esp_err_t sync_next_record_with_ble();
static esp_err_t transfer_storage_queue_to_sync(void);
static esp_err_t store_pending_records(const nvs_handle_t storageHandle);
static void get_comm_task_params(communication_task_param_t* const out_params, void* const pvParameters);

static esp_err_t hx711_setup(void);
static void IRAM_ATTR hx711_data_isr_handler(void* arg);

static esp_err_t i2c_setup(void);
static esp_err_t mpu_setup(void);
static void IRAM_ATTR mpu_data_rdy_isr(void* arg);

static void init_power_management(BaseType_t* const wakeupFromEXT);
static void init_battery_monitoring(void);
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
        xTaskCreatePinnedToCore(hydration_inference_task, "hydr_infer_task", 2048, NULL, 3, &xHydrationInferenceTask, APP_CPU_NUM);
        xTaskCreatePinnedToCore(communication_task, "comm_sync_task", 4096, (void*) &comm_task_params, 4, &xCommunicationTask, APP_CPU_NUM);

        ESP_LOGI(TAG, "Memoria heap disponible = %u bytes", esp_get_minimum_free_heap_size());

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
}

static void storage_task(void* pvParameters) 
{
    // Abrir y obtener un handle para almacenamiento.
    nvs_handle_t storageHandle;

    esp_err_t nvs_status = storage_open(&storageHandle);

    if (xStorageQueue == NULL || nvs_status != ESP_OK)  {
        // Terminar inmediatamente este task, no debe seguir.
        ESP_LOGE(TAG, "NVS initialization error (%s)", esp_err_to_name(nvs_status));
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "NVS inicializado");

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
        if (shouldSync) 
        {
            esp_err_t queueTransferStatus = transfer_storage_queue_to_sync();

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

    // Close handle for NVS.
    storage_close(storageHandle);

    vTaskDelete(NULL);
}

static void hydration_inference_task(void* pvParameters) 
{
    int32_t num_of_records_sent_for_sync = 0;
    int32_t numRecordsSentForStorage = 0; 

    static const TickType_t waitForReadingsTimeoutTicks = pdMS_TO_TICKS(10000);

    static sensor_measures_t readingsBuffer[MAX_SENSOR_DATA_BUF_LEN] = {};
    size_t indexOfLatestSensorReadings = 0;

    static int64_t maxPeriodWithNoHydrationUs = 30000000LL;
    int64_t timestampOfLastHydrationUs = esp_timer_get_time();

    while (true) 
    {
        BaseType_t mpu6050_measurements_received = xQueueReceive(
            xMpu6050DataQueue,
            &(readingsBuffer[indexOfLatestSensorReadings].accel_gyro_measurements),
            waitForReadingsTimeoutTicks
        );

        BaseType_t hx711_data_available = xQueueReceive(
            xHx711DataQueue,
            &(readingsBuffer[indexOfLatestSensorReadings].weight_measurements),
            (TickType_t) 0
        );

        bool sensor_measurements_available = true;

        if (sensor_measurements_available) 
        {
            record_measurements_timestamp(&readingsBuffer[indexOfLatestSensorReadings]);

            sensor_measures_t latest_sensor_data = readingsBuffer[indexOfLatestSensorReadings];

            ESP_LOGI(TAG, "Timestamp de datos de sensores: %lld", latest_sensor_data.timestamp_ms);

            ESP_LOGI(
                TAG, 
                "Lecturas de HX711: { raw_weight: %d, volume_ml: %u, timestamp: %lld }", 
                latest_sensor_data.weight_measurements.raw_weight, latest_sensor_data.weight_measurements.volume_ml,
                latest_sensor_data.weight_measurements.timestamp_ms
            );

            ESP_LOGI(
                TAG, 
                "Lecturas de MPU6050: { accel: (%.2f, %.2f, %.2f), gyro: (%.2f, %.2f, %.2f), temp: %.2f }", 
                latest_sensor_data.accel_gyro_measurements.acceleration.x, latest_sensor_data.accel_gyro_measurements.acceleration.y, latest_sensor_data.accel_gyro_measurements.acceleration.z, 
                latest_sensor_data.accel_gyro_measurements.gyroscope.x, latest_sensor_data.accel_gyro_measurements.gyroscope.y, latest_sensor_data.accel_gyro_measurements.gyroscope.z, 
                latest_sensor_data.accel_gyro_measurements.temperature
            );

            hydration_record_t hydrationRecord = {};
            
            // Algoritmo para determinar si las mediciones en readingsBuffer
            // son indicativas de consumo de agua.
            // bool was_hydration_recorded = hydration_record_from_measures(readingsBuffer, MAX_SENSOR_DATA_BUF_LEN, &hydrationRecord);
            bool was_hydration_recorded = false;
            // ESP_LOGI(TAG, "Do measures represent hydration: %s", (was_hydration_recorded ? "yes" : "no" ));

            if (was_hydration_recorded) 
            {
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
                        ++num_of_records_sent_for_sync;
                    }
                }

                int32_t totalRecordsSent = numRecordsSentForStorage + num_of_records_sent_for_sync;

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
            } else if (timestampOfLastHydrationUs + maxPeriodWithNoHydrationUs > esp_timer_get_time())
            {
                ESP_LOGI(TAG, "Pasaron mas de %lld us sin un registro de hidratacion", maxPeriodWithNoHydrationUs);
                EventBits_t resultBits = xEventGroupSetBits(xDeepSleepConditionEvents, NO_RECENT_HYDRATION_BIT);

                if ((resultBits & NO_RECENT_HYDRATION_BIT) != NO_RECENT_HYDRATION_BIT)
                {
                    ESP_LOGW(TAG, "El bit NO_RECENT_HYDRATION_BIT de eventos de deep sleep no fue set");
                }
            }
            
            ++indexOfLatestSensorReadings;
            indexOfLatestSensorReadings = (indexOfLatestSensorReadings >= MAX_SENSOR_DATA_BUF_LEN)
                ? indexOfLatestSensorReadings % MAX_SENSOR_DATA_BUF_LEN
                : indexOfLatestSensorReadings;
        }
    }

    vTaskDelete(NULL);
}

static void communication_task(void* pvParameters)
{
    communication_task_param_t* params = &((communication_task_param_t) {});
    get_comm_task_params(params, pvParameters);

    static int32_t advAttemptCount = 1;
    static const uint32_t waitForPairedStatusTimeoutMS = 5000;
    static const TickType_t FAILED_INIT_RETRY_DELAY = pdMS_TO_TICKS(1000);

    // El valor "watchdog" para evitar sincronizar registros infinitamente.
    static const uint16_t maxRecordsToSync = MAX_SYNC_QUEUE_LEN + MAX_STORED_RECORD_COUNT;
    static const uint16_t MAX_SYNC_ATTEMPTS = 3;
    static const TickType_t SYNC_FAILED_BACKOFF_DELAY = pdMS_TO_TICKS(5000);

    static const char* communication_task_tag = "communication_task";

    esp_err_t ble_init_status = ESP_FAIL;

    if (!params->useLazyBleEnable)
    {
        ble_init_status = ble_driver_init(CONFIG_BLE_DEVICE_NAME);

        if (NULL == xSyncQueue || ble_init_status != ESP_OK)  {
            ESP_LOGE(TAG, "Error de inicializacion del driver de BLE (%s)", esp_err_to_name(ble_init_status));
        }
    }

    esp_err_t ble_energy_saver_add_status = add_module_to_notify_before_deep_sleep(communication_task_tag);

    if (ESP_OK != ble_energy_saver_add_status)
    {
        ESP_LOGW(TAG, "Unable to register the communication task for energy saver notifications");
    }

    while (true) 
    {
        int32_t numOfRecordsPendingSync = uxQueueMessagesWaiting(xSyncQueue);

        if (ESP_OK == ble_init_status)
        {
            ble_status_t ble_pair_status = ble_wait_for_state(PAIRED, true, waitForPairedStatusTimeoutMS);

            if (PAIRED == ble_pair_status) 
            {
                int16_t synchronizedRecordsCount = 0;
                int16_t syncAttemptsForCurrentRecord = 0;

                int32_t totalRecordsPendingSync = uxQueueMessagesWaiting(xSyncQueue);

                BaseType_t hasRemainingRecordsToSync = totalRecordsPendingSync > 0;

                // Iterar mientras la extensión esté PAIRED, queden registros de hidratación 
                // por sincronizar, los intentos de sincronización no superen a MAX_SYNC_ATTEMPTS
                //  y la cuenta total de registros sincronizados no supere a maxRecordsToSync.      
                while (PAIRED == ble_pair_status && hasRemainingRecordsToSync && 
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
                    ble_pair_status = ble_wait_for_state(PAIRED, true, 0);

                    hasRemainingRecordsToSync = synchronizedRecordsCount < totalRecordsPendingSync;
                }

                if (syncAttemptsForCurrentRecord >= MAX_SYNC_ATTEMPTS) {
                    vTaskDelay(SYNC_FAILED_BACKOFF_DELAY);
                    syncAttemptsForCurrentRecord = 0;
                }
            } else if (ADVERTISING == ble_pair_status)
            {
                BaseType_t advAttemptLimitReached = advAttemptCount > CONFIG_BLE_ADDITIONAL_ADV_ATTEMPT_COUNT;

                if (advAttemptLimitReached && NULL != xDeepSleepConditionEvents)
                {
                    xEventGroupSetBits(xDeepSleepConditionEvents, BLE_INACTIVE_BIT);
                    //TODO: determinar cuando ha pasado mas de cierto tiempo en ADV. 
                } else if (pdTRUE)
                {
                    // esp_err_t ble_shutdown_status = ble_driver_shutdown();
                }
            }
        } else if (params->useLazyBleEnable == pdFALSE || numOfRecordsPendingSync > 0)
        {
            ble_init_status = ble_driver_init(CONFIG_BLE_DEVICE_NAME);

            if (NULL == xSyncQueue || ble_init_status != ESP_OK)  {
                ESP_LOGE(TAG, "Error de inicializacion del driver de BLE (%s)", esp_err_to_name(ble_init_status));
                vTaskDelay(FAILED_INIT_RETRY_DELAY);
            }
        }

        if (ESP_OK == ble_init_status)
        {
            uint32_t notify_value = 0;
            BaseType_t power_mgmt_notify_received = xTaskNotifyWait(0, ULONG_MAX, &notify_value, (TickType_t) 0);

            if (pdPASS == power_mgmt_notify_received)
            {
                ESP_LOGI(TAG, "A power management notification was received by communications task. Notify value is %u", notify_value);

                // if (isPaired) 
                // {
                //     //TODO: desconectar BLE
                // } 

                esp_err_t ble_shutdown_status = ble_driver_shutdown();

                if (ESP_OK != ble_shutdown_status)
                {
                    ESP_LOGW(TAG, "Error al intentar detener el task de BLE: %s", esp_err_to_name(ble_shutdown_status));
                } else 
                {
                    esp_err_t shutdown_notify_status = set_module_ready_for_deep_sleep(communication_task_tag, true);

                    if (ESP_OK != shutdown_notify_status) 
                    {
                        ESP_LOGW(TAG, "Could not set Hx711 task is ready for deep sleep");
                    }
                }
            }
        }

        // Esperar un momento antes de volver a revisar el estado de 
        // la conexión, evitar acaparar todo el tiempo del scheduler 
        // en revisar la conexion.
        vTaskDelay(pdMS_TO_TICKS(250));
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
    static const mpu6050_config_t mpu_config = {
        .i2c_port_num = I2C_NUM_0,
        .address = CONFIG_MPU_I2C_ADDRESS,
        .enabled_interrupts = MPU_INT_DATA_RDY_BIT | MPU_INT_MOTION_BIT,
        .mpu_int = CONFIG_MPU_INT_GPIO,
        .isr_handler = mpu_data_rdy_isr,
        .min_read_interval_us = CONFIG_MPU_DATA_SAMPLE_INTERVAL_MS
    };

    esp_err_t mpu_init_status = mpu6050_init(&mpu_config);

    if (ESP_OK != mpu_init_status)
    {
        ESP_LOGW(TAG, "MPU6050 init error, status (%s)", esp_err_to_name(mpu_init_status));
    }

    return mpu_init_status;
}

static void IRAM_ATTR mpu_data_rdy_isr(void* arg)
{
    const mpu_handle_t mpu6050 = *((mpu_handle_t*) arg);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (NULL != mpu6050)
    {
        mpu6050_measures_t mpu6050_data = {};
        esp_err_t read_status = mpu6050_get_measurements(mpu6050, &mpu6050_data);

        if (ESP_OK == read_status)
        {
            uint8_t interrupt_status = 0x00;
            mpu6050_read_interrupt_status(mpu6050, &interrupt_status);

            if (NULL != xMpu6050DataQueue)
            {
                xQueueSendToBackFromISR(xMpu6050DataQueue, &mpu6050_data, &xHigherPriorityTaskWoken);
            }
        }
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

        if (is_battery_low(&battery_measurement) && NULL != xDeepSleepConditionEvents)
        {
            ESP_LOGI(TAG, "Bateria baja");
            xEventGroupSetBits(xDeepSleepConditionEvents, LOW_BAT_BIT);
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

    EventBits_t deepSleepConditions = xEventGroupGetBits(xDeepSleepConditionEvents);
    EventBits_t inactiveBits = NO_RECENT_HYDRATION_BIT | BLE_INACTIVE_BIT;

    BaseType_t isBatteryLow = (deepSleepConditions & LOW_BAT_BIT) == LOW_BAT_BIT;
    BaseType_t shouldEnterSleep = isBatteryLow || (deepSleepConditions & inactiveBits) == inactiveBits;  

    const int32_t sleepEnterAttemptCount = increment_sleep_attempt_count();
    BaseType_t hasReachedRetryLimit = sleepEnterAttemptCount >= CONFIG_MAX_DEEP_SLEEP_ATTEMPTS;

    ESP_LOGI(TAG, "Deep sleep enter attempt count: %d", sleepEnterAttemptCount);

    if (CONFIG_DEEP_SLEEP_ENABLED && (shouldEnterSleep || hasReachedRetryLimit))
    {
        BaseType_t areTasksReadyForDeepSleep = pdFALSE;
        esp_err_t sleep_check_status = is_ready_for_deep_sleep(&areTasksReadyForDeepSleep);

        if (ESP_OK == sleep_check_status)
        {
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
                    increment_sleep_attempt_count();

                    // Notificar a tasks que el chip va a iniciar deep sleep.
                    xTaskNotifyGive(xStorageTask);
                    xTaskNotifyGive(xCommunicationTask);
                    xTaskNotifyGive(xHydrationInferenceTask);
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
        BaseType_t wakeupFromEXT = pdFALSE;
        init_power_management(&wakeupFromEXT);

        if (NULL != outCommunicationTaskParams)
        {
            outCommunicationTaskParams->useLazyBleEnable = wakeupFromEXT;
        }
    }

    if (ESP_OK == setup_status)
    {
        // Inicializar los objetos de FreeRTOS usados para controlar 
        // los procesos de almacenar y obtener registros de hidratación. 
        xStorageQueue = xQueueCreate( MAX_RECORD_QUEUE_LEN, sizeof(hydration_record_t) );
        xSyncQueue = xQueueCreate( MAX_SYNC_QUEUE_LEN, sizeof(hydration_record_t) );
        xMpu6050DataQueue = xQueueCreate( MAX_MPU6050_DATA_QUEUE_LEN, sizeof(mpu6050_measures_t) );
        xHx711DataQueue = xQueueCreate( MAX_HX711_DATA_QUEUE_LEN, sizeof(hx711_measures_t) );

        xBatteryLevelQueue = xQueueCreate( BATTERY_LVL_QUEUE_SIZE, sizeof(battery_measurement_t) );
        xLatestHydrationTimestampQueue = xQueueCreate(LATEST_HYDR_TIMESTAMP_QUEUE_SIZE, sizeof(int64_t));
        xBleAdvertisingStartTime = xQueueCreate(BLE_ADV_DURATION_QUEUE_SIZE, sizeof(int64_t));

        xDeepSleepConditionEvents = xEventGroupCreate();

        if (NULL == xStorageQueue || NULL == xSyncQueue || NULL == xMpu6050DataQueue || NULL == xHx711DataQueue|| NULL == xBatteryLevelQueue) 
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

    if (ESP_OK == setup_status)
    {
        setup_status = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    }

    return setup_status;
}

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

static esp_err_t transfer_storage_queue_to_sync(void)
{
    int32_t recordsInStorageQueue = uxQueueMessagesWaiting(xStorageQueue);

    if (recordsInStorageQueue == 0) return ESP_OK;

    esp_err_t status = ESP_OK;
    hydration_record_t xReceivedRecord = { 0, 0, 0, 0 };
    ESP_LOGD(TAG, "Se encontraron %d registros en queue de almacenamiento durante sincronizacion", recordsInStorageQueue);

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
                status = ESP_FAIL;
                ESP_LOGE(TAG, "El registro no pudo ser enviado para sincronizacion");
            }
        } else {
            status = ESP_FAIL;
            ESP_LOGE(
                TAG, 
                "Esperaba encontrar %d registros en xStorageQueue, pero no fue posible obtenerlos", 
                recordsInStorageQueue
            );
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
