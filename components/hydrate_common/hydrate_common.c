#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <esp_log.h>

#include "hydrate_common.h"

#define NUM_SENSOR_MEASURES_PER_SECOND 5
#define MAX_SENSOR_DATA_BUF_SECONDS 5
#define MAX_SENSOR_DATA_BUF_LEN (MAX_SENSOR_DATA_BUF_SECONDS * NUM_SENSOR_MEASURES_PER_SECOND)

static const char* TAG = "HYDRATE";

static sensor_measures_t sensorDataQueue[MAX_SENSOR_DATA_BUF_LEN] = {};
static int32_t sensorDataQueueSize = 0;
static int32_t sensorDataQueueTailIdx = -1;

static const int32_t min_water_volume_ml = 10;
static const int32_t max_water_volume_ml = 250;
static const float min_temperature_celsius = -40.0f;
static const float max_temperature_celsius = 75.0f;
static const int32_t min_battery_level = 0;
static const int32_t max_battery_level = 100;

static const uint16_t min_volume_delta_ml = 10;
static const uint16_t max_volume_delta_ml = 250;

static const float accel_maintained_threshold = 0.01f;

static const int32_t lifted_raw_weight = 25000;

static const int64_t hydration_cooldown_ms = 2000;

static int64_t latest_hydration_timestamp_ms = 0;

static const uint32_t rngSeed = 390; 

// Utils
hydration_record_t create_random_record(void);
static float constrain(float number, float min, float max) ;
static int32_t random_next_int(int32_t range, int32_t min);
static bool is_accel_maintained(mpu6050_acce_value_t* previous, mpu6050_acce_value_t* current);

esp_err_t hydration_record_to_string(char* out_buf, const hydration_record_t* hydration_record)
{
    if (NULL == out_buf || NULL == hydration_record) 
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    snprintf(
        out_buf, 
        100, 
        "Registro de Hidratacion { water_ml: %i, temp_celsius: %i, bat_percent: %i, time_s: %lld}",
        hydration_record->water_amount, hydration_record->temperature, 
        hydration_record->battery_level, hydration_record->timestamp
    );

    return ESP_OK;
}

esp_err_t record_measurements_timestamp(sensor_measures_t* measurement)
{
    if (NULL == measurement) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    struct timeval now;
    gettimeofday(&now, NULL);

    int64_t millis = (((int64_t) now.tv_sec) * 1000) + (((int64_t) now.tv_usec) / 1000);
    ESP_LOGD(TAG, "Millis since UNIX epoch: %lld", millis);
    measurement->timestamp_ms = millis;

    return ESP_OK;
}

esp_err_t add_sensor_measurements(const sensor_measures_t* const measurement)
{
    esp_err_t status = ESP_OK;

    if (NULL == measurement)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        if (sensorDataQueueSize < MAX_SENSOR_DATA_BUF_LEN) 
        {
            ++sensorDataQueueSize;
        }

        ++sensorDataQueueTailIdx;
        sensorDataQueueTailIdx %= MAX_SENSOR_DATA_BUF_LEN;
    }

    if (sensorDataQueueTailIdx < 0 || sensorDataQueueTailIdx >= MAX_SENSOR_DATA_BUF_LEN)
    {
        status = ESP_ERR_INVALID_STATE;
    }

    if (ESP_OK == status)
    {
        sensorDataQueue[sensorDataQueueTailIdx] = *measurement;
        ESP_LOGD(TAG, "Sensor data item was set at index = %d", (sensorDataQueueTailIdx - 1));
    }

    return status;
}

BaseType_t infer_hydration_from_sensors(hydration_record_t* const out_record)
{
    if (NULL == out_record)
    {
        return pdFALSE;
    }

    // Verificar si ya sea ha registrado consumo de agua en los últimos x segundos.
    // si es así, no es posible que el usuario esté tomando agua nuevamente.
    int64_t inferenceEnableTimestamp = latest_hydration_timestamp_ms + hydration_cooldown_ms;
    BaseType_t hydration_already_recorded = (sensorDataQueue[sensorDataQueueTailIdx].timestamp_ms) < inferenceEnableTimestamp;

    if (hydration_already_recorded)
    {
        ESP_LOGE(
            TAG, "Hydration record already created a few seconds ago. %lld ms < %lld ms",
            sensorDataQueue[sensorDataQueueTailIdx].timestamp_ms, inferenceEnableTimestamp
        );
        return false;
    }

    int32_t unusedQueueItems = MAX_SENSOR_DATA_BUF_LEN - sensorDataQueueSize; // 20
    int32_t sensorDataQueueHeadIdx = (sensorDataQueueTailIdx + 1 + unusedQueueItems) % MAX_SENSOR_DATA_BUF_LEN;

    ESP_LOGD(TAG, "Sensor data queue size = %d, head index = %d, tail index = %d", sensorDataQueueSize, sensorDataQueueHeadIdx,  sensorDataQueueTailIdx);

    size_t measures_in_hydration_record = 0;
    float temperature_accumulator = 0.0f;

    int32_t start_stationary_duration_ms = 0;
    int32_t lifted_duration_ms = 0;
    int32_t end_stationary_duration_ms = 0;

    uint16_t initial_volume_ml = 0;
    int32_t final_volume_ml = 0;
    uint16_t volume_delta_ml = 0;

    int32_t record_index = sensorDataQueueHeadIdx;

    while (record_index != sensorDataQueueTailIdx)
    {
        const sensor_measures_t* current_measurement = &sensorDataQueue[record_index];
        const sensor_measures_t* next_measurement = &sensorDataQueue[(record_index + 1) % sensorDataQueueSize];

        const int64_t measurement_duration_ms = next_measurement->timestamp_ms - current_measurement->timestamp_ms; 

        // Verificar la diferencia de peso. Para representar un consumo de agua, 
        // debería cambiar una sola vez a lo largo de sensorDataQueue y tener un delta
        // positivo (si es negativo, significa que aumentó el volumen total?).
        if (current_measurement->weight_measurements.raw_weight > lifted_raw_weight) 
        {
            if (lifted_duration_ms <= 0) 
            {
                start_stationary_duration_ms += measurement_duration_ms;
                initial_volume_ml = current_measurement->weight_measurements.volume_ml;
            } else
            {
                end_stationary_duration_ms += measurement_duration_ms;
                final_volume_ml = current_measurement->weight_measurements.volume_ml;
            }
        } else 
        {
            lifted_duration_ms += measurement_duration_ms;
        }

        temperature_accumulator += constrain(current_measurement->accel_gyro_measurements.temperature, min_temperature_celsius, max_temperature_celsius);

        ++measures_in_hydration_record;

        ++record_index;
        record_index %= MAX_SENSOR_DATA_BUF_LEN;
    }

    volume_delta_ml = initial_volume_ml - final_volume_ml;

    ESP_LOGI(TAG, "Data: [vol. delta = %d, start_ms = %d, lifted_ms = %d, end_ms = %d]", volume_delta_ml, start_stationary_duration_ms, lifted_duration_ms, end_stationary_duration_ms);

    BaseType_t started_ended_stationary = start_stationary_duration_ms > 0 && lifted_duration_ms > 0 && end_stationary_duration_ms > 0;
    BaseType_t volume_changed = volume_delta_ml > min_volume_delta_ml && volume_delta_ml < max_volume_delta_ml;

    BaseType_t measures_represent_hydration = volume_changed && started_ended_stationary;

    if (measures_represent_hydration) 
    {
        out_record->water_amount = volume_delta_ml;

        float temperature_celsius = temperature_accumulator / measures_in_hydration_record;
        out_record->temperature = (int16_t) temperature_celsius * 100;

        latest_hydration_timestamp_ms = sensorDataQueue[sensorDataQueueTailIdx].timestamp_ms;
        out_record->timestamp = latest_hydration_timestamp_ms * 1000;
    }

    return measures_represent_hydration;
}

hydration_record_t create_random_record(void) 
{
    uint16_t rand_water_amount = random_next_int(max_water_volume_ml, min_water_volume_ml);
    int16_t rand_temperature = random_next_int(max_temperature_celsius, min_temperature_celsius);
    uint8_t rand_battery_lvl = random_next_int(max_battery_level, min_battery_level);
    int32_t now = (int64_t) time(NULL);

    hydration_record_t randomRecord = { 
        .water_amount = rand_water_amount, 
        .temperature  = rand_temperature, 
        .battery_level  = rand_battery_lvl, 
        .timestamp    = now 
    };

    return randomRecord;
}

static float constrain(float number, float min, float max) 
{
    if (min > max) {
        float temp = min;
        min = max;
        max = temp;
    }

    if (number > max) {
        return max;
    } else if (number < min) {
        return min;
    } else {
        return number;
    }
}

static int32_t random_next_int(int32_t range, int32_t min) 
{
    static BaseType_t hasAlreadySeeded = pdFALSE;

    if (!hasAlreadySeeded) {
        srand(rngSeed);
        hasAlreadySeeded = pdTRUE;
    }

    int32_t random_number = rand() % range + min;

    return random_number;
}

static bool is_accel_maintained(mpu6050_acce_value_t* previous, mpu6050_acce_value_t* current)
{
    bool is_x_axis_maintained = fabsf(previous->acce_x - current->acce_x) < accel_maintained_threshold; 
    bool is_y_axis_maintained = fabsf(previous->acce_y - current->acce_y) < accel_maintained_threshold; 
    bool is_z_axis_maintained = fabsf(previous->acce_z - current->acce_z) < accel_maintained_threshold; 

    return is_x_axis_maintained && is_y_axis_maintained && is_z_axis_maintained;
}
