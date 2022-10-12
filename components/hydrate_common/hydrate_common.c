#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <freertos/FreeRTOS.h>

#include "hydrate_common.h"

static const uint32_t seed = 234; 
static BaseType_t hasAlreadySeeded = pdFALSE;

static const int32_t min_water_amount = 10;
static const int32_t max_water_amount = 1000;
static const int32_t min_temperature = -40;
static const int32_t max_temperature = 75;
static const int32_t min_battery_level = 0;
static const int32_t max_battery_level = 100;

static int32_t random_next_int(int32_t range, int32_t min);

hydration_record_t create_random_record(void) {

    uint16_t rand_water_amount = random_next_int(max_water_amount, min_water_amount);
    int16_t rand_temperature = random_next_int(max_temperature, min_temperature);
    uint8_t rand_battery_lvl = random_next_int(max_battery_level, min_battery_level);

    int64_t now = (int64_t) time(NULL);

    hydration_record_t randomRecord = { 
        .water_amount = rand_water_amount, 
        .temperature  = rand_temperature, 
        .battery_level  = rand_battery_lvl, 
        .timestamp    = now 
    };

    return randomRecord;
}

static int32_t random_next_int(int32_t range, int32_t min) {
    if (!hasAlreadySeeded) {
        srand(seed);
        hasAlreadySeeded = pdTRUE;
    }

    int32_t random_number = rand() % range + min;

    return random_number;
}

esp_err_t hydration_record_to_string(const char* out_buf, const hydration_record_t* hydration_record)
{
    if (NULL == out_buf || NULL == hydration_record) 
    {
        return ESP_ERR_INVALID_ARG;
    }
    
    snprintf(
        out_buf, 
        sizeof(out_buf), 
        "Registro de Hidratacion { water_ml: %i, temp_celsius: %i, bat_percent: %i, time_s: %lld}",
        hydrationRecord->water_amount, hydrationRecord->temperature, 
        hydrationRecord->battery_level, hydrationRecord->timestamp
    );

    return ESP_OK;
}

void start_measurement_period(const sensor_measures_t* measurement)
{
    if (NULL == measurement) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    measurement->start_time_ms = (int64_t) time(NULL);
}

void end_measurement_period(const sensor_measures_t* measurement)
{
    if (NULL == measurement) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    measurement->end_time_ms = (int64_t) time(NULL);
}

bool 

