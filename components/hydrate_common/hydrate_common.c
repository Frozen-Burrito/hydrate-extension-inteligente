#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "freertos/FreeRTOS.h"

#include "hydrate-common.h"

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

    int32_t now = (int64_t) time(NULL);

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