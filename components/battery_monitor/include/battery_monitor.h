#ifndef _BATTERY_MONITOR_H_
#define _BATTERY_MONITOR_H_

#include <stdbool.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <soc/adc_channel.h>
#include <esp_adc_cal.h>

typedef struct {
    uint8_t remaining_charge;
    int32_t aprox_ms_until_discharge;
} battery_measurement_t;

esp_err_t battery_monitor_init(void);

esp_err_t get_battery_level(battery_measurement_t* out_bat_measurement);

BaseType_t is_battery_low(const battery_measurement_t* const bat_measurement);

esp_err_t multi_sample_battery_level(battery_measurement_t* out_bat_measurement, size_t number_of_samples);

#endif /** _BATTERY_MONITOR_H_ **/
