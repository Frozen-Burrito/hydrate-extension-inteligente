#ifndef _HX711_H_
#define _HX711_H_

#include <driver/gpio.h>
#include <stdbool.h>
#include <math.h>
#include <esp_err.h>

typedef enum {
    HX711_GAIN_A_128 = 0,
    HX711_GAIN_B_32,
    HX711_GAIN_A_64,
} hx711_gain_t;

typedef struct {
    gpio_num_t data_out;
    gpio_num_t pd_sck;
    hx711_gain_t gain;
} hx711_t;

typedef struct {
    int32_t raw_weight;
    uint16_t volume_ml;
} hx711_measures_t;

esp_err_t hx711_init(hx711_t* device);

esp_err_t hx711_set_power(hx711_t* device, bool down);

esp_err_t hx711_set_gain(hx711_t* device, hx711_gain_t gain);

esp_err_t hx711_is_data_ready(hx711_t* device, bool* ready);

esp_err_t hx711_wait_for_data(hx711_t* device, size_t timeout_ms);

esp_err_t hx711_read_data(hx711_t* device, int32_t* data);

esp_err_t hx711_read_average(hx711_t* device, size_t num_samples, int32_t* data);

esp_err_t hx711_get_measurements(hx711_t* device, hx711_measures_t* const out_measurements, size_t timeout_ms);

esp_err_t hx711_calibrate(hx711_t* device);

uint16_t hx711_volume_ml_from_measurement(const int32_t* measurement);

#endif /** _HX711_H_ **/
