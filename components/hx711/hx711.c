#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp32/rom/ets_sys.h>

#include "hx711.h"

static const char* TAG = "HX711";

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static const int16_t NUM_DATA_BITS = 24;

static const int32_t lifted_raw_weight = 25000;
static const int32_t raw_weight_with_container = 42000;
static const int32_t raw_weight_per_ml = 150;

static uint32_t read_raw(gpio_num_t data_out, gpio_num_t pd_sck, hx711_gain_t gain) 
{
    ESP_LOGD(TAG, "Reading raw data (DOUT = %d, SCK = %d, GAIN = %d)", data_out, pd_sck, gain);
    ESP_LOGD(TAG, "Total PD SCK pulses = %d", (NUM_DATA_BITS + gain + 1));

    portENTER_CRITICAL(&mux);

    uint32_t data = 0;
    for (int16_t i = 0; i < NUM_DATA_BITS; ++i) 
    {
        gpio_set_level(pd_sck, 1);
        ets_delay_us(1);
        data |= gpio_get_level(data_out) << (NUM_DATA_BITS - 1 - i);
        gpio_set_level(pd_sck, 0);
        ets_delay_us(1);
    }

    for (int16_t i = 0; i <= gain; ++i)
    {
        gpio_set_level(pd_sck, 1);
        ets_delay_us(1);
        gpio_set_level(pd_sck, 0);
        ets_delay_us(1);
    }

    portEXIT_CRITICAL(&mux);

    return data;
}

esp_err_t hx711_init(hx711_t* device) 
{
    if (NULL == device) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t init_status = ESP_OK;

    if (ESP_OK == init_status) 
    {
        init_status = gpio_set_direction(device->data_out, GPIO_MODE_INPUT);
    }

    if (ESP_OK == init_status) 
    {
        init_status = gpio_set_direction(device->pd_sck, GPIO_MODE_OUTPUT);
    }

    if (ESP_OK == init_status) 
    {
        init_status = hx711_set_power(device, false);
    }

    if (ESP_OK == init_status) 
    {
        init_status = hx711_set_gain(device, device->gain);
    }

    return init_status;
}

esp_err_t hx711_set_power(hx711_t* device, bool down) 
{
    if (NULL == device) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t status = gpio_set_level(device->pd_sck, down);
    vTaskDelay(1);

    return status;
}

esp_err_t hx711_set_gain(hx711_t* device, hx711_gain_t gain) 
{
    if (NULL == device || gain > HX711_GAIN_A_64) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t status = ESP_OK;

    status = hx711_wait_for_data(device, 10);

    if (ESP_OK == status) 
    {
        read_raw(device->data_out, device->pd_sck, device->gain);
        device->gain = gain;
    }

    return status;
}

esp_err_t hx711_is_data_ready(hx711_t* device, bool* ready) 
{
    if (NULL == device || NULL == ready) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    *ready = !(gpio_get_level(device->data_out));

    return ESP_OK;
}

esp_err_t hx711_wait_for_data(hx711_t* device, size_t timeout_ms) 
{
    int64_t start_us = esp_timer_get_time();
    int64_t timeout_us = timeout_ms * 1000;

    while ((esp_timer_get_time() - start_us) < timeout_us)
    {
        if (!gpio_get_level(device->data_out))
        {
            return ESP_OK;
        }
        vTaskDelay(1);
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t hx711_read_data(hx711_t* device, int32_t* data) 
{
    if (NULL == device || NULL == data) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t raw_data = read_raw(device->data_out, device->pd_sck, device->gain);

    if (raw_data & 0x800000) 
    {
        raw_data |= 0xFF000000;
    }

    *data = *((int32_t*) &raw_data);

    return ESP_OK;
}

esp_err_t hx711_read_average(hx711_t* device, size_t num_samples, int32_t* data) 
{
    if (NULL == device || num_samples == 0 || NULL == data) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t v;
    *data = 0;
    for (size_t i = 0; i < num_samples; ++i) 
    {
        esp_err_t read_status = ESP_OK;

        read_status = hx711_wait_for_data(device, 125);

        if (ESP_OK != read_status) 
        {
            ESP_LOGW(TAG, "Error while waiting for data to become available (%s)", esp_err_to_name(read_status));
            return read_status;
        }

        read_status = hx711_read_data(device, &v);

        if (ESP_OK != read_status) 
        {
            ESP_LOGW(TAG, "Error while reading available data (%s)", esp_err_to_name(read_status));
            return read_status;
        } 

        *data += v;
    }

    *data /= num_samples;

    ESP_LOGD(TAG, "HX711 read average = %d (%d samples)", *data, num_samples);

    return ESP_OK;
}

esp_err_t hx711_get_measurements(hx711_t* device, hx711_measures_t* const out_measurements, size_t timeout_ms)
{
    esp_err_t status = ESP_OK; 

    if (NULL == device || NULL == out_measurements) 
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status && timeout_ms > 0)
    {
        status = hx711_wait_for_data(device, timeout_ms);
    }

    uint32_t raw_data = 0;

    if (ESP_OK == status) 
    {
        raw_data = read_raw(device->data_out, device->pd_sck, device->gain);

        if (raw_data & 0x800000) 
        {
            raw_data |= 0xFF000000;
        }
    }

    if (ESP_OK == status)
    {
        out_measurements->raw_weight = (int32_t) raw_data;

        out_measurements->volume_ml = hx711_volume_ml_from_measurement(&out_measurements->raw_weight);
    }

    return status;
}

uint16_t hx711_volume_ml_from_measurement(const int32_t* measurement)
{
    // Considerar el peso base del dispositivo, el peso cuando está 
    // siendo sostemido por arriba y la ligera diferencia entre el peso
    // base y el peso con contenedor.
    uint16_t volume_ml = 0;

    if (NULL == measurement) 
    {
        ESP_LOGW(TAG, "Measurement for raw weight was null");
        volume_ml = 0;
    }

    bool is_not_lifted = *measurement > lifted_raw_weight;
    bool has_container = *measurement > raw_weight_with_container;

    if (is_not_lifted) 
    {
        if (has_container) 
        {
            int32_t raw_liquid_weight = *measurement - raw_weight_with_container;
            assert(raw_liquid_weight >= 0);

            volume_ml = raw_liquid_weight / raw_weight_per_ml;

        } else 
        {
            // Si no tiene el peso del contenedor, no puede tener un volumen de 
            // liquido.
            ESP_LOGI(TAG, "No hay un contenedor de agua sobre la extension");
            volume_ml = 0;
        }
    } else 
    {
        ESP_LOGI(TAG, "El dispositivo esta levantado");
    }

    return volume_ml;
}
