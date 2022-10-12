#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp32/rom/ets_sys.h>

#include "hx711.h"

static const char* TAG = "HX711";

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static const int16_t NUM_DATA_BITS = 24;

static int32_t base_measurement = 0;

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
