#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "battery_monitor.h"

static const char* TAG = "BATTERY";

#define DEFAULT_VREF adc_vref_to_gpio(unit, CONFIG_BATTERY_INPUT_GPIO)

static const adc_channel_t bat_level_channel = ADC1_GPIO35_CHANNEL;
static const adc_bits_width_t bit_width = ADC_WIDTH_BIT_12;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_atten_t attenuation = ADC_ATTEN_DB_11;

static esp_adc_cal_characteristics_t* adc_chars;

static const uint32_t battery_charge_mv[CONFIG_BATTERY_CHARGE_CURVE_POINT_COUNT] = 
{
    3000, 3125, 3250, 3400, 3525, 3600, 3675, 3710, 3750, 3775, 3800, 3815, 3835, 3860, 3880, 3900, 3920, 3945, 3975, 4025, 4100
};

static const uint8_t battery_charge_levels[CONFIG_BATTERY_CHARGE_CURVE_POINT_COUNT] = 
{
    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100
};

static void find_calibration_in_efuse(void);

static esp_err_t sample_adc(uint32_t* out_voltage_mv, size_t num_samples);

esp_err_t battery_monitor_init(void)
{
    esp_err_t status = ESP_OK;

    find_calibration_in_efuse();

    if (ESP_OK == status)
    {
        if (unit == ADC_UNIT_1)
        {
            status = adc1_config_width(bit_width);
            status = adc1_config_channel_atten(bat_level_channel, attenuation);
        } else 
        {
            status = adc2_config_channel_atten((adc2_channel_t) bat_level_channel, attenuation);
        }
    }

    if (ESP_OK == status) 
    {
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t value_type = esp_adc_cal_characterize(unit, attenuation, bit_width, DEFAULT_VREF, adc_chars);

        if (NULL == adc_chars) 
        {
            return ESP_FAIL;
        }

        if (ESP_ADC_CAL_VAL_EFUSE_TP == value_type) {
            ESP_LOGI(TAG, "Characterized using Two Point Value\n");
        } else if (ESP_ADC_CAL_VAL_EFUSE_VREF == value_type) 
        {
            ESP_LOGI(TAG, "Characterized using eFuse Vref\n");
        } else {
            ESP_LOGI(TAG, "Characterized using Default Vref\n");
        }
    }

    return status;
}

static void find_calibration_in_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    if (ESP_OK == esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP))
    {
        ESP_LOGI(TAG, "eFuse Two Point: soportado");
    } else 
    {
        ESP_LOGI(TAG, "eFuse Two Point: no soportado");
    }
#else
#error "Este driver fue configurado para ESP32/ESP32S2"
#endif
}

static esp_err_t sample_adc(uint32_t* out_voltage_mv, size_t num_samples) 
{
    if (NULL == out_voltage_mv || num_samples < 1) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t adc_read_status = ESP_OK;

    uint32_t adc_reading = 0; 

    for (size_t i = 0; i < num_samples; ++i) 
    {
        if (ADC_UNIT_1 == unit) 
        {
            adc_reading += adc1_get_raw((adc1_channel_t) bat_level_channel);
        } else 
        {
            int32_t raw;
            adc2_get_raw((adc2_channel_t) bat_level_channel, bit_width, &raw);
            adc_reading += raw;
        }

        ESP_LOGI(TAG, "ADC%d aggregated voltage: %d mV / %d", ADC_UNIT_1 + 1, adc_reading, num_samples);
    }

    adc_reading /= num_samples;

    *out_voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    return adc_read_status;
}

esp_err_t get_battery_level(battery_measurement_t* out_bat_measurement)
{
    esp_err_t adc_read_status = ESP_OK;

    adc_read_status = multi_sample_battery_level(out_bat_measurement, 1);

    return adc_read_status;
}

esp_err_t multi_sample_battery_level(battery_measurement_t* out_bat_measurement, size_t number_of_samples)
{
    uint32_t battery_voltage = 0;

    esp_err_t adc_read_status = ESP_OK;

    if (ESP_OK == adc_read_status) 
    {
        adc_read_status = sample_adc(&battery_voltage, number_of_samples);

        if (ESP_OK == adc_read_status)
        {
            ESP_LOGI(TAG, "ADC%d Channel[%d] voltage: %d mV", ADC_UNIT_1 + 1, bat_level_channel, battery_voltage);
        }    
    }

    if (ESP_OK == adc_read_status) 
    {
        // Asegurar que CONFIG_BATTERY_MAX_CHARGE_VOLTS es mayor a 
        // CONFIG_BATTERY_MIN_CHARGE_VOLTS.
        ESP_ERROR_CHECK(!(CONFIG_BATTERY_MAX_CHARGE_VOLTS > CONFIG_BATTERY_MIN_CHARGE_VOLTS));

        int32_t current_charge_mv = battery_voltage - CONFIG_BATTERY_MIN_CHARGE_VOLTS;

        int32_t charge_percent = -1;
        
        for (size_t i = 0; i < CONFIG_BATTERY_CHARGE_CURVE_POINT_COUNT; ++i)
        {
            if (current_charge_mv <= battery_charge_mv[i])
            {
                charge_percent = battery_charge_levels[i];
            }
        }

        // Saturar el valor a 0% o 100%, según sea el caso.
        if (charge_percent > 100) 
        {
            charge_percent = 100;
        } else if (charge_percent < 0) 
        {
            charge_percent = 0;
        }

        out_bat_measurement->remaining_charge = charge_percent;
    }

    return adc_read_status;
}