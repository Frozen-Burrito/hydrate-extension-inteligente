#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "power-manager.h"

static const char* TAG = "POWER_MGMT";

static RTC_DATA_ATTR struct timeval sleep_enter_time;

static volatile bool has_configured_wakeup_sources = false;

static int32_t latest_sleep_duration_ms = 0;

static const char* wakeup_cause_to_name(const esp_sleep_wakeup_cause_t wakeup_cause);

static void start_ulp_program(void);

void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entrando en deep sleep");

    if (!has_configured_wakeup_sources) 
    {
        ESP_LOGW(TAG, "Entrando a deep sleep sin haber configurado wakeup sources");
        return;
    }

#if CONFIG_IDF_TARGET_ESP32
    // Aislar el pin GPIO12 pin de circuitos externos. Esto es necesario para módulos
    // que contienen un resisor pull-up externo en GPIO12 (como el ESP32-WROVER)
    // para minimizar el consumo de corriente.
    rtc_gpio_isolate(GPIO_NUM_12);
#endif

    // Registar el timestamp de inicio de sueño profundo.
    gettimeofday(&sleep_enter_time, NULL);

    start_ulp_program();

    esp_deep_sleep_start();
}

esp_err_t config_auto_light_sleep(bool enable_auto_light_sleep)
{
    esp_err_t light_sleep_setup_status = ESP_OK;

    if (ESP_OK == light_sleep_setup_status)
    {
        esp_pm_config_esp32_t pm_config = {
            .max_freq_mhz = 160,
            .min_freq_mhz = 80,
            .light_sleep_enable = enable_auto_light_sleep
        };

        light_sleep_setup_status = esp_pm_configure(&pm_config);
    }
}

esp_err_t setup_sleep_wakeup_sources(void)
{
    // IMPORTANTE: en revisiones 0 y 1 de ESP32, EXT0 no es compatible
    // con fuentes de wakeup de ULP y touch pad. Por esto se usa EXT1 en 
    // este caso.
    esp_err_t setup_status = ESP_OK;

    if (ESP_OK == setup_status)
    {
        // Activar wakeup desde EXT1.
        const gpio_num_t ext_wakeup_pin = CONFIG_POWER_ON_GPIO_NUM;
        const uint64_t ext1_wakeup_power_on_mask = 1ULL << ext_wakeup_pin;

        ESP_LOGI(TAG, "Activando wakeup desde EXT1 con GPIO%d", ext_wakeup_pin);

        setup_status = esp_sleep_enable_ext1_wakeup(ext1_wakeup_power_on_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    }

    if (ESP_OK == setup_status && CONFIG_ULP_MOVEMENT_WAKEUP) 
    {
        // Activar wakeup por programa de ULP.
        ESP_LOGI(TAG, "Activando wakeup por ULP");
        setup_status = esp_sleep_enable_ulp_wakeup();
    }

    has_configured_wakeup_sources = (ESP_OK == setup_status);

    return setup_status;
}

void calculate_sleep_duration(void) 
{
    struct timeval now;
    gettimeofday(&now, NULL);

    latest_sleep_duration_ms = ((now.tv_sec - sleep_enter_time.tv_sec) * 1000) + ((not.tv_usec - sleep_enter_time.tv_usec) / 1000);
}

void log_sleep_wakeup_cause(void)
{
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();

    ESP_LOGI(TAG, wakeup_cause_to_name(wakeup_cause));
}

static const char* wakeup_cause_to_name(const esp_sleep_wakeup_cause_t wakeup_cause)
{
    switch (wakeup_cause) 
    {
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            return "WAKEUP UNDEFINED, reset not caused by sleep wakeup.";
        case ESP_SLEEP_WAKEUP_EXT0:
            return "WAKEUP from EXT0";
        case ESP_SLEEP_WAKEUP_EXT1:
            return "WAKEUP from EXT1";
        case ESP_SLEEP_WAKEUP_TIMER:
            return "WAKEUP from timer";
        case ESP_SLEEP_WAKEUP_ULP:
            return "WAKEUP caused by ULP program";
        default:
            return "WAKEUP wakeup was caused by unsupported source";
    }
}

static void start_ulp_program(void)
{

}
