#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

#include "ulp_main.h"
#include "power-manager.h"

static const char* TAG = "POWER_MGMT";

#define MAX_MODULES_REQUIRING_SHUTDOWN 10

static RTC_DATA_ATTR struct timeval sleep_enter_time;

static volatile bool has_configured_wakeup_sources = false;

static int32_t latest_sleep_duration_ms = 0;

static EventGroupHandle_t xReadyForDeepSleepEvents = NULL;

static const char* modules_requiring_shutdown[MAX_MODULES_REQUIRING_SHUTDOWN] = {};
static size_t num_of_modules_requiring_shutdown = 0;

static TickType_t await_modules_before_sleep_timeout = pdMS_TO_TICKS(5000);

// ULP
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

static uint32_t ulp_wakeup_period_ms = 100;

static void init_ulp_program(void);
static void start_ulp_program(void);

static void init_ulp_rtc_gpio(void);
static void deinit_ulp_rtc_gpio(void);

// Utilidades
static void calculate_sleep_duration(void);
static const char* wakeup_cause_to_name(const esp_sleep_wakeup_cause_t wakeup_cause);
static size_t index_of_module_with_shutdown(const char* const module_tag);

esp_err_t after_wakeup(void)
{
    calculate_sleep_duration();

    esp_err_t wakeup_status = ESP_OK;

    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();

    const char* cause_name = wakeup_cause_to_name(wakeup_cause);

    switch (wakeup_cause)
    {
    case ESP_SLEEP_WAKEUP_ULP:
        //TODO: manejar wakeups por ULP
        ESP_LOGI(TAG, "Sistema despertado de deep sleep por ULP, tiempo en deep sleep: %d", latest_sleep_duration_ms);
        wakeup_status = ESP_OK;
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        //TODO: manejo especifico de wakeups por EXT1
        ESP_LOGI(TAG, "Sistema despertado de deep sleep por EXT1, tiempo en deep sleep: %d", latest_sleep_duration_ms);
        wakeup_status = ESP_OK;
        break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        // El reset no fue causado por deep sleep. Iniciar el programa del ULP.
        if (CONFIG_ULP_MOVEMENT_WAKEUP)
        {
            init_ulp_program();
        }
        break;
    default:
        ESP_LOGI(TAG, "Sistema despertado de deep sleep, causa inesperada: %s", cause_name);
        break;
    }

    esp_err_t wakeup_sources_status = setup_wakeup_sources();

    if (ESP_OK != wakeup_sources_status)
    {
        ESP_LOGW(TAG, "Error al configurar fuentes de activacion de deep sleep (%s)", esp_err_to_name(wakeup_sources_status));
    }

    deinit_ulp_rtc_gpio();
    
    xReadyForDeepSleepEvents = xEventGroupCreate();

    return wakeup_status;
}

void begin_deep_sleep_when_ready()
{
    if (!has_configured_wakeup_sources) 
    {
        ESP_LOGW(TAG, "Se intento comenzar deep sleep sin antes haber configurado wakeup sources");
        return;
    }

    EventBits_t resources_to_wait = 0x00;

    for (size_t i = 0; i < num_of_modules_requiring_shutdown; ++i)
    {
        resources_to_wait |= (1 << i);
    }

    EventBits_t modules_ready_bits = xEventGroupWaitBits(
        xReadyForDeepSleepEvents,
        resources_to_wait,
        pdTRUE,
        pdTRUE,
        await_modules_before_sleep_timeout
    );

    if (resources_to_wait != modules_ready_bits)
    {
        ESP_LOGW(TAG, "No todos los modulos estan listos para deep sleep, timeout.");
    }

    ESP_LOGI(TAG, "Comenzando deep sleep");

    if (CONFIG_ULP_MOVEMENT_WAKEUP)
    {
        start_ulp_program();
    }

    // Registar el timestamp de inicio de sueño profundo.
    gettimeofday(&sleep_enter_time, NULL);

    esp_deep_sleep_start();
}

esp_err_t setup_light_sleep(bool enable_auto_light_sleep)
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

    return light_sleep_setup_status;
}

esp_err_t setup_wakeup_sources(void)
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

esp_err_t add_module_for_deep_sleep_confirmation(const char* const module_tag)
{
    esp_err_t status = ESP_OK;

    size_t existing_module_idx = index_of_module_with_shutdown(module_tag);

    if (existing_module_idx == -1) 
    {
        modules_requiring_shutdown[num_of_modules_requiring_shutdown] = module_tag;
        ++num_of_modules_requiring_shutdown;
    } else 
    {
        status = ESP_FAIL;
    }

    return status;
}

esp_err_t set_module_ready_for_deep_sleep(const char* const module_tag, bool is_ready)
{
    esp_err_t status = ESP_OK;

    size_t existing_module_idx = index_of_module_with_shutdown(module_tag);
    bool module_is_registered = existing_module_idx >= -1;

    if (module_is_registered) 
    {
        EventBits_t bit_for_module = 1 << existing_module_idx;

        xEventGroupSetBits(xReadyForDeepSleepEvents, bit_for_module);
    } else 
    {
        status = ESP_FAIL;
    }

    return status;
}

static size_t index_of_module_with_shutdown(const char* const module_tag)
{
    size_t index_of_module = -1;

    for (size_t i = 0; i < num_of_modules_requiring_shutdown; ++i) 
    {
        bool is_same_module = 0 == strcmp(module_tag, modules_requiring_shutdown[i]);
        if (is_same_module) 
        {
            index_of_module = i;
            break;
        }
    }

    return index_of_module;
}

static void calculate_sleep_duration(void) 
{
    struct timeval now;
    gettimeofday(&now, NULL);

    latest_sleep_duration_ms = ((now.tv_sec - sleep_enter_time.tv_sec) * 1000) + ((now.tv_usec - sleep_enter_time.tv_usec) / 1000);
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

static void init_ulp_program(void)
{
    size_t program_size = (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t);
    esp_err_t ulp_load_status = ulp_load_binary(0, ulp_main_bin_start, program_size);

    if (ESP_OK != ulp_load_status)
    {
        ESP_LOGW(TAG, "El binario para el programa del ULP no pudo ser cargado (%s)", esp_err_to_name(ulp_load_status));
        return;
    } 

    uint32_t ulp_wakeup_period_us = ulp_wakeup_period_ms * 1000;
    esp_err_t set_wakeup_period_status = ulp_set_wakeup_period(0, ulp_wakeup_period_us);

    if (ESP_OK != set_wakeup_period_status)
    {
        ESP_LOGW(TAG, "Error al configurar periodo de activación de ULP (%s)", esp_err_to_name(set_wakeup_period_status));
        return;
    } 
}

static void init_ulp_rtc_gpio(void)
{
#if CONFIG_IDF_TARGET_ESP32
    // Aislar GPIO12 y GPIO15 de sus circuitos externos. Esto es necesario para módulos
    // que contienen un resisor pull-up externo en GPIO12 (como el ESP32-WROVER)
    // para minimizar el consumo de corriente.
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
#endif

    // Configurar perifericos usados por el ULP, incluyendo I2C.
    rtc_gpio_init((gpio_num_t) CONFIG_I2C_SCL_IO);
    rtc_gpio_init((gpio_num_t) CONFIG_I2C_SDA_IO);

    rtc_gpio_set_direction((gpio_num_t) CONFIG_I2C_SCL_IO, RTC_GPIO_MODE_INPUT_OUTPUT);
    rtc_gpio_set_direction((gpio_num_t) CONFIG_I2C_SDA_IO, RTC_GPIO_MODE_INPUT_OUTPUT);

    // Mantener activos los perifericos RTC durante sueño profundo.
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
}

static void deinit_ulp_rtc_gpio(void)
{
    rtc_gpio_deinit((gpio_num_t) CONFIG_I2C_SCL_IO);
    rtc_gpio_deinit((gpio_num_t) CONFIG_I2C_SDA_IO);
}

static void start_ulp_program(void)
{
    init_ulp_rtc_gpio();

    //TODO: inicializar variables usadas por ULP.

    esp_err_t run_status = ulp_run(&ulp_entry - RTC_SLOW_MEM);

    if (ESP_OK != run_status)
    {
        ESP_LOGW(TAG, "Error iniciar ejecución del programa del ULP (%s)", esp_err_to_name(run_status));
        return;
    } 
}
