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
static volatile bool has_configured_timer_wakeup = false;

static int32_t latest_sleep_duration_ms = 0;

static EventGroupHandle_t xReadyForDeepSleepEvents = NULL;

static const char* modules_requiring_shutdown[MAX_MODULES_REQUIRING_SHUTDOWN] = {};
static size_t num_of_modules_requiring_shutdown = 0;

static int32_t deep_sleep_enter_retry_count = 0;

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
static int32_t index_of_module_with_shutdown(const char* const module_tag);

esp_err_t after_wakeup(BaseType_t* const outIsWakeupExternal)
{
    calculate_sleep_duration();

    esp_err_t wakeup_status = ESP_OK;

    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();

    if (NULL != outIsWakeupExternal)
    {
        *outIsWakeupExternal = wakeup_cause == ESP_SLEEP_WAKEUP_EXT1; 
    }

    const char* cause_name = wakeup_cause_to_name(wakeup_cause);

    if (ESP_SLEEP_WAKEUP_UNDEFINED != wakeup_cause)
    {
        ESP_LOGI(TAG, "Causa de wakeup: %s, tiempo en deep sleep: %dms", cause_name, latest_sleep_duration_ms);
    }

    switch (wakeup_cause)
    {
        // Fall-through intencional, solo confirma que wakeup sea por una causa esperada.
    case ESP_SLEEP_WAKEUP_ULP:
    case ESP_SLEEP_WAKEUP_EXT0:
    case ESP_SLEEP_WAKEUP_EXT1:
    case ESP_SLEEP_WAKEUP_TIMER:
        wakeup_status = ESP_OK;
        break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        // El reset no fue causado por deep sleep. Iniciar el programa del ULP.
        if (CONFIG_WAKEUP_ON_WEIGHT || CONFIG_WAKEUP_ON_MOTION)
        {
            init_ulp_program();
        }
        break;
    default:
        ESP_LOGI(TAG, "Sistema despertado de deep sleep, causa inesperada: %s", cause_name);
        break;
    }

    deinit_ulp_rtc_gpio();
    
    xReadyForDeepSleepEvents = xEventGroupCreate();
    
    if (NULL == xReadyForDeepSleepEvents)
    {
        ESP_LOGW(TAG, "The event group for shutdown notification could not be created");
        wakeup_status = ESP_ERR_NO_MEM;
    }

    return wakeup_status;
}

esp_err_t is_ready_for_deep_sleep(BaseType_t* const out_is_ready)
{
    if (NULL == out_is_ready)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (NULL == xReadyForDeepSleepEvents)
    {
        return ESP_ERR_NO_MEM;
    }

    EventBits_t tasks_to_await_before_sleep = 0x00;

    for (size_t i = 0; i < num_of_modules_requiring_shutdown; ++i)
    {
        tasks_to_await_before_sleep |= (1 << i);
    }

    EventBits_t tasks_ready_for_sleep = xEventGroupGetBits(xReadyForDeepSleepEvents);

    *out_is_ready = (BaseType_t) tasks_to_await_before_sleep == tasks_ready_for_sleep;

    return ESP_OK;
}

void enter_deep_sleep()
{
    if (!(has_configured_wakeup_sources || has_configured_timer_wakeup)) 
    {
        ESP_LOGW(TAG, "Se intento comenzar deep sleep sin antes haber configurado wakeup sources");
        return;
    }

    ESP_LOGI(TAG, "Comenzando deep sleep");

    if (CONFIG_WAKEUP_ON_WEIGHT)
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
            .max_freq_mhz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ,
            .min_freq_mhz = CONFIG_MIN_CPU_FREQ_MHZ,
#ifdef CONFIG_FREERTOS_USE_TICKLESS_IDLE
#ifdef CONFIG_LIGHT_SLEEP_ENABLED
            // .light_sleep_enable = enable_auto_light_sleep
            .light_sleep_enable = false
#endif
#endif
        };

        light_sleep_setup_status = esp_pm_configure(&pm_config);
    }

    return light_sleep_setup_status;
}

esp_err_t enter_light_sleep(uint64_t duration_us)
{
    esp_err_t status = ESP_OK;

    if (ESP_OK == status)
    {
        status = esp_sleep_enable_timer_wakeup(duration_us);
    }

    if (ESP_OK == status)
    {
        ESP_LOGI(TAG, "Iniciando light sleep, duracion = %lldus", duration_us);
        status = esp_light_sleep_start();
    }

    return status;
}

esp_err_t setup_wakeup_sources(const gpio_num_t motionIntPin)
{
    // IMPORTANTE: en revisiones 0 y 1 de ESP32, EXT0 no es compatible
    // con fuentes de wakeup de ULP y touch pad. Por esto se usa EXT1 en 
    // este caso.
    esp_err_t setup_status = ESP_OK;

    if (ESP_OK == setup_status)
    {
        // Activar wakeup desde EXT1.
        uint64_t motionWakeupPinMask = 0ULL;
#ifndef CONFIG_MPU_INT_ACTIVE_LOW
        if (CONFIG_WAKEUP_ON_MOTION)
        {
            if (esp_sleep_is_valid_wakeup_gpio(motionIntPin))
            {
                motionWakeupPinMask = (1ULL << motionIntPin);
            }
        }
#endif

        const uint64_t buttonWakeupPinMask = (1ULL << CONFIG_POWER_ON_GPIO_NUM);
        const uint64_t ext1HighMask = (buttonWakeupPinMask | motionWakeupPinMask);

        ESP_LOGI(TAG, "Activando wakeup desde EXT1 con mask de GPIOs %llu", ext1HighMask);

        rtc_gpio_pullup_dis(CONFIG_POWER_ON_GPIO_NUM);
        rtc_gpio_pulldown_en(CONFIG_POWER_ON_GPIO_NUM);

        setup_status = esp_sleep_enable_ext1_wakeup(ext1HighMask, ESP_EXT1_WAKEUP_ANY_HIGH);
    }

    if (ESP_OK == setup_status && CONFIG_WAKEUP_ON_WEIGHT)
    {
        // Activar wakeup por programa de ULP.
        ESP_LOGI(TAG, "Activando wakeup por ULP");
        setup_status = esp_sleep_enable_ulp_wakeup();
    }

    has_configured_wakeup_sources = (ESP_OK == setup_status);

    return setup_status;
}

esp_err_t set_max_deep_sleep_duration(const int64_t sleep_duration_us)
{
    esp_err_t status = ESP_OK;

    if (ESP_OK == status) 
    {
        ESP_LOGI(TAG, "Activando wakeup por timer, periodo = %llius", sleep_duration_us);
        status = esp_sleep_enable_timer_wakeup(sleep_duration_us);
    }
    
    has_configured_timer_wakeup = (ESP_OK == status);

    return status;
}

esp_err_t add_module_to_notify_before_deep_sleep(const char* const module_tag)
{
    esp_err_t status = ESP_OK;

    int32_t existing_module_idx = index_of_module_with_shutdown(module_tag);

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

    int32_t existing_module_idx = index_of_module_with_shutdown(module_tag);
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

int32_t increment_sleep_attempt_count()
{
    return ++deep_sleep_enter_retry_count;
}

void reset_sleep_attempt_count()
{
    deep_sleep_enter_retry_count = 0;
}

static int32_t index_of_module_with_shutdown(const char* const module_tag)
{
    int32_t index_of_module = -1;

    for (int32_t i = 0; i < num_of_modules_requiring_shutdown; ++i) 
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
