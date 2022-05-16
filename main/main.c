#include <stdio.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <nvs_flash.h>
#include <esp_log.h>

#include "driver_ble.h"

#define ID_APP_BLE 0x55

static const char* TAG = "MAIN";

/** @brief Activa el sueño del chip, cuando el timer se completa. */
static void callback_timer_sleep(TimerHandle_t timer) 
{
    uint32_t tiempo_actual = xTaskGetTickCount();

    ESP_LOGI(TAG, "Timer de activación de sueño profundo completado: %d", pdTICKS_TO_MS(tiempo_actual));
}

/** 
 * @brief Inicia el driver BLE, emparejamiento y conexión. Envía una 
 * notificación cuando hay nuevos registros.  
 */
void task_conexion_ble(void* pvParameters) 
{
    /* Inicializar el driver BLE. */
    esp_err_t status_ble = init_driver_ble("HYDRATE-0001", ID_APP_BLE);

    if (ESP_OK != status_ble) 
    {
        ESP_LOGE(TAG, "Error inicializando el driver BLE: %s", esp_err_to_name(status_ble));
        while(true) {}
    }

    /* Determinar el timer para la activación de sueño profundo, 1 minuto
       después de iniciar el escaneo BLE. */
    TimerHandle_t timer_emparejamiento_ble = xTimerCreate(
        "timer_emparejamiento_ble",
        pdMS_TO_TICKS(60 * 1000), /* 1 minuto = 60 * 1000ms */
        pdFALSE,
        0,
        callback_timer_sleep
    );

    xTimerStart(timer_emparejamiento_ble, 0);

    while (true) 
    {
        if (EMPAREJADO == estado_dispositivo) {
            ESP_LOGI(TAG, "Dispositivo BLE emparejado, perfil es accesible.");
        } else {
            ESP_LOGI(TAG, "Dispositivo BLE no está emparejado: %d", estado_dispositivo);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelete(NULL);
}

void task_time(void* pvParameters) 
{
    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;

    time(&now);

    setenv("TZ", "CST-8", 1);
    tzset();

    while (true) 
    {
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

        ESP_LOGI(TAG, "The current date/time in Shanghai is: %s", strftime_buf);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    //Inicializar NVS
    esp_err_t status_nvs = nvs_flash_init();

    if (status_nvs == ESP_ERR_NVS_NO_FREE_PAGES || status_nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {   
        // Borrar páginas de NVS si no hay espacio disponible.
        ESP_ERROR_CHECK(nvs_flash_erase());
        status_nvs = nvs_flash_init();
    }

    // Asegurar que NVS haya sido inicializado.
    ESP_ERROR_CHECK(status_nvs);

    xTaskCreatePinnedToCore(task_conexion_ble, "Task Driver BLE", 4096, NULL, 5, NULL, 1);
    // xTaskCreatePinnedToCore(task_time, "task_tiempo", 2048, NULL, 3, NULL, 1);
}
