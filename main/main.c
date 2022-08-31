#include <stdio.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <freertos/event_groups.h>
#include <nvs_flash.h>
#include <esp_log.h>

#include "driver_ble.h"

static const char* TAG = "MAIN";

/**
 * @brief Distintos modos de funcionamiento del programa, para pruebas.
 */ 
typedef enum modos_func {
    MODO_COMPLETO, /** Funcionalidad completa */
    MODO_SOLO_BLE, /** Sólo BLE */
    MODO_SOLO_SENSORES, /** Sólo sensores */
    MODO_SIN_SLEEP /** Todas las funciones, sin ahorro de energía (no se desactiva la ESP32) */
} modo_func_t;

/**
 * Determina el modo de funcionamiento del dispositivo. Cad modo de 
 * funcionamiento incluye distintas características.
 */
#define MODO_FUNCIONAMIENTO MODO_COMPLETO

#define ID_APP_BLE 0x55

// Bits para eventos de estado del driver BLE.
#define EVT_BLE_INACTIVO (1 << 0)
#define EVT_BLE_INICIALIZADO (1 << 1)
#define EVT_BLE_ANUNCIANDO (1 << 2)
#define EVT_BLE_EMPAREJANDO (1 << 3)
#define EVT_BLE_CONECTADO (1 << 4)
#define EVT_BLE_DESCONECTADO (1 << 5)

#define EVT_SENSOR_GIRO_LEVANTADO (1 << 0)
#define EVT_SENSOR_GIRO_INCLINADO (1 << 1)
#define EVT_SENSOR_CAMBIO_PESO (1 << 2)
#define EVT_SENSOR_PESO_CALIBRADO (1 << 3)

#define EVT_SLEEP_CONEXION_INACTIVA (1 << 0)
#define EVT_SLEEP_NVS_LIBRE (1 << 1)
#define EVT_SLEEP_SIN_REGISTROS_RECIENTES (1 << 2)

EventGroupHandle_t xEventosBle;
EventGroupHandle_t xEventosSensores;
EventGroupHandle_t xEventosSleep;

/** @brief Activa el sueño del chip, cuando el timer se completa. 
 * TODO: Esta es una función temporal de prueba.
*/
static void callback_timer_sleep(TimerHandle_t timer) 
{
    uint32_t tiempo_actual = xTaskGetTickCount();

    ESP_LOGI(TAG, "Timer de activación de sueño profundo completado: %d", pdTICKS_TO_MS(tiempo_actual));
}

/**
 * TODO: Esta es una función temporal de prueba. 
 */ 
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

/** 
 * @brief Inicia el driver BLE, maneja el estado de conexión con el 
 * dispositivo central y actualiza los datos en el perfil GATT.
 */
void task_conexion_ble(void* pvParameters) 
{
    xEventosBle = xEventGroupCreate();

    if (xEventosBle == NULL) 
    {
        // El grupo de eventos no fue creado, porque no hay suficiente 
        // memoria heap disponible. 
        //TODO: Manejar memoria no disponible para crear xEventosBle.
    }

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
        // if (EMPAREJADO == estado_dispositivo) {
        //     ESP_LOGI(TAG, "Dispositivo BLE emparejado, perfil es accesible.");
        // } else {
        //     ESP_LOGI(TAG, "Dispositivo BLE no está emparejado: %d", estado_dispositivo);
        // }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelete(NULL);
}

/**
 * @brief Obtiene mediciones del giroscopio, acelerómetro y termómetro.
 * 
 * Calibra el sensor, si es necesario.
 * 
 * @param pvParameters Parámeteros opcionales para la tarea. No son 
 * usados por el momento.
 */ 
void task_medidas_giroscopio(void* pvParameters)
{

}

/**
 * @brief Obtiene mediciones del sensor de peso, después de calibrarlo.
 */ 
void task_medidas_peso(void* pvParameters)
{

}

/**
 * @brief Identifica si el usuario consume agua, si es así, crea un 
 * registro de hidratación.
 */ 
void task_identificacion_hidratacion(void* pvParameters)
{

}

/**
 * @brief Almacena y accesa registros de hidratación encontrados en NVS.
 * 
 * Prioridad: 
 * 
 */
void task_almacenamiento_registros(void* pvParameters) 
{

}

/**
 * @brief Activa los modos de sueño ligero y profundo cuando se cumplen sus condiciones.
 * 
 * Las condiciones para activar sueño ligero son:
 *  - 
 * 
 * Las condiciones para el sueño profundo son:
 *  -  
 */
void task_ahorro_energia(void* pvParameters) 
{

}

/**
 * @brief El punto de entrada de la aplicación.
 */ 
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

    bool conSleep = MODO_FUNCIONAMIENTO == MODO_COMPLETO
        || MODO_FUNCIONAMIENTO != MODO_SIN_SLEEP;

    bool incluirBle = MODO_FUNCIONAMIENTO == MODO_COMPLETO 
        || MODO_FUNCIONAMIENTO == MODO_SOLO_BLE
        || MODO_FUNCIONAMIENTO == MODO_SIN_SLEEP;

    bool incluirSensores = MODO_FUNCIONAMIENTO == MODO_COMPLETO 
        || MODO_FUNCIONAMIENTO == MODO_SOLO_SENSORES
        || MODO_FUNCIONAMIENTO == MODO_SIN_SLEEP;

    xTaskCreatePinnedToCore(task_almacenamiento_registros, "almacenamiento", 2048, NULL, 5 NULL, 1);
    xTaskCreatePinnedToCore(task_identificacion_hidratacion, "identificacion", 2048, NULL, 5 NULL, 1);

    // Crear tasks, según el modo de operación.
    if (incluirBle)
    {
        xTaskCreatePinnedToCore(task_conexion_ble, "Task Driver BLE", 4096, NULL, 5, NULL, 1);
    }

    if (incluirSensores) 
    {
        xTaskCreatePinnedToCore(task_medidas_giroscopio, "medidas_gyro", 2048, NULL, 5 NULL, 1);
        xTaskCreatePinnedToCore(task_medidas_peso, "medidas_peso", 2048, NULL, 5 NULL, 1);
    }

    if (conSleep)
    {
        xTaskCreatePinnedToCore(task_ahorro_energia, "ahorro_energia", 2048, NULL, 3 NULL, 1);
    }
    // xTaskCreatePinnedToCore(task_time, "task_tiempo", 2048, NULL, 3, NULL, 1);
}
