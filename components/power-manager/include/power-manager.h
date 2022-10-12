#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_

#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <soc/rtc.h>
#include <esp_pm.h>

#if CONFIG_IDF_TARGET_ESP32
#include <esp32/ulp.h>
#elif CONFIG_IDF_TARGET_ESP32S2
#include <esp32s2/ulp.h>
#elif CONFIG_IDF_TARGET_ESP32S3
#include <esp32s3/ulp.h>
#endif

#include <esp_err.h>
#include <esp_log.h>

/**
 * @brief Permite activar o desactivar el sueño ligero automático.
 */
esp_err_t setup_light_sleep(bool enable_auto_light_sleep);

/**
 * @brief Configura las fuentes que pueden despertar al sistema cuando está en sueño profundo.
 */
esp_err_t setup_wakeup_sources(void);

/**
 * @brief Debería ser invocada por el programa principal después de despertar de 
 * sueño profundo, para configurar el manejo de poder y manejar la causa de wakeup. 
 */
esp_err_t after_wakeup(void);

/**
 * @brief Registra un módulo que necesita teardown antes de que el 
 * administrador de poder comience un sueño profundo.
 */ 
esp_err_t add_module_for_deep_sleep_confirmation(const char* const module_tag);

/**
 * @brief Le indica al administrador de poder si el módulo identificado con
 * module_tag está listo o no para que el sistema entre en sueño proufndo. 
 */ 
esp_err_t set_module_ready_for_deep_sleep(const char* const module_tag, bool is_ready);

/**
 * @brief Prepara al sistema para entrar en sueño profundo. Espera a que 
 * todos los módulos señalen que están listos y finalmente comienza el
 * sueño profundo. 
 */
void begin_deep_sleep_when_ready();

#endif /* _POWER_MANAGER_H_ */
