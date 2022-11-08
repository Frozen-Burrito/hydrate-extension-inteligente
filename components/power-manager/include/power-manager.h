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
esp_err_t add_module_to_notify_before_deep_sleep(const char* const module_tag);

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
void enter_deep_sleep();

/**
 * @brief Revisa si el dispositivo esta en un estado consistente para iniciar el 
 * sueño profundo. 
 * 
 * Esta funcion nunca bloquea la task que la invoca.
 */
esp_err_t is_ready_for_deep_sleep(bool* const out_is_ready);

/**
 * @brief Registra un nuevo intento de entrar en modo de sueño profundo. Retorna
 * el numero actualizado de intentos.
 */
int32_t increment_sleep_attempt_count();

/**
 * @brief Reinicia el número de intentos de entrar en modo de sueño profundo.
 */
void reset_sleep_attempt_count();

/**
 * @brief Activa el wakeup por timer, haciendo que el sueño profundo dure sleep_duration_us, 
 * a menos que el chip sea despertado antes por alguna otra fuente. 
 */
esp_err_t set_max_deep_sleep_duration(const int64_t sleep_duration_us);

#endif /* _POWER_MANAGER_H_ */
