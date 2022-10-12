#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_

#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <soc/rtc.h>

#if CONFIG_IDF_TARGET_ESP32
#include <esp32/ulp.h>
#elif CONFIG_IDF_TARGET_ESP32S2
#include <esp32s2/ulp.h>
#elif CONFIG_IDF_TARGET_ESP32S3
#include <esp32s3/ulp.h>
#endif

#include <esp_err.h>
#include <esp_log.h>

#ifdef CONFIG_ULP_SENSOR_WAKEUP
#if CONFIG_IDF_TARGET_ESP32
/*
 * 
 */
#define ULP_DATA_OFFSET 36
#endif

void enter_deep_sleep(void);

esp_err_t setup_sleep_wakeup_sources(void);

esp_err_t config_auto_light_sleep(bool enable_auto_light_sleep);

void calculate_sleep_duration(void); 

void log_sleep_wakeup_cause(void);

#endif /* _POWER_MANAGER_H_ */
