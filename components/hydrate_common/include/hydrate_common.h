#ifndef _HYDRATE_COMMON_H_
#define _HYDRATE_COMMON_H_

#include "hx711.h"
#include "mpu6050_sensor.h"

/**
 * @brief Contiene los datos básicos de un registro de 
 * hidratación.
 */
typedef struct {
    uint16_t water_amount;
    int16_t temperature;
    uint8_t battery_level;
    int64_t timestamp;
} hydration_record_t;

/**
 * @brief Contiene los datos obtenidos para un muestreo determinado de 
 * los sensores.
 */
typedef struct {
    mpu6050_measures_t accel_gyro_measurements;
    hx711_measures_t weight_measurements;
    int64_t timestamp_ms;
} sensor_measures_t;

hydration_record_t create_random_record(void);

esp_err_t add_sensor_measurements(const sensor_measures_t* const measurement);

esp_err_t hydration_record_to_string(char* out_buf, const hydration_record_t* hydration_record);

esp_err_t record_measurements_timestamp(sensor_measures_t* measurement);

/**
 * @brief Determina si un conjunto de [sensor_measures_t] está asociado
 * a un consumo de agua. 
 * 
 * @returns pdTRUE - Las medidas representan un consumo de agua, out_record fue actualizado con sus datos.
 * @returns pdFALSE - No hubo un consumo de agua, o out_record es NULL o measurement_count es 0.
 */
BaseType_t infer_hydration_from_sensors(hydration_record_t* const out_record);

#endif // _HYDRATE_COMMON_H_