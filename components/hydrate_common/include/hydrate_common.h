#ifndef _HYDRATE_COMMON_H_
#define _HYDRATE_COMMON_H_

#include <mpu6050.h>

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
    int32_t raw_weight_data;
    int32_t volume_ml;
    mpu6050_acce_value_t acceleration;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temperature;
    int64_t start_time_ms;
    int64_t end_time_ms;
} sensor_measures_t;

esp_err_t hydration_record_to_string(char* out_buf, const hydration_record_t* hydration_record);

esp_err_t start_measurement_period(sensor_measures_t* measurement);
esp_err_t end_measurement_period(sensor_measures_t* measurement);

/**
 * @brief Determina si un conjunto de [sensor_measures_t] está asociado
 * a un consumo de agua. 
 * 
 * @returns true - Las medidas representan un consumo de agua, out_record fue actualizado con sus datos.
 * @returns false - No hubo un consumo de agua, o out_record es NULL o measurement_count es 0.
 */
bool hydration_record_from_measures(
    const sensor_measures_t measurements[], 
    const size_t measurement_count,
    hydration_record_t* out_record
);

#endif // _HYDRATE_COMMON_H_