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

typedef struct {
    int32_t raw_weight_data;
    mpu6050_acce_value_t acceleration;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temperature;
} sensor_measures_t;

hydration_record_t create_random_record(void);

#endif // _HYDRATE_COMMON_H_