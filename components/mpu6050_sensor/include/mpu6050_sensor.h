#ifndef _MPU6050_SENSOR_H_
#define _MPU6050_SENSOR_H_

#include <stdbool.h>
#include <esp_err.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/i2c.h>

#include <mpu6050.h>

typedef struct {
    float x;
    float y;
    float z;
} vector_3f_t;

typedef struct {
    vector_3f_t acceleration;
    vector_3f_t gyroscope;
    float temperature;
} mpu6050_measures_t;

/**
 * @brief Registra un nuevo MPU6050 y lo configura con senstividad por default. Finalmente, despierta
 * al MPU6050.
 */
esp_err_t mpu6050_init(bool init_i2c_bus);

esp_err_t mpu6050_get_i2c_id(uint8_t* const out_mpu6050_id);

/**
 * @brief Configura la sensitividad del acelerómetro y del giroscopio para el MPU6050 que tiene el
 * ID I2C especificado.
 */
esp_err_t mpu6050_set_sensitivity(const mpu6050_acce_fs_t acce_range, const mpu6050_gyro_fs_t gyro_range);

/**
 * @brief Activa el modo de sueño de un sensor MPU6050.
 */
esp_err_t mpu6050_power_down();

/**
 * @brief Libera la memoria dedicada al sensor MPU6050.
 */
esp_err_t mpu6050_free_resources(bool also_delete_i2c_driver);

/**
 * @brief Lee las mediciones del acelerómetro, giroscopio y termómetro del MPU6050.
 */
esp_err_t mpu6050_get_measurements(mpu6050_measures_t* const out_measurements);

#endif /* _MPU6050_SENSOR_H_ */
