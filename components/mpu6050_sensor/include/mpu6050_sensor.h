#ifndef _MPU6050_SENSOR_H_
#define _MPU6050_SENSOR_H_

#include <stdbool.h>
#include <esp_err.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/i2c.h>

#include <mpu6050.h>

#define MPU_INT_NONE ((uint8_t) 0)
#define MPU_INT_DATA_RDY_BIT ((uint8_t) 1 << 0)
#define MPU_INT_I2C_MAST_BIT ((uint8_t) 1 << 3)
#define MPU_INT_FIFO_OFLOW_BIT ((uint8_t) 1 << 4)
#define MPU_INT_MOTION_BIT ((uint8_t) 1 << 6)

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

typedef struct {
    i2c_port_t i2c_port_num;
    uint16_t address;
    uint8_t enabled_interrupts;
    gpio_num_t mpu_int;
    gpio_isr_t isr_handler;
    uint32_t min_read_interval_us;
} mpu6050_config_t;

typedef mpu6050_handle_t mpu_handle_t;

/**
 * @brief Registra un nuevo MPU6050 y lo configura con senstividad por default. Finalmente, despierta
 * al MPU6050.
 */
esp_err_t mpu6050_init(const mpu6050_config_t* const sensor_config);

esp_err_t mpu6050_get_i2c_id(const mpu_handle_t sensor, uint8_t* const out_mpu6050_id);

/**
 * @brief Configura la sensitividad del acelerómetro y del giroscopio para el MPU6050 que tiene el
 * ID I2C especificado.
 */
esp_err_t mpu6050_set_sensitivity(const mpu_handle_t sensor, const mpu6050_acce_fs_t acce_range, const mpu6050_gyro_fs_t gyro_range);

/**
 * @brief Activa el modo de sueño de un sensor MPU6050.
 */
esp_err_t mpu6050_power_down(const mpu_handle_t sensor);

/**
 * @brief Libera la memoria dedicada al sensor MPU6050.
 */
esp_err_t mpu6050_free_resources(const mpu_handle_t sensor);

/**
 * @brief Lee las mediciones del acelerómetro, giroscopio y termómetro del MPU6050.
 */
esp_err_t mpu6050_get_measurements(const mpu_handle_t sensor, mpu6050_measures_t* const out_measurements);

esp_err_t mpu6050_enable_interrupts(const mpu_handle_t sensor, const uint8_t interrupt_bits);

esp_err_t mpu6050_disable_interrupts(const mpu_handle_t sensor, const uint8_t interrupt_bits);

esp_err_t mpu6050_read_interrupt_status(const mpu_handle_t sensor, uint8_t* const out_interrupt_status);

#endif /* _MPU6050_SENSOR_H_ */
