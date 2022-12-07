#include <stdio.h>
#include "mpu6050_sensor.h"

static const char* TAG = "MPU6050";

#define ESP_INTR_FLAG_DEFAULT 0

#define MPU6050_INT_ENABLE 0x38u
#define MPU6050_INT_STATUS 0x3Au

static esp_err_t mpu6050_write_byte(mpu_handle_t sensor, const uint8_t reg_addr, const uint8_t data);
static esp_err_t mpu6050_read(mpu_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const out_data_buf, const size_t data_len);

esp_err_t mpu6050_init(const mpu6050_config_t* const sensor_config)
{
    esp_err_t status = ESP_OK;
    mpu_handle_t sensor = NULL;

    if (ESP_OK == status) 
    {
        sensor = (mpu_handle_t) mpu6050_create(sensor_config->i2c_port_num, sensor_config->address);

        if (NULL == sensor)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == status) 
    {
        status = mpu6050_set_sensitivity(sensor, ACCE_FS_4G, GYRO_FS_500DPS);
    }

    if (ESP_OK == status && sensor_config->enabled_interrupts != MPU_INT_NONE)
    {
        gpio_config_t int_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_NEGEDGE,
            .pin_bit_mask = (1ULL << sensor_config->mpu_int),
            .pull_up_en = 1,
        };
        
        status = gpio_config(&int_config);
    }

    if (ESP_OK == status && sensor_config->enabled_interrupts != MPU_INT_NONE)
    {
        status = gpio_isr_handler_add(
            sensor_config->mpu_int,
            *(sensor_config->isr_handler),
            (void*) sensor
        );
    }

    if (ESP_OK == status && sensor_config->enabled_interrupts != MPU_INT_NONE)
    {
        status = gpio_intr_enable(sensor_config->mpu_int);
    }

    if (ESP_OK == status && sensor_config->enabled_interrupts != MPU_INT_NONE)
    {
        status = mpu6050_enable_interrupts(sensor, sensor_config->enabled_interrupts);
    }

    if (ESP_OK == status && NULL != sensor) 
    {
        status = mpu6050_wake_up(sensor);
    }    

    return status;
}

esp_err_t mpu6050_get_i2c_id(const mpu_handle_t sensor, uint8_t* const out_mpu6050_id)
{
    esp_err_t status = ESP_OK;

    if (NULL == sensor)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        status = mpu6050_get_deviceid(sensor, out_mpu6050_id);
    }

    return status;
}

esp_err_t mpu6050_set_sensitivity(const mpu_handle_t sensor, const mpu6050_acce_fs_t acce_range, const mpu6050_gyro_fs_t gyro_range)
{
    esp_err_t status = ESP_OK;

    if (NULL == sensor)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        status = mpu6050_config(sensor, acce_range, gyro_range);
    }

    return status;
}

esp_err_t mpu6050_power_down(const mpu_handle_t sensor)
{
    esp_err_t status = ESP_OK;

    if (NULL == sensor)
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        status = mpu6050_sleep(sensor);
    }

    return status;
}

esp_err_t mpu6050_free_resources(const mpu_handle_t sensor)
{
    esp_err_t status = ESP_OK;

    if (NULL == sensor)
    {
        status = ESP_FAIL;
    }

    if (ESP_OK == status)
    {
        mpu6050_delete(sensor);
    }

    return status;
}

esp_err_t mpu6050_get_measurements(const mpu_handle_t sensor, mpu6050_measures_t* const out_measurements)
{
    esp_err_t status = ESP_OK; 

    if (NULL == sensor)
    {
        status = ESP_FAIL;
    }

    if (ESP_OK == status) 
    {
        mpu6050_acce_value_t mpu6050_acceleration = {};
        status = mpu6050_get_acce(sensor, &mpu6050_acceleration);

        if (ESP_OK == status)
        {
            out_measurements->acceleration.x = mpu6050_acceleration.acce_x;
            out_measurements->acceleration.y = mpu6050_acceleration.acce_y; 
            out_measurements->acceleration.z = mpu6050_acceleration.acce_z; 
        }
    }

    if (ESP_OK == status) 
    {
        mpu6050_gyro_value_t mpu6050_gyroscope_val = {};
        status = mpu6050_get_gyro(sensor, &mpu6050_gyroscope_val);

        if (ESP_OK == status)
        {
            out_measurements->gyroscope.x = mpu6050_gyroscope_val.gyro_x;
            out_measurements->gyroscope.y = mpu6050_gyroscope_val.gyro_y; 
            out_measurements->gyroscope.z = mpu6050_gyroscope_val.gyro_z; 
        }
    }

    if (ESP_OK == status) 
    {
        mpu6050_temp_value_t mpu6050_temperature_val = {};
        status = mpu6050_get_temp(sensor, &mpu6050_temperature_val);

        if (ESP_OK == status)
        {
            out_measurements->temperature = mpu6050_temperature_val.temp;
        }
    }

    return status;
}

// enabled interrupts = 0x01000001
// interrupt_bits =     0x01000000
// ~interrupt_bits =    0x10111111
// result =             0x00000001

esp_err_t mpu6050_enable_interrupts(const mpu_handle_t sensor, const uint8_t interrupt_bits)
{
    esp_err_t status = ESP_OK;
    
    if (NULL == sensor) 
    {
        status = ESP_ERR_INVALID_ARG;
    }

    uint8_t enabled_interrupts = 0x00;

    if (ESP_OK == status)
    {
        status = mpu6050_read(sensor, MPU6050_INT_ENABLE, &enabled_interrupts, sizeof(enabled_interrupts));
        ESP_LOGI(TAG, "MPU INT register (%x)", enabled_interrupts);
    }

    if (ESP_OK == status)
    {
        enabled_interrupts |= interrupt_bits;

        status = mpu6050_write_byte(sensor, MPU6050_INT_ENABLE, enabled_interrupts);
        ESP_LOGI(TAG, "MPU INT register (%x)", enabled_interrupts);
    }

    if (ESP_OK == status)
    {
        status = mpu6050_read(sensor, MPU6050_INT_ENABLE, &enabled_interrupts, sizeof(enabled_interrupts));
        ESP_LOGI(TAG, "MPU INT register confirm (%x)", enabled_interrupts);
    }

    return status;
}

esp_err_t mpu6050_disable_interrupts(const mpu_handle_t sensor, const uint8_t interrupt_bits)
{
    esp_err_t status = ESP_OK;

    if (NULL == sensor) 
    {
        status = ESP_ERR_INVALID_ARG;
    }

    uint8_t enabled_interrupts;

    if (ESP_OK == status)
    {
        status = mpu6050_read(sensor, MPU6050_INT_ENABLE, &enabled_interrupts, 1);
    }

    if (ESP_OK == status)
    {
        enabled_interrupts &= (~interrupt_bits);
        status = mpu6050_write_byte(sensor, MPU6050_INT_ENABLE, enabled_interrupts);
    }

    return status;
}

esp_err_t mpu6050_read_interrupt_status(const mpu_handle_t sensor, uint8_t* const out_interrupt_status)
{
    esp_err_t status = ESP_OK;

    if (NULL == sensor) 
    {
        status = ESP_ERR_INVALID_ARG;
    }

    uint8_t interrupt_status;

    if (ESP_OK == status)
    {
        status = mpu6050_read(sensor, MPU6050_INT_STATUS, &interrupt_status, 1);
    }

    if (ESP_OK == status) 
    {
        // Solo asignar valor a out_interrupt_status cuando el read fue ESP_OK.
        *out_interrupt_status = interrupt_status;
    }

    return status;
}

static esp_err_t mpu6050_write_byte(mpu_handle_t sensor, const uint8_t reg_addr, const uint8_t data)
{
    //TODO: solucionar problema de obtener estos dos valores.
    i2c_port_t bus = I2C_NUM_0;
    uint16_t dev_addr = CONFIG_MPU_I2C_ADDRESS;
    esp_err_t status = ESP_OK;

    if (ESP_OK == status)
    {
        uint8_t write_buf[2] = { reg_addr, data };
        status = i2c_master_write_to_device(bus, dev_addr, write_buf, sizeof(write_buf), pdMS_TO_TICKS(5));
    }

    return status;
}

static esp_err_t mpu6050_read(mpu_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const out_data_buf, const size_t data_len)
{
    //TODO: solucionar problema de obtener estos dos valores.
    i2c_port_t bus = I2C_NUM_0;
    uint16_t dev_addr = CONFIG_MPU_I2C_ADDRESS;
    esp_err_t status = ESP_OK;

    status = i2c_master_write_read_device(bus, dev_addr, &reg_start_addr, 1, out_data_buf, data_len, pdMS_TO_TICKS(5));

    return status;
}
