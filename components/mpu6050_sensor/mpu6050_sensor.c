#include <stdio.h>
#include "mpu6050_sensor.h"

static const char* TAG = "MPU6050";

// Configruacion del MPU6050
#define MPU6050_I2C_PORT_NUM I2C_NUM_0
#define AVAILABLE_MPU6050_ADDRESS_COUNT 2
#define I2C_MASTER_FREQ_HZ 100000
static const gpio_num_t I2C_SCL_IO = (gpio_num_t) CONFIG_I2C_SCL_IO;
static const gpio_num_t I2C_SDA_IO = (gpio_num_t) CONFIG_I2C_SDA_IO;

static mpu6050_handle_t mpu6050 = NULL;

static esp_err_t i2c_bus_init(void);

esp_err_t mpu6050_init(bool init_i2c_bus)
{
    esp_err_t status = ESP_OK;

    if (init_i2c_bus && ESP_OK == status) 
    {
        status = i2c_bus_init();
    }

    ESP_LOGI(TAG, "I2C init status (%s)", esp_err_to_name(status));

    if (ESP_OK == status) 
    {
        mpu6050 = mpu6050_create(MPU6050_I2C_PORT_NUM, MPU6050_I2C_ADDRESS);
    }

    if (ESP_OK == status) 
    {
        status = mpu6050_set_sensitivity(ACCE_FS_4G, GYRO_FS_500DPS);
    }

    if (ESP_OK == status && NULL != mpu6050) 
    {
        status = mpu6050_wake_up(mpu6050);
    }    

    return status;
}

esp_err_t mpu6050_get_i2c_id(uint8_t* const out_mpu6050_id)
{
    esp_err_t status = ESP_OK;

    if (NULL == mpu6050)
    {
        status = ESP_FAIL;
    }

    if (ESP_OK == status)
    {
        status = mpu6050_get_deviceid(mpu6050, out_mpu6050_id);
    }

    return status;
}

esp_err_t mpu6050_set_sensitivity(const mpu6050_acce_fs_t acce_range, const mpu6050_gyro_fs_t gyro_range)
{
    esp_err_t status = ESP_OK;

    if (NULL == mpu6050)
    {
        status = ESP_FAIL;
    }

    if (ESP_OK == status)
    {
        status = mpu6050_config(mpu6050, acce_range, gyro_range);
    }

    return status;
}

esp_err_t mpu6050_power_down()
{
    esp_err_t status = ESP_OK;

    if (NULL == mpu6050)
    {
        status = ESP_FAIL;
    }

    if (ESP_OK == status && NULL != mpu6050)
    {
        status = mpu6050_sleep(mpu6050);
    }

    return status;
}

esp_err_t mpu6050_free_resources(bool also_delete_i2c_driver)
{
    esp_err_t status = ESP_OK;

    if (NULL == mpu6050)
    {
        status = ESP_FAIL;
    }

    if (ESP_OK == status)
    {
        mpu6050_delete(mpu6050);
    }

    if (also_delete_i2c_driver && ESP_OK == status)
    {
        status = i2c_driver_delete(MPU6050_I2C_PORT_NUM);
    }

    return status;
}

esp_err_t mpu6050_get_measurements(sensor_measures_t* const out_measurements)
{
    esp_err_t status = ESP_OK; 

    if (NULL == mpu6050)
    {
        status = ESP_FAIL;
    }

    if (ESP_OK == status) 
    {
        mpu6050_acce_value_t mpu6050_acceleration = {};
        status = mpu6050_get_acce(mpu6050, &mpu6050_acceleration);

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
        status = mpu6050_get_gyro(mpu6050, &mpu6050_gyroscope_val);

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
        status = mpu6050_get_temp(mpu6050, &mpu6050_temperature_val);

        if (ESP_OK == status)
        {
            out_measurements->temperature = mpu6050_temperature_val.temp;
        }
    }

    return status;
}

static esp_err_t i2c_bus_init(void) 
{
    i2c_config_t i2c_bus_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };

    esp_err_t i2c_init_status = ESP_OK; 

    if (ESP_OK == i2c_init_status) 
    {
        i2c_init_status = i2c_param_config(MPU6050_I2C_PORT_NUM, &i2c_bus_config);

        if (i2c_init_status != ESP_OK) 
        {
            ESP_LOGW(TAG, "I2C initialization error (%s)", esp_err_to_name(i2c_init_status));
        }
    }

    if (ESP_OK == i2c_init_status) 
    {
        i2c_init_status = i2c_driver_install(MPU6050_I2C_PORT_NUM, i2c_bus_config.mode, 0, 0, 0);

        if (i2c_init_status != ESP_OK) 
        {
            ESP_LOGW(TAG, "I2C initialization error (%s)", esp_err_to_name(i2c_init_status));
        }
    }

    return i2c_init_status;
}
