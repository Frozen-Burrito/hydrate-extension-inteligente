#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include "mpu6050_sensor.h"

static const char* TAG = "MPU6050";

#define ESP_INTR_FLAG_DEFAULT 0

#define MPU6050_ACCEL_CFG 0x1C
#define MPU6050_MOT_THRESH 0x1F
#define MPU6050_MOT_DURATION 0x20
#define MPU6050_MOT_DET_CTRL 0x69
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_STATUS 0x3A

#define MPU_DHPF_CTRL_5KHZ (1 << 0)
#define MPU_INT_ACTIVE_LOW_BIT (1 << 7)
#define MPU_INT_OPEN_DRAIN_BIT (1 << 6)
#define MPU_INT_LATCH_LVL_BIT  (1 << 5)
#define MPU_INT_READ_CLEAR_BIT (1 << 4)

static esp_err_t mpu6050_write_byte(mpu_handle_t sensor, const uint8_t reg_addr, const uint8_t data);
static esp_err_t mpu6050_read(mpu_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const out_data_buf, const size_t data_len);

static esp_err_t mpu6050_configure_isr(const mpu_handle_t sensor, const mpu6050_config_t* const sensor_config);
static esp_err_t mpu6050_configure_int_pin(const mpu_handle_t sensor, const mpu6050_config_t* const sensor_config);

esp_err_t mpu6050_init(const mpu6050_config_t* const sensor_config, mpu_handle_t* out_sensor)
{
    esp_err_t status = ESP_OK;
    mpu_handle_t sensor = NULL;

    if (ESP_OK == status) 
    {
        sensor = (mpu_handle_t) mpu6050_create(sensor_config->i2c_port_num, sensor_config->address);

        if (NULL != sensor)
        {
            *out_sensor = sensor;
        } else 
        {
            status = ESP_ERR_NO_MEM;
        }
    }

    if (ESP_OK == status && NULL != sensor) 
    {
        status = mpu6050_wake_up(sensor);
    }    

    if (ESP_OK == status) 
    {
        status = mpu6050_set_sensitivity(sensor, ACCE_FS_4G, GYRO_FS_500DPS);
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

esp_err_t mpu6050_enable_motion_detect(const mpu_handle_t sensor, mpu6050_config_t* const sensor_config)
{
    esp_err_t status = ESP_OK;
    // Actualizar la configuración del sensor para reflejar
    // la nueva fuente de interrupts.
    sensor_config->enabled_interrupts |= MPU_INT_MOTION_BIT;

    if (ESP_OK == status)
    {
        status = mpu6050_configure_int_pin(sensor, sensor_config);
    }

    if (ESP_OK == status)
    {
        status = mpu6050_configure_isr(sensor, sensor_config);
    }

    if (ESP_OK == status)
    {
        uint8_t accel_cfg = 0x00;

        status = mpu6050_read(sensor, MPU6050_ACCEL_CFG, &accel_cfg, sizeof(accel_cfg));
        ESP_LOGI(TAG, "MPU ACCEL_CFG register (%0X)", accel_cfg);

        if (ESP_OK == status)
        {
            accel_cfg |= MPU_DHPF_CTRL_5KHZ;
            status = mpu6050_write_byte(sensor, MPU6050_ACCEL_CFG, accel_cfg);
            ESP_LOGI(TAG, "MPU accelerometer configuration set to = %0X", accel_cfg);
        }
    }

    if (ESP_OK == status)
    {
        uint8_t motionThreshold = (uint8_t) CONFIG_MPU_MOT_DETECT_THRESHOLD;
        status = mpu6050_write_byte(sensor, MPU6050_MOT_THRESH, motionThreshold);
        ESP_LOGI(TAG, "MPU motion threshold (%0X)", motionThreshold);
    }

    if (ESP_OK == status)
    {
        uint8_t motionDuration = (uint8_t) CONFIG_MPU_MOT_DETECT_DURATION_MS;
        status = mpu6050_write_byte(sensor, MPU6050_MOT_DURATION, motionDuration);
        ESP_LOGI(TAG, "MPU motion duration (%0X)", motionDuration);
    }

    if (ESP_OK == status)
    {
        status = mpu6050_write_byte(sensor, MPU6050_MOT_DET_CTRL, 0x15);
        ESP_LOGI(TAG, "Set motion detect ctrl (%s)", esp_err_to_name(status));
    }


    if (ESP_OK == status)
    {
        uint8_t enabled_interrupts = 0x00;
        status = mpu6050_read(sensor, MPU6050_INT_ENABLE, &enabled_interrupts, sizeof(enabled_interrupts));
        ESP_LOGI(TAG, "MPU INT register (%0X)", enabled_interrupts);

        BaseType_t motIntIsNotEnabled = (enabled_interrupts & MPU_INT_MOTION_BIT) != MPU_INT_MOTION_BIT;

        if (ESP_OK == status && motIntIsNotEnabled)
        {
            enabled_interrupts |= MPU_INT_MOTION_BIT;

            status = mpu6050_write_byte(sensor, MPU6050_INT_ENABLE, enabled_interrupts);
            ESP_LOGI(TAG, "MPU INT register (%0X)", enabled_interrupts);
        }
    }

    return status;
}

esp_err_t mpu6050_enable_interrupts(const mpu_handle_t sensor, const mpu6050_config_t* const sensor_config)
{
    esp_err_t status = ESP_OK;
    
    if (NULL == sensor) 
    {
        status = ESP_ERR_INVALID_ARG;
    }

    if (ESP_OK == status)
    {
        status = mpu6050_configure_int_pin(sensor, sensor_config);
    }

    if (ESP_OK == status)
    {
        uint8_t enabled_interrupts = 0x0 | sensor_config->enabled_interrupts;

        status = mpu6050_write_byte(sensor, MPU6050_INT_ENABLE, enabled_interrupts);
        ESP_LOGI(TAG, "MPU enabled interrupts (%0X)", enabled_interrupts);
    }

    if (ESP_OK == status)
    {
        status = mpu6050_configure_isr(sensor, sensor_config);
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

    if (ESP_OK == status)
    {
        uint8_t interrupt_status;
        status = mpu6050_read(sensor, MPU6050_INT_STATUS, &interrupt_status, 1);

        if (ESP_OK == status) 
        {
            // Solo asignar valor a out_interrupt_status cuando el read fue ESP_OK.
            *out_interrupt_status = interrupt_status;
        }
    }

    return status;
}

bool mpu6050_is_intr_for_data(uint8_t intStatus)
{
    return ((intStatus & MPU_INT_DATA_RDY_BIT) == MPU_INT_DATA_RDY_BIT);
}

bool mpu6050_is_intr_for_motion(uint8_t intStatus)
{
    return ((intStatus & MPU_INT_MOTION_BIT) == MPU_INT_MOTION_BIT);
}

static esp_err_t mpu6050_configure_int_pin(const mpu_handle_t sensor, const mpu6050_config_t* const sensor_config)
{
    uint8_t int_pin_cfg = 0x00;

#ifdef CONFIG_MPU_INT_ACTIVE_LOW
    int_pin_cfg |= MPU_INT_ACTIVE_LOW_BIT;
#endif

#ifdef CONFIG_MPU_INT_OPEN_DRAIN
    int_pin_cfg |= MPU_INT_OPEN_DRAIN_BIT;
#endif

#ifdef CONFIG_MPU_INT_LATCH_EN
    int_pin_cfg |= MPU_INT_LATCH_LVL_BIT;
#endif

#ifdef CONFIG_MPU_INT_CLEAR_ON_ANY_READ
    int_pin_cfg |= MPU_INT_READ_CLEAR_BIT;
#endif

    esp_err_t status = mpu6050_write_byte(sensor, MPU6050_INT_PIN_CFG, int_pin_cfg);

    if (ESP_OK != status) 
    {
        ESP_LOGW(
            TAG, 
            "Error writing to INT_PIN_CFG reg, value = %0X (%s)",
            int_pin_cfg, 
            esp_err_to_name(status)
        );
    } else {
        ESP_LOGI(TAG, "Wrote %0X to INT_PIN_CFG", int_pin_cfg);
    }

    return status;
}

static esp_err_t mpu6050_configure_isr(const mpu_handle_t sensor, const mpu6050_config_t* const sensor_config)
{
    esp_err_t status = ESP_OK;

    // Si no hay fuentes de interrupts activadas, reiniciar 
    // la configuración del pin para evitar instalar el ISR
    // más de una vez.
    if (sensor_config->enabled_interrupts == MPU_INT_NONE)
    {
        status = gpio_reset_pin(sensor_config->mpu_int);
        return status;
    }

    if (ESP_OK == status)
    {
#ifdef CONFIG_MPU_INT_ACTIVE_LOW
        bool intActiveLow = true;
#else
        bool intActiveLow = false;
#endif
        gpio_config_t int_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = intActiveLow ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE,
            .pin_bit_mask = (1ULL << sensor_config->mpu_int),
            // .pull_up_en = intActiveLow ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            // .pull_down_en = intActiveLow ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE
        };
        
        status = gpio_config(&int_config);
    }

    if (ESP_OK == status)
    {
        status = gpio_isr_handler_add(
            sensor_config->mpu_int,
            *(sensor_config->isr_handler),
            NULL
        );
    }

    if (ESP_OK == status)
    {
        status = gpio_intr_enable(sensor_config->mpu_int);
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
