// system header
#include <stdint.h>
// esp-idf header
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
// external header
#include "tasks/bmi160_Task.h"
#include "bmi160.h"
#include "include/sdk_config.h"


/*
    local macro definitions
*/

/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_I2C  1

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_SPI  0

#if (!((BMI160_INTERFACE_I2C == 1) && (BMI160_INTERFACE_SPI == 0)) && \
    (!((BMI160_INTERFACE_I2C == 0) && (BMI160_INTERFACE_SPI == 1))))
#error "Invalid value given for the macros BMI160_INTERFACE_I2C / BMI160_INTERFACE_SPI"
#endif

/*! bmi160 shuttle id */
#define BMI160_SHUTTLE_ID     0x38

/*! bmi160 Device address */
#define BMI160_DEV_ADDR     CONFIG_I2C_ADDR


/*
    create structure
*/

// esp log tag
static const char *log_TAG = "IMU_BMI160";

/*! @brief This structure containing relevant bmi160 info */
struct bmi160_dev bmi160dev;

/*! @brief variable to hold the bmi160 accel data */
struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
struct bmi160_sensor_data bmi160_gyro;


/*
    static functions
*/

// delay ms "function"
static void task_delay_ms(uint32_t delay_time)
{
    // 延迟可能并不稳定
    // vTaskDelay(pdMS_TO_TICKS(delay_time));
    esp_rom_delay_us(delay_time*1000);
}

// i2c write function
static int8_t i2c_write_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    // TODO 现在并没有理会请求的地址
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_handle_1, read_data, len, -1));
    return 0;
}

int8_t i2c_read_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // TODO 原因与写入函数相同，现在并没有理会请求的地址
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_handle_1, data, len, -1));
    return 0;
}

// init driver config structure
static void init_bmi160_sensor_driver_structure()
{
#if BMI160_INTERFACE_I2C == 1

    /* I2C setup */

    /* link read/write/delay function of host system to appropriate
     * bmi160 function call prototypes */
    bmi160dev.write = i2c_write_for_bmi160Api;
    bmi160dev.read = i2c_read_for_bmi160Api;
    bmi160dev.delay_ms = task_delay_ms;

    /* set correct i2c address */
    bmi160dev.id = BMI160_DEV_ADDR;
    bmi160dev.intf = BMI160_I2C_INTF;
#endif
#if BMI160_INTERFACE_SPI == 1

    /* SPI setup */

    /* link read/write/delay function of host system to appropriate
     *  bmi160 function call prototypes */
    bmi160dev.write = coines_write_spi;
    bmi160dev.read = coines_read_spi;
    bmi160dev.delay_ms = coines_delay_msec;
    bmi160dev.id = COINES_SHUTTLE_PIN_7;
    bmi160dev.intf = BMI160_SPI_INTF;
#endif
}

// init sensor
static void init_bmi160()
{
    int8_t rslt;

    rslt = bmi160_init(&bmi160dev);

    if (rslt == BMI160_OK)
    {
        ESP_LOGI(log_TAG, "BMI160 initialization success !\n");
        ESP_LOGI(log_TAG, "Chip ID 0x%X\n", bmi160dev.chip_id);
    }
    else
    {
        ESP_LOGI(log_TAG, "BMI160 initialization failure !\n");
        vTaskDelete(NULL);
    }

    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi160dev);
}

// task function
void bmi160(void *pvParameters)
{
    ESP_LOGI("IMU_BMI160", "BMI160 task test");
    init_bmi160_sensor_driver_structure();
    /* After sensor init introduce 200 msec sleep */
    vTaskDelay(pdMS_TO_TICKS(200));
    init_bmi160();
    // TODO 读取传感器信息
    vTaskDelete(NULL);
}
