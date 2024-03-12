#include "tasks/bmi160_Task.h"


/*
    create structure
*/

// esp log tag
static const char *logTAG = "IMU_BMI160";

// spi
// static spi_transaction_t spi_write_conf;
// static spi_transaction_t spi_read_conf;

/*! @brief This structure containing relevant bmi160 info */
static struct bmi160_dev bmi160dev;

/*! @brief variable to hold the bmi160 accel data */
static struct bmi160_sensor_data bmi160_accel;

/*! @brief variable to hold the bmi160 gyro data */
static struct bmi160_sensor_data bmi160_gyro;


/*
    static function prototypes
*/

// delay ms "function"
static void delay_ms_for_bmi160Api(uint32_t delay_time);

// i2c write function
static int8_t i2c_write_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);

// i2c read function
static int8_t i2c_read_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

// bmi160Api spi write
static int8_t spi_write_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

// bmi160Api spi read
static int8_t spi_read_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

// init driver config structure
static void init_bmi160_sensor_driver_structure(void);

// init sensor
static void init_bmi160(void);


/*
    functions
*/

// delay ms "function"
static void delay_ms_for_bmi160Api(uint32_t delay_time)
{
    // 延迟可能并不稳定
    // vTaskDelay(pdMS_TO_TICKS(delay_time));
    esp_rom_delay_us(delay_time*1000);
}

// bmi160Api i2c write
static int8_t i2c_write_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    ESP_LOGD(logTAG, "bmi160API write(I2C) dev_addr: %#x, reg_addr: %#x, read_data: %#x, len: %d", dev_addr, reg_addr, *read_data, len);
    /* TODO imu需要 start-地址-ack-传感器寄存器地址-ack-需要写入的数据-stop
    但现在只有 start-地址-ack-8bit数据-stop */
    // TODO 现在并没有理会请求的地址
    // ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_handle_1, buffer, len, -1));
    return 0;
}

// bmi160Api i2c read
static int8_t i2c_read_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    ESP_LOGD(logTAG, "bmi160API read(I2C) dev_addr: %#x, reg_addr: %#x, data: %#x, len: %d", dev_addr, reg_addr, *data, len);
    // TODO 原因与写入函数相同，现在并没有理会请求的地址
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_device_handle_1, &reg_addr, 1, data, len, -1));
    return 0;
}

// bmi160Api spi write
static int8_t spi_write_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len)
{
    ESP_LOGD(logTAG, "SPI2 write: dev_addr:%#x, reg_addr:%#x, read_data:%#x, len:%d", dev_addr, reg_addr, *read_data, len);

    // spi_write_conf.cmd = reg_addr;
    // spi_write_conf.length = len;
    // spi_write_conf.tx_buffer = read_data;
    spi_transaction_t spi_write_conf = {
        .cmd = reg_addr,
        .length = len,
        .rxlength = 0,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
        .tx_data[1] = *read_data,
        .flags = SPI_TRANS_USE_TXDATA,
    };
    // ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_1, &spi_write_conf));

    ESP_LOGD(logTAG, "SPI2 write: finished reg_addr:%#x, read_data:%#x", reg_addr, *read_data);

    return 0;
}

// bmi160Api spi read
static int8_t spi_read_for_bmi160Api(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    ESP_LOGD(logTAG, "reg_addr IS WHAT? 16:%#x 10:%d", reg_addr, reg_addr);
    ESP_LOGD(logTAG, "SPI2 read: dev_addr:%#x, reg_addr:%#x, data:%#x, len:%d", dev_addr, reg_addr, *data, len);

    // spi_read_conf.cmd = reg_addr;
    // spi_read_conf.length = len;
    // spi_read_conf.rx_buffer = data;
    spi_transaction_t spi_read_conf = {
        .cmd = reg_addr,
        .length = len,
        .rxlength = len,
        .tx_buffer = NULL,
        .rx_buffer = NULL,
        .rx_data[1] = *data,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    // ESP_ERROR_CHECK(spi_device_transmit(spi_device_handle_1, &spi_read_conf));

    ESP_LOGD(logTAG, "SPI2 read finished: reg_addr:%#x, data:%#x", reg_addr, *data);

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
    bmi160dev.delay_ms = delay_ms_for_bmi160Api;

    /* set correct i2c address */
    bmi160dev.id = BMI160_DEV_ADDR;
    bmi160dev.intf = BMI160_I2C_INTF;
#endif
#if BMI160_INTERFACE_SPI == 1

    /* SPI setup */

    /* link read/write/delay function of host system to appropriate
     *  bmi160 function call prototypes */
    bmi160dev.write = spi_write_for_bmi160Api;
    bmi160dev.read = spi_read_for_bmi160Api;
    bmi160dev.delay_ms = delay_ms_for_bmi160Api;
    bmi160dev.id = CONFIG_PIN_NUM_CS;
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
        ESP_LOGI(logTAG, "BMI160 initialization success !\n");
        ESP_LOGI(logTAG, "Chip ID 0x%X\n", bmi160dev.chip_id);
    }
    else
    {
        ESP_LOGI(logTAG, "BMI160 initialization failure !\n");
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
void BMI160_task(void *pvParameters)
{
    // TODO 读取传感器信息

    ESP_LOGI("IMU_BMI160", "BMI160 task start");
    // init imu
    init_bmi160_sensor_driver_structure();
    /* After sensor init introduce 200 msec sleep */
    vTaskDelay(pdMS_TO_TICKS(200));
    init_bmi160();

    goto selfDelete;

    int times_to_read = 0;
    while (times_to_read < 100)
    {
        /* To read both Accel and Gyro data */
        bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &bmi160_accel, &bmi160_gyro, &bmi160dev);

        ESP_LOGI(logTAG, "ax:%d\tay:%d\taz:%d\n", bmi160_accel.x, bmi160_accel.y, bmi160_accel.z);
        ESP_LOGI(logTAG, "gx:%d\tgy:%d\tgz:%d\n", bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z);

        delay_ms_for_bmi160Api(10);
        times_to_read = times_to_read + 1;
    }

    selfDelete:
    ESP_LOGI("IMU_BMI160", "BMI160 task self delete");
    vTaskDelete(NULL);
}
