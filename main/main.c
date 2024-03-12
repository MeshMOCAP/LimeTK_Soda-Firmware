#include <stdbool.h>
// freeRTOS header
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
// esp-idf header
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
// external header
#include "tasks/bmi160_Task.h"
#include "tasks/imu_dataT.h"

// create structures
// from freeRTOS
QueueHandle_t xQueueTrans;
// from esp-idf i2c driver
i2c_master_bus_handle_t i2c_bus_handle_1;
i2c_master_dev_handle_t i2c_device_handle_1;
// from esp-idf spi driver
spi_device_handle_t spi_device_handle_1;

static const char *logTAG = "App_main";

static void start_i2c(void)
{
    esp_err_t ret;

    // config master bus
    // https://docs.espressif.com/projects/esp-idf/zh_CN/v5.2.1/esp32c2/api-reference/peripherals/i2c.html#_CPPv423i2c_master_bus_config_t
    i2c_master_bus_config_t i2c_bus_conf_1 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .sda_io_num = CONFIG_GPIO_SDA,
        .scl_io_num = CONFIG_GPIO_SCL,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = true,
    };
    // create master bus
    ret = i2c_new_master_bus(&i2c_bus_conf_1, &i2c_bus_handle_1);
    ESP_ERROR_CHECK(ret);

    // config device
    i2c_device_config_t i2c_device_conf_1 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = CONFIG_I2C_ADDR,
        .scl_speed_hz = 400*1000, // 400KHz
    };
    // create device
    ret = i2c_master_bus_add_device(i2c_bus_handle_1, &i2c_device_conf_1, &i2c_device_handle_1);
    ESP_ERROR_CHECK(ret);
}

static void start_spi(void)
{
    esp_err_t ret;

    // 文档链接"SPI 主机驱动程序"
    // https://docs.espressif.com/projects/esp-idf/zh_CN/v5.2/esp32c2/api-reference/peripherals/spi_master.html
    // init bus
    spi_bus_config_t spi_bus_conf_spi2 = {
        .miso_io_num = CONFIG_PIN_NUM_MISO,
        .mosi_io_num = CONFIG_PIN_NUM_MOSI,
        .sclk_io_num = CONFIG_PIN_NUM_CLK,
        .max_transfer_sz = 1024,
    };
    ret = spi_bus_initialize(SPI2_HOST, &spi_bus_conf_spi2, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ESP_LOGD(logTAG, "SPI(2) bus init finished");

    // add device
    spi_device_interface_config_t spi_device_conf_1 = {
        .command_bits = 1,
        .address_bits = 0,
        .clock_speed_hz = 600*1000, // 600kHz
        .mode = 0,
        .spics_io_num = CONFIG_PIN_NUM_CS,
        .queue_size = 2,
    };
    ret = spi_bus_add_device(SPI2_HOST, &spi_device_conf_1, &spi_device_handle_1);
    ESP_ERROR_CHECK(ret);
    ESP_LOGD(logTAG, "SPI(2) add device finished");
}

void app_main(void)
{
    // start_i2c();
    start_spi();

    // Create Queue
    xQueueTrans = xQueueCreate(10, sizeof(POSE_t));
    configASSERT( xQueueTrans );

    // Start imu task
    xTaskCreate(BMI160_task, "IMU", 1024*8, NULL, 5, NULL);

    return;
}
