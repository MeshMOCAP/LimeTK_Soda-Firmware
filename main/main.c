#include <stdbool.h>
// esp-idf header
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
// external header
#include "tasks/bmi160_Task.h"
#include "include/sdk_config.h"
#include "tasks/imu_dataT.h"

// create structures
// from freeRTOS
QueueHandle_t xQueueTrans;
// from esp-idf i2c driver
// master bus 1
i2c_master_bus_config_t i2c_bus_conf_1;
i2c_master_bus_handle_t i2c_bus_handle_1;
// master device 1
i2c_device_config_t i2c_device_conf_1;
i2c_master_dev_handle_t i2c_device_handle_1;
// from esp-idf spi driver
spi_bus_config_t spi_bus_conf_spi2;
spi_device_interface_config_t spi_device_conf_1;
spi_device_handle_t spi_device_handle_1;

void start_i2c(void)
{
    // config master bus
    // https://docs.espressif.com/projects/esp-idf/zh_CN/v5.2.1/esp32c2/api-reference/peripherals/i2c.html#_CPPv423i2c_master_bus_config_t
    i2c_bus_conf_1.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_conf_1.i2c_port = -1;
    i2c_bus_conf_1.sda_io_num = CONFIG_GPIO_SDA;
    i2c_bus_conf_1.scl_io_num = CONFIG_GPIO_SCL;
    i2c_bus_conf_1.glitch_ignore_cnt = 7;
    i2c_bus_conf_1.intr_priority = 0;
    i2c_bus_conf_1.flags.enable_internal_pullup = true;
    // create master bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_conf_1, &i2c_bus_handle_1));

    // config device
    i2c_device_conf_1.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    i2c_device_conf_1.device_address = CONFIG_I2C_ADDR;
    i2c_device_conf_1.scl_speed_hz = 400*1000; // 400KHz
    // create device
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle_1, &i2c_device_conf_1, &i2c_device_handle_1));
}

void start_spi(void)
{
    spi_bus_conf_spi2.miso_io_num = PIN_NUM_MISO;
    spi_bus_conf_spi2.mosi_io_num = PIN_NUM_MOSI;
    spi_bus_conf_spi2.sclk_io_num = PIN_NUM_CLK;
    spi_bus_conf_spi2.max_transfer_sz = 1024;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_conf_spi2, SPI_DMA_CH_AUTO));

    spi_device_conf_1.command_bits = 0;
    spi_device_conf_1.address_bits = 0;
    spi_device_conf_1.clock_speed_hz = 800*1000;
    spi_device_conf_1.mode = 0;
    spi_device_conf_1.spics_io_num = PIN_NUM_CS;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &spi_device_conf_1, &spi_device_handle_1));
}

void app_main()
{
    start_i2c();
    start_spi();

    // Create Queue
    xQueueTrans = xQueueCreate(10, sizeof(POSE_t));
    configASSERT( xQueueTrans );

    // Start imu task
    xTaskCreate(BMI160_task, "IMU", 1024*8, NULL, 5, NULL);

    return;
}
