// system header
#include <stdint.h>
// esp-idf header
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
// external header
#include "bmi160.h"


/*
    local macro definitions
*/

/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_I2C  0

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI160_INTERFACE_SPI  1

#if (!((BMI160_INTERFACE_I2C == 1) && (BMI160_INTERFACE_SPI == 0)) && \
    (!((BMI160_INTERFACE_I2C == 0) && (BMI160_INTERFACE_SPI == 1))))
#error "Invalid value given for the macros BMI160_INTERFACE_I2C / BMI160_INTERFACE_SPI"
#endif

/*! bmi160 shuttle id */
// #define BMI160_SHUTTLE_ID     0x38

/*! bmi160 Device address */
#define BMI160_DEV_ADDR     CONFIG_I2C_ADDR


/* 
    externs variables
*/
extern i2c_master_dev_handle_t i2c_device_handle_1;
extern spi_device_handle_t spi_device_handle_1;


// task function
void BMI160_task(void *pvParameters);
