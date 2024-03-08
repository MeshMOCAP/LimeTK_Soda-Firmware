extern i2c_master_dev_handle_t i2c_device_handle_1;

// delay ms "function"
static void task_delay_ms(uint32_t delay_time);

// init driver config structure
static void init_bmi160_sensor_driver_structure(void);

// init sensor
static void init_bmi160(void);

// task function
void bmi160(void *pvParameters);
