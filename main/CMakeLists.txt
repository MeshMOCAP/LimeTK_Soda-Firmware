idf_component_register(
    SRCS "main.c" "tasks/bmi160_Task.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES "driver" "esp_timer" "BMI160_SensorAPI" "KalmanFilter.c"
)
