idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES qmi8658a esp_driver_gpio esp_driver_i2c esp_timer)
