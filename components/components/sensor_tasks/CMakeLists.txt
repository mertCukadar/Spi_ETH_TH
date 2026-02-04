idf_component_register(SRCS "src/sensor_tasks.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES aht10 bmp180 esp_timer driver espnow_sender)