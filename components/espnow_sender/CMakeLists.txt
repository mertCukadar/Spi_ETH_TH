
idf_component_register(SRCS "src/espnow_sender.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES driver esp_driver_i2c esp_wifi sensor_tasks nvs_flash
)