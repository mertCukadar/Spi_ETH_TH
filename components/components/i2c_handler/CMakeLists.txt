
idf_component_register(SRCS "src/i2c_handler.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES driver esp_driver_i2c
)