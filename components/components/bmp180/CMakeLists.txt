
idf_component_register(SRCS "src/bmp180.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES driver i2c_handler
)