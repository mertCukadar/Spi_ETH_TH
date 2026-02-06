idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES aht10 i2c_handler sensor_tasks esp_netif esp_eth ethernet_init udp_sender)
