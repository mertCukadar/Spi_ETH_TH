idf_component_register(SRCS "src/udp_sender.c"
                       PRIV_REQUIRES esp_driver_gpio esp_eth esp_netif json aht10 esp_timer
                       INCLUDE_DIRS "inc")
