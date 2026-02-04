idf_component_register(SRCS "src/ethernet_init.c"
                       PRIV_REQUIRES esp_driver_gpio esp_eth esp_netif
                       INCLUDE_DIRS "inc")
