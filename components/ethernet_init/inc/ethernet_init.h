#ifndef ETHERNET_INIT_H
#define ETHERNET_INIT_H

#include "esp_eth_driver.h"
#include "esp_err.h"

/**
 * @brief Initialize the Ethernet interface.
 * 
 * this function sets up the Ethernet hardware and network stack.
 * **/
esp_err_t ethernet_init();

#endif // ETHERNET_INIT_H