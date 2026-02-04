#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "ethernet_init.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_err.h"

#if CONFIG_SPI_TH_ETHERNET_INIT
#include "driver/spi_master.h"
#endif //CONFIG_SPI_TH_ETHERNET_INIT


typedef struct {
    uint8_t spi_cs_gpio;
    int8_t int_gpio;
    uint32_t polling_ms;
    int8_t phy_reset_gpio;
    uint8_t phy_addr;
    uint8_t *mac_addr;
}spi_eth_module_config_t;


#if CONFIG_W5500_ETHERNET_CONTROLLER

static const char *TAG = "ethernet_init";

// SPI bus initialization function
static esp_err_t spi_bus_init(void){
    esp_err_t ret = ESP_OK;

    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_ETH_SPI_MISO_GPIO,
        .mosi_io_num = CONFIG_ETH_SPI_MOSI_GPIO,
        .sclk_io_num = CONFIG_ETH_SPI_CLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ret = spi_bus_initialize(CONFIG_ETH_SPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
  
    return ret;
}

static esp_eth_handle_t eth_init_spi(spi_eth_module_config_t *spi_eth_module_config, esp_eth_mac_t **mac_out, esp_eth_phy_t **phy_out){

    esp_eth_handle_t ret = NULL;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();      // apply default common MAC configuration
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();      // apply default PHY configuration
    // Install GPIO interrupt service (as the SPI-Ethernet module is interrupt-driven)
    gpio_install_isr_service(0);
    // SPI bus configuration

    
    spi_bus_init();

    spi_device_interface_config_t spi_devcfg = {
    .mode = 0,
    .clock_speed_hz = CONFIG_ETH_SPI_CLOCK_SPEED_HZ* 1000 * 1000,
    .spics_io_num = CONFIG_ETH_SPI_CS_GPIO,
    .queue_size = 20
    };



    /* W5500 ethernet driver is based on SPI device */
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(CONFIG_ETH_SPI_HOST, &spi_devcfg);
    w5500_config.int_gpio_num = spi_eth_module_config->int_gpio;
    w5500_config.poll_period_ms = spi_eth_module_config->polling_ms;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_handle_t eth_handle = NULL;
    esp_eth_config_t eth_config_spi = ETH_DEFAULT_CONFIG(mac, phy);

    esp_err_t err = esp_eth_driver_install(&eth_config_spi, &eth_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install SPI Ethernet driver: %s", esp_err_to_name(err));
        goto err;
    }


    // The SPI Ethernet module might not have a burned factory MAC address, we can set it manually.
    if (spi_eth_module_config->mac_addr != NULL) {
        err = esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, spi_eth_module_config->mac_addr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set MAC address: %s", esp_err_to_name(err));
            goto err;
        }
                                      
    }

    if (mac_out) {
        *mac_out = mac;
    }
    if (phy_out) {
        *phy_out = phy;
    }

    return eth_handle;
err:
    if (eth_handle != NULL) {
        esp_eth_driver_uninstall(eth_handle);
    }
    if (mac != NULL) {
        mac->del(mac);
    }
    if (phy != NULL) {
        phy->del(phy);
    }
    return ret;

}


esp_err_t ethernet_init() {
    // Placeholder for Ethernet initialization code
    // This function should set up the Ethernet hardware and network stack.
    return ESP_OK;
}


#endif // CONFIG_W5500_ETHERNET_CONTROLLER