#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "ethernet_init.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_netif_sntp.h"

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

// Hardware reset for W5500 via GPIO 10
static void w5500_hardware_reset(void) {
    ESP_LOGI(TAG, "Performing W5500 hardware reset on GPIO 10...");
    
    // Configure GPIO 10 as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << 10),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Reset pulse: LOW -> HIGH -> LOW
    gpio_set_level(10, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(10, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "W5500 hardware reset complete");
}

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

    ret = spi_bus_initialize(CONFIG_ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_eth_handle_t eth_init_spi(spi_eth_module_config_t *spi_eth_module_config, esp_eth_mac_t **mac_out, esp_eth_phy_t **phy_out){

    esp_eth_handle_t ret = NULL;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();      // apply default common MAC configuration
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();      // apply default PHY configuration
    // Install GPIO interrupt service (as the SPI-Ethernet module is interrupt-driven)
    gpio_install_isr_service(0);
    
    // Perform hardware reset on W5500
    w5500_hardware_reset();
    
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
    w5500_config.int_gpio_num = -1;  // Use polling mode (no interrupt GPIO)
    w5500_config.poll_period_ms = 10;  // Poll every 10ms
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


// Event handler for ethernet events (connected/disconnected)
/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

esp_err_t ethernet_init() {
    ESP_LOGI(TAG, "Starting Ethernet initialization...");
    
    spi_eth_module_config_t spi_eth_module_config = {
        .spi_cs_gpio = CONFIG_ETH_SPI_CS_GPIO,
        .int_gpio = -1,  // Use polling mode, not interrupt
        .polling_ms = 10,
        .phy_reset_gpio = -1,
        .phy_addr = 0,
        .mac_addr = NULL,
    };

    // Initialize SPI Ethernet and get driver handle
    esp_eth_mac_t *mac = NULL;
    esp_eth_phy_t *phy = NULL;
    esp_eth_handle_t eth_handle = eth_init_spi(&spi_eth_module_config, &mac, &phy);
    if (eth_handle == NULL) {
        ESP_LOGE(TAG, "Failed to initialize SPI Ethernet");
        return ESP_FAIL;
    }

    // Create netif for Ethernet
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    // Attach ethernet driver to netif
    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
    if (glue == NULL) {
        ESP_LOGE(TAG, "Failed to create netif glue");
        esp_eth_driver_uninstall(eth_handle);
        return ESP_FAIL;
    }

    esp_netif_attach(eth_netif, glue);

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // Start Ethernet
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    ESP_LOGI(TAG, "Ethernet started successfully");

    return ESP_OK;
}


#endif // CONFIG_W5500_ETHERNET_CONTROLLER