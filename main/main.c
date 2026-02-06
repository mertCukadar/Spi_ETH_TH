#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "i2c_handler.h"
#include "aht10.h"
#include "ethernet_init.h"
#include "udp_sender.h"

static const char *MAIN_TAG = "APP_MAIN";

void app_main(void)
{
    // Initialize Ethernet driver
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    ESP_ERROR_CHECK(ethernet_init(&eth_handles, &eth_port_cnt));



    esp_err_t ret;

    // 1) NVS (network stack için şart)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    // 2) TCP/IP + event loop
    ESP_ERROR_CHECK(esp_netif_init());

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        // INVALID_STATE: zaten oluşturulmuş demek, sorun değil
        ESP_ERROR_CHECK(ret);
    }

    // 3) Ethernet init (W5500)
    ret = ethernet_init();
    if (ret != ESP_OK) {
        ESP_LOGE(MAIN_TAG, "ethernet_init failed: %s", esp_err_to_name(ret));
        // Ethernet yoksa UDP başlatma
        return;
    }

    // 4) I2C + AHT10 init (tek sefer)
    i2c_init();
    vTaskDelay(pdMS_TO_TICKS(75));

    ret = aht10_cmd_init();
    if (ret != ESP_OK) {
        ESP_LOGW(MAIN_TAG, "AHT10 init failed: %s", esp_err_to_name(ret));
        // Sensör yoksa yine de UDP başlatabilirsin ama payload boş olur.
        // Şimdilik devam edelim.
    }

    // 5) UDP sender task
    start_udp_sender();

    ESP_LOGI(MAIN_TAG, "System initialized.");
}
