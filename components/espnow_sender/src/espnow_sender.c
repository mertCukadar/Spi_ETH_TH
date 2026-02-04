#include "espnow_sender.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h" // for MACSTR/MAC2STR
#include "nvs_flash.h"

static const char *TAG = "ESPNOW_SENDER";

// Broadcast MAC address
static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = ESPNOW_BROADCAST_MAC;

// ESP-NOW send callback
static void on_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL) {
        ESP_LOGE(TAG, "Send callback: invalid MAC address");
        return;
    }
    
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "Broadcast sent successfully to " MACSTR, MAC2STR(mac_addr));
    } else {
        ESP_LOGW(TAG, "Broadcast send failed to " MACSTR, MAC2STR(mac_addr));
    }
}

// Initialize ESP-NOW in broadcast mode
esp_err_t espnow_init(uint8_t channel)
{
    ESP_LOGI(TAG, "Initializing ESP-NOW in broadcast mode on channel %d", channel);
    
    // Initialize NVS (required for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize Wi-Fi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // Set Wi-Fi mode to STA (station) - no need to connect to AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    
    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    // Set Wi-Fi channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    
    ESP_LOGI(TAG, "Wi-Fi initialized in STA mode");
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ESP-NOW");
        return ESP_FAIL;
    }
    
    // Register send callback
    ESP_ERROR_CHECK(esp_now_register_send_cb((esp_now_send_cb_t)on_send_callback));
    
    // Add broadcast peer (no encryption, no LMK)
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = channel;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;  // No encryption for broadcast
    
    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add broadcast peer");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    return ESP_OK;
}

// Send sensor data via broadcast ESP-NOW
esp_err_t espnow_send_sensor_data(const sensor_data_t *data)
{
    if (data == NULL) {
        ESP_LOGE(TAG, "Invalid sensor data pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create ESP-NOW payload from sensor data
    espnow_payload_t payload = {
        .temperature = data->temperature,
        .pressure = data->pressure,
        .humidity = data->aht10_datas.humidity,
        .altitude = data->altitude,
        .timestamp = data->timestamp
    };
    
    // Send via broadcast (all 0xFF MAC)
    esp_err_t ret = esp_now_send(broadcast_mac, 
                                 (uint8_t *)&payload, 
                                 sizeof(espnow_payload_t));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send ESP-NOW data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Sensor data sent via broadcast (size: %d bytes)", sizeof(espnow_payload_t));
    return ESP_OK;
}