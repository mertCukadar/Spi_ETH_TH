#ifndef ESPNOW_SENDER_H
#define ESPNOW_SENDER_H

#include "esp_err.h"
#include "sensor_tasks.h"  // For sensor_data_t

// Define broadcast MAC address
#define ESPNOW_BROADCAST_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// Payload structure for ESP-NOW (packed to minimize size)
typedef struct {
    float temperature;
    int32_t pressure;
    float humidity;
    float altitude;
    uint32_t timestamp;
} __attribute__((packed)) espnow_payload_t;

// Initialize ESP-NOW in broadcast mode
// @param channel: Wi-Fi channel (1-13 for most regions, 1-14 for Japan)
// @return ESP_OK on success
esp_err_t espnow_init(uint8_t channel);

// Send sensor data via broadcast ESP-NOW
// @param data: Pointer to sensor_data_t structure
// @return ESP_OK on success
esp_err_t espnow_send_sensor_data(const sensor_data_t *data);

#endif // ESPNOW_SENDER_H
