#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "aht10.h"
#include "cJSON.h"
#include "esp_timer.h"
#include "udp_sender.h"

static const char *TAG = "udp_sender";

// Configure destination here; change port if needed
#define UDP_SERVER_IP "<server_ip_here>"
#define UDP_SERVER_PORT 5005
#define SEND_INTERVAL_MS 1000 * 60 * 5

void udp_sender_task(void *pvParameters)
{
    aht10_data_t sample;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SERVER_PORT);

    int sock = -1;
    
    // Wait for ethernet interface to be up before starting sends
    ESP_LOGI(TAG, "Waiting for ethernet interface to be up...");
    int wait_count = 0;
    while (!esp_netif_is_netif_up(esp_netif_get_handle_from_ifkey("ETH_DEF"))) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        wait_count++;
        if (wait_count % 5 == 0) {
            ESP_LOGI(TAG, "Still waiting for network... (%d seconds)", wait_count);
        }
    }
    ESP_LOGI(TAG, "Ethernet interface is up! Starting to send data.");
    ESP_LOGI(TAG, "Destination: %s:%d", UDP_SERVER_IP, UDP_SERVER_PORT);

    while (1) {
        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) {
                ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            ESP_LOGI(TAG, "Socket created");
        }

        // Read sensor (non-blocking but may use I2C driver delays internally)
        if (aht10_get_event(&sample) != ESP_OK) {
            ESP_LOGE(TAG, "AHT10 read failed");
        } else {
            // Build JSON payload
            cJSON *root = cJSON_CreateObject();
            if (root) {
                cJSON_AddNumberToObject(root, "temperature", sample.temperature);
                cJSON_AddNumberToObject(root, "humidity", sample.humidity);
                // timelaps in ms since boot
                int64_t ms = esp_timer_get_time() / 1000LL;
                cJSON_AddNumberToObject(root, "timelaps_ms", (double)ms);

                char *payload = cJSON_PrintUnformatted(root);
                if (payload) {
                    ssize_t sent = sendto(sock, payload, strlen(payload), 0,
                                          (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    if (sent < 0) {
                        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                        // close socket and attempt re-create next loop
                        close(sock);
                        sock = -1;
                        // Wait a bit before retrying to avoid spamming logs
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    } else {
                        ESP_LOGI(TAG, "Sent %d bytes: %s", (int)sent, payload);
                    }
                    free(payload);
                }
                cJSON_Delete(root);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }

    if (sock >= 0) {
        close(sock);
    }
    vTaskDelete(NULL);
}

// Helper to start the sender from other code
void start_udp_sender(void)
{
    xTaskCreate(udp_sender_task, "udp_sender", 4096, NULL, 5, NULL);
}
