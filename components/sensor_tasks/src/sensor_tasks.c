#include "sensor_tasks.h"
#include "bmp180.h"
#include "aht10.h"
#include "esp_timer.h"
#include "espnow_sender.h"
#include <inttypes.h>

static const char *SENSOR_TASKS_TAG = "SENSOR_TASKS";

QueueHandle_t sensor_data_queue = NULL;
TaskHandle_t sensor_read_task_handle = NULL;

esp_err_t sensor_tasks_init(void) {
    // Create the queue to hold sensor data
    sensor_data_queue = xQueueCreate(5, sizeof(sensor_data_t));
    if (sensor_data_queue == NULL) {
        ESP_LOGE(SENSOR_TASKS_TAG, "Failed to create sensor data queue");
        return ESP_FAIL;
    }

    // Create the sensor read task
    xTaskCreate(sensor_read_task, "sensor_read_task", 4096, NULL, 5 , &sensor_read_task_handle);
    if (sensor_read_task_handle == NULL) {
        ESP_LOGE(SENSOR_TASKS_TAG, "Failed to create sensor read task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void sensor_read_task(void *pvParameters) {
    sensor_data_t sensor_data;
    for(;;) {
        // Read BMP180 data
        esp_err_t err = bmp180_read_pressure_temp(&sensor_data.pressure, &sensor_data.temperature);
        if (err != ESP_OK) {
            ESP_LOGE(SENSOR_TASKS_TAG, "Failed to read BMP180 data: %s", esp_err_to_name(err));
        } else {
            // Calculate altitude
            bmp180_cal_altitude(sensor_data.pressure, &sensor_data.altitude);
        }

        // Read AHT10 data with status check
        uint8_t aht10_status = 0;
        int retry_count = 0;
        do {
            if (retry_count > 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            aht10_get_status(&aht10_status);
            retry_count++;
        } while ((aht10_status & 0x80) && retry_count < 10);
        
        err = aht10_get_event(&sensor_data.aht10_datas);
        if (err != ESP_OK) {
            ESP_LOGE(SENSOR_TASKS_TAG, "Failed to read AHT10 data: %s", esp_err_to_name(err));
        }

        // Get current timestamp
        sensor_data.timestamp = (uint32_t)(esp_timer_get_time() / 1000000); // Convert microseconds to seconds

        // Send the sensor data to the queue
        if (xQueueSend(sensor_data_queue, &sensor_data, pdMS_TO_TICKS(1000)) != pdPASS) {
            ESP_LOGW(SENSOR_TASKS_TAG, "Sensor data queue full, failed to send data");
        } else {
            ESP_LOGI(SENSOR_TASKS_TAG, "Sensor data sent: Temp=%.2f C, Pressure=%" PRIi32 " Pa, Humidity=%.2f %%, Altitude=%.2f m",
                     sensor_data.temperature,
                     sensor_data.pressure,
                     sensor_data.aht10_datas.humidity,
                     sensor_data.altitude);
        }

        // Send data via ESP-NOW broadcast
        if (espnow_send_sensor_data(&sensor_data) != ESP_OK) {
            ESP_LOGW(SENSOR_TASKS_TAG, "Failed to send data via ESP-NOW");
        }

        // Wait before next reading
        vTaskDelay(pdMS_TO_TICKS(5000)); // Read every 5 seconds
    }
}


//aht10 read task
void read_aht10_task(void *pvParameters){
    aht10_data_t *param = (aht10_data_t *)pvParameters;

    while(1){

        aht10_get_event(param);

        vTaskDelay(pdMS_TO_TICKS(1000));


    }


}