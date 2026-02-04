#ifndef SENSOR_TASKS_H
#define SENSOR_TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "bmp180.h"
#include "aht10.h"
#include "esp_timer.h"
#include <inttypes.h>


typedef struct {
    float temperature;
    int32_t pressure;
    aht10_data_t aht10_datas;
    float altitude;
    uint32_t timestamp;
} sensor_data_t;


// queue handle (external to make it accessible in other files)
extern QueueHandle_t sensor_data_queue;

//function prototypes
esp_err_t sensor_tasks_init(void);
void sensor_read_task(void *pvParameters);

void read_aht10_task(void *pvParameters);

#endif //SENSOR_TASKS_H