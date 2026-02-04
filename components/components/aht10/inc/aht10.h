#ifndef AHT10_H
#define AHT10_H

#include "i2c_handler.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define AHTX0_I2CADDR_DEFAULT 0x38   ///< AHT default i2c address
#define AHTX0_I2CADDR_ALTERNATE 0x39 ///< AHT alternate i2c address
#define AHTX0_CMD_CALIBRATE 0xE1     ///< Calibration command
#define AHTX0_CMD_TRIGGER 0xAC       ///< Trigger reading command
#define AHTX0_CMD_SOFTRESET 0xBA     ///< Soft reset command


typedef struct {
    float temperature;
    float humidity;
} aht10_data_t;


esp_err_t aht10_get_status(uint8_t *out_statusid);


esp_err_t aht10_add_device();

//cmd and sizeof
esp_err_t aht10_cmd_init();


esp_err_t aht10_get_event(aht10_data_t *data);


#endif // AHT10_H