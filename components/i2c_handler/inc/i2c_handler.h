#ifndef IC2C_HANDLER_H
#define IC2C_HANDLER_H

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/*
@brief Initialize the I2C master bus.
@return ESP_OK on success, or an error code on failure.
*/
esp_err_t i2c_init();



/*
@brief add a device to the I2C bus.

@param device_handle Pointer to the device handle to be initialized.
@param device_config Pointer to the device configuration structure.

@return ESP_OK on success, or an error code on failure.
*/

esp_err_t i2c_add_bus_device(i2c_master_dev_handle_t *device_handle,
                        const i2c_device_config_t *device_config);

#endif // IC2C_HANDLER_H