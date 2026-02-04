#include "i2c_handler.h"
#include "driver/i2c_types.h" 


#define I2C_PORT I2C_NUM_0
#define I2C_MASTER_SDA GPIO_NUM_8
#define I2C_MASTER_SCL GPIO_NUM_9

const static char *I2C_TAG = "i2c_handler";
i2c_master_bus_handle_t bus_handle;



esp_err_t i2c_init()
{
    ESP_LOGI(I2C_TAG, "Initializing I2C master bus...");

    i2c_master_bus_config_t i2c_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = 8,   // SDA = GPIO8
        .scl_io_num = 9,   // SCL = GPIO9
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    //NOTE: if statement to check initialization
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_new_master_bus(&i2c_config, &bus_handle)); 
    ESP_LOGI(I2C_TAG, "I2C master bus initialized successfully.");

    return ESP_OK;
}

esp_err_t i2c_add_bus_device(
                            i2c_master_dev_handle_t *device_handle,
                            const i2c_device_config_t *device_config
                        )
{
    if (bus_handle == NULL) {
        ESP_LOGE(I2C_TAG,
            "I2C bus handle is NULL. Please initialize the I2C bus first.");
        return ESP_ERR_INVALID_STATE;
    }

    if(device_handle == NULL || device_config == NULL) {
        ESP_LOGE(I2C_TAG,
            "Invalid device handle or configuration.");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(I2C_TAG, "Adding I2C device...");
  

    esp_err_t ret = i2c_master_bus_add_device(bus_handle,
                                            device_config,
                                            device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(I2C_TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(I2C_TAG, "I2C device added successfully.");
    return ESP_OK;
}