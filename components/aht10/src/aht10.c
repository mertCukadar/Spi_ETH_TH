#include "aht10.h"

#define AHT10_CMD_INIT 0xE1
#define AHT10_CMD_MEASURE 0xAC


#define AHT10_CMD_MEASURE_DATA 0x33
#define AHT10_CMD_SOFT_RESET 0xBA
#define AHT10_CMD_READ_STATUS 0x71
#define AHT10_STATUS_BUSY 0x08

i2c_master_dev_handle_t aht10_dev_handle = NULL;


// device configuration
i2c_device_config_t aht_10_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = AHTX0_I2CADDR_DEFAULT,
    .scl_speed_hz = 100000,
};

//TODO: check [7 bit] if 0 ready to read measurements
esp_err_t aht10_get_status(uint8_t *out_status) {
    uint8_t cmd = AHT10_CMD_READ_STATUS; 
    esp_err_t err = i2c_master_transmit_receive(
        aht10_dev_handle,
        &cmd, 1,
        out_status, 1,
        1000 / portTICK_PERIOD_MS
    );
    if (err != ESP_OK) {
        ESP_LOGE("AHT10", "Status read failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}


esp_err_t aht10_add_device() {

    return i2c_add_bus_device(&aht10_dev_handle, &aht_10_dev_cfg);
}



esp_err_t aht10_cmd_init(void)
{
    esp_err_t err = aht10_add_device();
    if (err != ESP_OK){
        ESP_LOGE("AHT10", "Failed to add device: %s", esp_err_to_name(err));
    }

    uint8_t soft_reset = AHTX0_CMD_SOFTRESET; 
    
    //SOFT RESET
    err = i2c_master_transmit(aht10_dev_handle, &soft_reset, 1, -1);
    if (err != ESP_OK) {
        ESP_LOGE("AHT10", "Soft reset failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI("AHT10", "Soft reset command sent successfully.");
    vTaskDelay(pdMS_TO_TICKS(30)); // Wait for reset to complete
    return ESP_OK;
}   



esp_err_t aht10_get_event(aht10_data_t *data)
{
    uint8_t cmd[3] = {AHTX0_CMD_TRIGGER, 0x33, 0x00}; // AHT10 için 0xAC 0x33 0x00

    ESP_ERROR_CHECK_WITHOUT_ABORT(
        i2c_master_transmit(aht10_dev_handle, cmd, sizeof(cmd), -1)
    );
    ESP_LOGI("AHT10", "Command sent successfully.");

    vTaskDelay(pdMS_TO_TICKS(100)); // AHT10'un ölçüm yapması için bekleme süresi
    uint8_t buf[6];

    esp_err_t err = i2c_master_receive(aht10_dev_handle, buf, sizeof(buf), -1);
    if (err != ESP_OK) {
        ESP_LOGE("AHT10", "Data read failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI("AHT", "bytes: %02X %02X %02X %02X %02X %02X",
         buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);

    uint32_t raw_humidity = (((uint32_t)buf[1] << 12)|
                            ((uint32_t)buf[2] << 4) |
                            ((buf[3] & 0xF0) >> 4));

    uint32_t raw_temperature = (((uint32_t)(buf[3] & 0x0F) << 16)|
                                        ((uint32_t)buf[4] << 8) |
                                        buf[5]);
    
    
    data->humidity = ((float)raw_humidity / 1048576.0f) * 100.0f; // RH[%] = (S_RH / 2^20) * 100%
    data->temperature = ((float)raw_temperature / 1048576.0f) * 200.0f - 50.0f; // T(°C) = (S_T / 2^20) * 200 - 50

    ESP_LOGI("AHT10", "Temperature: %.2f °C, Humidity: %.2f %%", data->temperature, data->humidity);

    return ESP_OK;
}