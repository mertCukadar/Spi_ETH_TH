#include "bmp180.h"

i2c_master_dev_handle_t bmp180_dev_handle = NULL;

i2c_device_config_t bmp180_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = BMP180_I2C_ADDRESS,
    .scl_speed_hz = 100000,
};

// Calibration coefficients
int16_t AC1;
int16_t AC2;
int16_t AC3;
uint16_t AC4;
uint16_t AC5;
uint16_t AC6;
int16_t B1;
int16_t B2;
int16_t MB;
int16_t MC;
int16_t MD;

esp_err_t bmp180_add_device() {
    return i2c_add_bus_device(&bmp180_dev_handle, &bmp180_dev_cfg);
}

esp_err_t bmp180_read_calibration() {
    uint8_t cal_data[22] = {0};
    uint8_t reg_addr = BMP180_REG_CAL_AC1;
    
    // Read all 22 calibration bytes (0xAA to 0xBF) at once
    esp_err_t err = i2c_master_transmit_receive(
        bmp180_dev_handle,
        &reg_addr, 1,
        cal_data, 22,
        1000 / portTICK_PERIOD_MS
    );
    
    if (err != ESP_OK) {
        ESP_LOGE("BMP180", "Failed to read calibration data: %s", esp_err_to_name(err));
        return err;
    }
    
    // Parse calibration data (all values are MSB first)
    AC1 = (int16_t)((cal_data[0] << 8) | cal_data[1]);
    AC2 = (int16_t)((cal_data[2] << 8) | cal_data[3]);
    AC3 = (int16_t)((cal_data[4] << 8) | cal_data[5]);
    AC4 = (uint16_t)((cal_data[6] << 8) | cal_data[7]);
    AC5 = (uint16_t)((cal_data[8] << 8) | cal_data[9]);
    AC6 = (uint16_t)((cal_data[10] << 8) | cal_data[11]);
    B1 = (int16_t)((cal_data[12] << 8) | cal_data[13]);
    B2 = (int16_t)((cal_data[14] << 8) | cal_data[15]);
    MB = (int16_t)((cal_data[16] << 8) | cal_data[17]);
    MC = (int16_t)((cal_data[18] << 8) | cal_data[19]);
    MD = (int16_t)((cal_data[20] << 8) | cal_data[21]);
    
    ESP_LOGI("BMP180", "Calibration coefficients read successfully");
    ESP_LOGI("BMP180", "AC1=%d, AC2=%d, AC3=%d, AC4=%u, AC5=%u, AC6=%u", 
             AC1, AC2, AC3, AC4, AC5, AC6);
    ESP_LOGI("BMP180", "B1=%d, B2=%d, MB=%d, MC=%d, MD=%d", 
             B1, B2, MB, MC, MD);
    
    return ESP_OK;
}

esp_err_t bmp180_read_temperature(float *temperature) {
    // Step 1: Write control register to start temperature measurement
    uint8_t control_cmd[2] = {BMP180_REG_CONTROL, 0x2E}; // Register address + control byte
    esp_err_t err = i2c_master_transmit(bmp180_dev_handle, control_cmd, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE("BMP180", "Failed to send temperature read command: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for conversion to complete (4.5ms for temperature)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Step 2: Read temperature data from result register
    uint8_t reg_addr = BMP180_REG_TEMP_MSB; // The register to read from
    uint8_t temp_data[2] = {0};
    err = i2c_master_transmit_receive(
        bmp180_dev_handle,
        &reg_addr, 1,  // Send only register address (1 byte)
        temp_data, 2,   // Receive 2 bytes
        1000 / portTICK_PERIOD_MS
    );
    if (err != ESP_OK) {
        ESP_LOGE("BMP180", "Failed to read temperature data: %s", esp_err_to_name(err));
        return err;
    }
    
    uint16_t UT = (temp_data[0] << 8) | temp_data[1]; // Raw temperature (unsigned!)
    
    ESP_LOGI("BMP180", "Raw UT=%u (0x%04X)", UT, UT);
    
    // Calculate true temperature using calibration coefficients
    int32_t X1 = (((int32_t)UT - (int32_t)AC6) * (int32_t)AC5) >> 15; // X1 = (UT - AC6) * AC5 / 2^15
    int32_t X2 = ((int32_t)MC << 11) / (X1 + (int32_t)MD);   // X2 = MC * 2^11 / (X1 + MD)
    int32_t B5 = X1 + X2;                   // B5 = X1 + X2
    int32_t T = (B5 + 8) >> 4;              // T = (B5 + 8) / 2^4
    
    ESP_LOGI("BMP180", "X1=%d, X2=%d, B5=%d, T=%d", (int)X1, (int)X2, (int)B5, (int)T);
    
    *temperature = T / 10.0; // Convert to degrees Celsius (0.1°C resolution)
    
    return ESP_OK;
}




esp_err_t bmp180_read_pressure_temp(int32_t *pressure , float *temperature) {
    // Step 1: Read temperature first to get B5 (required for pressure calculation)
    uint8_t temp_cmd[2] = {BMP180_REG_CONTROL, 0x2E};
    esp_err_t err = i2c_master_transmit(bmp180_dev_handle, temp_cmd, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE("BMP180", "Failed to send temperature command: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    uint8_t temp_reg = BMP180_REG_TEMP_MSB;
    uint8_t temp_data[2] = {0};
    err = i2c_master_transmit_receive(bmp180_dev_handle, &temp_reg, 1, temp_data, 2, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE("BMP180", "Failed to read temperature: %s", esp_err_to_name(err));
        return err;
    }

    uint16_t UT = (temp_data[0] << 8) | temp_data[1]; // Raw temperature (unsigned!)
    
       // Calculate true temperature using calibration coefficients
    int32_t X1 = (((int32_t)UT - (int32_t)AC6) * (int32_t)AC5) >> 15; // X1 = (UT - AC6) * AC5 / 2^15
    int32_t X2 = ((int32_t)MC << 11) / (X1 + (int32_t)MD);   // X2 = MC * 2^11 / (X1 + MD)
    int32_t B5 = X1 + X2;                   // B5 = X1 + X2
    int32_t T = (B5 + 8) >> 4;              // T = (B5 + 8) / 2^4
    
    ESP_LOGI("BMP180", "X1=%d, X2=%d, B5=%d, T=%d", (int)X1, (int)X2, (int)B5, (int)T);
    
    *temperature = T / 10.0; // Convert to degrees Celsius (0.1°C resolution)
    
    
    // Step 2: Start pressure measurement (OSS=0, standard mode)
    uint8_t pressure_cmd[2] = {BMP180_REG_CONTROL, 0x34}; // 0x34 = pressure with OSS=0
    err = i2c_master_transmit(bmp180_dev_handle, pressure_cmd, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE("BMP180", "Failed to send pressure command: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for conversion (≥ one tick; 4.5ms min for OSS=0)
    
    // Step 3: Read raw pressure data
    uint8_t press_reg = BMP180_REG_PRESSURE_MSB;
    uint8_t pressure_data[3] = {0};
    err = i2c_master_transmit_receive(bmp180_dev_handle, &press_reg, 1, pressure_data, 3, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE("BMP180", "Failed to read pressure data: %s", esp_err_to_name(err));
        return err;
    }
    
    int32_t UP = ((pressure_data[0] << 16) | (pressure_data[1] << 8) | pressure_data[2]) >> 8; // OSS=0, so shift right 8
    
    // Step 4: Calculate true pressure using calibration
    int32_t B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((AC1 * 4 + X3) << 0) + 2) >> 2; // OSS=0
    X1 = (AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = (AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)UP - B3) * (50000 >> 0); // OSS=0
    
    int32_t p;
    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }
    
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);
    
    *pressure = p; // Pressure in Pa
    return ESP_OK;
}



//TODO: alternative Formula: Formul: h = (R * T / g) * ln(P0 / P)
// where:
// h = altitude (in meters)
// R = specific gas constant for dry air (approximately 287.05 J/(kg·K))
// T = absolute temperature (in Kelvin)
// g = acceleration due to gravity (approximately 9.80665 m/s²)
// P0 = standard sea-level pressure (101325 Pa)
// P = measured pressure (in Pa)


esp_err_t bmp180_cal_altitude(float pressure, float *altitude) {
    // Calculate altitude using the barometric formula
    // Altitude (in meters) = 44330 * (1 - (P / P0)^(1/5.255))
    // where P0 is the standard sea-level pressure (101325 Pa)
    const float P0 = 101325.0f; // Standard sea-level pressure in Pa
    *altitude = 44330.0f * (1.0f - powf(pressure / P0, 0.1903f));
    return ESP_OK;
}