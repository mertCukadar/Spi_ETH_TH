#ifndef BMP180_H
#define BMP180_H

#include "esp_log.h"
#include "esp_err.h"
#include "i2c_handler.h"
#include "math.h"


//TODO: DEFINE BMP180 CONSTANTS AND FUNCTION PROTOTYPES HERE

#define BMP180_I2C_ADDRESS      0x77
#define BMP180_REG_CONTROL      0xF4
#define BMP180_REG_TEMP_MSB     0xF6
#define BMP180_REG_TEMP_LSB     0xF7
#define BMP180_REG_PRESSURE_MSB 0xF6
#define BMP180_REG_PRESSURE_LSB 0xF7
#define BMP180_REG_PRESSURE_XLSB 0xF8

// EEPROM calibration registers
#define BMP180_REG_CAL_AC1      0xAA
#define BMP180_REG_CAL_AC2      0xAC
#define BMP180_REG_CAL_AC3      0xAE
#define BMP180_REG_CAL_AC4      0xB0
#define BMP180_REG_CAL_AC5      0xB2
#define BMP180_REG_CAL_AC6      0xB4
#define BMP180_REG_CAL_B1       0xB6
#define BMP180_REG_CAL_B2       0xB8
#define BMP180_REG_CAL_MB       0xBA
#define BMP180_REG_CAL_MC       0xBC
#define BMP180_REG_CAL_MD       0xBE

// Calibration coefficients (read from EEPROM)
extern int16_t AC1;
extern int16_t AC2;
extern int16_t AC3;
extern uint16_t AC4;
extern uint16_t AC5;
extern uint16_t AC6;
extern int16_t B1;
extern int16_t B2;
extern int16_t MB;
extern int16_t MC;
extern int16_t MD;


esp_err_t bmp180_add_device();
esp_err_t bmp180_read_calibration();
esp_err_t bmp180_read_pressure_temp(int32_t *pressure , float *temperature);
esp_err_t bmp180_cal_altitude(float pressure, float *altitude);





#endif //BMP180_H