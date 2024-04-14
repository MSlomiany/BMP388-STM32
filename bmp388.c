/**
 * @file bmp388.c
 * @author Michal Slomiany (mslomiany@icloud.com)
 * @brief API for Bosch BMP388 pressure sensor
 * @version 0.1
 * @date 2021-08-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/* Includes */
#include "bmp388.h"

/* Structure with calibration data */
bmp_calibration_data calibration_data;

/* Structure with sensor status values */
bmp_status status;

/**
 * @brief This function read 8-bit data from single device register.
 * 
 */
uint8_t BMP_read8(uint8_t address)
{
    uint8_t tmp;
    HAL_I2C_Mem_Read(status.i2c_h, BMP388_I2C_ADDR, address, 1, &tmp, 1, 10);
    return tmp;
}

/**
 * @brief This function read and parse 16-bit data from single device register. Since because data is 
 * stored in consecutive registers from least significant to most significant part, functions perform
 * bit shifting operatrions to parse final value.
 * 
 */
uint16_t BMP_read16(uint8_t address)
{
    uint8_t tmp[2];
    HAL_I2C_Mem_Read(status.i2c_h, BMP388_I2C_ADDR, address, 1, tmp, 2, 10);
    return (tmp[1] << 8 | tmp[0]);
}

/**
 * @brief This function read and parse 24-bit data from single device register. Since data is 
 * stored in consecutive registers from least significant to most significant part, functions perform
 * bit shifting operatrions to parse final value which is stored as 32-bit unsigneg integer. Beacuse 
 * burst read is recommended, function read pressure and temperature registers at the same time.
 * 
 */
void BMP_read24(uint8_t address, uint32_t *pressure, uint32_t *temperature)
{
    uint8_t tmp[6];
    HAL_I2C_Mem_Read(status.i2c_h, BMP388_I2C_ADDR, address, 1, tmp, 6, 10);
    *pressure = (tmp[2] << 16 | tmp[1] << 8 | tmp[0]);
    *temperature = (tmp[5] << 16 | tmp[4] << 8 | tmp[3]);
}

/**
 * @brief This function write 8-bit data to desired register. It's used for device control pusposes.
 * 
 */
void BMP_write8(uint8_t address, uint8_t data)
{
    uint8_t tmp = data;
    HAL_I2C_Mem_Write(status.i2c_h, BMP388_I2C_ADDR, address, 1, &tmp, 1, 10);
}

/**
 * @brief API entry point. Initalize sensor with selected values and I2C bus. Perform self-test and
 * parse calibration coefficients.
 * 
 */
void BMP_init(I2C_HandleTypeDef *i2c_handler)
{
    uint8_t result;
    status.i2c_h = i2c_handler;
    result = checkChipId();
    if (result == STATUS_OK)
    {
        result = checkCalibrationCoeffs();
    }
    if (result == STATUS_OK)
    {
        saveCalibrationCoeffs();
        setOversampling(OVERSAMPLING_16, OVERSAMPLING_2);
        setFilterCoefficient(FILTER_31);
        setOutputDataRate(DATA_RATE_25);
        setPowerMode(PRESSURE_ON, TEMPERATURE_ON, MODE_NORMAL);
    }
    while (result != STATUS_OK)
    {
    };
}

/**
 * @brief Function check sensor ID and indicate whether valid device is communicating.
 *
 */
uint8_t checkChipId()
{
    uint8_t result = STATUS_OK;
    uint8_t tmp = BMP_read8(REG_CHIP_ID);
    if (tmp != BMP_CHIP_ID)
    {
        result = ERR_WRONG_CHIP_ID;
    }
    return result;
}

/**
 * @brief Function check if calibration coefficients stored in device's memory are valid.
 * 
 */
uint8_t checkCalibrationCoeffs()
{
    int8_t result = STATUS_OK;
    uint8_t crc = 0xFF;
    uint8_t stored_crc;
    uint8_t coeffsArray[21];
    uint8_t i;
    HAL_I2C_Mem_Read(status.i2c_h, BMP388_I2C_ADDR, REG_T1, 1, coeffsArray, 21, 10);
    for (i = 0; i < 21; i++)
    {
        crc = (uint8_t)calculateCrc(crc, coeffsArray[i]);
    }
    crc = (crc ^ 0xFF);
    stored_crc = BMP_read8(REG_CRC);
    if (stored_crc != crc)
    {
        result = ERR_INVALID_COEFFICIENTS;
    }
    return result;
}

/**
 * @brief Function calculate cyclic redundancy code for coefficient values
 * 
 */
int8_t calculateCrc(uint8_t seed, uint8_t data)
{
    int8_t poly = 0x1D;
    int8_t var2;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if ((seed & 0x80) ^ (data & 0x80))
        {
            var2 = 1;
        }
        else
        {
            var2 = 0;
        }

        seed = (seed & 0x7F) << 1;
        data = (data & 0x7F) << 1;
        seed = seed ^ (uint8_t)(poly * var2);
    }

    return (int8_t)seed;
}

/**
 * @brief Function read, parse and save calibration coefficients to bmp_calibration_data structure. 
 * 
 */
void saveCalibrationCoeffs()
{
    calibration_data.t1 = BMP_read16(REG_T1);
    calibration_data.t2 = BMP_read16(REG_T2);
    calibration_data.t3 = BMP_read8(REG_T3);
    calibration_data.p1 = BMP_read16(REG_P1);
    calibration_data.p2 = BMP_read16(REG_P2);
    calibration_data.p3 = BMP_read8(REG_P3);
    calibration_data.p4 = BMP_read8(REG_P4);
    calibration_data.p5 = BMP_read16(REG_P5);
    calibration_data.p6 = BMP_read16(REG_P6);
    calibration_data.p7 = BMP_read8(REG_P7);
    calibration_data.p8 = BMP_read8(REG_P8);
    calibration_data.p9 = BMP_read16(REG_P9);
    calibration_data.p10 = BMP_read8(REG_P10);
    calibration_data.p11 = BMP_read8(REG_P11);
}

/**
 * @brief Function parse measured temperature in Celsius degrees as float value
 * 
 */
float readTemperature(uint32_t raw_temperature)
{
    float temperature = compensateTemperature(raw_temperature) / 100.00;
    return temperature;
}

/**
 * @brief Function calculate compensated temperature
 * 
 */
int64_t compensateTemperature(uint32_t temperature)
{
    int64_t data1;
    int64_t data2;
    int64_t data3;
    int64_t data4;
    int64_t data5;
    int64_t data6;
    int64_t compensated_temperature;

    data1 = ((int64_t)temperature - (256 * calibration_data.t1));
    data2 = calibration_data.t2 * data1;
    data3 = (data1 * data1);
    data4 = (int64_t)data3 * calibration_data.t3;
    data5 = ((int64_t)(data2 * 262144) + data4);
    data6 = data5 / 4294967296;
    calibration_data.t_comp = data6;
    compensated_temperature = (int64_t)((data6 * 25) / 16384);

    return compensated_temperature;
}

/**
 * @brief Function read and parse measured pressure. Since temperature is neccessary to 
 * compensate raw readed pressure, function also read and parse temperature.
 * 
 */
void readPressure(float *pressure, float *temperature)
{
    uint32_t tmp_pressure;
    uint32_t tmp_temperature;
    BMP_read24(REG_PRESSURE, &tmp_pressure, &tmp_temperature);
    *temperature = readTemperature(tmp_temperature);
    float tmp_ps;
    tmp_ps = compensatePressure(tmp_pressure) / 100.0;
    *pressure = tmp_ps;
}

/**
 * @brief Function calculate compensated pressure
 * 
 */
uint64_t compensatePressure(uint32_t pressure)
{
    int64_t data1;
    int64_t data2;
    int64_t data3;
    int64_t data4;
    int64_t data5;
    int64_t data6;
    int64_t out1;
    int64_t out2;
    uint64_t compensated_pressure;

    data1 = calibration_data.t_comp * calibration_data.t_comp;
    data2 = data1 / 64;
    data3 = (data2 * calibration_data.t_comp) / 256;
    data4 = (calibration_data.p8 * data3) / 32;
    data5 = (calibration_data.p7 * data1) * 16;
    data6 = (calibration_data.p6 * calibration_data.t_comp) * 4194304;
    out1 = (calibration_data.p5 * 140737488355328) + data4 + data5 + data6;
    data2 = (calibration_data.p4 * data3) / 32;
    data4 = (calibration_data.p3 * data1) * 4;
    data5 = (calibration_data.p2 - 16384) * calibration_data.t_comp * 2097152;
    out2 = ((calibration_data.p1 - 16384) * 70368744177664) + data2 + data4 + data5;
    data1 = (out2 / 16777216) * pressure;
    data2 = calibration_data.p10 * calibration_data.t_comp;
    data3 = data2 + (65536 * calibration_data.p9);
    data4 = (data3 * pressure) / 8192;
    data5 = (pressure * (data4 / 10)) / 512;
    data5 = data5 * 10;
    data6 = (int64_t)((uint64_t)pressure * (uint64_t)pressure);
    data2 = (calibration_data.p11 * data6) / 65536;
    data3 = (data2 * pressure) / 128;
    data4 = (out1 / 4) + data1 + data5 + data3;
    compensated_pressure = (((uint64_t)data4 * 25) / (uint64_t)1099511627776);

    return compensated_pressure;
}

/**
 * @brief Set pressure IIR filter coefficient
 * 
 */
void setFilterCoefficient(uint8_t coeff)
{
    uint8_t tmp;
    tmp = (coeff << 1);
    BMP_write8(REG_IIR_CONFIG, tmp);
    status.filter_coefficient = coeff;
}

/**
 * @brief Set sensor output data rate
 * 
 */
void setOutputDataRate(uint8_t data_rate)
{
    BMP_write8(REG_ODR, data_rate);
    status.data_rate = data_rate;
}

/**
 * @brief Set selected pressure and temperature oversampling
 * 
 */
void setOversampling(uint8_t press_osr, uint8_t temp_osr)
{
    uint8_t tmp;
    tmp = (temp_osr << 3 | press_osr);
    BMP_write8(REG_OSR, tmp);
    status.pressure_oversampling = press_osr;
    status.temperature_oversampling = temp_osr;
}

/**
 * @brief Set sensor power mode
 * 
 */
void setPowerMode(uint8_t press_en, uint8_t temp_en, uint8_t power_mode)
{
    uint8_t tmp;
    tmp = (power_mode << 4 | temp_en << 1 | press_en);
    BMP_write8(REG_PWR_CNTRL, tmp);
    status.pressure_enable = press_en;
    status.temperature_enable = temp_en;
    status.power_mode = power_mode;
}
