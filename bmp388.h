/**
 * @file bmp388.h
 * @author Michal Slomiany (mslomiany@icloud.com)
 * @brief API for Bosch BMP388 pressure sensor
 * @version 0.1
 * @date 2021-08-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/**
 * @defgroup bmp388 BMP388
 * @brief BMP388 API functions
 */

#ifndef BMP388_H
#define BMP388_H

/* Includes */
#include "main.h"
#include "bmp388_regs.h"

/**
 * \ingroup bmp388
 * \defgroup bmp388InterfFunc Interface functions 
 * @brief I2C interface functions
 */

/**
 * \ingroup bmp388InterfFunc
 * \page bmp388_api_BMP_read_8 BMP_read8
 * \code
 * uint8_t BMP_read8(uint8_t address);
 * \endcode
 * @brief This function read 8-bit data from single device register.
 * 
 * @param[in] address Register address
 * @return uint8_t - Data from register
 */
uint8_t BMP_read8(uint8_t address);

/**
 * \ingroup bmp388InterfFunc
 * \page bmp388_api_BMP_read_16 BMP_read16
 * \code
 * uint16_t BMP_read16(uint8_t address);
 * \endcode
 * @brief This function read and parse 16-bit data from single device register. Since because data is 
 * stored in consecutive registers from least significant to most significant part, functions perform
 * bit shifting operatrions to parse final value.
 * 
 * @param[in] address Register address
 * @return uint16_t - Data from register
 */
uint16_t BMP_read16(uint8_t address);

/**
 * \ingroup bmp388InterfFunc
 * \page bmp388_api_BMP_read_24 BMP_read24
 * \code
 * uint16_t BMP_read24(uint8_t address);
 * \endcode
 * @brief This function read and parse 24-bit data from single device register. Since data is 
 * stored in consecutive registers from least significant to most significant part, functions perform
 * bit shifting operatrions to parse final value which is stored as 32-bit unsigneg integer. Beacuse 
 * burst read is recommended, function read pressure and temperature registers at the same time.
 * 
 * @param[in] address Register address
 * @param[in] pressure Pointer to variable storing readed raw pressure
 * @param[in] temperature  Pointer to variable storing readed raw temperature
 */
void BMP_read24(uint8_t address, uint32_t *pressure, uint32_t *temperature);

/**
 * \ingroup bmp388InterfFunc
 * \page bmp388_api_BMP_write8 BMP_write8
 * \code
 * void BMP_write8(uint8_t address, uint8_t data);
 * \endcode
 * @brief This function write 8-bit data to desired register. It's used for device control pusposes.
 * 
 * @param [in] address Register address
 * @param [in] data Data to be write to register
 */
void BMP_write8(uint8_t address, uint8_t data);

/**
 * \ingroup bmp388
 * \defgroup bmp388Init Initialization
 * @brief Sensor initialization
 */

/**
 * \ingroup bmp388Init
 * \page bmp388_api_BMP_init BMP_init
 * \code
 * void BMP_init(I2C_HandleTypeDef *i2c_handler);
 * \endcode
 * @brief API entry point. Initalize sensor with selected values and I2C bus. Perform self-test and
 * parse calibration coefficients.
 * 
 * @param [in] i2c_handler Pointer to selected I2C handler
 */
void BMP_init(I2C_HandleTypeDef *i2c_handler);

/**
 * \ingroup bmp388
 * \defgroup bmp388SelfTest Self Test
 * @brief Sensor self testing functions
 */

/**
 * \ingroup bmp388SelfTest
 * \page bmp388_api_checkChipId checkChipId
 * \code
 * uint8_t checkChipId();
 * \endcode
 * @brief Function check sensor ID and indicate whether valid device is communicating.
 * 
 * @return uint8_t - indicates whether sensor id is valid
 * @retval 0 valid id
 * @retval >0 invalid id
 */
uint8_t checkChipId();

/**
 * \ingroup bmp388SelfTest
 * \page bmp388_api_checkCalibrationCoeffs checkCalibrationCoeffs
 * \code
 * uint8_t checkCalibrationCoeffs();
 * \endcode
 * @brief Function check if calibration coefficients stored in device's memory are valid.
 *
 * @return uint8_t - indicates whether calibration coefficients are valid
 * @retval 0 valid coefficients
 * @retval >0 invalid coefficients
 */
uint8_t checkCalibrationCoeffs();

/**
*  \ingroup bmp388SelfTest
 * \page bmp388_api_calculateCrc calculateCrc
 * \code
 * int8_t calculateCrc(uint8_t seed, uint8_t data);
 * \endcode
 * @brief Function calculate cyclic redundancy code for coefficient values
 * 
 * @param [in] seed CRC seed
 * @param [in] data Data to validate
 * @return int8_t - calculated CRC
 */
int8_t calculateCrc(uint8_t seed, uint8_t data);

/**
 * \ingroup bmp388SelfTest
 * \page bmp388_api_saveCalibrationCoeffs saveCalibrationCoeffs
 * \code
 * void saveCalibrationCoeffs();
 * \endcode
 * @brief Function read, parse and save calibration coefficients to bmp_calibration_data structure. 
 *
 */
void saveCalibrationCoeffs();

/**
 * \ingroup bmp388
 * \defgroup bmp388dataMeas Measurement
 * @brief Pressure and temperature measurement functions
 */

/**
 * \ingroup bmp388dataMeas
 * \page bmp388_api_readTemperature readTemperature
 * \code
 * float readTemperature();
 * \endcode 
 * @brief Function read and parse measured temperature in Celsius degrees
 * 
 * @param [in] raw_temperature Raw temperature from device registers
 * @return float - measured temperature
 */
float readTemperature(uint32_t raw_temperature);

/**
 * \ingroup bmp388dataMeas
 * \page bmp388_api_compensateTemperature compensateTemperature
 * \code
 * int64_t compensateTemperature(uint32_t temperature);
 * \endcode 
 * @brief Function calculate compensated temperature
 * 
 * @param [in] temperature Uncompensated raw temperature
 * @return int64_t - compensated temperature
 */
int64_t compensateTemperature(uint32_t temperature);

/**
 * \ingroup bmp388dataMeas
 * \page bmp388_api_readPressure readPressure
 * \code
 * void readPressure(float *pressure, float *temperature);
 * \endcode 
 * @brief Function read and parse measured pressure. Since temperature is neccessary to 
 * compensate raw readed pressure, function also read and parse temperature.
 * 
 * @param pressure Pointer to variable storing pressure measurement
 * @param temperature Pointer to variable storing temperature measurement
 */
void readPressure(float *pressure, float *temperature);

/**
 * \ingroup bmp388dataMeas
 * \page bmp388_api_compensatePressure compensatePressure
 * \code
 * uint64_t compensatePressure(uint32_t pressure);
 * \endcode 
 * @brief Function calculate compensated pressure
 * 
 * @param [in] pressure Uncompensated raw pressure
 * @return uint64_t - compensated pressure
 */
uint64_t compensatePressure(uint32_t pressure);

/**
 * \ingroup bmp388
 * \defgroup bmp388set Settings
 * @brief Sensor settings functions
 */

/**
 * \ingroup bmp388set
 * \page bmp388_api_setFilterCoefficient setFilterCoefficient
 * \code
 * void setFilterCoefficient(uint8_t coeff);
 * \endcode
 * @brief Set pressure IIR filter coefficient
 * 
 * @param coeff Selected filter coefficient
 */
void setFilterCoefficient(uint8_t coeff);

/**
 * \ingroup bmp388set
 * \page bmp388_api_setOutputDataRate setOutputDataRate
 * \code
 * void setOutputDataRate(uint8_t data_rate);
 * \endcode
 * @brief Set sensor output data rate
 * 
 * @param data_rate Selectoed output data rate
 */
void setOutputDataRate(uint8_t data_rate);

/**
 * \ingroup bmp388set
 * \page bmp388_api_setOversampling setOversampling
 * \code
 * void setOversampling(uint8_t press_osr, uint8_t temp_osr);
 * \endcode
 * @brief Set selected pressure and temperature oversampling
 * 
 * @param press_osr Selected pressure oversampling value
 * @param temp_osr Selected temperature oversampling value
 */
void setOversampling(uint8_t press_osr, uint8_t temp_osr);

/**
 * \ingroup bmp388set
 * \page bmp388_api_setPowerMode setPowerMode
 * \code
 * void setPowerMode(uint8_t press_en, uint8_t temp_en, uint8_t power_mode);
 * \endcode
 * @brief Set sensor power mode
 * 
 * @param press_en Pressure measurement setting
 * @param temp_en Temperature measurement setting
 * @param power_mode Selected power mode
 */
void setPowerMode(uint8_t press_en, uint8_t temp_en, uint8_t power_mode);

#endif