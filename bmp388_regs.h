/**
 * @file bmp388_regs.h
 * @author Michał Słomiany (mslomiany@outlook.com)
 * @brief Registers definitios for BMP388
 * @version 0.1
 * @date 2021-08-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef BMP388_REGS_H
#define BMP388_REGS_H

/*
 *  Device address and id
 */

#define BMP388_I2C_ADDR 0x76 << 1
#define BMP_CHIP_ID 0x50

/*
 *  Status defines
 */

#define STATUS_OK 0x00
#define ERR_WRONG_CHIP_ID 0x02
#define ERR_INVALID_COEFFICIENTS 0x03

/*
 *  Registers addresses
 */

#define REG_CHIP_ID 0x00       // Chip ID
#define REG_ERR_REG 0x02       // Errors register
#define REG_STATUS 0x03        // Conversion status flags
#define REG_EVENT 0x10         // After powerup or reset
#define REG_INT_STATUS 0x00    // Interrupt settings
#define REG_FIFO_LENGTH 0x12   // FIFO
#define REG_FIFO_DATA 0x14     // FIFO
#define REG_FIFO_WTM 0x15      // FIFO
#define REG_FIFO_CONFIG_1 0x17 // FIFO
#define REG_FIFO_CONFIG_2 0x18 // FIFO
#define REG_INT_CTRL 0x19      // Interrupt pin configuration
#define REG_IF_CONF 0x1A       // Interface configuration
#define REG_PWR_CNTRL 0x1B     // Power control settings
#define REG_OSR 0x1C           // Oversampling settings
#define REG_ODR 0x1D           // Output data rate
#define REG_IIR_CONFIG 0x1F    // Filter coefficient setting
#define REG_CMD 0x7E           // Command register (soft reset)

#define REG_DATA 0x04        // Data register
#define REG_PRESSURE 0x04    // Pressure data register
#define REG_TEMPERATURE 0x07 // Temperature data register

// Trimming parameters
#define REG_CRC 0x30 // CRC sum
#define REG_T1 0x31
#define REG_T2 0x33
#define REG_T3 0x35
#define REG_P1 0x36
#define REG_P2 0x38
#define REG_P3 0x3A
#define REG_P4 0x3B
#define REG_P5 0x3C
#define REG_P6 0x3E
#define REG_P7 0x40
#define REG_P8 0x41
#define REG_P9 0x42
#define REG_P10 0x44
#define REG_P11 0x45

/*
 *  Command macros
 */

// PWR_CNTRL - power control
#define PRESSURE_ON 0x01
#define PRESSURE_OFF 0x00
#define TEMPERATURE_ON 0x01
#define TEMPERATURE_OFF 0x00
#define MODE_SLEEP 0x00
#define MODE_FORCED 0X01
#define MODE_NORMAL 0x03

// OSR - oversampling settings
#define OVERSAMPLING_1 0x00
#define OVERSAMPLING_2 0x01
#define OVERSAMPLING_4 0x02
#define OVERSAMPLING_8 0x03
#define OVERSAMPLING_16 0x04
#define OVERSAMPLING_32 0x05

// ODR - output data rate
#define DATA_RATE_200 0x00
#define DATA_RATE_100 0x01
#define DATA_RATE_50 0x02
#define DATA_RATE_25 0x03
#define DATA_RATE_12_5 0x04
#define DATA_RATE_6_25 0x05

// IIR - filter coefficient
#define FILTER_0 0x00
#define FILTER_1 0x01
#define FILTER_3 0x02
#define FILTER_7 0x03
#define FILTER_15 0x04
#define FILTER_31 0x05
#define FILTER_63 0x06
#define FILTER_127 0x07

/**
 * @brief Structure with parsed calibration data read from registers
 * 
 */
typedef struct bmp_calibration_data
{
    /*! Temperature coefficients */
    uint16_t t1;
    uint16_t t2;
    int8_t t3;
    /*! Pressure coefficients */
    int16_t p1;
    int16_t p2;
    int8_t p3;
    int8_t p4;
    uint16_t p5;
    uint16_t p6;
    int8_t p7;
    int8_t p8;
    int16_t p9;
    int8_t p10;
    int8_t p11;
    /*! Calibrated temperature for pressure calculations */
    int64_t t_comp;
} bmp_calibration_data;

/**
 * @brief Structure with sensor actual settings
 * 
 */
typedef struct bmp_status
{
    /*! Temperature oversampling */
    uint8_t temperature_oversampling;
    /*! Pressure oversampling */
    uint8_t pressure_oversampling;
    /*! Pressure measurement enable */
    uint8_t pressure_enable;
    /*! Temperature measurement enable */
    uint8_t temperature_enable;
    /*! Sensor power mode */
    uint8_t power_mode;
    /*! Selected filter coefficient */
    uint8_t filter_coefficient;
    /*! Selected data rate */
    uint8_t data_rate;
    /*! Pointer to selected I2C handler */
    I2C_HandleTypeDef *i2c_h;
} bmp_status;

#endif
