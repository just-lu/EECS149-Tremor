// BNO055 driver
//
// Read from BNO055 3-axis accelerometer/gyro/magnetometer over I2C

#pragma once

#include "app_error.h"
#include "nrf_twi_mngr.h"

// Types

typedef struct {
	float x_axis;
	float y_axis;
	float z_axis;
} bno055_measurement_t;


// Function prototypes

// Initialize and configure the MPU-9250
//
// i2c - pointer to already initialized and enabled twim instance
void bno055_init(const nrf_twi_mngr_t* i2c);

// Read all three axes on the accelerometer
//
// Return measurements as floating point values in g's
bno055_measurement_t bno055_read_accelerometer();

// Read all three axes on the gyro
//
// Return measurements as floating point values in degrees/second
bno055_measurement_t bno055_read_gyro();

// Read all three axes on the magnetometer
//
// Return measurements as floating point values in uT
// bno055_measurement_t bno055_read_magnetometer();

// Start integration on the gyro
//
// Return an NRF error code
//  - must be stopped before starting
ret_code_t bno055_start_gyro_integration();

// Stop integration on the gyro
void bno055_stop_gyro_integration();

// Read the value of the integrated gyro
//
// Note: this function also performs the integration and needs to be called
// periodically
//
// Return the integrated value as floating point in degrees
bno055_measurement_t bno055_read_gyro_integration();

// Definitions

typedef enum {
    BNO055_REG_ADDR_CHIP_ID	         = 0x00,
    BNO055_REG_ADDR_ACC_ID	         = 0x01,
    BNO055_REG_ADDR_MAG_ID	         = 0x02,
    BNO055_REG_ADDR_GYR_REV_ID	     = 0x03,
    BNO055_REG_ADDR_SW_REV_ID_LSB	 = 0x04,
    BNO055_REG_ADDR_SW_REV_ID_MSB	 = 0x05,
    BNO055_REG_ADDR_BL_REV_ID	     = 0X06,
    BNO055_REG_ADDR_PAGE_ID	         = 0X07,
    BNO055_REG_ADDR_ACC_DATA_X_LSB   = 0X08,
    BNO055_REG_ADDR_ACC_DATA_X_MSB   = 0X09,
    BNO055_REG_ADDR_ACC_DATA_Y_LSB   = 0X0A,
    BNO055_REG_ADDR_ACC_DATA_Y_MSB   = 0X0B,
    BNO055_REG_ADDR_ACC_DATA_Z_LSB   = 0X0C,
    BNO055_REG_ADDR_ACC_DATA_Z_MSB   = 0X0D,
    BNO055_REG_ADDR_MAG_DATA_X_LSB   = 0X0E,
    BNO055_REG_ADDR_MAG_DATA_X_MSB   = 0X0F,
    BNO055_REG_ADDR_MAG_DATA_Y_LSB   = 0X10,
    BNO055_REG_ADDR_MAG_DATA_Y_MSB   = 0X11,
    BNO055_REG_ADDR_MAG_DATA_Z_LSB   = 0X12,
    BNO055_REG_ADDR_MAG_DATA_Z_MSB   = 0X13,
    BNO055_REG_ADDR_GYR_DATA_X_LSB   = 0X14,
    BNO055_REG_ADDR_GYR_DATA_X_MSB   = 0X15,
    BNO055_REG_ADDR_GYR_DATA_Y_LSB   = 0X16,
    BNO055_REG_ADDR_GYR_DATA_Y_MSB   = 0X17,
    BNO055_REG_ADDR_GYR_DATA_Z_LSB   = 0X18,
    BNO055_REG_ADDR_GYR_DATA_Z_MSB   = 0X19,
    BNO055_REG_ADDR_EUL_HEADING_LSB  = 0X1A,
    BNO055_REG_ADDR_EUL_HEADING_MSB  = 0X1B,
    BNO055_REG_ADDR_EUL_ROLL_LSB	 = 0X1C,
    BNO055_REG_ADDR_EUL_ROLL_MSB	 = 0X1D,
    BNO055_REG_ADDR_EUL_PITCH_LSB	 = 0X1E,
    BNO055_REG_ADDR_EUL_PITCH_MSB	 = 0X1F,
    BNO055_REG_ADDR_QUA_DATA_W_LSB   = 0X20,
    BNO055_REG_ADDR_QUA_DATA_W_MSB   = 0X21,
    BNO055_REG_ADDR_QUA_DATA_X_LSB   = 0X22,
    BNO055_REG_ADDR_QUA_DATA_X_MSB   = 0X23,
    BNO055_REG_ADDR_QUA_DATA_Y_LSB   = 0X24,
    BNO055_REG_ADDR_QUA_DATA_Y_MSB   = 0X25,
    BNO055_REG_ADDR_QUA_DATA_Z_LSB   = 0X26,
    BNO055_REG_ADDR_QUA_DATA_Z_MSB   = 0X27,
    BNO055_REG_ADDR_LIA_DATA_X_LSB   = 0X28,
    BNO055_REG_ADDR_LIA_DATA_X_MSB   = 0X29,
    BNO055_REG_ADDR_LIA_DATA_Y_LSB   = 0X2A,
    BNO055_REG_ADDR_LIA_DATA_Y_MSB   = 0X2B,
    BNO055_REG_ADDR_LIA_DATA_Z_LSB   = 0X2C,
    BNO055_REG_ADDR_LIA_DATA_Z_MSB   = 0X2D,
    BNO055_REG_ADDR_GRV_DATA_X_LSB   = 0X2E,
    BNO055_REG_ADDR_GRV_DATA_X_MSB   = 0X2F,
    BNO055_REG_ADDR_GRV_DATA_Y_LSB   = 0X30,
    BNO055_REG_ADDR_GRV_DATA_Y_MSB   = 0X31,
    BNO055_REG_ADDR_GRV_DATA_Z_LSB   = 0X32,
    BNO055_REG_ADDR_GRV_DATA_Z_MSB   = 0X33,
    BNO055_REG_ADDR_TEMP             = 0X34,
    BNO055_REG_ADDR_CALIB_STAT	     = 0X35,
    BNO055_REG_ADDR_ST_RESULT	     = 0X36,
    BNO055_REG_ADDR_INT_STA	         = 0X37,
    BNO055_REG_ADDR_SYS_CLK_STATUS   = 0X38,
    BNO055_REG_ADDR_SYS_STATUS	     = 0X39,
    BNO055_REG_ADDR_SYS_ERR	         = 0X3A,
    BNO055_REG_ADDR_UNIT_SEL	     = 0X3B,
    /** 0x3C is reserved */
    BNO055_REG_ADDR_OPR_MODE	     = 0X3D,
    BNO055_REG_ADDR_PWR_MODE	     = 0X3E,
    BNO055_REG_ADDR_SYS_TRIGGER	     = 0X3F,
    BNO055_REG_ADDR_TEMP_SOURCE	     = 0X40,
    BNO055_REG_ADDR_AXIS_MAP_CONFIG  = 0X41,
    BNO055_REG_ADDR_AXIS_MAP_SIGN	 = 0X42,
    /** 0x43 - 0x54 are reserved */
    BNO055_REG_ADDR_ACC_OFFSET_X_LSB = 0X55,
    BNO055_REG_ADDR_ACC_OFFSET_X_MSB = 0X56,
    BNO055_REG_ADDR_ACC_OFFSET_Y_LSB = 0X57,
    BNO055_REG_ADDR_ACC_OFFSET_Y_MSB = 0X58,
    BNO055_REG_ADDR_ACC_OFFSET_Z_LSB = 0X59,
    BNO055_REG_ADDR_ACC_OFFSET_Z_MSB = 0X5A,
    BNO055_REG_ADDR_MAG_OFFSET_X_LSB = 0X5B,
    BNO055_REG_ADDR_MAG_OFFSET_X_MSB = 0X5C,
    BNO055_REG_ADDR_MAG_OFFSET_Y_LSB = 0X5D,
    BNO055_REG_ADDR_MAG_OFFSET_Y_MSB = 0X5E,
    BNO055_REG_ADDR_MAG_OFFSET_Z_LSB = 0X5F,
    BNO055_REG_ADDR_MAG_OFFSET_Z_MSB = 0X60,
    BNO055_REG_ADDR_GYR_OFFSET_X_LSB = 0X61,
    BNO055_REG_ADDR_GYR_OFFSET_X_MSB = 0X62,
    BNO055_REG_ADDR_GYR_OFFSET_Y_LSB = 0X63,
    BNO055_REG_ADDR_GYR_OFFSET_Y_MSB = 0X64,
    BNO055_REG_ADDR_GYR_OFFSET_Z_LSB = 0X65,
    BNO055_REG_ADDR_GYR_OFFSET_Z_MSB = 0X66,
    BNO055_REG_ADDR_ACC_RADIUS_LSB   = 0X67,
    BNO055_REG_ADDR_ACC_RADIUS_MSB   = 0X68,
    BNO055_REG_ADDR_MAG_RADIUS_LSB   = 0X69,
    BNO055_REG_ADDR_MAG_RADIUS_MSB   = 0X6A
} bno055_reg_addr_t;
