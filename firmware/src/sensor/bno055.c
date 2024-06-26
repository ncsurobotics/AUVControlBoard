/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#include <sensor/bno055.h>
#include <util/angles.h>
#include <hardware/i2c.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <calibration.h>
#include <debug.h>
#include <cmdctrl.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// BNO055 Info Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Sensor info
#define BNO055_ADDR                         (0x28)
#define BNO055_ID                           (0xA0)

// BNO055 Registers
/* Page id register definition*/
#define BNO055_PAGE_ID_ADDR                 (0X07)

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                 (0x00)
#define BNO055_ACCEL_REV_ID_ADDR            (0x01)
#define BNO055_MAG_REV_ID_ADDR              (0x02)
#define BNO055_GYRO_REV_ID_ADDR             (0x03)
#define BNO055_SW_REV_ID_LSB_ADDR           (0x04)
#define BNO055_SW_REV_ID_MSB_ADDR           (0x05)
#define BNO055_BL_REV_ID_ADDR               (0X06)

/* Accel data register*/
#define BNO055_ACCEL_DATA_X_LSB_ADDR        (0X08)
#define BNO055_ACCEL_DATA_X_MSB_ADDR        (0X09)
#define BNO055_ACCEL_DATA_Y_LSB_ADDR        (0X0A)
#define BNO055_ACCEL_DATA_Y_MSB_ADDR        (0X0B)
#define BNO055_ACCEL_DATA_Z_LSB_ADDR        (0X0C)
#define BNO055_ACCEL_DATA_Z_MSB_ADDR        (0X0D)

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR          (0X0E)
#define BNO055_MAG_DATA_X_MSB_ADDR          (0X0F)
#define BNO055_MAG_DATA_Y_LSB_ADDR          (0X10)
#define BNO055_MAG_DATA_Y_MSB_ADDR          (0X11)
#define BNO055_MAG_DATA_Z_LSB_ADDR          (0X12)
#define BNO055_MAG_DATA_Z_MSB_ADDR          (0X13)

/*Gyro data registers*/
#define BNO055_GYRO_DATA_X_LSB_ADDR         (0X14)
#define BNO055_GYRO_DATA_X_MSB_ADDR         (0X15)
#define BNO055_GYRO_DATA_Y_LSB_ADDR         (0X16)
#define BNO055_GYRO_DATA_Y_MSB_ADDR         (0X17)
#define BNO055_GYRO_DATA_Z_LSB_ADDR         (0X18)
#define BNO055_GYRO_DATA_Z_MSB_ADDR         (0X19)

/*Euler data registers*/
#define BNO055_EULER_H_LSB_ADDR             (0X1A)
#define BNO055_EULER_H_MSB_ADDR             (0X1B)

#define BNO055_EULER_R_LSB_ADDR             (0X1C)
#define BNO055_EULER_R_MSB_ADDR             (0X1D)

#define BNO055_EULER_P_LSB_ADDR             (0X1E)
#define BNO055_EULER_P_MSB_ADDR             (0X1F)

/*Quaternion data registers*/
#define BNO055_QUATERNION_DATA_W_LSB_ADDR   (0X20)
#define BNO055_QUATERNION_DATA_W_MSB_ADDR   (0X21)
#define BNO055_QUATERNION_DATA_X_LSB_ADDR   (0X22)
#define BNO055_QUATERNION_DATA_X_MSB_ADDR   (0X23)
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR   (0X24)
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR   (0X25)
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR   (0X26)
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR   (0X27)

/* Linear acceleration data registers*/
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR (0X28)
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR (0X29)
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR (0X2A)
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR (0X2B)
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR (0X2C)
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR (0X2D)

/*Gravity data registers*/
#define BNO055_GRAVITY_DATA_X_LSB_ADDR      (0X2E)
#define BNO055_GRAVITY_DATA_X_MSB_ADDR      (0X2F)
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR      (0X30)
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR      (0X31)
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR      (0X32)
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR      (0X33)

/* Temperature data register*/
#define BNO055_TEMP_ADDR                    (0X34)

/* Status registers*/
#define BNO055_CALIB_STAT_ADDR              (0X35)
#define BNO055_SELFTEST_RESULT_ADDR         (0X36)
#define BNO055_INTR_STAT_ADDR               (0X37)
#define BNO055_SYS_CLK_STAT_ADDR            (0X38)
#define BNO055_SYS_STAT_ADDR                (0X39)
#define BNO055_SYS_ERR_ADDR                 (0X3A)

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR                (0X3B)
#define BNO055_DATA_SELECT_ADDR             (0X3C)

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR                (0X3D)
#define BNO055_PWR_MODE_ADDR                (0X3E)

#define BNO055_SYS_TRIGGER_ADDR             (0X3F)
#define BNO055_TEMP_SOURCE_ADDR             (0X40)

/* Axis remap registers*/
#define BNO055_AXIS_MAP_CONFIG_ADDR         (0X41)
#define BNO055_AXIS_MAP_SIGN_ADDR           (0X42)

/* SIC registers*/
#define BNO055_SIC_MATRIX_0_LSB_ADDR        (0X43)
#define BNO055_SIC_MATRIX_0_MSB_ADDR        (0X44)
#define BNO055_SIC_MATRIX_1_LSB_ADDR        (0X45)
#define BNO055_SIC_MATRIX_1_MSB_ADDR        (0X46)
#define BNO055_SIC_MATRIX_2_LSB_ADDR        (0X47)
#define BNO055_SIC_MATRIX_2_MSB_ADDR        (0X48)
#define BNO055_SIC_MATRIX_3_LSB_ADDR        (0X49)
#define BNO055_SIC_MATRIX_3_MSB_ADDR        (0X4A)
#define BNO055_SIC_MATRIX_4_LSB_ADDR        (0X4B)
#define BNO055_SIC_MATRIX_4_MSB_ADDR        (0X4C)
#define BNO055_SIC_MATRIX_5_LSB_ADDR        (0X4D)
#define BNO055_SIC_MATRIX_5_MSB_ADDR        (0X4E)
#define BNO055_SIC_MATRIX_6_LSB_ADDR        (0X4F)
#define BNO055_SIC_MATRIX_6_MSB_ADDR        (0X50)
#define BNO055_SIC_MATRIX_7_LSB_ADDR        (0X51)
#define BNO055_SIC_MATRIX_7_MSB_ADDR        (0X52)
#define BNO055_SIC_MATRIX_8_LSB_ADDR        (0X53)
#define BNO055_SIC_MATRIX_8_MSB_ADDR        (0X54)

/* Accelerometer Offset registers*/
#define BNO055_ACCEL_OFFSET_X_LSB_ADDR      (0X55)
#define BNO055_ACCEL_OFFSET_X_MSB_ADDR      (0X56)
#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR      (0X57)
#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR      (0X58)
#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR      (0X59)
#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR      (0X5A)

/* Magnetometer Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB_ADDR        (0X5B)
#define BNO055_MAG_OFFSET_X_MSB_ADDR        (0X5C)
#define BNO055_MAG_OFFSET_Y_LSB_ADDR        (0X5D)
#define BNO055_MAG_OFFSET_Y_MSB_ADDR        (0X5E)
#define BNO055_MAG_OFFSET_Z_LSB_ADDR        (0X5F)
#define BNO055_MAG_OFFSET_Z_MSB_ADDR        (0X60)

/* Gyroscope Offset registers*/
#define BNO055_GYRO_OFFSET_X_LSB_ADDR       (0X61)
#define BNO055_GYRO_OFFSET_X_MSB_ADDR       (0X62)
#define BNO055_GYRO_OFFSET_Y_LSB_ADDR       (0X63)
#define BNO055_GYRO_OFFSET_Y_MSB_ADDR       (0X64)
#define BNO055_GYRO_OFFSET_Z_LSB_ADDR       (0X65)
#define BNO055_GYRO_OFFSET_Z_MSB_ADDR       (0X66)

/* Radius registers*/
#define BNO055_ACCEL_RADIUS_LSB_ADDR        (0X67)
#define BNO055_ACCEL_RADIUS_MSB_ADDR        (0X68)
#define BNO055_MAG_RADIUS_LSB_ADDR          (0X69)
#define BNO055_MAG_RADIUS_MSB_ADDR          (0X6A)

/* PAGE0 REGISTERS DEFINITION END*/
/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define BNO055_ACCEL_CONFIG_ADDR            (0X08)
#define BNO055_MAG_CONFIG_ADDR              (0X09)
#define BNO055_GYRO_CONFIG_ADDR             (0X0A)
#define BNO055_GYRO_MODE_CONFIG_ADDR        (0X0B)
#define BNO055_ACCEL_SLEEP_CONFIG_ADDR      (0X0C)
#define BNO055_GYRO_SLEEP_CONFIG_ADDR       (0X0D)
#define BNO055_MAG_SLEEP_CONFIG_ADDR        (0x0E)

/* Interrupt registers*/
#define BNO055_INT_MASK_ADDR                (0X0F)
#define BNO055_INT_ADDR                     (0X10)
#define BNO055_ACCEL_ANY_MOTION_THRES_ADDR  (0X11)
#define BNO055_ACCEL_INTR_SETTINGS_ADDR     (0X12)
#define BNO055_ACCEL_HIGH_G_DURN_ADDR       (0X13)
#define BNO055_ACCEL_HIGH_G_THRES_ADDR      (0X14)
#define BNO055_ACCEL_NO_MOTION_THRES_ADDR   (0X15)
#define BNO055_ACCEL_NO_MOTION_SET_ADDR     (0X16)
#define BNO055_GYRO_INTR_SETING_ADDR        (0X17)
#define BNO055_GYRO_HIGHRATE_X_SET_ADDR     (0X18)
#define BNO055_GYRO_DURN_X_ADDR             (0X19)
#define BNO055_GYRO_HIGHRATE_Y_SET_ADDR     (0X1A)
#define BNO055_GYRO_DURN_Y_ADDR             (0X1B)
#define BNO055_GYRO_HIGHRATE_Z_SET_ADDR     (0X1C)
#define BNO055_GYRO_DURN_Z_ADDR             (0X1D)
#define BNO055_GYRO_ANY_MOTION_THRES_ADDR   (0X1E)
#define BNO055_GYRO_ANY_MOTION_SET_ADDR     (0X1F)

// Operating modes
#define OPMODE_CFG              0x00
#define OPMODE_IMU              0X08

// Axis remap register values
#define REMAP_P0                0x21
#define REMAP_P1                0x24
#define REMAP_P2                0x24
#define REMAP_P3                0x21
#define REMAP_P4                0x24
#define REMAP_P5                0x21
#define REMAP_P6                0x21
#define REMAP_P7                0x24

// Axis sign register values
#define SIGN_P0                 0x04
#define SIGN_P1                 0x00
#define SIGN_P2                 0x06
#define SIGN_P3                 0x02
#define SIGN_P4                 0x03
#define SIGN_P5                 0x00
#define SIGN_P6                 0x07
#define SIGN_P7                 0x05

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// BNO055 Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define WRITE_BUF_SIZE          16
#define READ_BUF_SIZE           16

// Need to ensure only one thread perfoming I2C comms with this sensor at a time
static SemaphoreHandle_t trans_mutex;

static i2c_trans trans;
static uint8_t write_buf[WRITE_BUF_SIZE];
static uint8_t read_buf[READ_BUF_SIZE];

// Last applied axis configuration
static uint8_t remap = REMAP_P1;
static uint8_t sign = SIGN_P1;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// BNO055 Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define bno055_perform(x)           i2c_perform_retries((x), 20, 5)

void bno055_init(void){
    trans_mutex = xSemaphoreCreateMutex();
    trans.address = BNO055_ADDR;
    trans.write_buf = write_buf;
    trans.read_buf = read_buf;
}



static inline bool bno055_configure_internal(void){
    // Read chip ID register to make sure right device is on bus
    trans.write_buf[0] = BNO055_CHIP_ID_ADDR;
    trans.write_count = 1;
    trans.read_count = 1;
    if(!bno055_perform(&trans)){
        return false;
    }
    if(trans.read_buf[0] != BNO055_ID)
        return false;

    // Put IMU in config mode
    trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
    trans.write_buf[1] = OPMODE_CFG;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(25));

    // Reset IMU
    trans.write_buf[0] = BNO055_SYS_TRIGGER_ADDR;
    trans.write_buf[1] = 0x20;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    
    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(500));
    unsigned int attempts = 0;
    do{
        trans.write_buf[0] = BNO055_CHIP_ID_ADDR;
        trans.write_count = 1;
        trans.read_count = 1;
        if(!bno055_perform(&trans))
            vTaskDelay(pdMS_TO_TICKS(20));
        attempts++;
        if(attempts > 50)
            return false;
    }while(trans.read_buf[0] != BNO055_ID);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Set to normal power mode
    trans.write_buf[0] = BNO055_PWR_MODE_ADDR;
    trans.write_buf[1] = 0x00;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Zero page ID register
    trans.write_buf[0] = BNO055_PAGE_ID_ADDR;
    trans.write_buf[1] = 0x00;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }

    // TODO: Set units
    // Windows fusion data output mode
    // Acceleration: m/s^2
    // Linear Accel & Grav Vec: m/s^2
    // Gyro: dps
    // Euler: degrees
    // Temperature: Celcius
    const uint8_t unitsel_val = 0b00000000;
    trans.write_buf[0] = BNO055_UNIT_SEL_ADDR;
    trans.write_buf[1] = unitsel_val;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }

    // Apply last configured axis remap
    trans.write_buf[0] = BNO055_AXIS_MAP_CONFIG_ADDR;
    trans.write_buf[1] = remap;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }

    // Apply last configured axis sign
    trans.write_buf[0] = BNO055_AXIS_MAP_SIGN_ADDR;
    trans.write_buf[1] = sign;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }

    // Clear sys trigger register
    trans.write_buf[0] = BNO055_SYS_TRIGGER_ADDR;
    trans.write_buf[1] = 0x00;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Apply stored calibrations (if any are stored)
    if(calibration_bno055.valid){
        // ACC_OFFSET_X
        trans.write_buf[0] = BNO055_ACCEL_OFFSET_X_LSB_ADDR;
        trans.write_buf[1] = calibration_bno055.accel_offset_x & 0xFF;
        trans.write_buf[2] = (calibration_bno055.accel_offset_x & 0xFF00) >> 8;
        trans.write_count = 3;
        trans.read_count = 0;
        if(!bno055_perform(&trans))
            return false;
        vTaskDelay(pdMS_TO_TICKS(10));

        // ACC_OFFSET_Y
        trans.write_buf[0] = BNO055_ACCEL_OFFSET_Y_LSB_ADDR;
        trans.write_buf[1] = calibration_bno055.accel_offset_y & 0xFF;
        trans.write_buf[2] = (calibration_bno055.accel_offset_y & 0xFF00) >> 8;
        trans.write_count = 3;
        trans.read_count = 0;
        if(!bno055_perform(&trans))
            return false;
        vTaskDelay(pdMS_TO_TICKS(10));

        // ACC_OFFSET_Z
        trans.write_buf[0] = BNO055_ACCEL_OFFSET_Z_LSB_ADDR;
        trans.write_buf[1] = calibration_bno055.accel_offset_z & 0xFF;
        trans.write_buf[2] = (calibration_bno055.accel_offset_z & 0xFF00) >> 8;
        trans.write_count = 3;
        trans.read_count = 0;
        if(!bno055_perform(&trans))
            return false;
        vTaskDelay(pdMS_TO_TICKS(10));

        // ACC_RADIUS
        trans.write_buf[0] = BNO055_ACCEL_RADIUS_LSB_ADDR;
        trans.write_buf[1] = calibration_bno055.accel_radius & 0xFF;
        trans.write_buf[2] = (calibration_bno055.accel_radius & 0xFF00) >> 8;
        trans.write_count = 3;
        trans.read_count = 0;
        if(!bno055_perform(&trans))
            return false;
        vTaskDelay(pdMS_TO_TICKS(10));

        // GYR_OFFSET_X
        trans.write_buf[0] = BNO055_GYRO_OFFSET_X_LSB_ADDR;
        trans.write_buf[1] = calibration_bno055.gyro_offset_x & 0xFF;
        trans.write_buf[2] = (calibration_bno055.gyro_offset_x & 0xFF00) >> 8;
        trans.write_count = 3;
        trans.read_count = 0;
        if(!bno055_perform(&trans))
            return false;
        vTaskDelay(pdMS_TO_TICKS(10));

        // GYR_OFFSET_Y
        trans.write_buf[0] = BNO055_GYRO_OFFSET_Y_LSB_ADDR;
        trans.write_buf[1] = calibration_bno055.gyro_offset_y & 0xFF;
        trans.write_buf[2] = (calibration_bno055.gyro_offset_y & 0xFF00) >> 8;
        trans.write_count = 3;
        trans.read_count = 0;
        if(!bno055_perform(&trans))
            return false;
        vTaskDelay(pdMS_TO_TICKS(10));

        // GYR_OFFSET_Z
        trans.write_buf[0] = BNO055_GYRO_OFFSET_Z_LSB_ADDR;
        trans.write_buf[1] = calibration_bno055.gyro_offset_z & 0xFF;
        trans.write_buf[2] = (calibration_bno055.gyro_offset_z & 0xFF00) >> 8;
        trans.write_count = 3;
        trans.read_count = 0;
        if(!bno055_perform(&trans))
            return false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Set to IMU (fusion) operating mode
    trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
    trans.write_buf[1] = OPMODE_IMU;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(600));

    // Done configuring IMU
    return true;   
}

bool bno055_configure(void){
    // Take I2C bus at beginning of each function communicating with sensor.
    // This also ensures accesses to trans are thread safe and prevents unexpected
    // interleaving of messages to the same sensor (if multiple threads call
    // functions of the same sensor).
    if(xSemaphoreTake(trans_mutex, portMAX_DELAY) == pdFALSE){
        return false;
    }
    bool ret = bno055_configure_internal();
    xSemaphoreGive(trans_mutex);
    return ret;
}



static inline bool bno055_set_axis_internal(uint8_t mode){
    // Set remap and sign registers
    switch(mode){
    case BNO055_AXIS_P0:
        remap = REMAP_P0;
        sign = SIGN_P0;
        break;
    case BNO055_AXIS_P1:
        remap = REMAP_P1;
        sign = SIGN_P1;
        break;
    case BNO055_AXIS_P2:
        remap = REMAP_P2;
        sign = SIGN_P2;
        break;
    case BNO055_AXIS_P3:
        remap = REMAP_P3;
        sign = SIGN_P3;
        break;
    case BNO055_AXIS_P4:
        remap = REMAP_P4;
        sign = SIGN_P4;
        break;
    case BNO055_AXIS_P5:
        remap = REMAP_P5;
        sign = SIGN_P5;
        break;
    case BNO055_AXIS_P6:
        remap = REMAP_P6;
        sign = SIGN_P6;
        break;
    case BNO055_AXIS_P7:
        remap = REMAP_P7;
        sign = SIGN_P7;
        break;
    }

    // Accumulated angles are invalid b/c definition of axes has changed
    imu_reset_data();

    // Only actually apply this now if the BNO055 is the active IMU
    if(imu_get_sensor() != IMU_BNO055)
        return false;

    // Put in CONFIG mode
    trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
    trans.write_buf[1] = OPMODE_CFG;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(25));

    // Set remap
    trans.write_buf[0] = BNO055_AXIS_MAP_CONFIG_ADDR;
    trans.write_buf[1] = remap;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set sign
    trans.write_buf[0] = BNO055_AXIS_MAP_SIGN_ADDR;
    trans.write_buf[1] = sign;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Restore to IMU mode
    trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
    trans.write_buf[1] = OPMODE_IMU;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(600));

    return true;
}

bool bno055_set_axis(uint8_t mode){
    // Take I2C bus at beginning of each function communicating with sensor.
    // This also ensures accesses to trans are thread safe and prevents unexpected
    // interleaving of messages to the same sensor (if multiple threads call
    // functions of the same sensor).
    if(xSemaphoreTake(trans_mutex, portMAX_DELAY) == pdFALSE){
        return false;
    }
    bool ret = bno055_set_axis_internal(mode);
    xSemaphoreGive(trans_mutex);
    return ret;
}



static inline bool bno055_read_internal(imu_data_t *data){
    int16_t tmp16;

    // Read Orientation Quaternion
    // Ignore failure if cmdctrl_sim_hijacked since real data wouldn't be used anyway
    trans.write_buf[0] = BNO055_QUATERNION_DATA_W_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 8;
    if(!bno055_perform(&trans) && !cmdctrl_sim_hijacked){
        return false;
    }
    tmp16 = (((uint16_t)trans.read_buf[1]) << 8) | ((uint16_t)trans.read_buf[0]);
    data->quat.w = tmp16 / 16384.0f;
    tmp16 = (((uint16_t)trans.read_buf[3]) << 8) | ((uint16_t)trans.read_buf[2]);
    data->quat.x = tmp16 / 16384.0f;
    tmp16 = (((uint16_t)trans.read_buf[5]) << 8) | ((uint16_t)trans.read_buf[4]);
    data->quat.y = tmp16 / 16384.0f;
    tmp16 = (((uint16_t)trans.read_buf[7]) << 8) | ((uint16_t)trans.read_buf[6]);
    data->quat.z = tmp16 / 16384.0f;

    // Raw gyroscope data
    trans.write_buf[0] = BNO055_GYRO_DATA_X_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 6;
    if(!bno055_perform(&trans)){
        return false;
    }
    tmp16 = trans.read_buf[0] | (trans.read_buf[1] << 8);
    data->raw_gyro.x = tmp16 / 16.0f;
    tmp16 = trans.read_buf[2] | (trans.read_buf[3] << 8);
    data->raw_gyro.y = tmp16 / 16.0f;
    tmp16 = trans.read_buf[4] | (trans.read_buf[5] << 8);
    data->raw_gyro.z = tmp16 / 16.0f;

    // Raw accelerometer data
    trans.write_buf[0] = BNO055_ACCEL_DATA_X_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 6;
    if(!bno055_perform(&trans)){
        return false;
    }
    tmp16 = trans.read_buf[0] | (trans.read_buf[1] << 8);
    data->raw_accel.x = tmp16 / 100.0f;
    tmp16 = trans.read_buf[2] | (trans.read_buf[3] << 8);
    data->raw_accel.y = tmp16 / 100.0f;
    tmp16 = trans.read_buf[4] | (trans.read_buf[5] << 8);
    data->raw_accel.z = tmp16 / 100.0f;

    // Success
    return true;
}

bool bno055_read(imu_data_t *data){
    // Take I2C bus at beginning of each function communicating with sensor.
    // This also ensures accesses to trans are thread safe and prevents unexpected
    // interleaving of messages to the same sensor (if multiple threads call
    // functions of the same sensor).
    if(xSemaphoreTake(trans_mutex, portMAX_DELAY) == pdFALSE){
        return false;
    }
    bool ret = bno055_read_internal(data);
    xSemaphoreGive(trans_mutex);
    return ret;
}



static inline bool bno055_read_calibration_status_internal(uint8_t *status){
    // Read CALIB_STAT
    trans.write_buf[0] = BNO055_CALIB_STAT_ADDR;
    trans.write_count = 1;
    trans.read_count = 1;
    if(!bno055_perform(&trans)){
        return false;
    }
    *status = trans.read_buf[0];
    vTaskDelay(pdMS_TO_TICKS(10));
    return true;
}

bool bno055_read_calibration_status(uint8_t *status){
    // Take I2C bus at beginning of each function communicating with sensor.
    // This also ensures accesses to trans are thread safe and prevents unexpected
    // interleaving of messages to the same sensor (if multiple threads call
    // functions of the same sensor).
    if(xSemaphoreTake(trans_mutex, portMAX_DELAY) == pdFALSE){
        return false;
    }
    bool ret = bno055_read_calibration_status_internal(status);
    xSemaphoreGive(trans_mutex);
    return ret;
}



static inline bool bno055_read_calibration_internal(int16_t *acc_offset_x, int16_t *acc_offset_y, int16_t *acc_offset_z, 
        int16_t *acc_radius, int16_t *gyr_offset_x, int16_t *gyr_offset_y, int16_t *gyr_offset_z){
    //  Note: Can only read calibration status if the sensor has been fully calibrated (3 in CALIB_STAT)
    //        and the sensor is put into CONFIG mode AFTER!
    
    // Put in CONFIG mode
    trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
    trans.write_buf[1] = OPMODE_CFG;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(25));

    // Read ACC_OFFSET_X
    trans.write_buf[0] = BNO055_ACCEL_OFFSET_X_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 2;
    if(!bno055_perform(&trans)){
        return false;
    }
    *acc_offset_x = trans.read_buf[0] | (trans.read_buf[1] << 8);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read ACC_OFFSET_Y
    trans.write_buf[0] = BNO055_ACCEL_OFFSET_Y_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 2;
    if(!bno055_perform(&trans)){
        return false;
    }
    *acc_offset_y = trans.read_buf[0] | (trans.read_buf[1] << 8);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read ACC_OFFSET_Z
    trans.write_buf[0] = BNO055_ACCEL_OFFSET_Z_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 2;
    if(!bno055_perform(&trans)){
        return false;
    }
    *acc_offset_z = trans.read_buf[0] | (trans.read_buf[1] << 8);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read ACC_RADIUS
    trans.write_buf[0] = BNO055_ACCEL_RADIUS_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 2;
    if(!bno055_perform(&trans)){
        return false;
    }
    *acc_radius = trans.read_buf[0] | (trans.read_buf[1] << 8);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read GYR_OFFSET_X
    trans.write_buf[0] = BNO055_GYRO_OFFSET_X_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 2;
    if(!bno055_perform(&trans)){
        return false;
    }
    *gyr_offset_x = trans.read_buf[0] | (trans.read_buf[1] << 8);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read GYR_OFFSET_Y
    trans.write_buf[0] = BNO055_GYRO_OFFSET_Y_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 2;
    if(!bno055_perform(&trans)){
        return false;
    }
    *gyr_offset_y = trans.read_buf[0] | (trans.read_buf[1] << 8);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read GYR_OFFSET_Z
    trans.write_buf[0] = BNO055_GYRO_OFFSET_Z_LSB_ADDR;
    trans.write_count = 1;
    trans.read_count = 2;
    if(!bno055_perform(&trans)){
        return false;
    }
    *gyr_offset_z = trans.read_buf[0] | (trans.read_buf[1] << 8);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Restore to IMU mode
    trans.write_buf[0] = BNO055_OPR_MODE_ADDR;
    trans.write_buf[1] = OPMODE_IMU;
    trans.write_count = 2;
    trans.read_count = 0;
    if(!bno055_perform(&trans)){
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(35));

    return true;
}

bool bno055_read_calibration(int16_t *acc_offset_x, int16_t *acc_offset_y, int16_t *acc_offset_z, 
        int16_t *acc_radius, int16_t *gyr_offset_x, int16_t *gyr_offset_y, int16_t *gyr_offset_z){
    // Take I2C bus at beginning of each function communicating with sensor.
    // This also ensures accesses to trans are thread safe and prevents unexpected
    // interleaving of messages to the same sensor (if multiple threads call
    // functions of the same sensor).
    if(xSemaphoreTake(trans_mutex, portMAX_DELAY) == pdFALSE){
        return false;
    }
    bool ret = bno055_read_calibration_internal(acc_offset_x, acc_offset_y, acc_offset_z, acc_radius, gyr_offset_x, 
            gyr_offset_y, gyr_offset_z);
    xSemaphoreGive(trans_mutex);
    return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
