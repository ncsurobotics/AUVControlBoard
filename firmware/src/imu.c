/*
 * Copyright 2023 Marcus Behel
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

#include <imu.h>
#include <sensor/bno055.h>
#include <cmdctrl.h>
#include <FreeRTOS.h>
#include <semphr.h>


static uint8_t imu_which;
static imu_data_t imu_data;

static imu_data_t new_data;
static unsigned int read_failures;

SemaphoreHandle_t imu_mutex;


static void calc_accum_angles(void){
    // imu_data holds the old quaternion
    // new_data holds the new quaternion

    // Use data->quat to calculate data->accum_euler
    bool quat_same = (imu_data.quat.w == new_data.quat.w) && 
            (imu_data.quat.x == new_data.quat.x) &&
            (imu_data.quat.y == new_data.quat.y) &&
            (imu_data.quat.z == new_data.quat.z);

    // Old data is valid if it's quat is not all zeros
    bool data_valid = (imu_data.quat.w != 0) || 
            (imu_data.quat.x != 0) || 
            (imu_data.quat.y != 0) || 
            (imu_data.quat.z != 0);

    if(data_valid && !quat_same){
        // Accumulation math
        // Note that floating point errors lead to a small diff even if quaternions are the same
        // thus, don't run the math if quaternions are unchanged
        quaternion_t diff_quat;
        float dot_f;
        quat_dot(&dot_f, &new_data.quat, &imu_data.quat);
        if(dot_f < 0){
            quat_multiply_scalar(&diff_quat, &imu_data.quat, -1);
        }else{
            quat_multiply_scalar(&diff_quat, &imu_data.quat, 1);
        }
        quat_inverse(&diff_quat, &diff_quat);
        quat_multiply(&diff_quat, &new_data.quat, &diff_quat);
        euler_t diff_euler;
        quat_to_euler(&diff_euler, &diff_quat);
        euler_rad2deg(&diff_euler, &diff_euler);
        new_data.accum_angles.pitch = imu_data.accum_angles.pitch + diff_euler.pitch;
        new_data.accum_angles.roll = imu_data.accum_angles.roll + diff_euler.roll;
        new_data.accum_angles.yaw = imu_data.accum_angles.yaw + diff_euler.yaw;
        new_data.accum_angles.is_deg = true;
    }
}

static void reset_data(void){
    xSemaphoreTake(imu_mutex, portMAX_DELAY);
    imu_data.quat.w = 0;
    imu_data.quat.x = 0;
    imu_data.quat.y = 0;
    imu_data.quat.z = 0;
    imu_data.raw_gyro.x = 0;
    imu_data.raw_gyro.y = 0;
    imu_data.raw_gyro.z = 0;
    imu_data.raw_accel.x = 0;
    imu_data.raw_accel.y = 0;
    imu_data.raw_accel.z = 0;
    imu_data.accum_angles.is_deg = true;
    imu_data.accum_angles.pitch = 0;
    imu_data.accum_angles.roll = 0;
    imu_data.accum_angles.yaw = 0;
    xSemaphoreGive(imu_mutex);
}

void imu_init(void){
    // Default / initial values
    read_failures = 0;
    imu_data.quat.w = 0;
    imu_data.quat.x = 0;
    imu_data.quat.y = 0;
    imu_data.quat.z = 0;
    imu_data.raw_gyro.x = 0;
    imu_data.raw_gyro.y = 0;
    imu_data.raw_gyro.z = 0;
    imu_data.raw_accel.x = 0;
    imu_data.raw_accel.y = 0;
    imu_data.raw_accel.z = 0;
    imu_data.accum_angles.is_deg = true;
    imu_data.accum_angles.pitch = 0;
    imu_data.accum_angles.roll = 0;
    imu_data.accum_angles.yaw = 0;

    // Reading imu_data is multiple read operations
    // Want to ensure a write of imu_data cannot interrupt a read causing mixed data
    imu_mutex = xSemaphoreCreateMutex();

    // Init code for all supported IMUs
    bno055_init();
}

static void imu_configure_if_needed(void){
    // If sim hijacked, always use sim IMU
    if(cmdctrl_sim_hijacked){
        imu_which = IMU_SIM;
        read_failures = 0;

        // Reset IMU data whenever IMU changes (prevents angle accumulation issues)
        reset_data();
    }

    // If no longer sim hijacked, cannot use sim IMU
    if(!cmdctrl_sim_hijacked && imu_which == IMU_SIM){
        imu_which = IMU_NONE;
        read_failures = 0;
        
        // Reset IMU data whenever IMU changes (prevents angle accumulation issues)
        reset_data();
    }

    // If too many read failures, assume IMU is no longer connected
    if(read_failures >= 5){
        imu_which = IMU_NONE;
        read_failures = 0;

        // Reset IMU data whenever IMU changes (prevents angle accumulation issues)
        reset_data();
    }

    // Already an active IMU (no need to configure one)
    if(imu_which != IMU_NONE){
        return;
    }

    // Try to configure each IMU until one succeeds
    if(bno055_configure()){
        imu_which = IMU_BNO055;
    }else{
        // Failed to configure all IMUs
        imu_which = IMU_NONE;
    }

    // No need to reset data here. It was reset when imu_which became IMU_NONE (see if cases above)
}

bool imu_read(void){

    // Configure IMU if needed
    imu_configure_if_needed();

    // If still no IMU, abort read
    if(imu_which == IMU_NONE){
        return false;
    }

    // Read the active IMU
    bool success = false;
    switch(imu_which){
    case IMU_SIM:
        // Read SIM IMU
        new_data.quat = cmdctrl_sim_quat;
        new_data.raw_gyro.x = 0;
        new_data.raw_gyro.y = 0;
        new_data.raw_gyro.z = 0;
        new_data.raw_accel.x = 0;
        new_data.raw_accel.y = 0;
        new_data.raw_accel.z = 0;
        success = true;
        break;
    case IMU_BNO055:
        success = bno055_read(&new_data);
        break;
    }

    if(success){
        // Calculate accumulated angles after data from IMU exists
        calc_accum_angles();

        // Update imu_data while holding mutex
        xSemaphoreTake(imu_mutex, portMAX_DELAY);
        imu_data = new_data;
        xSemaphoreGive(imu_mutex);

    }else{
        read_failures++;
    }

    return success;
}

imu_data_t imu_get_data(void){
    imu_data_t ret_data;

    // Read under mutex because reading a struct is multiple reads
    // If reading from one thread and writing imu_data (via imu_read() function) on another thread
    // imu_data could change part way through the read here.
    xSemaphoreTake(imu_mutex, portMAX_DELAY);
    ret_data = imu_data;
    xSemaphoreGive(imu_mutex);

    return ret_data;
}

uint8_t imu_get_sensor(void){
    return imu_which;
}