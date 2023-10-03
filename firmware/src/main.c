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

#include <hardware/led.h>
#include <hardware/delay.h>
#include <hardware/thruster.h>
#include <hardware/eeprom.h>
#include <hardware/i2c.h>
#include <hardware/wdt.h>

#include <util/conversions.h>

#include <framework.h>
#include <FreeRTOS.h>
#include <task.h>
#include <math.h>
#include <hardware/usb.h>
#include <app.h>
#include <cmdctrl.h>
#include <motor_control.h>
#include <debug.h>
#include <pccomm.h>
#include <calibration.h>


#if defined(CONTROL_BOARD_V1)
__attribute__((section(".noinit"))) volatile uint32_t first_run;
__attribute__((section(".noinit"))) volatile uint32_t reset_cause_persist;
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Program Entry point / startup
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void){
    // -------------------------------------------------------------------------
    // System & Peripheral Initialization
    // -------------------------------------------------------------------------
    init_frameworks();
    conversions_init();

#ifdef NDEBUG
    // Enable watchdog if not debug build
    wdt_init();
#endif

    delay_init();
    led_init();
    usb_init();
    pccomm_init();
    thruster_init();
    mc_init();
    cmdctrl_init();
    i2c_init();
    eeprom_init();
    calibration_load();
    // -------------------------------------------------------------------------
    
    app_init();

    // -------------------------------------------------------------------------
    // RTOS Startup
    // -------------------------------------------------------------------------
    vTaskStartScheduler();
    // -------------------------------------------------------------------------

    // Start scheduler should never return. This should never run, but is
    // included to make debugging easier in case it does
    debug_halt(HALT_EC_SCHEDRET);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
