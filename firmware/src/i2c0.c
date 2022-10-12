/**
 * @file i2c0.c
 * @author Marcus Behel
 */

#include <i2c0.h>
#include <flags.h>
#include <timers.h>
#include <atmel_start.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C                 I2C_0                               // Which ASF I2C object to use

// States
#define STATE_IDLE          0
#define STATE_WRITE         1
#define STATE_READ          2

#define QUEUE_SIZE          16                                  // Size of I2C transaction queue

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void i2c0_init(void){
    // i2c_m_async_get_io_descriptor(&I2C, &io);                   // Store for later
    i2c_m_async_enable(&I2C);                                   // Enable bus (ASF layer)
}