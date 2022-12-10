
#include <motor_control.h>
#include <app.h>
#include <thruster.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>

#define MOTOR_WDOG_PERIOD_MS            1500

// Globals
bool mc_invert[8];

// For motor watchdog
static bool motors_killed;
TimerHandle_t motor_wdog_timer;

// Ensures thread safe API for motor management
// Necessary because motor watchdog callback runs in timer task context
SemaphoreHandle_t motor_mutex;

// Forward declaration b/c used in init
static void mc_wdog_timeout(TimerHandle_t timer);

void mc_init(void){
    // Create required RTOS objects
    motor_mutex = xSemaphoreCreateRecursiveMutex();
    motor_wdog_timer = xTimerCreate(
        "mwdog_tim",
        pdMS_TO_TICKS(MOTOR_WDOG_PERIOD_MS),
        pdFALSE,
        NULL,
        mc_wdog_timeout
    );

    // Kill motors at startup
    motors_killed = true;

    // Default all motors to non-inverted
    for(unsigned int i = 0; i < 8; ++i){
        mc_invert[i] = false;
    }
}

void mc_set_motor_matrix(matrix *motor_mat){
    // TODO: Implement actual math
}

static void mc_wdog_timeout(TimerHandle_t timer){
    xSemaphoreTake(motor_mutex, portMAX_DELAY);
    motors_killed = true;
    thruster_set((float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    xSemaphoreGive(motor_mutex);
}

void mc_wdog_feed(void){

}

void mc_set_raw(float *speeds){
    xSemaphoreTake(motor_mutex, portMAX_DELAY);   

    // Don't allow speed set while motors are killed
    if(motors_killed)
        return;

    // Apply thruster inversions
    for(unsigned int i = 0; i < 8; ++i){
        if(mc_invert[i])
            speeds[i] *= -1;
    }

    // Actually set thruster speeds
    thruster_set(speeds);

    xSemaphoreGive(motor_mutex);
}
