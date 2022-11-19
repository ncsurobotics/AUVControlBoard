
#include <tusb.h>

extern void xPortSysTickHandler( void );

#if defined(CONTROL_BOARD_V1)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Control Board V1 IRQ Handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void NMI_Handler(void){
    while (1);
}

void HardFault_Handler(void){
    while (1);
}

void MemManage_Handler(void){
    while (1);
}

void BusFault_Handler(void){
    while (1);
}

void UsageFault_Handler(void){
    while (1);
}

void DebugMon_Handler(void){}

// Defined by FreeRTOS
// void SysTick_Handler(void){}

// Defined by FreeRTOS
// void SVCall_Handler(void){}

// Defined by FreeRTOS
// void PendSV_Handler(void){}

void USB_0_Handler (void){
    tud_int_handler(0);
}

void USB_1_Handler (void){
    tud_int_handler(0);
}

void USB_2_Handler (void){
    tud_int_handler(0);
}

void USB_3_Handler (void){
    tud_int_handler(0);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#elif defined(CONTROL_BOARD_V2)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Control Board V2 IRQ Handlers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Adapted from stm32f4xx_it.c generated by STM32CubeMX

#include <stm32f4xx_hal.h>
#include  <stm32f4xx_hal_tim.h>

extern TIM_HandleTypeDef htim11;

void NMI_Handler(void){
    while (1);
}

void HardFault_Handler(void){
    while (1);
}

void MemManage_Handler(void){
    while (1);
}

void BusFault_Handler(void){
    while (1);
}

void UsageFault_Handler(void){
    while (1);
}

void DebugMon_Handler(void){}

// Defined by FreeRTOS
// void SysTick_Handler(void){}

// Defined by FreeRTOS
// void PendSV_Handler(void){}

// Defined by FreeRTOS
// void SVC_Handler(void){}

void TIM1_TRG_COM_TIM11_IRQHandler(void){
    HAL_TIM_IRQHandler(&htim11);
}

void OTG_FS_IRQHandler(void){
    tud_int_handler(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif
