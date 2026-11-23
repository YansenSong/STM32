/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines for the vehicle control project.
  ******************************************************************************
  */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

/******************************************************************************/
/* Cortex-M4 Processor Exceptions Handlers                                     */
/******************************************************************************/

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void DebugMon_Handler(void)
{
}

extern void xPortSysTickHandler(void);
extern volatile uint8_t g_freertos_tick_enabled;

void SysTick_Handler(void)
{
    HAL_IncTick();
    if (g_freertos_tick_enabled != 0u)
        xPortSysTickHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                     */
/******************************************************************************/

void EXTI0_IRQHandler(void)
{
}

void EXTI4_IRQHandler(void)
{
}

void EXTI9_5_IRQHandler(void)
{
}

/*
 * 注意：
 * USART1_IRQHandler 已经在 bsp_uart.c 里实现。
 * 这里不要再写 USART1_IRQHandler，否则会重复定义。
 *
 * CAN 接收暂时不做，所以这里也不写 CAN1/CAN2 RX 中断。
 */