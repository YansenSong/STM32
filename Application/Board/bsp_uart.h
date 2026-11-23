#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;

extern volatile uint32_t dbg_uart_rx_byte_cnt;
extern volatile uint32_t dbg_uart_rx_overflow_cnt;
extern volatile uint32_t dbg_uart_irq_cnt;
extern volatile uint8_t  dbg_uart_last_byte;
extern volatile uint8_t  dbg_uart_last_bytes[64];
extern volatile uint32_t dbg_uart_last_index;

extern volatile uint32_t dbg_uart5_rx_byte_cnt;
extern volatile uint32_t dbg_uart5_rx_overflow_cnt;
extern volatile uint32_t dbg_uart5_irq_cnt;
extern volatile uint8_t  dbg_uart5_last_byte;
extern volatile uint8_t  dbg_uart5_last_bytes[64];
extern volatile uint32_t dbg_uart5_last_index;

void BSP_UART1_Init(uint32_t baudrate);
uint8_t BSP_UART1_ReadByte(uint8_t *out);

void BSP_UART5_Init(uint32_t baudrate);
uint8_t BSP_UART5_ReadByte(uint8_t *out, uint8_t *discontinuity);

#endif
