#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f4xx.h"

#define configCPU_CLOCK_HZ                      SystemCoreClock
#define configTICK_RATE_HZ                      1000u
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  0
#define configMAX_PRIORITIES                    3
#define configMINIMAL_STACK_SIZE                128
#define configMAX_TASK_NAME_LEN                 16
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_TICKLESS_IDLE                  0
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        0
#define configKERNEL_PROVIDED_STATIC_MEMORY     1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_TIMERS                         0
#define configUSE_TASK_NOTIFICATIONS            0
#define configUSE_MUTEXES                        0
#define configQUEUE_REGISTRY_SIZE               0
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_uxTaskGetStackHighWaterMark     1

#define configKERNEL_INTERRUPT_PRIORITY         (15u << (8u - __NVIC_PRIO_BITS))
/* USART 中断的优先级高于此阈值，中断处理函数不调用内核 API。 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (5u << (8u - __NVIC_PRIO_BITS))
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler

void FreeRTOS_AssertFailed(unsigned line);
#define configASSERT(condition) do { if (!(condition)) FreeRTOS_AssertFailed(__LINE__); } while (0)

#endif
