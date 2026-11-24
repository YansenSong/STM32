#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "health_task.h"
#include "ota_boot.h"
#include "ota_update.h"
#include "vehicle_config.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_eth.h"
#include "upper_cmd.h"
#include "vehicle_control.h"
#include "remote_control.h"
#include "vehicle_status.h"

#if (VEH_STA_PERIOD_MS != VEH_CONTROL_PERIOD_MS)
#error Status task shares the existing control period
#endif

static void SystemClock_Config(void);
#define VEHICLE_STACK_WORDS 768u
#define VEHICLE_PRIORITY    2u

static void Vehicle_CreateTask(void);
static void VehicleTask(void *argument);
static StaticTask_t s_vehicle_task;
static StackType_t s_vehicle_stack[VEHICLE_STACK_WORDS];
volatile uint8_t g_freertos_tick_enabled;
volatile uint32_t dbg_freertos_assert_line;
volatile uint32_t dbg_freertos_stack_overflow;
volatile UBaseType_t dbg_vehicle_stack_free_words;

volatile uint32_t dbg_main_loop_cnt = 0u;
volatile uint32_t dbg_last_20ms_tick = 0u;
volatile uint32_t dbg_boot_step = 0;

volatile uint32_t dbg_control_max_late_us;
volatile uint32_t dbg_control_max_run_us;
volatile uint32_t dbg_control_missed_periods;

int main(void)
{
    dbg_boot_step = 1;
    HAL_Init();

    dbg_boot_step = 2;
    SystemClock_Config();

    dbg_boot_step = 3;
    BSP_UART1_Init(VEH_UART1_BAUDRATE);

    dbg_boot_step = 31;
    BSP_UART5_Init(VEH_REMOTE_UART5_BAUDRATE);

    dbg_boot_step = 4;
    dbg_can1_init_ret = BSP_CAN1_Init_EPS();

    dbg_boot_step = 5;
    dbg_can2_init_ret = BSP_CAN2_Init_DriveBrake();

    dbg_boot_step = 6;
    UpperCmd_Init();

    dbg_boot_step = 61;
    Remote_Init();

    dbg_boot_step = 7;
    Vehicle_Init();
    VehicleStatus_Init();
    BSP_CAN_RX_Init();

    dbg_boot_step = 8;
    /* APB1 的预分频系数为 1，TIM2 的输入时钟等于 PCLK1。 */
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->PSC = HAL_RCC_GetPCLK1Freq() / 1000000u - 1u;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 = TIM_CR1_CEN;
    /* 先创建三个应用任务，再启动调度器。 */
    Vehicle_CreateTask();
    Health_CreateTask();
    OtaUpdate_CreateTask();
    vTaskStartScheduler();
    FreeRTOS_AssertFailed(__LINE__);
    return 1;
}

static void Vehicle_CreateTask(void)
{
    TaskHandle_t task = xTaskCreateStatic(VehicleTask, "Vehicle", VEHICLE_STACK_WORDS, NULL,
                                          VEHICLE_PRIORITY, s_vehicle_stack, &s_vehicle_task);
    configASSERT(task != NULL);
}

static void VehicleTask(void *argument)
{
    const uint32_t period_us = VEH_CONTROL_PERIOD_MS * 1000u;
    uint32_t next_control = TIM2->CNT + period_us;
    uint32_t stack_sample_tick = HAL_GetTick();
    (void)argument;
    g_freertos_tick_enabled = 1u;

    while (1)
    {
        uint32_t start = TIM2->CNT;
        uint32_t late = start - next_control;
        uint8_t control_ran = 0u;
        dbg_main_loop_cnt++;
        if (OtaUpdate_InMaintenance())
        {
            OtaBoot_Service(HAL_GetTick(), 0u);
            BSP_ETH_Service(HAL_GetTick());
            BSP_ETH_Process();
            OtaUpdate_SendReply();
            vTaskDelay(1u);
            continue;
        }

        if ((int32_t)late >= 0)
        {
            uint32_t missed = late / period_us;
            uint32_t elapsed;
            dbg_control_missed_periods += missed;
            next_control += (missed + 1u) * period_us;
            if (late > dbg_control_max_late_us) dbg_control_max_late_us = late;
            dbg_last_20ms_tick = HAL_GetTick();
            if (Remote_ShouldOverride() != 0u)
            {
                Remote_UpdateVehicleCmd();
                VehicleStatus_SetControlSource(VEH_SOURCE_REMOTE);
            }
            Vehicle_20msTask();
            elapsed = TIM2->CNT - start;
            if (elapsed > dbg_control_max_run_us) dbg_control_max_run_us = elapsed;
            control_ran = 1u;
        }

        UpperCmd_Process();
        BSP_ETH_Service(HAL_GetTick());
        BSP_ETH_Process();
        Remote_Process();
        BSP_CAN_ProcessRX();
        if (control_ran != 0u)
        {
            VehicleStatus_20msTask();
            OtaBoot_Service(HAL_GetTick(), (uint8_t)(BSP_ETH_IsReady() &&
                            dbg_control_missed_periods == 0u));
        }

        if ((uint32_t)(HAL_GetTick() - stack_sample_tick) >= 1000u)
        {
            stack_sample_tick = HAL_GetTick();
            dbg_vehicle_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
            const HealthSnapshot snapshot = {
                .uptime_ms = stack_sample_tick,
                .max_control_late_us = dbg_control_max_late_us,
                .max_control_run_us = dbg_control_max_run_us,
                .missed_periods = dbg_control_missed_periods,
                .vehicle_stack_free_words = dbg_vehicle_stack_free_words,
                .can_online_mask = g_vehicle_status.can_online_mask,
                .ethernet_retries = dbg_eth_retries,
            };
            Health_Publish(&snapshot);
        }
        Health_SendPending();
        vTaskDelay(1u);
    }
}

void FreeRTOS_AssertFailed(unsigned line)
{
    __disable_irq();
    dbg_freertos_assert_line = line;
    for (;;) {}
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    dbg_freertos_stack_overflow = 1u;
    FreeRTOS_AssertFailed(__LINE__);
}

static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* 使用内部 16 MHz 高速振荡器（HSI）作为系统时钟。 */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        dbg_boot_step = 200;
        while (1)
        {
        }
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_HCLK   |
                                  RCC_CLOCKTYPE_PCLK1  |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        dbg_boot_step = 201;
        while (1)
        {
        }
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000u);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0u, 0u);
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    (void)file;
    (void)line;
    while (1) {}
}
#endif
