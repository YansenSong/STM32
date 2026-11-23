#include "bsp_uart.h"
#include "vehicle_config.h"
#include "stm32f4xx_hal.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart5;

/* 串口环形缓冲区 */
#define UART1_RX_BUF_SIZE 256u

static volatile uint8_t  s_uart1_rx_buf[UART1_RX_BUF_SIZE];
static volatile uint16_t s_uart1_rx_head = 0u;
static volatile uint16_t s_uart1_rx_tail = 0u;

/* 调试变量 */
volatile uint32_t dbg_uart_rx_byte_cnt = 0u;
volatile uint32_t dbg_uart_rx_overflow_cnt = 0u;
volatile uint32_t dbg_uart_irq_cnt = 0u;
volatile uint8_t  dbg_uart_last_byte = 0u;
volatile uint8_t  dbg_uart_last_bytes[64];
volatile uint32_t dbg_uart_last_index = 0u;

#define UART5_RX_BUF_SIZE 512u

static volatile uint16_t s_uart5_rx_buf[UART5_RX_BUF_SIZE];
static volatile uint16_t s_uart5_rx_head = 0u;
static volatile uint16_t s_uart5_rx_tail = 0u;
static uint8_t s_uart5_discontinuity;
static uint32_t s_uart5_last_tick;
#define UART5_RX_BREAK 0x100u

volatile uint32_t dbg_uart5_rx_byte_cnt = 0u;
volatile uint32_t dbg_uart5_rx_overflow_cnt = 0u;
volatile uint32_t dbg_uart5_irq_cnt = 0u;
volatile uint8_t  dbg_uart5_last_byte = 0u;
volatile uint8_t  dbg_uart5_last_bytes[64];
volatile uint32_t dbg_uart5_last_index = 0u;

static void UART1_RxPush(uint8_t ch)
{
    uint16_t next_head;

    next_head = (uint16_t)((s_uart1_rx_head + 1u) % UART1_RX_BUF_SIZE);

    if (next_head == s_uart1_rx_tail)
    {
        dbg_uart_rx_overflow_cnt++;
        return;
    }

    s_uart1_rx_buf[s_uart1_rx_head] = ch;
    s_uart1_rx_head = next_head;

    dbg_uart_rx_byte_cnt++;
    dbg_uart_last_byte = ch;
    dbg_uart_last_bytes[dbg_uart_last_index & 0x3Fu] = ch;
    dbg_uart_last_index++;
}

static void UART5_RxPush(uint8_t ch)
{
    uint16_t next_head;

    next_head = (uint16_t)((s_uart5_rx_head + 1u) % UART5_RX_BUF_SIZE);

    if (next_head == s_uart5_rx_tail)
    {
        s_uart5_discontinuity = 1u;
        dbg_uart5_rx_overflow_cnt++;
        return;
    }

    s_uart5_rx_buf[s_uart5_rx_head] = (uint16_t)ch |
        (s_uart5_discontinuity ? UART5_RX_BREAK : 0u);
    s_uart5_discontinuity = 0u;
    s_uart5_rx_head = next_head;

    dbg_uart5_rx_byte_cnt++;
    dbg_uart5_last_byte = ch;
    dbg_uart5_last_bytes[dbg_uart5_last_index & 0x3Fu] = ch;
    dbg_uart5_last_index++;
}

void BSP_UART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /*
     * USART1:
     * PB6 = TX
     * PB7 = RX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = baudrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart1);

    s_uart1_rx_head = 0u;
    s_uart1_rx_tail = 0u;

    dbg_uart_rx_byte_cnt = 0u;
    dbg_uart_rx_overflow_cnt = 0u;
    dbg_uart_irq_cnt = 0u;
    dbg_uart_last_byte = 0u;
    dbg_uart_last_index = 0u;

    /*
     * 直接打开 RXNE 中断。
     * 不使用 HAL_UART_Receive_IT，避免每字节重启接收导致漏字节。
     */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

    HAL_NVIC_SetPriority(USART1_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/*
 * 从环形缓冲区取 1 字节。
 * 返回 1 表示取到，返回 0 表示没有数据。
 */
uint8_t BSP_UART1_ReadByte(uint8_t *out)
{
    if (out == 0)
    {
        return 0u;
    }

    if (s_uart1_rx_head == s_uart1_rx_tail)
    {
        return 0u;
    }

    *out = s_uart1_rx_buf[s_uart1_rx_tail];
    s_uart1_rx_tail = (uint16_t)((s_uart1_rx_tail + 1u) % UART1_RX_BUF_SIZE);

    return 1u;
}


void BSP_UART5_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();

    /* Native SBUS receive path on the controller board:
     * CN5 pin 29 (SBUS_RD) -> onboard Q4 inverter -> PD2/UART5_RX.
     * PC12/UART5_TX/SBUS_TD is intentionally unused.
     */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* SBUS: 100000 baud, 8 data bits + even parity, 2 stop bits.
     * STM32 HAL represents 8 data bits plus parity as WORDLENGTH_9B.
     */
    huart5.Instance = UART5;
    huart5.Init.BaudRate = baudrate;
    huart5.Init.WordLength = UART_WORDLENGTH_9B;
    huart5.Init.StopBits = UART_STOPBITS_2;
    huart5.Init.Parity = UART_PARITY_EVEN;
    huart5.Init.Mode = UART_MODE_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart5);

    s_uart5_rx_head = 0u;
    s_uart5_rx_tail = 0u;
    s_uart5_discontinuity = 1u;
    s_uart5_last_tick = HAL_GetTick();

    dbg_uart5_rx_byte_cnt = 0u;
    dbg_uart5_rx_overflow_cnt = 0u;
    dbg_uart5_irq_cnt = 0u;
    dbg_uart5_last_byte = 0u;
    dbg_uart5_last_index = 0u;

    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_PE);

    HAL_NVIC_SetPriority(UART5_IRQn, 1u, 1u);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

uint8_t BSP_UART5_ReadByte(uint8_t *out, uint8_t *discontinuity)
{
    uint16_t entry;
    if ((out == 0) || (discontinuity == 0))
    {
        return 0u;
    }

    if (s_uart5_rx_head == s_uart5_rx_tail)
    {
        return 0u;
    }

    entry = s_uart5_rx_buf[s_uart5_rx_tail];
    *out = (uint8_t)entry;
    *discontinuity = (uint8_t)((entry & UART5_RX_BREAK) != 0u);
    s_uart5_rx_tail = (uint16_t)((s_uart5_rx_tail + 1u) % UART5_RX_BUF_SIZE);

    return 1u;
}

void USART1_IRQHandler(void)
{
    uint32_t sr;
    uint8_t ch;

    dbg_uart_irq_cnt++;

    sr = USART1->SR;

    /*
     * RXNE：收到数据
     * ORE/FE/NE/PE：错误标志
     * STM32F4 清这些标志的方法是先读 SR，再读 DR。
     */
    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u)
    {
        ch = (uint8_t)(USART1->DR & 0xFFu);

        if ((sr & USART_SR_RXNE) != 0u)
        {
            UART1_RxPush(ch);
        }
    }
}

void UART5_IRQHandler(void)
{
    uint32_t sr;
    uint8_t ch;

    dbg_uart5_irq_cnt++;

    sr = UART5->SR;

    if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u)
    {
        ch = (uint8_t)(UART5->DR & 0xFFu);

        if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u)
        {
            s_uart5_discontinuity = 1u;
        }
        else if ((sr & USART_SR_RXNE) != 0u)
        {
            uint32_t now = HAL_GetTick();
            if ((uint32_t)(now - s_uart5_last_tick) >= 3u)
                s_uart5_discontinuity = 1u;
            s_uart5_last_tick = now;
            UART5_RxPush(ch);
        }
    }
}
