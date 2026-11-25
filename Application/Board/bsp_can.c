#include "bsp_can.h"
#include "vehicle_config.h"
#include "vehicle_feedback.h"
#include "stm32f4xx_hal.h"

static uint8_t s_can1_started = 0u;
static uint8_t s_can2_started = 0u;
CAN_HandleTypeDef hcan1_eps;
CAN_HandleTypeDef hcan2_drive;

volatile uint32_t dbg_can1_tx_ok_cnt = 0u;
volatile uint32_t dbg_can1_tx_fail_cnt = 0u;
volatile uint32_t dbg_can2_tx_ok_cnt = 0u;
volatile uint32_t dbg_can2_tx_fail_cnt = 0u;

volatile uint32_t dbg_can_init_step = 0u;
volatile uint32_t dbg_can1_init_ret = 0u;
volatile uint32_t dbg_can2_init_ret = 0u;

static uint8_t can_enter_init(CAN_TypeDef *CANx)
{
    uint32_t timeout = 1000000u;

    CANx->MCR &= ~CAN_MCR_SLEEP;

    while ((CANx->MSR & CAN_MSR_SLAK) != 0u)
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    CANx->MCR |= CAN_MCR_INRQ;

    timeout = 1000000u;
    while ((CANx->MSR & CAN_MSR_INAK) == 0u)
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    return 1u;
}

static uint8_t can_leave_init(CAN_TypeDef *CANx)
{
    uint32_t timeout = 1000000u;

    CANx->MCR &= ~CAN_MCR_INRQ;

    while ((CANx->MSR & CAN_MSR_INAK) != 0u)
    {
        if (--timeout == 0u)
        {
            return 0u;
        }
    }

    return 1u;
}

static void can_set_btr(CAN_TypeDef *CANx, uint16_t prescaler)
{
    /*
     * APB1 = 16MHz
     * Tq = 1 + BS1(13) + BS2(2) = 16
     *
     * CAN1 250K: prescaler = 4
     * CAN2 500K: prescaler = 2
     */
    /* Only CAN1 timing changes; CAN2 stays SJW1. Configure in init mode. */
    uint32_t sjw = (CANx == CAN1) ? VEH_CAN1_SJW_TQ : 1u;
    uint32_t bs1 = 13u;
    uint32_t bs2 = 2u;

    CANx->BTR =
        ((sjw - 1u) << 24) |
        ((bs2 - 1u) << 20) |
        ((bs1 - 1u) << 16) |
        ((uint32_t)(prescaler - 1u));
}

static void can_config_common(CAN_TypeDef *CANx)
{
    /*
     * 保持 INRQ=1，不要提前退出初始化模式。
     * ABOM: 自动离线恢复
     * NART: 禁止自动重发，调试阶段避免无 ACK 时邮箱一直占用
     */
    CANx->MCR |= CAN_MCR_INRQ | CAN_MCR_ABOM | CAN_MCR_NART;

    CANx->MCR &= ~(CAN_MCR_TTCM |
                   CAN_MCR_AWUM |
                   CAN_MCR_RFLM |
                   CAN_MCR_TXFP);
}
uint8_t BSP_CAN1_Init_EPS(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    s_can1_started = 0u;
    dbg_can_init_step = 101u;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();

    /*
     * CAN1:
     * PA11 = RX
     * PA12 = TX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hcan1_eps.Instance = CAN1;

    dbg_can_init_step = 102u;

    if (can_enter_init(CAN1) == 0u)
    {
        dbg_can1_init_ret = 1u;
        dbg_can_init_step = 191u;
        return 0u;
    }

    dbg_can_init_step = 103u;

    can_config_common(CAN1);
    can_set_btr(CAN1, 4u);     /* 250K @ APB1 16MHz */

    dbg_can_init_step = 104u;

    if (can_leave_init(CAN1) == 0u)
    {
        dbg_can1_init_ret = 2u;
        dbg_can_init_step = 192u;
        return 0u;
    }

    s_can1_started = 1u;
    dbg_can_init_step = 105u;
    dbg_can1_init_ret = 2u;

    return 1u;
}

uint8_t BSP_CAN2_Init_DriveBrake(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    s_can2_started = 0u;
    dbg_can_init_step = 201u;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*
     * CAN2 依赖 CAN1 时钟。
     */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();

    /*
     * CAN2:
     * PB12 = RX
     * PB13 = TX
     */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hcan2_drive.Instance = CAN2;

    dbg_can_init_step = 202u;

    if (can_enter_init(CAN2) == 0u)
    {
        dbg_can2_init_ret = 1u;
        dbg_can_init_step = 291u;
        return 0u;
    }

    dbg_can_init_step = 203u;

    can_config_common(CAN2);
    can_set_btr(CAN2, 2u);     /* 500K @ APB1 16MHz */

    dbg_can_init_step = 204u;

    if (can_leave_init(CAN2) == 0u)
    {
        dbg_can2_init_ret = 2u;
        dbg_can_init_step = 292u;
        return 0u;
    }

    s_can2_started = 1u;
    dbg_can_init_step = 205u;
    dbg_can2_init_ret = 2u;

    return 1u;
}

static uint8_t can_send_std(CAN_TypeDef *CANx,
                            uint16_t std_id,
                            const uint8_t *data,
                            uint8_t dlc,
                            volatile uint32_t *ok_cnt,
                            volatile uint32_t *fail_cnt)
{
    uint32_t mailbox;

    if (data == 0 || dlc > 8u)
    {
        (*fail_cnt)++;
        return 0u;
    }

    if ((CANx->TSR & CAN_TSR_TME0) != 0u)
    {
        mailbox = 0u;
    }
    else if ((CANx->TSR & CAN_TSR_TME1) != 0u)
    {
        mailbox = 1u;
    }
    else if ((CANx->TSR & CAN_TSR_TME2) != 0u)
    {
        mailbox = 2u;
    }
    else
    {
        (*fail_cnt)++;
        return 0u;
    }

    CANx->sTxMailBox[mailbox].TIR = ((uint32_t)(std_id & 0x7FFu) << 21);
    CANx->sTxMailBox[mailbox].TDTR = (uint32_t)(dlc & 0x0Fu);

    CANx->sTxMailBox[mailbox].TDLR =
        ((uint32_t)data[0]) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);

    CANx->sTxMailBox[mailbox].TDHR =
        ((uint32_t)data[4]) |
        ((uint32_t)data[5] << 8) |
        ((uint32_t)data[6] << 16) |
        ((uint32_t)data[7] << 24);

    CANx->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;

    (*ok_cnt)++;
    return 1u;
}

uint8_t BSP_CAN1_SendStd(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    return can_send_std(CAN1,
                        std_id,
                        data,
                        dlc,
                        &dbg_can1_tx_ok_cnt,
                        &dbg_can1_tx_fail_cnt);
}

uint8_t BSP_CAN2_SendStd(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    return can_send_std(CAN2,
                        std_id,
                        data,
                        dlc,
                        &dbg_can2_tx_ok_cnt,
                        &dbg_can2_tx_fail_cnt);
}

/*
 * Compatibility for the old STM32F4 HAL CAN driver in this project.
 * We do not use HAL_CAN_Init() now, but stm32f4xx_hal_can.c is still compiled,
 * so these symbols must exist to satisfy the linker.
 */
void HAL_CAN1_MspInit(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}

void HAL_CAN2_MspInit(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}
volatile uint32_t dbg_can_rx_ready = 0u;
volatile uint32_t dbg_can1_rx_cnt = 0u;
volatile uint32_t dbg_can2_rx_cnt = 0u;
volatile uint32_t dbg_can1_rx_overrun_cnt = 0u;
volatile uint32_t dbg_can2_rx_overrun_cnt = 0u;
volatile uint32_t dbg_can_rx_reject_cnt = 0u;
volatile uint16_t dbg_can1_last_rx_id = 0u;
volatile uint16_t dbg_can2_last_rx_id = 0u;
volatile uint8_t dbg_can1_last_rx_data[8];
volatile uint8_t dbg_can2_last_rx_data[8];

static void can_rx_filter(uint8_t bank, uint16_t id, uint8_t enabled)
{
    uint32_t bit;
    bit = (uint32_t)1u << bank;
    CAN1->FA1R &= ~bit;
    CAN1->FM1R &= ~bit;
    CAN1->FS1R |= bit;
    CAN1->FFA1R &= ~bit;
    CAN1->sFilterRegister[bank].FR1 = (uint32_t)id << 21;
    /* Match all 11 standard-ID bits, IDE=0 and RTR=0; FIFO0. */
    CAN1->sFilterRegister[bank].FR2 = 0xFFE00006u;
    if (enabled != 0u) CAN1->FA1R |= bit;
}

void BSP_CAN_RX_Init(void)
{
    /* Shared bxCAN filters: banks0..13 CAN1, banks14..27 CAN2.
     * Original TX-only code does not install filters or CAN IRQs. */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FMR = (CAN1->FMR & ~0x00003F00u) | (14u << 8) | CAN_FMR_FINIT;
    can_rx_filter(0u, VEH_CAN_ID_EPS_STATUS, s_can1_started);
    can_rx_filter(1u, VEH_CAN_ID_EPS_FAULT, s_can1_started);
    can_rx_filter(14u, VEH_CAN_ID_RT49_STATUS, s_can2_started);
    can_rx_filter(15u, VEH_CAN_ID_SEB_STATUS, s_can2_started);
    can_rx_filter(16u, VEH_CAN_ID_SEB_FAULT, s_can2_started);
#if VEH_EPS_RX_DIAGNOSTIC
    /* All STANDARD DATA IDs on CAN1. No guessed angle decoding for other IDs. */
    CAN1->sFilterRegister[0].FR1 = 0u;
    CAN1->sFilterRegister[0].FR2 = 6u;
#endif
    CAN1->FMR &= ~CAN_FMR_FINIT;
    dbg_can_rx_ready = (uint32_t)s_can1_started | ((uint32_t)s_can2_started << 1);
}

#define CAN1_DIAG_TRACE_LEN 16u
volatile uint32_t dbg_can1_diag_cnt = 0u;
volatile uint32_t dbg_can1_diag_18e_cnt = 0u;
volatile uint32_t dbg_can1_diag_18f_cnt = 0u;
volatile uint32_t dbg_can1_diag_19f_cnt = 0u;
volatile uint32_t dbg_can1_diag_700_cnt = 0u;
volatile uint32_t dbg_can1_diag_other_cnt = 0u;
volatile uint32_t dbg_can1_diag_esr = 0u;
volatile uint32_t dbg_can1_diag_msr = 0u;
volatile uint32_t dbg_can1_diag_btr = 0u;
volatile uint16_t dbg_can1_diag_last_id = 0u;
volatile uint8_t dbg_can1_diag_last_dlc = 0u;
volatile uint8_t dbg_can1_diag_last_data[8];
volatile uint8_t dbg_can1_diag_next = 0u;
volatile uint16_t dbg_can1_diag_ids[CAN1_DIAG_TRACE_LEN];
volatile uint8_t dbg_can1_diag_dlcs[CAN1_DIAG_TRACE_LEN];
volatile uint32_t dbg_can1_diag_ticks[CAN1_DIAG_TRACE_LEN];
volatile uint8_t dbg_can1_diag_data[CAN1_DIAG_TRACE_LEN][8];

static void can1_diag_record(uint32_t rir, uint8_t dlc, uint32_t low,
                             uint32_t high, uint32_t now)
{
    uint8_t i;
    uint8_t slot;
    uint8_t byte;
    uint16_t id;
    if ((rir & 6u) != 0u) return; /* Standard DATA only. */
    id = (uint16_t)((rir >> 21) & 0x7FFu);
    slot = dbg_can1_diag_next;
    dbg_can1_diag_last_id = id;
    dbg_can1_diag_last_dlc = dlc;
    dbg_can1_diag_ids[slot] = id;
    dbg_can1_diag_dlcs[slot] = dlc;
    dbg_can1_diag_ticks[slot] = now;
    for (i = 0u; i < 8u; i++)
    {
        byte = i < 4u ? (uint8_t)(low >> (8u * i)) :
                         (uint8_t)(high >> (8u * (i - 4u)));
        if (i >= dlc) byte = 0u; /* Never present mailbox tail as real bytes. */
        dbg_can1_diag_last_data[i] = byte;
        dbg_can1_diag_data[slot][i] = byte;
    }
    if (id == 0x18Eu) dbg_can1_diag_18e_cnt++;
    else if (id == 0x18Fu) dbg_can1_diag_18f_cnt++;
    else if (id == 0x19Fu) dbg_can1_diag_19f_cnt++;
    else if (id == 0x700u) dbg_can1_diag_700_cnt++;
    else dbg_can1_diag_other_cnt++;
    dbg_can1_diag_cnt++;
    dbg_can1_diag_next = (uint8_t)((slot + 1u) % CAN1_DIAG_TRACE_LEN);
}

static void can_rx_poll(CAN_TypeDef *can, uint8_t bus)
{
    uint8_t count;
    uint8_t i;
    uint8_t dlc;
    uint8_t data[8];
    uint16_t id;
    uint32_t rir;
    uint32_t low;
    uint32_t high;
    uint32_t now;
    for (count = 0u; count < VEH_CAN_RX_MAX_PER_BUS; count++)
    {
        if ((can->RF0R & CAN_RF0R_FOVR0) != 0u)
        {
            if (bus == 1u) dbg_can1_rx_overrun_cnt++;
            else dbg_can2_rx_overrun_cnt++;
            can->RF0R = CAN_RF0R_FOVR0;
        }
        if ((can->RF0R & CAN_RF0R_FMP0) == 0u) break;
        rir = can->sFIFOMailBox[0].RIR;
        dlc = (uint8_t)(can->sFIFOMailBox[0].RDTR & 15u);
        low = can->sFIFOMailBox[0].RDLR;
        high = can->sFIFOMailBox[0].RDHR;
        now = HAL_GetTick();
        /* Write only the release bit: avoid accidental W1C flag clearing. */
        can->RF0R = CAN_RF0R_RFOM0;
        /* Record even non-8 DLC for diagnosis; strict status parser still rejects. */
        if (bus == 1u) can1_diag_record(rir, dlc, low, high, now);
        if (((rir & 6u) != 0u) || (dlc != 8u))
        {
            dbg_can_rx_reject_cnt++;
            continue;
        }
        id = (uint16_t)((rir >> 21) & 0x7FFu);
        for (i = 0u; i < 4u; i++)
        {
            data[i] = (uint8_t)(low >> (8u * i));
            data[i + 4u] = (uint8_t)(high >> (8u * i));
        }
        if (bus == 1u)
        {
            dbg_can1_rx_cnt++;
            dbg_can1_last_rx_id = id;
            for (i = 0u; i < 8u; i++) dbg_can1_last_rx_data[i] = data[i];
        }
        else
        {
            dbg_can2_rx_cnt++;
            dbg_can2_last_rx_id = id;
            for (i = 0u; i < 8u; i++) dbg_can2_last_rx_data[i] = data[i];
        }
        VehicleFeedback_OnCAN(bus, id, data, dlc, now);
    }
}

void BSP_CAN_ProcessRX(void)
{
    dbg_can1_diag_esr = CAN1->ESR;
    dbg_can1_diag_msr = CAN1->MSR;
    dbg_can1_diag_btr = CAN1->BTR;
    if ((dbg_can_rx_ready & 1u) != 0u) can_rx_poll(CAN1, 1u);
    if ((dbg_can_rx_ready & 2u) != 0u) can_rx_poll(CAN2, 2u);
}
