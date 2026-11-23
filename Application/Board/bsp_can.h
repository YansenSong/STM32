#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

extern CAN_HandleTypeDef hcan1_eps;
extern CAN_HandleTypeDef hcan2_drive;

extern volatile uint32_t dbg_can1_tx_ok_cnt;
extern volatile uint32_t dbg_can1_tx_fail_cnt;
extern volatile uint32_t dbg_can2_tx_ok_cnt;
extern volatile uint32_t dbg_can2_tx_fail_cnt;
extern volatile uint32_t dbg_can_init_step;
extern volatile uint32_t dbg_can1_init_ret;
extern volatile uint32_t dbg_can2_init_ret;


uint8_t BSP_CAN1_Init_EPS(void);
uint8_t BSP_CAN2_Init_DriveBrake(void);

uint8_t BSP_CAN1_SendStd(uint16_t std_id, const uint8_t *data, uint8_t dlc);
uint8_t BSP_CAN2_SendStd(uint16_t std_id, const uint8_t *data, uint8_t dlc);

void BSP_CAN_RX_Init(void);
void BSP_CAN_ProcessRX(void);
extern volatile uint32_t dbg_can_rx_ready;
extern volatile uint32_t dbg_can1_rx_cnt, dbg_can2_rx_cnt;
extern volatile uint32_t dbg_can1_rx_overrun_cnt, dbg_can2_rx_overrun_cnt;
extern volatile uint32_t dbg_can_rx_reject_cnt;
extern volatile uint16_t dbg_can1_last_rx_id, dbg_can2_last_rx_id;
extern volatile uint8_t dbg_can1_last_rx_data[8], dbg_can2_last_rx_data[8];
/* EPS receive diagnostic Watch, raw trace without guessed signal semantics. */
extern volatile uint32_t dbg_can1_diag_cnt;
extern volatile uint32_t dbg_can1_diag_18e_cnt, dbg_can1_diag_18f_cnt;
extern volatile uint32_t dbg_can1_diag_19f_cnt, dbg_can1_diag_700_cnt;
extern volatile uint32_t dbg_can1_diag_other_cnt;
extern volatile uint32_t dbg_can1_diag_esr, dbg_can1_diag_msr, dbg_can1_diag_btr;
extern volatile uint16_t dbg_can1_diag_last_id;
extern volatile uint8_t dbg_can1_diag_last_dlc, dbg_can1_diag_last_data[8];
extern volatile uint8_t dbg_can1_diag_next;
extern volatile uint16_t dbg_can1_diag_ids[16];
extern volatile uint8_t dbg_can1_diag_dlcs[16], dbg_can1_diag_data[16][8];
extern volatile uint32_t dbg_can1_diag_ticks[16];
#endif
