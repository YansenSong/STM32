#ifndef BSP_ETH_H
#define BSP_ETH_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Ethernet UDP command receiver.
 * Default: static IP 192.168.5.50, UDP port 5000.
 * PC/path-planning host sends the same 26-byte cmd__ frame by UDP payload.
 */

void BSP_ETH_Service(uint32_t now);
void BSP_ETH_Process(void);
uint8_t BSP_ETH_IsReady(void);
/* Fixed status peer; returns DMA queue success, not host acknowledgement.
 * ARP pending/DMA busy returns immediately. Called from main loop only.
 */
uint8_t BSP_ETH_SendStatus(const uint8_t *payload, uint16_t len);

uint8_t BSP_ETH_SendTelemetry(const uint8_t *payload, uint16_t len, uint16_t port);
extern volatile uint32_t dbg_eth_retries;

extern volatile uint8_t dbg_sta_target_ip[4];
extern volatile uint16_t dbg_sta_target_port;
extern volatile uint8_t dbg_sta_peer_mac[6];
extern volatile uint32_t dbg_sta_peer_valid;
extern volatile uint32_t dbg_sta_arp_request_cnt;
/* 0=queued, 1=ETH unavailable, 2=ARP pending, 3=DMA busy, 4=bad payload,
 * 5=HAL send failed, 6=peer outside local subnet. */
extern volatile uint32_t dbg_sta_tx_result;

extern volatile uint32_t dbg_eth_init_step;
extern volatile uint32_t dbg_eth_init_ret;
extern volatile uint32_t dbg_eth_ready;
extern volatile uint32_t dbg_eth_rx_frame_cnt;
extern volatile uint32_t dbg_eth_rx_udp_cnt;
extern volatile uint32_t dbg_eth_rx_cmd_cnt;
extern volatile uint32_t dbg_eth_cmd_ok_cnt;
extern volatile uint32_t dbg_eth_cmd_checksum_err_cnt;
extern volatile uint32_t dbg_eth_cmd_header_err_cnt;
extern volatile uint32_t dbg_eth_arp_req_cnt;
extern volatile uint32_t dbg_eth_arp_reply_cnt;
extern volatile uint32_t dbg_eth_tx_ok_cnt;
extern volatile uint32_t dbg_eth_tx_fail_cnt;
extern volatile uint32_t dbg_eth_last_rx_len;
extern volatile uint8_t  dbg_eth_last_cmd[26];
extern volatile uint8_t  dbg_eth_last_src_ip[4];
extern volatile uint16_t dbg_eth_last_src_port;

#ifdef __cplusplus
}
#endif

#endif /* BSP_ETH_H */
