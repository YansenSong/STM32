#include "bsp_eth.h"
#include "ota_update.h"
#include "vehicle_config.h"
#include "vehicle_control.h"
#include "vehicle_status.h"
#include <string.h>

/*
 * Lightweight Ethernet receiver for STM32F407 + LAN8720.
 * It avoids adding a full lwIP stack. The PC/path-planning host sends one
 * 26-byte cmd__ frame as the UDP payload to VEH_ETH_LOCAL_IP:VEH_ETH_UDP_PORT.
 *
 * Supported:
 *   1) ARP reply for VEH_ETH_LOCAL_IP
 *   2) IPv4 UDP receive, payload scan for 26-byte cmd__ frame
 *   3) Forward valid command to Vehicle_ParseCmd26(), same as USART1 command
 *
 * Hardware pins from the schematic:
 *   PA1  ETH_RMII_REF_CLK
 *   PA2  ETH_MDIO
 *   PA7  ETH_RMII_CRS_DV
 *   PC1  ETH_MDC
 *   PC4  ETH_RMII_RXD0
 *   PC5  ETH_RMII_RXD1
 *   PG11 ETH_RMII_TX_EN
 *   PG13 ETH_RMII_TXD0
 *   PG14 ETH_RMII_TXD1
 */

#ifndef VEH_ETH_ENABLE
#define VEH_ETH_ENABLE                 1u
#endif

#ifndef VEH_ETH_UDP_PORT
#define VEH_ETH_UDP_PORT               5000u
#endif

#ifndef VEH_ETH_ACCEPT_ANY_UDP_PORT
#define VEH_ETH_ACCEPT_ANY_UDP_PORT    0u
#endif

#ifndef VEH_ETH_LOCAL_IP0
#define VEH_ETH_LOCAL_IP0              192u
#define VEH_ETH_LOCAL_IP1              168u
#define VEH_ETH_LOCAL_IP2              5u
#define VEH_ETH_LOCAL_IP3              50u
#endif

#ifndef VEH_ETH_MAC0
#define VEH_ETH_MAC0                   0x02u
#define VEH_ETH_MAC1                   0x00u
#define VEH_ETH_MAC2                   0x00u
#define VEH_ETH_MAC3                   0x00u
#define VEH_ETH_MAC4                   0x00u
#define VEH_ETH_MAC5                   0x50u
#endif

#define ETH_TYPE_IPV4                  0x0800u
#define ETH_TYPE_ARP                   0x0806u
#define IP_PROTO_UDP                   17u
#define ARP_HTYPE_ETHERNET             0x0001u
#define ARP_PTYPE_IPV4                 0x0800u
#define ARP_OPER_REQUEST               0x0001u
#define ARP_OPER_REPLY                 0x0002u

#define ETH_FRAME_MIN_LEN              60u
#define ETH_MAX_POLL_FRAMES_PER_CALL   4u
#define ETH_RX_OWN_VALUE               0x80000000u
#define ETH_DMASR_RBUS_VALUE           0x00000080u

ETH_HandleTypeDef heth;

/* STM32F4 ETH DMA descriptors and buffers must be 4-byte aligned.
 * __ALIGN_BEGIN/__ALIGN_END are provided by CMSIS/ST HAL and work with ARMCC V5.
 */
__ALIGN_BEGIN ETH_DMADescTypeDef DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;
__ALIGN_BEGIN ETH_DMADescTypeDef DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;
__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END;
__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END;

static uint8_t s_local_mac[6] = {
    VEH_ETH_MAC0, VEH_ETH_MAC1, VEH_ETH_MAC2,
    VEH_ETH_MAC3, VEH_ETH_MAC4, VEH_ETH_MAC5
};

static const uint8_t s_local_ip[4] = {
    VEH_ETH_LOCAL_IP0, VEH_ETH_LOCAL_IP1,
    VEH_ETH_LOCAL_IP2, VEH_ETH_LOCAL_IP3
};

static const uint8_t s_status_ip[4] = {
    VEH_STA_HOST_IP0, VEH_STA_HOST_IP1, VEH_STA_HOST_IP2, VEH_STA_HOST_IP3
};
static const uint8_t s_netmask[4] = {
    VEH_ETH_NETMASK0, VEH_ETH_NETMASK1, VEH_ETH_NETMASK2, VEH_ETH_NETMASK3
};
static uint8_t s_peer_mac[6];
static uint32_t s_peer_tick = 0u;
static uint32_t s_arp_tick = 0u;
static uint8_t s_arp_sent = 0u;
static uint16_t s_ip_id = 0u;
volatile uint8_t dbg_sta_target_ip[4];
volatile uint16_t dbg_sta_target_port = VEH_STA_UDP_PORT;
volatile uint8_t dbg_sta_peer_mac[6];
volatile uint32_t dbg_sta_peer_valid = 0u;
volatile uint32_t dbg_sta_arp_request_cnt = 0u;
volatile uint32_t dbg_sta_tx_result = 1u;

volatile uint32_t dbg_eth_init_step = 0u;
volatile uint32_t dbg_eth_init_ret = 0u;
volatile uint32_t dbg_eth_ready = 0u;
volatile uint32_t dbg_eth_rx_frame_cnt = 0u;
volatile uint32_t dbg_eth_rx_udp_cnt = 0u;
volatile uint32_t dbg_eth_rx_cmd_cnt = 0u;
volatile uint32_t dbg_eth_cmd_ok_cnt = 0u;
volatile uint32_t dbg_eth_cmd_checksum_err_cnt = 0u;
volatile uint32_t dbg_eth_cmd_header_err_cnt = 0u;
volatile uint32_t dbg_eth_arp_req_cnt = 0u;
volatile uint32_t dbg_eth_arp_reply_cnt = 0u;
volatile uint32_t dbg_eth_tx_ok_cnt = 0u;
volatile uint32_t dbg_eth_tx_fail_cnt = 0u;
volatile uint32_t dbg_eth_last_rx_len = 0u;
volatile uint8_t  dbg_eth_last_cmd[26];
volatile uint8_t  dbg_eth_last_src_ip[4];
volatile uint16_t dbg_eth_last_src_port = 0u;

static uint16_t rd16be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t rd32be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
           ((uint32_t)p[3]);
}

static void wr16be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)((v >> 8) & 0xFFu);
    p[1] = (uint8_t)(v & 0xFFu);
}

static void wr32be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)((v >> 24) & 0xFFu);
    p[1] = (uint8_t)((v >> 16) & 0xFFu);
    p[2] = (uint8_t)((v >> 8) & 0xFFu);
    p[3] = (uint8_t)(v & 0xFFu);
}

static uint8_t ip_is_local_or_broadcast(const uint8_t *ip)
{
    if (memcmp(ip, s_local_ip, 4u) == 0)
    {
        return 1u;
    }
    if ((ip[0] == 255u) && (ip[1] == 255u) &&
        (ip[2] == 255u) && (ip[3] == 255u))
    {
        return 1u;
    }
    return 0u;
}

static uint32_t checksum_sum22(const uint8_t *p)
{
    uint32_t sum = 0u;
    uint8_t i;
    for (i = 0u; i < 22u; i++)
    {
        sum += p[i];
    }
    return sum;
}

static uint8_t eth_send_frame(const uint8_t *frame, uint16_t len)
{
    uint16_t tx_len = len;

    if ((dbg_eth_ready == 0u) || (frame == 0) || (len == 0u))
    {
        dbg_eth_tx_fail_cnt++;
        return 0u;
    }

    if (tx_len < ETH_FRAME_MIN_LEN)
    {
        tx_len = ETH_FRAME_MIN_LEN;
    }
    if (tx_len > ETH_TX_BUF_SIZE)
    {
        dbg_eth_tx_fail_cnt++;
        return 0u;
    }

    /* Check ownership BEFORE writing. A busy DMA buffer must remain untouched. */
    if ((heth.TxDesc == 0) || ((heth.TxDesc->Status & ETH_DMATXDESC_OWN) != 0u))
    {
        dbg_eth_tx_fail_cnt++;
        return 0u;
    }
    memset((void *)heth.TxDesc->Buffer1Addr, 0, tx_len);
    memcpy((void *)heth.TxDesc->Buffer1Addr, frame, len);

    __DMB();
    if (HAL_ETH_TransmitFrame(&heth, tx_len) == HAL_OK)
    {
        dbg_eth_tx_ok_cnt++;
        return 1u;
    }

    dbg_eth_tx_fail_cnt++;
    return 0u;
}

static void eth_send_arp_reply(const uint8_t *rx)
{
    uint8_t tx[60];
    const uint8_t *sender_mac;
    const uint8_t *sender_ip;

    sender_mac = &rx[22];
    sender_ip  = &rx[28];

    memset(tx, 0, sizeof(tx));

    memcpy(&tx[0], sender_mac, 6u);       /* Ethernet dst */
    memcpy(&tx[6], s_local_mac, 6u);      /* Ethernet src */
    wr16be(&tx[12], ETH_TYPE_ARP);

    wr16be(&tx[14], ARP_HTYPE_ETHERNET);
    wr16be(&tx[16], ARP_PTYPE_IPV4);
    tx[18] = 6u;
    tx[19] = 4u;
    wr16be(&tx[20], ARP_OPER_REPLY);

    memcpy(&tx[22], s_local_mac, 6u);     /* sender hw */
    memcpy(&tx[28], s_local_ip, 4u);      /* sender ip */
    memcpy(&tx[32], sender_mac, 6u);      /* target hw */
    memcpy(&tx[38], sender_ip, 4u);       /* target ip */

    if (eth_send_frame(tx, 42u) != 0u)
    {
        dbg_eth_arp_reply_cnt++;
    }
}

static void eth_process_arp(const uint8_t *frame, uint16_t len)
{
    uint16_t oper;

    if (len < 42u)
    {
        return;
    }

    if ((rd16be(&frame[14]) != ARP_HTYPE_ETHERNET) ||
        (rd16be(&frame[16]) != ARP_PTYPE_IPV4) ||
        (frame[18] != 6u) || (frame[19] != 4u))
    {
        return;
    }

    oper = rd16be(&frame[20]);
    /* Learn only the configured host, from ARP directed at this local IP.
     * For replies additionally require this board's target MAC. */
    if (((oper == ARP_OPER_REQUEST) ||
         ((oper == ARP_OPER_REPLY) && (memcmp(&frame[32], s_local_mac, 6u) == 0))) &&
        (memcmp(&frame[28], s_status_ip, 4u) == 0) &&
        (memcmp(&frame[38], s_local_ip, 4u) == 0) &&
        (memcmp(&frame[6], &frame[22], 6u) == 0) &&
        ((frame[22] & 1u) == 0u) &&
        ((frame[22] | frame[23] | frame[24] | frame[25] | frame[26] | frame[27]) != 0u))
    {
        memcpy(s_peer_mac, &frame[22], 6u);
        memcpy((void *)dbg_sta_peer_mac, s_peer_mac, 6u);
        s_peer_tick = HAL_GetTick();
        dbg_sta_peer_valid = 1u;
    }
    if ((oper == ARP_OPER_REQUEST) && (memcmp(&frame[38], s_local_ip, 4u) == 0))
    {
        dbg_eth_arp_req_cnt++;
        eth_send_arp_reply(frame);
    }
}

static void eth_accept_cmd26(const uint8_t *cmd)
{
    uint32_t calc;
    uint32_t recv;

    dbg_eth_rx_cmd_cnt++;
    memcpy((void *)dbg_eth_last_cmd, cmd, 26u);

    if (memcmp(cmd, "cmd__", 5u) != 0)
    {
        dbg_eth_cmd_header_err_cnt++;
        return;
    }

    calc = checksum_sum22(cmd);
    recv = rd32be(&cmd[VEH_CMD_CHECKSUM]);

    if (calc != recv)
    {
        dbg_eth_cmd_checksum_err_cnt++;
        return;
    }

    if (Vehicle_ParseCmd26(cmd, VEH_CMD_LEN) != 0u)
    {
        VehicleStatus_OnUpperCmd(VEH_SOURCE_UDP, cmd[VEH_CMD_COUNTER]);
        dbg_eth_cmd_ok_cnt++;
    }
}

static void eth_process_udp_payload(const uint8_t *payload, uint16_t payload_len)
{
    uint16_t i;

    if (payload_len < VEH_CMD_LEN)
    {
        return;
    }

    /* Normal case: exactly one 26-byte frame. */
    if ((payload_len == VEH_CMD_LEN) && (memcmp(payload, "cmd__", 5u) == 0))
    {
        eth_accept_cmd26(payload);
        return;
    }

    /* Robust scan: allow payload to contain padding or multiple frames. */
    for (i = 0u; (uint16_t)(i + VEH_CMD_LEN) <= payload_len; i++)
    {
        if (memcmp(&payload[i], "cmd__", 5u) == 0)
        {
            eth_accept_cmd26(&payload[i]);
            i = (uint16_t)(i + VEH_CMD_LEN - 1u);
        }
    }
}

static void eth_process_ipv4_udp(const uint8_t *frame, uint16_t len)
{
    const uint8_t *ip;
    const uint8_t *udp;
    const uint8_t *payload;
    uint8_t ihl;
    uint16_t total_len;
    uint16_t udp_len;
    uint16_t dst_port;

    if (len < 42u)
    {
        return;
    }

    ip = &frame[14];

    if ((ip[0] >> 4) != 4u)
    {
        return;
    }

    ihl = (uint8_t)((ip[0] & 0x0Fu) * 4u);
    if ((ihl < 20u) || ((uint16_t)(14u + ihl + 8u) > len))
    {
        return;
    }

    total_len = rd16be(&ip[2]);
    if ((total_len < (uint16_t)(ihl + 8u)) || ((uint32_t)14u + total_len > len))
    {
        return;
    }

    /* Ignore fragmented packets. */
    if ((rd16be(&ip[6]) & 0x3FFFu) != 0u)
    {
        return;
    }

    if (ip[9] != IP_PROTO_UDP)
    {
        return;
    }

    if (ip_is_local_or_broadcast(&ip[16]) == 0u)
    {
        return;
    }

    memcpy((void *)dbg_eth_last_src_ip, &ip[12], 4u);

    udp = &ip[ihl];
    udp_len = rd16be(&udp[4]);
    dst_port = rd16be(&udp[2]);
    dbg_eth_last_src_port = rd16be(&udp[0]);

    if ((udp_len < 8u) || ((uint32_t)ihl + udp_len > total_len))
        return;
    if (dst_port == OTA_UPDATE_PORT)
    {
        if (rd16be(udp) == OTA_UPDATE_PORT &&
            memcmp(&ip[12], s_status_ip, 4u) == 0 &&
            memcmp(&ip[16], s_local_ip, 4u) == 0)
            OtaUpdate_Receive(&udp[8], (uint16_t)(udp_len - 8u));
        return;
    }
    if (OtaUpdate_InMaintenance()) return;
#if (VEH_ETH_ACCEPT_ANY_UDP_PORT == 0u)
    if (dst_port != VEH_ETH_UDP_PORT) return;
#endif

    dbg_eth_rx_udp_cnt++;
    payload = &udp[8];
    eth_process_udp_payload(payload, (uint16_t)(udp_len - 8u));
}

static void eth_release_rx_descriptors(void)
{
    ETH_DMADescTypeDef *dmarxdesc;
    uint32_t i;

    dmarxdesc = heth.RxFrameInfos.FSRxDesc;

    for (i = 0u; i < heth.RxFrameInfos.SegCount; i++)
    {
        dmarxdesc->Status = ETH_RX_OWN_VALUE;
        dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
    }

    heth.RxFrameInfos.SegCount = 0u;

    if ((heth.Instance->DMASR & ETH_DMASR_RBUS_VALUE) != 0u)
    {
        heth.Instance->DMASR = ETH_DMASR_RBUS_VALUE;
        heth.Instance->DMARPDR = 0u;
    }
}

/* Each service call advances at most one phase. Deadlines use HAL ticks. */
#define ETH_PHY_RESTART_AUTONEG 0x0200u

enum EthPhase {
    ETH_RETRY, ETH_MAC_RESET, ETH_PHY_RESET_WRITE, ETH_PHY_RESET_READ,
    ETH_PHY_AUTONEG, ETH_PHY_LINK, ETH_PHY_SPEED, ETH_MAC_ENABLE,
    ETH_FIFO_FLUSH, ETH_RUNNING, ETH_CHECK_LINK
};

static enum EthPhase s_eth_phase;
static uint32_t s_eth_due;
static uint32_t s_eth_timeout;
static uint32_t s_mdio_timeout;
volatile uint32_t dbg_eth_retries;
volatile uint32_t dbg_eth_failed_phase;

static void eth_phase(enum EthPhase phase)
{
    s_eth_phase = phase;
    dbg_eth_init_step = (uint32_t)phase;
}

static void eth_retry(uint32_t now)
{
    dbg_eth_failed_phase = (uint32_t)s_eth_phase;
    dbg_eth_retries++;
    dbg_eth_init_ret = 1u;
    dbg_eth_ready = 0u;
    dbg_sta_peer_valid = 0u;
    s_arp_sent = 0u;
    __HAL_RCC_ETHMAC_FORCE_RESET();
    __HAL_RCC_ETHMAC_RELEASE_RESET();
    s_eth_due = now + 1000u;
    eth_phase(ETH_RETRY);
}

static void eth_mdio(uint32_t reg, uint16_t value, uint32_t write,
                     enum EthPhase phase, uint32_t now)
{
    if (write != 0u) ETH->MACMIIDR = value;
    /* HCLK is 16 MHz on this board, giving a 1 MHz MDC. */
    ETH->MACMIIAR = ETH_MACMIIAR_CR_Div16 | (LAN8720_PHY_ADDRESS << 11u) |
                   (reg << 6u) | write | ETH_MACMIIAR_MB;
    s_mdio_timeout = now + 20u;
    eth_phase(phase);
}

void BSP_ETH_Service(uint32_t now)
{
    if ((VEH_ETH_ENABLE == 0u) || ((int32_t)(now - s_eth_due) < 0)) return;

    if ((s_eth_phase >= ETH_PHY_RESET_WRITE && s_eth_phase <= ETH_PHY_SPEED) ||
        s_eth_phase == ETH_CHECK_LINK)
    {
        if ((ETH->MACMIIAR & ETH_MACMIIAR_MB) != 0u)
        {
            if ((int32_t)(now - s_mdio_timeout) >= 0) eth_retry(now);
            return;
        }
    }

    switch (s_eth_phase)
    {
    case ETH_RETRY:
        memset(&heth, 0, sizeof(heth));
        heth.Instance = ETH;
        heth.Init.RxMode = ETH_RXPOLLING_MODE;
        HAL_ETH_MspInit(&heth);
        ETH->DMABMR |= ETH_DMABMR_SR;
        s_eth_timeout = now + 500u;
        eth_phase(ETH_MAC_RESET);
        break;
    case ETH_MAC_RESET:
        if ((ETH->DMABMR & ETH_DMABMR_SR) == 0u)
        {
            s_eth_timeout = now + 500u;
            eth_mdio(PHY_BCR, PHY_RESET, ETH_MACMIIAR_MW, ETH_PHY_RESET_WRITE, now);
        }
        else if ((int32_t)(now - s_eth_timeout) >= 0) eth_retry(now);
        break;
    case ETH_PHY_RESET_WRITE:
        eth_mdio(PHY_BCR, 0u, 0u, ETH_PHY_RESET_READ, now);
        break;
    case ETH_PHY_RESET_READ:
        if ((ETH->MACMIIDR & PHY_RESET) == 0u)
        {
            s_eth_timeout = now + 5000u;
            eth_mdio(PHY_BCR, PHY_AUTONEGOTIATION | ETH_PHY_RESTART_AUTONEG,
                     ETH_MACMIIAR_MW, ETH_PHY_AUTONEG, now);
        }
        else if ((int32_t)(now - s_eth_timeout) >= 0) eth_retry(now);
        else
        {
            s_eth_due = now + 2u;
            eth_phase(ETH_PHY_RESET_WRITE);
        }
        break;
    case ETH_PHY_AUTONEG:
        eth_mdio(PHY_BSR, 0u, 0u, ETH_PHY_LINK, now);
        s_eth_due = now + 10u;
        break;
    case ETH_PHY_LINK:
        if ((ETH->MACMIIDR & (PHY_LINKED_STATUS | PHY_AUTONEGO_COMPLETE)) ==
                (PHY_LINKED_STATUS | PHY_AUTONEGO_COMPLETE))
            eth_mdio(PHY_SR, 0u, 0u, ETH_PHY_SPEED, now);
        else if ((int32_t)(now - s_eth_timeout) >= 0) eth_retry(now);
        else eth_phase(ETH_PHY_AUTONEG);
        break;
    case ETH_PHY_SPEED:
    {
        uint32_t phy = ETH->MACMIIDR;
        ETH->MACCR = ((phy & PHY_SPEED_STATUS) ? ETH_SPEED_10M : ETH_SPEED_100M) |
                     ((phy & PHY_DUPLEX_STATUS) ? ETH_MODE_FULLDUPLEX : ETH_MODE_HALFDUPLEX);
        ETH->MACA0HR = (uint32_t)s_local_mac[4] | ((uint32_t)s_local_mac[5] << 8u);
        ETH->MACA0LR =
                      (uint32_t)s_local_mac[0] | ((uint32_t)s_local_mac[1] << 8u) |
                      ((uint32_t)s_local_mac[2] << 16u) | ((uint32_t)s_local_mac[3] << 24u);
        ETH->DMAOMR = ETH_DMAOMR_RSF | ETH_DMAOMR_TSF | ETH_DMAOMR_OSF;
        ETH->DMABMR = ETH_DMABMR_EDE | ETH_DMABMR_FB |
                      ETH_RXDMABURSTLENGTH_32BEAT | ETH_TXDMABURSTLENGTH_32BEAT | ETH_DMABMR_USP;
        HAL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
        HAL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
        s_eth_due = now + 2u;
        eth_phase(ETH_MAC_ENABLE);
        break;
    }
    case ETH_MAC_ENABLE:
        ETH->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;
        ETH->DMAOMR |= ETH_DMAOMR_FTF;
        s_eth_timeout = now + 20u;
        eth_phase(ETH_FIFO_FLUSH);
        break;
    case ETH_FIFO_FLUSH:
        if ((ETH->DMAOMR & ETH_DMAOMR_FTF) == 0u)
        {
            __DMB();
            ETH->DMAOMR |= ETH_DMAOMR_ST | ETH_DMAOMR_SR;
            dbg_eth_init_ret = 0u;
            dbg_eth_ready = 1u;
            s_eth_due = now + 250u;
            eth_phase(ETH_RUNNING);
        }
        else if ((int32_t)(now - s_eth_timeout) >= 0) eth_retry(now);
        break;
    case ETH_RUNNING:
        eth_mdio(PHY_BSR, 0u, 0u, ETH_CHECK_LINK, now);
        break;
    case ETH_CHECK_LINK:
        if ((ETH->MACMIIDR & PHY_LINKED_STATUS) == 0u) eth_retry(now);
        else
        {
            s_eth_due = now + 250u;
            eth_phase(ETH_RUNNING);
        }
        break;
    }
}

uint8_t BSP_ETH_IsReady(void)
{
    return (uint8_t)(dbg_eth_ready != 0u);
}

static uint32_t internet_sum(const uint8_t *p, uint16_t len, uint32_t sum)
{
    while (len >= 2u)
    {
        sum += rd16be(p);
        p += 2;
        len = (uint16_t)(len - 2u);
    }
    if (len != 0u) sum += (uint32_t)p[0] << 8;
    return sum;
}

static uint16_t internet_finish(uint32_t sum)
{
    while ((sum >> 16) != 0u) sum = (sum & 0xFFFFu) + (sum >> 16);
    return (uint16_t)~sum;
}

static void eth_request_status_arp(uint32_t now)
{
    uint8_t tx[60];
    if ((s_arp_sent != 0u) && ((uint32_t)(now - s_arp_tick) < VEH_STA_ARP_RETRY_MS))
        return;
    /* Rate-limit attempts even if DMA is busy; never wait in the control loop. */
    s_arp_tick = now;
    s_arp_sent = 1u;
    memset(tx, 0, sizeof(tx));
    memset(tx, 0xFF, 6u);
    memcpy(&tx[6], s_local_mac, 6u);
    wr16be(&tx[12], ETH_TYPE_ARP);
    wr16be(&tx[14], ARP_HTYPE_ETHERNET);
    wr16be(&tx[16], ARP_PTYPE_IPV4);
    tx[18] = 6u;
    tx[19] = 4u;
    wr16be(&tx[20], ARP_OPER_REQUEST);
    memcpy(&tx[22], s_local_mac, 6u);
    memcpy(&tx[28], s_local_ip, 4u);
    memcpy(&tx[38], s_status_ip, 4u);
    if (eth_send_frame(tx, 42u) != 0u) dbg_sta_arp_request_cnt++;
}

static uint32_t eth_send_telemetry(const uint8_t *payload, uint16_t len, uint16_t port)
{
    uint8_t tx[42u + VEH_STA_LEN];
    uint8_t i;
    uint16_t udp_checksum;
    uint32_t sum;
    uint32_t now;
    if ((payload == 0) || (len == 0u) || (len > VEH_STA_LEN))
    {
        return 4u;
    }
    if (dbg_eth_ready == 0u)
    {
        return 1u;
    }
    for (i = 0u; i < 4u; i++)
    {
        if ((s_local_ip[i] & s_netmask[i]) != (s_status_ip[i] & s_netmask[i]))
        {
            return 6u;
        }
    }
    now = HAL_GetTick();
    if ((dbg_sta_peer_valid != 0u) &&
        ((uint32_t)(now - s_peer_tick) >= VEH_STA_ARP_CACHE_MS))
    {
        dbg_sta_peer_valid = 0u;
        s_arp_sent = 0u;
    }
    if (dbg_sta_peer_valid == 0u)
    {
        eth_request_status_arp(now);
        return 2u;
    }
    if ((heth.TxDesc == 0) || ((heth.TxDesc->Status & ETH_DMATXDESC_OWN) != 0u))
    {
        return 3u;
    }
    memset(tx, 0, sizeof(tx));
    memcpy(&tx[0], s_peer_mac, 6u);
    memcpy(&tx[6], s_local_mac, 6u);
    wr16be(&tx[12], ETH_TYPE_IPV4);
    tx[14] = 0x45u;
    wr16be(&tx[16], (uint16_t)(28u + len));
    wr16be(&tx[18], s_ip_id++);
    wr16be(&tx[20], 0x4000u); /* Don't fragment */
    tx[22] = 64u;
    tx[23] = IP_PROTO_UDP;
    memcpy(&tx[26], s_local_ip, 4u);
    memcpy(&tx[30], s_status_ip, 4u);
    wr16be(&tx[24], internet_finish(internet_sum(&tx[14], 20u, 0u)));
    wr16be(&tx[34], port);
    wr16be(&tx[36], port);
    wr16be(&tx[38], (uint16_t)(8u + len));
    memcpy(&tx[42], payload, len);
    /* UDP checksum includes IPv4 pseudo-header and odd-byte padding. */
    sum = internet_sum(&tx[26], 8u, 0u);
    sum += IP_PROTO_UDP + (uint32_t)(8u + len);
    udp_checksum = internet_finish(internet_sum(&tx[34], (uint16_t)(8u + len), sum));
    if (udp_checksum == 0u) udp_checksum = 0xFFFFu;
    wr16be(&tx[40], udp_checksum);
    if (eth_send_frame(tx, (uint16_t)(42u + len)) != 0u)
    {
        return 0u;
    }
    return 5u;
}

uint8_t BSP_ETH_SendStatus(const uint8_t *payload, uint16_t len)
{
    memcpy((void *)dbg_sta_target_ip, s_status_ip, 4u);
    dbg_sta_target_port = VEH_STA_UDP_PORT;
    dbg_sta_tx_result = (len == VEH_STA_LEN) ?
        eth_send_telemetry(payload, len, VEH_STA_UDP_PORT) : 4u;
    return (uint8_t)(dbg_sta_tx_result == 0u);
}

uint8_t BSP_ETH_SendTelemetry(const uint8_t *payload, uint16_t len, uint16_t port)
{
    return (uint8_t)(eth_send_telemetry(payload, len, port) == 0u);
}

void BSP_ETH_Process(void)
{
#if (VEH_ETH_ENABLE != 0u)
    uint8_t poll_cnt;
    uint16_t frame_len;
    uint8_t *frame;
    uint16_t eth_type;

    if (dbg_eth_ready == 0u)
    {
        return;
    }

    for (poll_cnt = 0u; poll_cnt < ETH_MAX_POLL_FRAMES_PER_CALL; poll_cnt++)
    {
        if (HAL_ETH_GetReceivedFrame(&heth) != HAL_OK)
        {
            return;
        }

        frame_len = (uint16_t)heth.RxFrameInfos.length;
        frame = (uint8_t *)heth.RxFrameInfos.buffer;
        dbg_eth_last_rx_len = frame_len;
        dbg_eth_rx_frame_cnt++;

        if ((frame != 0) && (frame_len >= 14u) &&
            (heth.RxFrameInfos.length <= ETH_RX_BUF_SIZE) &&
            (heth.RxFrameInfos.SegCount == 1u))
        {
            eth_type = rd16be(&frame[12]);
            if (eth_type == ETH_TYPE_ARP)
            {
                eth_process_arp(frame, frame_len);
            }
            else if (eth_type == ETH_TYPE_IPV4)
            {
                eth_process_ipv4_udp(frame, frame_len);
            }
            else
            {
                /* Ignore other Ethernet types. */
            }
        }

        eth_release_rx_descriptors();
    }
#endif
}

void HAL_ETH_MspInit(ETH_HandleTypeDef *heth_arg)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    (void)heth_arg;

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

#ifdef __HAL_RCC_ETHMAC_CLK_ENABLE
    __HAL_RCC_ETHMAC_CLK_ENABLE();
    __HAL_RCC_ETHMACTX_CLK_ENABLE();
    __HAL_RCC_ETHMACRX_CLK_ENABLE();
#else
    __HAL_RCC_ETH_CLK_ENABLE();
#endif

    /* Select RMII interface. */
    SYSCFG->PMC |= SYSCFG_PMC_MII_RMII_SEL;

    /* PA1=REF_CLK, PA2=MDIO, PA7=CRS_DV */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PC1=MDC, PC4=RXD0, PC5=RXD1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PG11=TX_EN, PG13=TXD0, PG14=TXD1 */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
}
