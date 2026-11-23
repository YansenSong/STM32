#include "upper_cmd.h"
#include "bsp_uart.h"
#include "vehicle_control.h"
#include "vehicle_status.h"
#include "vehicle_config.h"
#include <string.h>

volatile uint32_t dbg_cmd_found_cnt = 0u;
volatile uint32_t dbg_cmd_ok_cnt = 0u;
volatile uint32_t dbg_cmd_checksum_err_cnt = 0u;
volatile uint32_t dbg_cmd_header_drop_cnt = 0u;
volatile uint8_t  dbg_last_cmd[26];
volatile uint32_t dbg_last_checksum_calc = 0u;
volatile uint32_t dbg_last_checksum_recv = 0u;

static uint8_t s_frame[VEH_CMD_LEN];
static uint8_t s_pos = 0u;
static uint8_t s_collecting = 0u;
static const uint8_t s_header[5] = { 'c', 'm', 'd', '_', '_' };

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

static uint32_t read_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
           ((uint32_t)p[3]);
}

static void reset_parser(void)
{
    s_pos = 0u;
    s_collecting = 0u;
}

static void feed_one_byte(uint8_t ch)
{
    uint32_t calc;
    uint32_t recv;

    if (s_collecting == 0u)
    {
        if (ch == s_header[s_pos])
        {
            s_frame[s_pos] = ch;
            s_pos++;
            if (s_pos == VEH_CMD_HEADER_LEN)
            {
                s_collecting = 1u;
            }
        }
        else
        {
            dbg_cmd_header_drop_cnt++;
            if (ch == s_header[0])
            {
                s_frame[0] = ch;
                s_pos = 1u;
            }
            else
            {
                s_pos = 0u;
            }
        }
        return;
    }

    s_frame[s_pos] = ch;
    s_pos++;

    if (s_pos >= VEH_CMD_LEN)
    {
        dbg_cmd_found_cnt++;
        memcpy((void *)dbg_last_cmd, s_frame, VEH_CMD_LEN);

        calc = checksum_sum22(s_frame);
        recv = read_u32_be(&s_frame[VEH_CMD_CHECKSUM]);
        dbg_last_checksum_calc = calc;
        dbg_last_checksum_recv = recv;

        if (calc == recv)
        {
            if (Vehicle_ParseCmd26(s_frame, VEH_CMD_LEN) != 0u)
            {
                VehicleStatus_OnUpperCmd(VEH_SOURCE_USART1, s_frame[VEH_CMD_COUNTER]);
                dbg_cmd_ok_cnt++;
            }
        }
        else
        {
            dbg_cmd_checksum_err_cnt++;
        }

        reset_parser();
    }
}

void UpperCmd_Init(void)
{
    reset_parser();
    memset((void *)dbg_last_cmd, 0, sizeof(dbg_last_cmd));
}

void UpperCmd_Process(void)
{
    uint8_t ch;
    for (uint32_t budget = 64u; (budget != 0u) && (BSP_UART1_ReadByte(&ch) != 0u); --budget)
    {
        feed_one_byte(ch);
    }
}
