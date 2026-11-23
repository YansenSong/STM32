#include "remote_control.h"
#include "bsp_uart.h"
#include "vehicle_control.h"
#include "vehicle_config.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#ifndef VEH_REMOTE_ENABLE
#define VEH_REMOTE_ENABLE 1u
#endif

volatile uint32_t dbg_remote_rx_byte_cnt = 0u;
volatile uint32_t dbg_remote_frame_cnt = 0u;
volatile uint32_t dbg_remote_parse_drop_cnt = 0u;
volatile uint32_t dbg_remote_timeout_cnt = 0u;
volatile uint32_t dbg_remote_override_cnt = 0u;
volatile uint32_t dbg_remote_safe_cnt = 0u;
volatile uint16_t dbg_remote_ch_us[16];
volatile uint8_t  dbg_remote_flags = 0u;
volatile uint8_t  dbg_remote_end_byte = 0u;
volatile uint8_t  dbg_remote_active = 0u;
volatile uint8_t  dbg_remote_armed = 0u;
volatile uint8_t  dbg_remote_brake_on = 0u;
volatile uint8_t  dbg_remote_fixed_speed_on = 0u;
volatile int16_t  dbg_remote_eps_raw = 0;
volatile int16_t  dbg_remote_v_mps_x1000 = 0;
volatile int16_t  dbg_remote_pressure_x100 = 0;

static uint8_t  s_frame[25];
static uint8_t  s_pos = 0u;
static uint8_t  s_has_frame = 0u;
static uint32_t s_last_frame_tick = 0u;
static uint16_t s_ch_us[16];
static uint8_t  s_flags = 0u;
static uint8_t  s_end_byte = 0u;

static uint8_t clamp_u8_channel(uint8_t ch)
{
    if (ch < 1u) return 1u;
    if (ch > 16u) return 16u;
    return ch;
}

static uint16_t sbus_raw_to_us(uint16_t raw)
{
    int32_t us;
    us = 1000 + (((int32_t)raw - 172) * 1000) / (1811 - 172);
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    return (uint16_t)us;
}

static uint16_t sbus_get_raw_channel(const uint8_t *data, uint8_t ch)
{
    uint16_t bit_index;
    uint8_t byte_index;
    uint8_t bit_offset;
    uint32_t value;

    bit_index = (uint16_t)ch * 11u;
    byte_index = (uint8_t)(bit_index >> 3);
    bit_offset = (uint8_t)(bit_index & 0x07u);

    value = data[byte_index];
    if ((uint8_t)(byte_index + 1u) < 22u)
    {
        value |= ((uint32_t)data[byte_index + 1u] << 8);
    }
    if ((uint8_t)(byte_index + 2u) < 22u)
    {
        value |= ((uint32_t)data[byte_index + 2u] << 16);
    }

    value = (value >> bit_offset) & 0x07FFu;
    return (uint16_t)value;
}

static uint8_t parse_sbus_frame_25(const uint8_t *frame)
{
    uint8_t i;

    if ((frame[0] != 0x0Fu) || ((frame[23] & 0xF0u) != 0u) ||
        ((frame[24] != 0u) && ((frame[24] & 0xCFu) != 0x04u)))
    {
        return 0u;
    }

    for (i = 0u; i < 16u; i++)
    {
        s_ch_us[i] = sbus_raw_to_us(sbus_get_raw_channel(&frame[1], i));
        dbg_remote_ch_us[i] = s_ch_us[i];
    }

    s_flags = frame[23];
    s_end_byte = frame[24];
    dbg_remote_flags = s_flags;
    dbg_remote_end_byte = s_end_byte;
    s_last_frame_tick = HAL_GetTick();
    s_has_frame = 1u;
    dbg_remote_frame_cnt++;

    return 1u;
}

static void feed_remote_byte(uint8_t ch)
{
    dbg_remote_rx_byte_cnt++;

    if (s_pos == 0u)
    {
        if (ch != 0x0Fu)
        {
            dbg_remote_parse_drop_cnt++;
            return;
        }
    }

    s_frame[s_pos] = ch;
    s_pos++;

    if (s_pos >= 25u)
    {
        if (parse_sbus_frame_25(s_frame) == 0u) dbg_remote_parse_drop_cnt++;
        s_pos = 0u;
    }
}

static uint16_t get_ch_us(uint8_t ch)
{
    ch = clamp_u8_channel(ch);
    return s_ch_us[ch - 1u];
}

static int16_t speed_channel_to_rpm(uint16_t us)
{
    int32_t delta;
    int32_t magnitude;
    int32_t active_span;
    int32_t rpm;

    delta = (int32_t)us - (int32_t)VEH_REMOTE_CENTER_US;
    magnitude = (delta >= 0) ? delta : -delta;

    if (magnitude <= (int32_t)VEH_REMOTE_SPEED_START_THRESHOLD_US)
    {
        return 0;
    }

    if (magnitude > 500)
    {
        magnitude = 500;
    }

    active_span = 500 - (int32_t)VEH_REMOTE_SPEED_START_THRESHOLD_US;
    rpm = (int32_t)VEH_REMOTE_MIN_MOTOR_RPM;
    if (active_span > 0)
    {
        rpm += ((magnitude - (int32_t)VEH_REMOTE_SPEED_START_THRESHOLD_US) *
                ((int32_t)VEH_REMOTE_MAX_MOTOR_RPM - (int32_t)VEH_REMOTE_MIN_MOTOR_RPM)) /
               active_span;
    }

    if (rpm > (int32_t)VEH_REMOTE_MAX_MOTOR_RPM)
    {
        rpm = (int32_t)VEH_REMOTE_MAX_MOTOR_RPM;
    }

    if (delta < 0)
    {
        rpm = -rpm;
    }

#if VEH_REMOTE_SPEED_REVERSE
    rpm = -rpm;
#endif

#if !VEH_REMOTE_ALLOW_REVERSE
    if (rpm < 0)
    {
        rpm = 0;
    }
#endif

    return (int16_t)rpm;
}

static void fill_safe_cmd(VehicleCmd_t *cmd)
{
    memset(cmd, 0, sizeof(*cmd));
    cmd->valid = 1u;
    cmd->version = VEH_CMD_VERSION_VALUE;
    cmd->mode = VEH_MODE_AUTO;
    cmd->flags = 0u;
    cmd->enable_mask = VEH_ENABLE_RT49;
    cmd->v_mps = 0.0f;
    cmd->eps_deg = 0.0f;
    cmd->seb_mode = VEH_SEB_MODE_NONE;
    cmd->seb_value_raw = 0;
    cmd->last_tick = HAL_GetTick();
    dbg_remote_safe_cnt++;
}

void Remote_Init(void)
{
    uint8_t i;
    s_pos = 0u;
    s_has_frame = 0u;
    s_last_frame_tick = 0u;
    s_flags = 0u;
    s_end_byte = 0u;
    for (i = 0u; i < 16u; i++)
    {
        s_ch_us[i] = 1500u;
        dbg_remote_ch_us[i] = 1500u;
    }
    dbg_remote_active = 0u;
    dbg_remote_armed = 0u;
    dbg_remote_brake_on = 0u;
    dbg_remote_fixed_speed_on = 0u;
    dbg_remote_eps_raw = 0;
    dbg_remote_v_mps_x1000 = 0;
    dbg_remote_pressure_x100 = 0;
}

void Remote_Process(void)
{
#if VEH_REMOTE_ENABLE
    uint8_t ch;
    uint8_t discontinuity;
    for (uint32_t budget = 64u; (budget != 0u) && (BSP_UART5_ReadByte(&ch, &discontinuity) != 0u); --budget)
    {
        if (discontinuity != 0u) s_pos = 0u;
        feed_remote_byte(ch);
    }
#endif
}

uint8_t Remote_ShouldOverride(void)
{
#if VEH_REMOTE_ENABLE
    uint32_t now;
    uint8_t timeout;
    uint8_t frame_lost;
    uint8_t failsafe;
    uint16_t arm_us;

    static uint8_t s_remote_takeover_latched = 0u;

    now = HAL_GetTick();

    /* 从来没收到过遥控器数据：不接管上位机/以太网 */
    if (s_has_frame == 0u)
    {
        s_remote_takeover_latched = 0u;
        return 0u;
    }

    timeout = ((uint32_t)(now - s_last_frame_tick) > VEH_REMOTE_TIMEOUT_MS) ? 1u : 0u;
    frame_lost = ((s_flags & 0x04u) != 0u) ? 1u : 0u;
    failsafe = ((s_flags & 0x08u) != 0u) ? 1u : 0u;

    /*
     * 如果遥控器曾经接管过，并且接管过程中出现超时/丢帧/失控，
     * 继续接管，让 Remote_UpdateVehicleCmd() 生成安全停车指令。
     */
    if ((timeout != 0u) || (frame_lost != 0u) || (failsafe != 0u))
    {
        if (s_remote_takeover_latched != 0u)
        {
            return 1u;
        }

        return 0u;
    }

    /*
     * 只有 CH5 总使能打开时，遥控器才接管。
     * CH5 不打开时，即使遥控器在线，也不覆盖 UDP / USART1。
     */
    arm_us = get_ch_us(VEH_REMOTE_ARM_CH);

    if (arm_us >= VEH_REMOTE_ARM_THRESHOLD_US)
    {
        s_remote_takeover_latched = 1u;
        return 1u;
    }

    /*
     * CH5 关闭后，遥控器退出接管，控制权交还给 UDP / USART1。
     */
    s_remote_takeover_latched = 0u;
    return 0u;

#else
    return 0u;
#endif
}

void Remote_UpdateVehicleCmd(void)
{
#if VEH_REMOTE_ENABLE
    VehicleCmd_t cmd;
    uint32_t now;
    uint8_t timeout;
    uint8_t frame_lost;
    uint8_t failsafe;
    uint16_t speed_us;
    uint16_t arm_us;
    uint16_t fixed_speed_us;
    uint8_t armed;
    int16_t speed_rpm;

    now = HAL_GetTick();
    timeout = ((s_has_frame == 0u) || ((uint32_t)(now - s_last_frame_tick) > VEH_REMOTE_TIMEOUT_MS)) ? 1u : 0u;
    frame_lost = ((s_flags & 0x04u) != 0u) ? 1u : 0u;
    failsafe = ((s_flags & 0x08u) != 0u) ? 1u : 0u;

    if ((timeout != 0u) || (frame_lost != 0u) || (failsafe != 0u))
    {
        if (timeout != 0u)
        {
            dbg_remote_timeout_cnt++;
        }
        dbg_remote_active = 0u;
        dbg_remote_armed = 0u;
        dbg_remote_brake_on = 0u;
        dbg_remote_fixed_speed_on = 0u;
        fill_safe_cmd(&cmd);
        __disable_irq();
        g_vehicle_cmd = cmd;
        __enable_irq();
        return;
    }

    dbg_remote_active = 1u;
    dbg_remote_override_cnt++;

    speed_us = get_ch_us(VEH_REMOTE_SPEED_CH);
    arm_us = get_ch_us(VEH_REMOTE_ARM_CH);
    fixed_speed_us = get_ch_us(VEH_REMOTE_FIXED_SPEED_CH);

    armed = (arm_us >= VEH_REMOTE_ARM_THRESHOLD_US) ? 1u : 0u;
    dbg_remote_armed = armed;

    if (armed == 0u)
    {
        dbg_remote_brake_on = 0u;
        dbg_remote_fixed_speed_on = 0u;
        fill_safe_cmd(&cmd);
        __disable_irq();
        g_vehicle_cmd = cmd;
        __enable_irq();
        return;
    }

    if (fixed_speed_us < VEH_REMOTE_FIXED_SPEED_THRESHOLD_US)
    {
        speed_rpm = (int16_t)VEH_REMOTE_FIXED_SPEED_RPM;
        dbg_remote_fixed_speed_on = 1u;
    }
    else
    {
        speed_rpm = speed_channel_to_rpm(speed_us);
        dbg_remote_fixed_speed_on = 0u;
    }
    dbg_remote_brake_on = 0u;

    memset(&cmd, 0, sizeof(cmd));
    cmd.valid = 1u;
    cmd.version = VEH_CMD_VERSION_VALUE;
    cmd.mode = VEH_MODE_AUTO;
    cmd.flags = 0u;
    cmd.enable_mask = VEH_ENABLE_RT49;
    cmd.v_mps = (float)speed_rpm / VEH_RT49_RPM_PER_MPS;
    cmd.eps_deg = 0.0f;
    cmd.seb_mode = VEH_SEB_MODE_NONE;
    cmd.seb_value_raw = 0;
    cmd.last_tick = HAL_GetTick();

    dbg_remote_eps_raw = 0;
    dbg_remote_v_mps_x1000 = (int16_t)(cmd.v_mps * 1000.0f);
    dbg_remote_pressure_x100 = 0;

    __disable_irq();
    g_vehicle_cmd = cmd;
    __enable_irq();
#else
    (void)0;
#endif
}
