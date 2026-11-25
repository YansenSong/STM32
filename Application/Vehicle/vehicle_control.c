#include "vehicle_control.h"
#include "bsp_can.h"
#include "vehicle_status.h"
#include <string.h>

volatile VehicleCmd_t g_vehicle_cmd;

volatile uint8_t dbg_rt49_tx_data[8];
volatile uint8_t dbg_eps_tx_data[8];
volatile uint8_t dbg_seb_tx_data[8];
volatile uint32_t dbg_rt49_tx_cnt = 0u;
volatile uint32_t dbg_eps_tx_cnt = 0u;
volatile uint32_t dbg_seb_tx_cnt = 0u;
volatile uint32_t dbg_vehicle_20ms_cnt = 0u;
volatile uint32_t dbg_vehicle_timeout_cnt = 0u;
volatile int16_t dbg_rt49_target_rpm = 0;
volatile int16_t dbg_eps_target_deg = 0;

static uint8_t s_rt49_tx[8];
static uint8_t s_eps_tx[8];
static uint8_t s_seb_tx[8];
static uint8_t s_seb_request_pending = 0u;

static int16_t clamp_i16(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return (int16_t)lo;
    if (v > hi) return (int16_t)hi;
    return (int16_t)v;
}

static float clamp_f32(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t read_i32_be(const uint8_t *p)
{
    uint32_t u = ((uint32_t)p[0] << 24) |
                 ((uint32_t)p[1] << 16) |
                 ((uint32_t)p[2] << 8)  |
                 ((uint32_t)p[3]);
    return (int32_t)u;
}

static int16_t read_i16_be(const uint8_t *p)
{
    uint16_t u = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
    return (int16_t)u;
}

static void build_rt49_301(int16_t rpm)
{
    memset(s_rt49_tx, 0, sizeof(s_rt49_tx));
    s_rt49_tx[0] = 0x01u;
    s_rt49_tx[1] = (uint8_t)(((uint16_t)rpm >> 8) & 0xFFu);
    s_rt49_tx[2] = (uint8_t)((uint16_t)rpm & 0xFFu);
    s_rt49_tx[3] = 0x00u;
}

static void build_eps_314(float eps_deg)
{
    int32_t angle_i32;
    int16_t angle;

    eps_deg = clamp_f32(eps_deg, -VEH_EPS_ANGLE_LIMIT_DEG, VEH_EPS_ANGLE_LIMIT_DEG);
    angle_i32 = (eps_deg >= 0.0f) ? (int32_t)(eps_deg + 0.5f) : (int32_t)(eps_deg - 0.5f);
    angle = clamp_i16(angle_i32, -32768, 32767);

    memset(s_eps_tx, 0, sizeof(s_eps_tx));
    s_eps_tx[0] = 0x01u;
    s_eps_tx[1] = (uint8_t)(((uint16_t)angle >> 8) & 0xFFu);
    s_eps_tx[2] = (uint8_t)((uint16_t)angle & 0xFFu);

    dbg_eps_target_deg = angle;
}

static uint8_t pressure_to_seb_signal(float pressure_mpa)
{
    uint16_t sig;
    if (pressure_mpa < 0.0f) pressure_mpa = 0.0f;
    if (pressure_mpa > 12.0f) pressure_mpa = 12.0f;
    sig = (uint16_t)(pressure_mpa / 0.05f + 0.5f);
    if (sig > 255u) sig = 255u;
    return (uint8_t)sig;
}

static void build_seb_7b9(uint8_t seb_mode, int16_t seb_value_raw, uint8_t flags, uint8_t estop)
{
    memset(s_seb_tx, 0, sizeof(s_seb_tx));

    if (estop != 0u)
    {
        /* 急停时使用压力模式（字节 0 为 0x06），字节 3 保存急停压力，每计数代表 0.05 MPa。 */
        s_seb_tx[0] = 0x06u;
        s_seb_tx[3] = pressure_to_seb_signal(VEH_SEB_ESTOP_PRESSURE_MPA);
    }
    else if (((flags & VEH_FLAG_SEB_ALIGN) != 0u) || (seb_mode == VEH_SEB_MODE_ALIGNMENT))
    {
        /* 对齐模式只置位字节 0 的最低位。 */
        s_seb_tx[0] = 0x01u;
    }
    else if (((flags & VEH_FLAG_SEB_AUTOBRAKE) != 0u) || (seb_mode == VEH_SEB_MODE_AUTOBRAKE))
    {
        /* 自动制动模式使用 0x0E，压力写入字节 3。 */
        float pressure_mpa = (float)seb_value_raw / 100.0f;

        s_seb_tx[0] = 0x0Eu;
        s_seb_tx[3] = pressure_to_seb_signal(pressure_mpa);
    }
    else if (seb_mode == VEH_SEB_MODE_PRESSURE)
    {
        /* 压力模式下，输入每计数代表 0.01 MPa，字节 3 每计数代表 0.05 MPa。
         * 例如输入 50 表示 0.5 MPa，编码后字节 3 为 0x0A。 */
        float pressure_mpa = (float)seb_value_raw / 100.0f;

        s_seb_tx[0] = 0x06u;
        s_seb_tx[3] = pressure_to_seb_signal(pressure_mpa);
    }
    else if (seb_mode == VEH_SEB_MODE_STROKE)
    {
        /* 行程模式下，输入每计数代表 0.1 mm。限幅后按 0.05 mm 为单位编码，
         * 高字节写入字节 2，低字节写入字节 3。 */
        float stroke_mm = (float)seb_value_raw / 10.0f;
        int16_t stroke_signal;
        uint16_t stroke_u16;

        if (stroke_mm < -5.0f)
        {
            stroke_mm = -5.0f;
        }

        if (stroke_mm > 27.0f)
        {
            stroke_mm = 27.0f;
        }

        stroke_signal = (int16_t)(stroke_mm / 0.05f);
        stroke_u16 = (uint16_t)stroke_signal;

        s_seb_tx[0] = 0x02u;
        s_seb_tx[2] = (uint8_t)((stroke_u16 >> 8) & 0xFFu);
        s_seb_tx[3] = (uint8_t)(stroke_u16 & 0xFFu);
    }
    else
    {
        /* 其余模式将字节 0 清零。 */
        s_seb_tx[0] = 0x00u;
    }

    /* 当前实现将字节 6 和字节 7 保持为零。 */
    s_seb_tx[6] = 0x00u;
    s_seb_tx[7] = 0x00u;
}

void Vehicle_Init(void)
{
    s_seb_request_pending = 0u;
    memset((void *)&g_vehicle_cmd, 0, sizeof(g_vehicle_cmd));
    build_rt49_301(0);
    build_eps_314(0.0f);
    build_seb_7b9(VEH_SEB_MODE_NONE, 0, 0u, 0u);
    memcpy((void *)dbg_rt49_tx_data, s_rt49_tx, 8u);
    memcpy((void *)dbg_eps_tx_data, s_eps_tx, 8u);
    memcpy((void *)dbg_seb_tx_data, s_seb_tx, 8u);
}

uint8_t Vehicle_ParseCmd26(const uint8_t *buf, uint16_t len)
{
    VehicleCmd_t cmd;

    if ((buf == 0) || (len != VEH_CMD_LEN))
    {
        return 0u;
    }
    if ((memcmp(buf, "cmd__", 5u) != 0) ||
        (buf[VEH_CMD_VERSION] != VEH_CMD_VERSION_VALUE))
    {
        return 0u;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.valid         = 1u;
    cmd.version       = buf[VEH_CMD_VERSION];
    cmd.mode          = buf[VEH_CMD_MODE];
    cmd.flags         = buf[VEH_CMD_FLAGS];
    cmd.enable_mask   = buf[VEH_CMD_ENABLE];
    cmd.v_mps         = (float)read_i32_be(&buf[VEH_CMD_V_RAW]) / 10000.0f;
    /* The upper-computer command carries road-wheel degrees x 10000. */
    cmd.eps_deg       = ((float)read_i32_be(&buf[VEH_CMD_EPS_RAW]) / 10000.0f -
                        VEH_STEERING_WHEEL_OFFSET_DEG) / VEH_STEERING_WHEEL_PER_EPS_DEG;
    cmd.seb_mode      = buf[VEH_CMD_SEB_MODE];
    cmd.seb_value_raw = read_i16_be(&buf[VEH_CMD_SEB_VALUE]);
    cmd.light         = buf[VEH_CMD_LIGHT];
    cmd.counter       = buf[VEH_CMD_COUNTER];
    cmd.last_tick     = HAL_GetTick();

    __disable_irq();
    g_vehicle_cmd = cmd;
    __enable_irq();

    return 1u;
}

void Vehicle_EStop(void)
{
    VehicleCmd_t cmd;
    VehicleStatus_SetControlSource(VEH_SOURCE_INTERNAL);
    memset(&cmd, 0, sizeof(cmd));
    cmd.valid = 1u;
    cmd.version = VEH_CMD_VERSION_VALUE;
    cmd.mode = VEH_MODE_AUTO;
    cmd.flags = VEH_FLAG_ESTOP;
    cmd.enable_mask = (uint8_t)(VEH_ENABLE_RT49 | VEH_ENABLE_EPS | VEH_ENABLE_SEB);
    cmd.v_mps = 0.0f;
    cmd.eps_deg = 0.0f;
    cmd.seb_mode = VEH_SEB_MODE_PRESSURE;
    cmd.seb_value_raw = (int16_t)(VEH_SEB_ESTOP_PRESSURE_MPA * 100.0f);
    cmd.last_tick = HAL_GetTick();

    __disable_irq();
    g_vehicle_cmd = cmd;
    __enable_irq();
}

void Vehicle_20msTask(void)
{
    VehicleCmd_t cmd;
    uint32_t now;
    uint32_t dt;
    uint8_t timeout;
    uint8_t estop;
    uint8_t enabled;
    uint8_t send_seb;
    uint8_t send_eps = 0u;
    uint8_t cancel_seb;
    int16_t rt49_rpm;
    float eps_deg;

    __disable_irq();
    cmd = g_vehicle_cmd;
    __enable_irq();

    now = HAL_GetTick();
    dt = now - cmd.last_tick;
    timeout = ((cmd.valid == 0u) || (dt > VEH_CMD_TIMEOUT_MS)) ? 1u : 0u;
    estop = ((cmd.flags & VEH_FLAG_ESTOP) != 0u) ? 1u : 0u;
    enabled = ((cmd.mode == VEH_MODE_AUTO) || (cmd.mode == VEH_MODE_DEBUG)) ? 1u : 0u;
    send_seb = 0u;
    rt49_rpm = 0;
    eps_deg = 0.0f;

    dbg_vehicle_20ms_cnt++;

    if (timeout != 0u)
    {
        dbg_vehicle_timeout_cnt++;
    }

    if ((timeout == 0u) && (enabled != 0u) && (estop == 0u))
    {
        if ((cmd.enable_mask & VEH_ENABLE_RT49) != 0u)
        {
            float rpm_f = cmd.v_mps * VEH_RT49_RPM_PER_MPS;
            int32_t rpm_i = (rpm_f >= 0.0f) ? (int32_t)(rpm_f + 0.5f) : (int32_t)(rpm_f - 0.5f);
            rt49_rpm = clamp_i16(rpm_i, -VEH_RT49_RPM_LIMIT, VEH_RT49_RPM_LIMIT);
        }

        if ((cmd.enable_mask & VEH_ENABLE_EPS) != 0u)
        {
            send_eps = 1u;
            eps_deg = ((cmd.flags & VEH_FLAG_EPS_CENTER) != 0u) ? 0.0f : cmd.eps_deg;
        }

        if ((cmd.enable_mask & VEH_ENABLE_SEB) != 0u)
        {
            send_seb = 1u;
        }
    }
    else
    {
        rt49_rpm = 0;
        eps_deg = 0.0f;
        if (estop != 0u)
        {
            send_seb = 1u;
        }
    }

    dbg_rt49_target_rpm = rt49_rpm;

    build_rt49_301(rt49_rpm);
    build_eps_314(eps_deg);
    cancel_seb = (uint8_t)((send_seb == 0u) && (s_seb_request_pending != 0u));
    if (send_seb != 0u)
        build_seb_7b9(cmd.seb_mode, cmd.seb_value_raw, cmd.flags, estop);
    else
        build_seb_7b9(VEH_SEB_MODE_NONE, 0, 0u, 0u);

    memcpy((void *)dbg_rt49_tx_data, s_rt49_tx, 8u);
    memcpy((void *)dbg_eps_tx_data, s_eps_tx, 8u);
    memcpy((void *)dbg_seb_tx_data, s_seb_tx, 8u);

    dbg_rt49_tx_cnt++;
    if (send_eps != 0u) dbg_eps_tx_cnt++;
    if ((send_seb != 0u) || (cancel_seb != 0u))
    {
        dbg_seb_tx_cnt++;
    }

#if VEH_REAL_CAN_TX_ENABLE
    (void)BSP_CAN2_SendStd(VEH_CAN_ID_RT49_CMD, s_rt49_tx, 8u);
    if (send_eps != 0u)
        (void)BSP_CAN1_SendStd(VEH_CAN_ID_EPS_CMD, s_eps_tx, 8u);
    if (((send_seb != 0u) || (cancel_seb != 0u)) &&
        (BSP_CAN2_SendStd(VEH_CAN_ID_SEB_REQ, s_seb_tx, 8u) != 0u))
    {
        s_seb_request_pending = send_seb;
    }
#endif
}
