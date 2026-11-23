#include "vehicle_status.h"
#include "vehicle_config.h"
#include "vehicle_control.h"
#include "remote_control.h"
#include "bsp_eth.h"
#include "vehicle_feedback.h"
#include <string.h>

volatile VehicleStatus_t g_vehicle_status;
volatile uint32_t dbg_sta_attempt_cnt = 0u;
volatile uint32_t dbg_sta_send_cnt = 0u;
volatile uint32_t dbg_sta_fail_cnt = 0u;
volatile uint32_t dbg_sta_can_valid_flags = 0u;
volatile uint8_t dbg_sta_last_data[VEH_STA_LEN];

static volatile VehicleStatus_t s_feedback;
static volatile uint32_t s_feedback_ticks[5];
static uint8_t s_source = VEH_SOURCE_NONE;
static uint8_t s_last_cmd_counter = 0u;
static uint8_t s_has_upper_cmd = 0u;
static uint32_t s_counter = 0u;
static const uint32_t s_groups[5] = {
    VEH_STA_VALID_SPEED, VEH_STA_VALID_STEERING, VEH_STA_VALID_ENCODER,
    VEH_STA_VALID_ENABLE, VEH_STA_VALID_FAULT
};

static void put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static void put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

void VehicleStatus_Init(void)
{
    uint32_t primask;
    VehicleFeedback_Init();
    primask = __get_PRIMASK();
    __disable_irq();
    memset((void *)&s_feedback, 0, sizeof(s_feedback));
    memset((void *)s_feedback_ticks, 0, sizeof(s_feedback_ticks));
    __set_PRIMASK(primask);
    memset((void *)&g_vehicle_status, 0, sizeof(g_vehicle_status));
    memset((void *)dbg_sta_last_data, 0, sizeof(dbg_sta_last_data));
    s_source = VEH_SOURCE_NONE;
    s_last_cmd_counter = 0u;
    s_has_upper_cmd = 0u;
    s_counter = 0u;
    dbg_sta_attempt_cnt = 0u;
    dbg_sta_send_cnt = 0u;
    dbg_sta_fail_cnt = 0u;
    dbg_sta_can_valid_flags = 0u;
}

void VehicleStatus_OnUpperCmd(uint8_t source, uint8_t counter)
{
    s_source = source;
    s_last_cmd_counter = counter;
    s_has_upper_cmd = 1u;
}

void VehicleStatus_SetControlSource(uint8_t source)
{
    s_source = source;
}

void VehicleStatus_UpdateCANFeedback(const VehicleStatus_t *measured,
                                      uint32_t mask, uint32_t sample_ms)
{
    uint32_t primask;
    uint8_t i;
    if (measured == 0) return;
    mask &= VEH_STA_BASE_CAN_MASK;
    primask = __get_PRIMASK();
    __disable_irq();
    if ((mask & VEH_STA_VALID_SPEED) != 0u)
        s_feedback.actual_speed_x10000 = measured->actual_speed_x10000;
    if ((mask & VEH_STA_VALID_STEERING) != 0u)
        s_feedback.actual_steering_x10000 = measured->actual_steering_x10000;
    if ((mask & VEH_STA_VALID_ENCODER) != 0u)
        s_feedback.encoder_total = measured->encoder_total;
    if ((mask & VEH_STA_VALID_ENABLE) != 0u)
    {
        s_feedback.enable_state = measured->enable_state;
        s_feedback.enable_valid_mask = 7u;
    }
    if ((mask & VEH_STA_VALID_FAULT) != 0u)
    {
        s_feedback.fault_flags = measured->fault_flags;
        s_feedback.fault_valid_mask = 31u;
    }
    for (i = 0u; i < 5u; i++)
        if ((mask & s_groups[i]) != 0u) s_feedback_ticks[i] = sample_ms;
    s_feedback.valid_flags |= mask;
    __set_PRIMASK(primask);
}

void VehicleStatus_InvalidateCANFeedback(uint32_t mask)
{
    uint32_t primask;
    primask = __get_PRIMASK();
    __disable_irq();
    s_feedback.valid_flags &= ~(mask & VEH_STA_CAN_VALID_MASK);
    __set_PRIMASK(primask);
}

static uint8_t status_wire_valid(const VehicleStatus_t *s)
{
    uint8_t valid;
    valid = 0u;
    if ((s->valid_flags & VEH_STA_VALID_ENCODER) != 0u) valid |= 1u;
    if ((s->valid_flags & VEH_STA_VALID_SPEED) != 0u) valid |= 2u;
    if ((s->valid_flags & VEH_STA_VALID_STEERING) != 0u) valid |= 4u;
    if ((s->valid_flags & VEH_STA_VALID_EPS) != 0u) valid |= 8u;
    if (((s->brake_mode == 1u) && ((s->valid_flags & VEH_STA_VALID_SEB_PRESSURE) != 0u)) ||
        ((s->brake_mode == 2u) && ((s->valid_flags & VEH_STA_VALID_SEB_STROKE) != 0u))) valid |= 16u;
    if ((s->valid_flags & VEH_STA_VALID_RT49) != 0u) valid |= 32u;
    if ((s->valid_flags & VEH_STA_VALID_EPS) != 0u) valid |= 64u;
    if ((s->valid_flags & VEH_STA_VALID_SEB) != 0u) valid |= 128u;
    return valid;
}

uint16_t VehicleStatus_Pack(const VehicleStatus_t *status, uint8_t *out,
                            uint16_t capacity)
{
    uint32_t sum;
    uint64_t encoder;
    uint8_t i;
    uint8_t valid;
    if ((status == 0) || (out == 0) || (capacity < VEH_STA_LEN)) return 0u;
    valid = status_wire_valid(status);
    memset(out, 0, VEH_STA_LEN);
    memcpy(out, "sta__", 5u);
    out[5] = VEH_STA_VERSION;
    out[6] = valid;
    out[7] = status->control_source;
    out[8] = (uint8_t)(status->enable_state & status->enable_valid_mask);
    out[9] = (uint8_t)status->counter;
    put32(&out[10], status->timestamp_ms);
    if ((valid & 1u) != 0u)
    {
        encoder = (uint64_t)status->encoder_total;
        put32(&out[14], (uint32_t)(encoder >> 32));
        put32(&out[18], (uint32_t)encoder);
    }
    if ((valid & 2u) != 0u) put32(&out[22], (uint32_t)status->actual_speed_x10000);
    if ((valid & 4u) != 0u) put32(&out[26], (uint32_t)status->actual_steering_x10000);
    /* Raw int16 EPS actual-angle count sign-extended to int32, 1 deg/count. */
    if ((valid & 8u) != 0u) put32(&out[30], (uint32_t)(status->eps_angle_x10000 / 10000));
    out[34] = status->brake_mode;
    if ((valid & 16u) != 0u) put16(&out[35], (uint16_t)status->brake_actual);
    if ((valid & 32u) != 0u)
    {
        out[37] = (uint8_t)status->gear;
        put16(&out[38], status->rt49_status);
    }
    if ((valid & 64u) != 0u) put16(&out[40], status->eps_status);
    if ((valid & 128u) != 0u) put16(&out[42], status->seb_status_word);
    put32(&out[44], status->fault_flags);
    if ((status->valid_flags & VEH_STA_VALID_LAST_CMD) != 0u) out[48] = status->last_cmd_counter;
    /* Reserved49..51 remain zero as required. */
    sum = 0u;
    for (i = 0u; i < VEH_STA_CHECKSUM; i++) sum += out[i];
    put32(&out[52], sum);
    return VEH_STA_LEN;
}

void VehicleStatus_20msTask(void)
{
    VehicleStatus_t status;
    VehicleCmd_t cmd;
    uint32_t now;
    uint32_t primask;
    uint8_t i;
    uint8_t data[VEH_STA_LEN];
    now = HAL_GetTick();
    primask = __get_PRIMASK();
    __disable_irq();
    cmd = g_vehicle_cmd;
    status = s_feedback;
    for (i = 0u; i < 5u; i++)
        if ((uint32_t)(now - s_feedback_ticks[i]) > VEH_STA_FEEDBACK_TIMEOUT_MS)
            status.valid_flags &= ~s_groups[i];
    __set_PRIMASK(primask);
    if ((status.valid_flags & VEH_STA_VALID_ENABLE) == 0u)
    {
        status.enable_state = 0u;
        status.enable_valid_mask = 0u;
    }
    if ((status.valid_flags & VEH_STA_VALID_FAULT) == 0u)
    {
        status.fault_flags = 0u;
        status.fault_valid_mask = 0u;
    }
    VehicleFeedback_FillStatus(&status, now);
    dbg_sta_can_valid_flags = status.valid_flags & VEH_STA_CAN_VALID_MASK;
    status.valid_flags |= VEH_STA_VALID_SOURCE | VEH_STA_VALID_TIMESTAMP;
    status.control_source = s_source;
    status.counter = s_counter++; /* increments per attempt, including failures */
    status.timestamp_ms = now;
    status.supervisor_flags = 0u;
    if ((cmd.valid == 0u) || ((uint32_t)(now - cmd.last_tick) > VEH_CMD_TIMEOUT_MS))
    {
        status.control_source = VEH_SOURCE_NONE;
        status.supervisor_flags |= VEH_SUP_NO_FRESH_CMD;
    }
    if (cmd.valid != 0u)
    {
        status.requested_enable = cmd.enable_mask;
        status.valid_flags |= VEH_STA_VALID_REQUESTED_EN;
        if ((cmd.flags & VEH_FLAG_ESTOP) != 0u)
            status.supervisor_flags |= VEH_SUP_ESTOP_REQUEST;
    }
    if ((cmd.mode != VEH_MODE_AUTO) && (cmd.mode != VEH_MODE_DEBUG))
        status.supervisor_flags |= VEH_SUP_MODE_DISABLED;
    if ((s_source == VEH_SOURCE_REMOTE) && (dbg_remote_active == 0u))
        status.supervisor_flags |= VEH_SUP_REMOTE_SAFE_CMD;
#if (VEH_REAL_CAN_TX_ENABLE == 0u)
    status.supervisor_flags |= VEH_SUP_CAN_TX_DISABLED;
#endif
    if (s_has_upper_cmd != 0u)
    {
        status.last_cmd_counter = s_last_cmd_counter;
        status.valid_flags |= VEH_STA_VALID_LAST_CMD;
    }
    /* Keep the Watch snapshot consistent with zeroed invalid wire fields. */
    if ((status.valid_flags & VEH_STA_VALID_SPEED) == 0u) status.actual_speed_x10000 = 0;
    if ((status.valid_flags & VEH_STA_VALID_STEERING) == 0u) status.actual_steering_x10000 = 0;
    if ((status.valid_flags & VEH_STA_VALID_ENCODER) == 0u) status.encoder_total = 0;
    if ((status.valid_flags & VEH_STA_VALID_ENABLE) == 0u) status.enable_state = 0u;
    if ((status.valid_flags & VEH_STA_VALID_FAULT) == 0u) status.fault_flags = 0u;
    status.wire_valid_flags = status_wire_valid(&status);
    /* Requirements section6.2: unified system faults, independent of validity. */
    status.fault_flags = 0u;
    if ((status.valid_flags & VEH_STA_VALID_RT49) == 0u) status.fault_flags |= 1u;
    if ((status.valid_flags & VEH_STA_VALID_EPS) == 0u) status.fault_flags |= 2u;
    if ((status.valid_flags & VEH_STA_VALID_SEB) == 0u) status.fault_flags |= 4u;
    if (((status.valid_flags & VEH_STA_VALID_RT49) != 0u) && (status.rt49_fault_code != 0u)) status.fault_flags |= 8u;
    if ((((status.valid_flags & VEH_STA_VALID_EPS) != 0u) && (status.eps_control_state >= 2u)) ||
        (((status.valid_flags & VEH_STA_VALID_EPS_FAULT0) != 0u) &&
         (memcmp(&status.eps_fault_table[0], "\0\0\0\0\0\0", 6u) != 0)) ||
        (((status.valid_flags & VEH_STA_VALID_EPS_FAULT1) != 0u) &&
         ((status.eps_fault_level != 0u) || (memcmp(&status.eps_fault_table[6], "\0\0\0\0\0\0", 6u) != 0)))) status.fault_flags |= 16u;
    if ((((status.valid_flags & VEH_STA_VALID_SEB) != 0u) &&
         (((status.seb_status & 0xC0u) != 0u) || ((status.seb_status & 12u) == 8u))) ||
        (((status.valid_flags & VEH_STA_VALID_SEB_ERRORS) != 0u) && (status.seb_error_bits != 0u))) status.fault_flags |= 32u;
    if ((status.supervisor_flags & VEH_SUP_ESTOP_REQUEST) != 0u) status.fault_flags |= 64u;
    if ((status.supervisor_flags & VEH_SUP_NO_FRESH_CMD) != 0u) status.fault_flags |= 128u;
    if (Remote_ShouldOverride() != 0u) status.fault_flags |= 256u;
    if ((status.valid_flags & VEH_STA_VALID_ENCODER) == 0u) status.fault_flags |= 512u;
    if ((status.valid_flags & VEH_STA_VALID_STEERING) == 0u) status.fault_flags |= 1024u;
    /* Frozen extension bits: speed invalid, remote-safe latch, unknown enables,
     * unknown EPS fault pages, invalid brake measurement, missing cmd counter. */
    if ((status.valid_flags & VEH_STA_VALID_SPEED) == 0u) status.fault_flags |= 2048u;
    if ((status.supervisor_flags & VEH_SUP_REMOTE_SAFE_CMD) != 0u) status.fault_flags |= 4096u;
    if ((status.enable_valid_mask & 1u) == 0u) status.fault_flags |= 8192u;
    if ((status.enable_valid_mask & 2u) == 0u) status.fault_flags |= 16384u;
    if ((status.enable_valid_mask & 4u) == 0u) status.fault_flags |= 32768u;
    if ((status.fault_valid_mask & 4u) == 0u) status.fault_flags |= 65536u;
    if ((status.wire_valid_flags & 16u) == 0u) status.fault_flags |= 131072u;
    if ((status.valid_flags & VEH_STA_VALID_LAST_CMD) == 0u) status.fault_flags |= 262144u;
#if VEH_FEEDBACK_USE_LEGACY_STANDARD && !VEH_FEEDBACK_SPEED_CALIBRATED
    status.fault_flags |= 0x00080000u; /* bit19 legacy speed coefficient, not calibrated */
#endif
#if VEH_FEEDBACK_USE_LEGACY_STANDARD && !VEH_FEEDBACK_STEER_CALIBRATED
    status.fault_flags |= 0x00100000u; /* bit20 direct EPS angle, wheel mapping unconfirmed */
#endif
    g_vehicle_status = status;
    if (VehicleStatus_Pack(&status, data, sizeof(data)) == 0u) return;
    memcpy((void *)dbg_sta_last_data, data, sizeof(data));
    dbg_sta_attempt_cnt++;
    if (BSP_ETH_SendStatus(data, sizeof(data)) != 0u) dbg_sta_send_cnt++;
    else dbg_sta_fail_cnt++;
}
