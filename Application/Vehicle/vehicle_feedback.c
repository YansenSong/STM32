#include "vehicle_feedback.h"
#include "vehicle_config.h"
#include <string.h>

volatile uint32_t dbg_feedback_rt49_cnt = 0u;
volatile uint32_t dbg_feedback_eps_cnt = 0u;
volatile uint32_t dbg_feedback_seb_cnt = 0u;
volatile uint32_t dbg_feedback_seb_error_cnt = 0u;
volatile uint32_t dbg_feedback_eps_fault_cnt = 0u;
volatile uint32_t dbg_feedback_reject_cnt = 0u;
volatile uint32_t dbg_feedback_seb_checksum_err_cnt = 0u;
volatile uint32_t dbg_feedback_seb_roll_duplicate_cnt = 0u;
volatile uint32_t dbg_feedback_seb_roll_gap_cnt = 0u;
volatile uint32_t dbg_feedback_online_mask = 0u;

static VehicleStatus_t s_fb;
static uint8_t s_seen[6];
static uint8_t s_seb_roll_seen = 0u;
static uint8_t s_seb_roll = 0u;
static uint32_t s_seb_roll_tick = 0u;

static uint16_t fb_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint16_t fb_eps_u16(const uint8_t *p)
{
#if VEH_EPS_RX_INTEL
    return (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
#else
    return fb_u16_be(p);
#endif
}

static int16_t fb_i16(uint16_t raw)
{
    int32_t value;
    value = (int32_t)raw;
    if ((raw & 0x8000u) != 0u) value -= 65536;
    return (int16_t)value;
}

void VehicleFeedback_Init(void)
{
    memset(&s_fb, 0, sizeof(s_fb));
    memset(s_seen, 0, sizeof(s_seen));
    s_seb_roll_seen = 0u;
    s_seb_roll = 0u;
    s_seb_roll_tick = 0u;
    dbg_feedback_rt49_cnt = 0u;
    dbg_feedback_eps_cnt = 0u;
    dbg_feedback_seb_cnt = 0u;
    dbg_feedback_seb_error_cnt = 0u;
    dbg_feedback_eps_fault_cnt = 0u;
    dbg_feedback_reject_cnt = 0u;
    dbg_feedback_seb_checksum_err_cnt = 0u;
    dbg_feedback_seb_roll_duplicate_cnt = 0u;
    dbg_feedback_seb_roll_gap_cnt = 0u;
    dbg_feedback_online_mask = 0u;
}

void VehicleFeedback_OnCAN(uint8_t bus, uint16_t id, const uint8_t *data,
                            uint8_t dlc, uint32_t received_ms)
{
    uint16_t raw;
    uint8_t sum;
    uint8_t i;
    uint8_t roll;
    if ((data == 0) || (dlc != 8u))
    {
        dbg_feedback_reject_cnt++;
        return;
    }
    /* Called only from nonblocking main-loop FIFO polling, never IRQ context. */
    if ((bus == 2u) && (id == VEH_CAN_ID_RT49_STATUS))
    {
        /* RT49 document's 11-byte examples include 3 transport bytes.
         * CAN payload starts at device type, then signed BE RPM at data[1:2]. */
        if (data[0] != 0x01u)
        {
            dbg_feedback_reject_cnt++;
            return;
        }
        s_fb.rt49_rpm = fb_i16(fb_u16_be(&data[1]));
        s_fb.rt49_current_a = data[3];
        s_fb.rt49_temperature_c = data[4];
        s_fb.rt49_fault_code = data[5];
        s_fb.rt49_sample_ms = received_ms;
        s_seen[0] = 1u;
        dbg_feedback_rt49_cnt++;
    }
    else if ((bus == 1u) && (id == VEH_CAN_ID_EPS_STATUS))
    {
        /* motorla314 uses Motorola LSB convention: 16-bit LSB starts at bit16,
         * so high byte1 / low byte2. Intel314 starts at bit8, low byte1/high2.
         * This matches the existing 0x314 TX byte positions; TX is unchanged. */
        s_fb.eps_mode = (uint8_t)(data[0] & 1u);
        /* Store signed EPS degrees (1 deg/count) scaled by 10000. */
        s_fb.eps_angle_x10000 = (int32_t)fb_i16(fb_eps_u16(&data[1])) * 10000;
        s_fb.eps_current_deci_a = fb_eps_u16(&data[3]);
        s_fb.eps_temperature_raw = data[6];
        s_fb.eps_control_state = (uint8_t)(data[7] & 3u);
        s_fb.eps_byte_order = VEH_EPS_RX_INTEL ? 2u : 1u;
        memcpy(s_fb.eps_status_raw, data, 8u);
        s_fb.eps_sample_ms = received_ms;
        s_seen[1] = 1u;
        dbg_feedback_eps_cnt++;
    }
    else if ((bus == 1u) && (id == VEH_CAN_ID_EPS_FAULT))
    {
        if (data[6] == 0x00u)
        {
            memcpy(&s_fb.eps_fault_table[0], data, 6u);
            s_fb.eps_fault0_sample_ms = received_ms;
            s_seen[4] = 1u;
        }
        else if (data[6] == 0xFFu)
        {
            memcpy(&s_fb.eps_fault_table[6], data, 6u);
            s_fb.eps_fault_level = data[7];
            s_fb.eps_fault1_sample_ms = received_ms;
            s_seen[5] = 1u;
        }
        else
        {
            dbg_feedback_reject_cnt++;
            return;
        }
        dbg_feedback_eps_fault_cnt++;
    }
    else if ((bus == 2u) && (id == VEH_CAN_ID_SEB_STATUS))
    {
        /* Verify checksum ONLY when the ECU says it is enabled (byte6 bit1). */
        if ((data[6] & 2u) != 0u)
        {
            sum = 0u;
            for (i = 0u; i < 7u; i++) sum = (uint8_t)(sum + data[i]);
            if ((uint8_t)(sum ^ 0xFFu) != data[7])
            {
                dbg_feedback_seb_checksum_err_cnt++;
                return;
            }
        }
        if ((data[6] & 1u) != 0u)
        {
            roll = (uint8_t)((data[6] >> 4) & 15u);
            if (s_seb_roll_seen != 0u)
            {
                if (roll == s_seb_roll)
                {
                    /* Repeated life counter must NOT keep stale sensors valid. */
                    dbg_feedback_seb_roll_duplicate_cnt++;
                    return;
                }
                if (((uint32_t)(received_ms - s_seb_roll_tick) <= VEH_STA_FEEDBACK_TIMEOUT_MS) &&
                    (roll != (uint8_t)((s_seb_roll + 1u) & 15u)))
                    dbg_feedback_seb_roll_gap_cnt++;
            }
            s_seb_roll = roll;
            s_seb_roll_tick = received_ms;
            s_seb_roll_seen = 1u;
        }
        else s_seb_roll_seen = 0u;
        s_fb.seb_status = data[0];
        /* Motorola LSB: stroke high1/low2, angle high4/low5.
         * Pressure byte3 is separate; do not overlap it with stroke. */
        raw = fb_u16_be(&data[1]);
        s_fb.seb_stroke_um = (int32_t)raw * 50 - 30000;
        s_fb.seb_pressure_kpa = (uint16_t)((uint16_t)data[3] * 50u);
        s_fb.seb_angle_half_deg = fb_i16(fb_u16_be(&data[4]));
        s_fb.seb_sample_ms = received_ms;
        s_seen[2] = 1u;
        dbg_feedback_seb_cnt++;
    }
    else if ((bus == 2u) && (id == VEH_CAN_ID_SEB_FAULT))
    {
        s_fb.seb_error_bits = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                              ((uint32_t)data[2] << 8) | data[3];
        s_fb.seb_error_sample_ms = received_ms;
        s_seen[3] = 1u;
        dbg_feedback_seb_error_cnt++;
    }
}

void VehicleFeedback_FillStatus(VehicleStatus_t *status, uint32_t now)
{
    uint8_t fresh[6];
    uint8_t i;
    uint8_t any_fault;
#if VEH_FEEDBACK_SPEED_CALIBRATED || VEH_FEEDBACK_STEER_CALIBRATED || VEH_FEEDBACK_USE_LEGACY_STANDARD
    float scaled;
#endif
    if (status == 0) return;
    fresh[0] = (uint8_t)(s_seen[0] && ((uint32_t)(now - s_fb.rt49_sample_ms) <= VEH_STA_FEEDBACK_TIMEOUT_MS));
    fresh[1] = (uint8_t)(s_seen[1] && ((uint32_t)(now - s_fb.eps_sample_ms) <= VEH_STA_FEEDBACK_TIMEOUT_MS));
    fresh[2] = (uint8_t)(s_seen[2] && ((uint32_t)(now - s_fb.seb_sample_ms) <= VEH_STA_FEEDBACK_TIMEOUT_MS));
    fresh[3] = (uint8_t)(s_seen[3] && ((uint32_t)(now - s_fb.seb_error_sample_ms) <= VEH_STA_ERROR_TIMEOUT_MS));
    fresh[4] = (uint8_t)(s_seen[4] && ((uint32_t)(now - s_fb.eps_fault0_sample_ms) <= VEH_STA_ERROR_TIMEOUT_MS));
    fresh[5] = (uint8_t)(s_seen[5] && ((uint32_t)(now - s_fb.eps_fault1_sample_ms) <= VEH_STA_ERROR_TIMEOUT_MS));
    status->can_online_mask = 0u;
    for (i = 0u; i < 6u; i++) if (fresh[i]) status->can_online_mask |= (uint32_t)1u << i;
    dbg_feedback_online_mask = status->can_online_mask;
    if (fresh[0])
    {
        status->valid_flags |= VEH_STA_VALID_RT49;
        status->rt49_rpm = s_fb.rt49_rpm;
        status->rt49_current_a = s_fb.rt49_current_a;
        status->rt49_temperature_c = s_fb.rt49_temperature_c;
        status->rt49_fault_code = s_fb.rt49_fault_code;
        status->rt49_sample_ms = s_fb.rt49_sample_ms;
        status->gear = s_fb.rt49_rpm > 0 ? 1 : (s_fb.rt49_rpm < 0 ? -1 : 0);
        status->rt49_status = (uint16_t)((uint16_t)s_fb.rt49_fault_code | 0x0100u);
        if (s_fb.rt49_rpm > 0) status->rt49_status |= 0x0200u;
        else if (s_fb.rt49_rpm < 0) status->rt49_status |= 0x0400u;
        else status->rt49_status |= 0x0800u;
        status->fault_valid_mask |= 1u;
        if (s_fb.rt49_fault_code != 0u) status->fault_flags |= 1u;
#if VEH_FEEDBACK_SPEED_CALIBRATED || VEH_FEEDBACK_USE_LEGACY_STANDARD
        /* Actual measured RPM; legacy mapping explicitly selected by user. */
#if (VEH_FEEDBACK_SPEED_METHOD == 2u)
        if ((VEH_FEEDBACK_GEAR_RATIO > 0.0f) && (VEH_FEEDBACK_WHEEL_RADIUS_M > 0.0f))
        {
            scaled = (float)s_fb.rt49_rpm * 6.28318530718f * VEH_FEEDBACK_WHEEL_RADIUS_M *
                10000.0f * VEH_FEEDBACK_SPEED_SIGN / (60.0f * VEH_FEEDBACK_GEAR_RATIO);
            if ((scaled >= -2147483648.0f) && (scaled <= 2147483520.0f))
            {
                status->actual_speed_x10000 = (int32_t)scaled;
                status->valid_flags |= VEH_STA_VALID_SPEED;
            }
        }
#elif (VEH_FEEDBACK_SPEED_METHOD == 1u)
        if (VEH_FEEDBACK_RPM_PER_MPS > 0.0f)
        {
            scaled = (float)s_fb.rt49_rpm *
                10000.0f * VEH_FEEDBACK_SPEED_SIGN / VEH_FEEDBACK_RPM_PER_MPS;
            if ((scaled >= -2147483648.0f) && (scaled <= 2147483520.0f))
            {
                status->actual_speed_x10000 = (int32_t)scaled;
                status->valid_flags |= VEH_STA_VALID_SPEED;
            }
        }
#endif
#endif
    }
    if (fresh[1])
    {
        status->valid_flags |= VEH_STA_VALID_EPS;
        status->eps_status = (uint16_t)((uint16_t)s_fb.eps_mode |
             ((uint16_t)s_fb.eps_control_state << 1));
        status->eps_mode = s_fb.eps_mode;
        status->eps_control_state = s_fb.eps_control_state;
        status->eps_temperature_raw = s_fb.eps_temperature_raw;
        status->eps_angle_x10000 = s_fb.eps_angle_x10000;
        status->eps_current_deci_a = s_fb.eps_current_deci_a;
        status->eps_byte_order = s_fb.eps_byte_order;
        status->eps_sample_ms = s_fb.eps_sample_ms;
        memcpy(status->eps_status_raw, s_fb.eps_status_raw, 8u);
        if (s_fb.eps_control_state != 0u)
        {
            status->fault_valid_mask |= 2u;
            if (s_fb.eps_control_state >= 2u) status->fault_flags |= 2u;
        }
#if VEH_FEEDBACK_STEER_CALIBRATED || VEH_FEEDBACK_USE_LEGACY_STANDARD
        scaled = (float)s_fb.eps_angle_x10000 *
             VEH_STEERING_WHEEL_PER_EPS_DEG + VEH_STEERING_WHEEL_OFFSET_DEG * 10000.0f;
        if ((scaled >= -2147483648.0f) && (scaled <= 2147483520.0f))
        {
            status->actual_steering_x10000 = (int32_t)scaled;
            status->valid_flags |= VEH_STA_VALID_STEERING;
        }
#endif
    }
    if (fresh[2])
    {
        status->valid_flags |= VEH_STA_VALID_SEB;
        status->seb_status = s_fb.seb_status;
        status->seb_status_word = s_fb.seb_status;
        status->seb_sample_ms = s_fb.seb_sample_ms;
        status->enable_valid_mask |= 4u;
        status->valid_flags |= VEH_STA_VALID_ENABLE;
        if ((s_fb.seb_status & 2u) != 0u) status->enable_state |= 4u;
        status->fault_valid_mask |= 8u;
        if ((s_fb.seb_status & 0xC0u) != 0u) status->fault_flags |= 8u;
        if ((s_fb.seb_stroke_um >= -5000) && (s_fb.seb_stroke_um <= 27000))
        {
            status->seb_stroke_um = s_fb.seb_stroke_um;
            status->valid_flags |= VEH_STA_VALID_SEB_STROKE;
        }
        if (s_fb.seb_pressure_kpa <= 5000u)
        {
            status->seb_pressure_kpa = s_fb.seb_pressure_kpa;
            status->valid_flags |= VEH_STA_VALID_SEB_PRESSURE;
        }
        if ((s_fb.seb_angle_half_deg >= -300) && (s_fb.seb_angle_half_deg <= 1680))
        {
            status->seb_angle_half_deg = s_fb.seb_angle_half_deg;
            status->valid_flags |= VEH_STA_VALID_SEB_ANGLE;
        }
    }
    if (fresh[3])
    {
        status->valid_flags |= VEH_STA_VALID_SEB_ERRORS;
        status->seb_error_bits = s_fb.seb_error_bits;
        status->seb_error_sample_ms = s_fb.seb_error_sample_ms;
        status->fault_valid_mask |= 16u;
        if (s_fb.seb_error_bits != 0u) status->fault_flags |= 16u;
        /* Known pressure sensor absence: don't present its zero as measurement. */
        if ((s_fb.seb_error_bits & 0x00080000u) != 0u)
        {
            status->valid_flags &= ~VEH_STA_VALID_SEB_PRESSURE;
            status->seb_pressure_kpa = 0u;
        }
    }
    if (fresh[4])
    {
        status->valid_flags |= VEH_STA_VALID_EPS_FAULT0;
        memcpy(&status->eps_fault_table[0], &s_fb.eps_fault_table[0], 6u);
        status->eps_fault0_sample_ms = s_fb.eps_fault0_sample_ms;
    }
    if (fresh[5])
    {
        status->valid_flags |= VEH_STA_VALID_EPS_FAULT1;
        memcpy(&status->eps_fault_table[6], &s_fb.eps_fault_table[6], 6u);
        status->eps_fault1_sample_ms = s_fb.eps_fault1_sample_ms;
        status->eps_fault_level = s_fb.eps_fault_level;
    }
    if (fresh[4] && fresh[5])
    {
        status->fault_valid_mask |= 4u;
        any_fault = s_fb.eps_fault_level;
        for (i = 0u; i < 12u; i++) any_fault |= s_fb.eps_fault_table[i];
        if (any_fault != 0u) status->fault_flags |= 4u;
    }
    if (status->fault_valid_mask != 0u) status->valid_flags |= VEH_STA_VALID_FAULT;
    if (fresh[2])
    {
        if ((s_fb.seb_status & 12u) == 4u)
        {
            status->brake_mode = 1u;
            if ((status->valid_flags & VEH_STA_VALID_SEB_PRESSURE) != 0u)
                status->brake_actual = (int16_t)(status->seb_pressure_kpa / 10u);
        }
        else if ((s_fb.seb_status & 12u) == 0u)
        {
            status->brake_mode = 2u;
            if ((status->valid_flags & VEH_STA_VALID_SEB_STROKE) != 0u)
                status->brake_actual = (int16_t)(status->seb_stroke_um / 100);
        }
    }
    /* Encoder counts are not in the three supplied protocols: remain invalid.
     * Reported data may describe a faulty sensor: raw fault fields remain visible.
     * No feedback observation changes existing command/CAN TX behavior. */
}
