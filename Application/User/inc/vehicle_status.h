#ifndef VEHICLE_STATUS_H
#define VEHICLE_STATUS_H

#include <stdint.h>

/* Requirements V1.0 section6, exactly 56 bytes. Not compatible with old
 * framework48 or real-feedback128. Explicit offsets, never transmit a struct. */
#define VEH_STA_LEN                 56u
#define VEH_STA_VERSION             1u
#define VEH_STA_VALID_FLAGS         6u
#define VEH_STA_CONTROL_SOURCE      7u
#define VEH_STA_ENABLE_STATE        8u
#define VEH_STA_COUNTER             9u
#define VEH_STA_TIMESTAMP_MS        10u
#define VEH_STA_ENCODER             14u
#define VEH_STA_ACTUAL_SPEED        22u
#define VEH_STA_ACTUAL_STEERING      26u
#define VEH_STA_STEERING_RAW         30u
#define VEH_STA_BRAKE_MODE           34u
#define VEH_STA_BRAKE_ACTUAL         35u
#define VEH_STA_GEAR                 37u
#define VEH_STA_RT49_STATUS          38u
#define VEH_STA_EPS_STATUS           40u
#define VEH_STA_SEB_STATUS           42u
#define VEH_STA_FAULT_FLAGS          44u
#define VEH_STA_LAST_CMD_COUNTER     48u
#define VEH_STA_CHECKSUM             52u

/* Internal uint32 flags below are NOT the wire's uint8 ValidFlags.
 * Pack maps encoder,speed,wheel,EPSraw,brake,RT49,EPS,SEB to wire bits0..7. */
#define VEH_STA_VALID_SOURCE        0x00000001u
#define VEH_STA_VALID_TIMESTAMP     0x00000002u
#define VEH_STA_VALID_LAST_CMD      0x00000004u
#define VEH_STA_VALID_REQUESTED_EN  0x00000008u
#define VEH_STA_VALID_SPEED         0x00000100u
#define VEH_STA_VALID_STEERING      0x00000200u
#define VEH_STA_VALID_ENCODER       0x00000400u
#define VEH_STA_VALID_ENABLE        0x00000800u
#define VEH_STA_VALID_FAULT         0x00001000u
#define VEH_STA_VALID_RT49          0x00010000u
#define VEH_STA_VALID_EPS           0x00020000u
#define VEH_STA_VALID_SEB           0x00040000u
#define VEH_STA_VALID_SEB_ERRORS    0x00080000u
#define VEH_STA_VALID_EPS_FAULT0    0x00100000u
#define VEH_STA_VALID_EPS_FAULT1    0x00200000u
#define VEH_STA_VALID_SEB_STROKE    0x00400000u
#define VEH_STA_VALID_SEB_PRESSURE  0x00800000u
#define VEH_STA_VALID_SEB_ANGLE     0x01000000u
#define VEH_STA_CAN_VALID_MASK      0x01FF1F00u
#define VEH_STA_BASE_CAN_MASK       0x00001F00u

#define VEH_SOURCE_NONE             0u
#define VEH_SOURCE_USART1          3u
#define VEH_SOURCE_UDP              1u
#define VEH_SOURCE_REMOTE           2u
#define VEH_SOURCE_INTERNAL         4u

/* Software observations, separate from the unknown hardware FaultFlags. */
#define VEH_SUP_NO_FRESH_CMD         0x0001u
#define VEH_SUP_ESTOP_REQUEST        0x0002u
#define VEH_SUP_REMOTE_SAFE_CMD      0x0004u
#define VEH_SUP_CAN_TX_DISABLED      0x0008u
#define VEH_SUP_MODE_DISABLED        0x0010u

typedef struct
{
    uint32_t valid_flags;
    uint8_t wire_valid_flags;    /* section6 uint8 mask for Watch */
    uint8_t brake_mode;          /* 0 unknown/fault, 1 pressure, 2 stroke */
    int16_t brake_actual;        /* MPa*100 or mm*10 according to brake_mode */
    int8_t gear;                /* observed rotation direction, NOT a gearbox sensor */
    uint16_t rt49_status;
    uint16_t eps_status;
    uint16_t seb_status_word;
    uint8_t control_source;
    uint8_t enable_state;         /* REAL partial state; see enable_valid_mask */
    uint32_t counter;
    uint32_t timestamp_ms;        /* snapshot/aggregation time at task entry */
    int32_t actual_speed_x10000;  /* calibrated measured m/s * 10000 */
    int32_t actual_steering_x10000; /* selected mapping degrees*10000; direct EPS in legacy profile, see fault bit20 */
    int64_t encoder_total;        /* measured cumulative drive encoder counts */
    uint32_t fault_flags;         /* known fault summaries; see fault_valid_mask */
    uint8_t last_cmd_counter;     /* last accepted UDP/USART1, NOT an apply ACK */
    uint8_t requested_enable;     /* software request, distinct from real state */
    uint16_t supervisor_flags;
    int16_t rt49_rpm;
    uint8_t rt49_current_a;
    uint8_t rt49_temperature_c;
    uint8_t rt49_fault_code;
    uint8_t eps_mode;
    uint8_t eps_control_state;
    uint8_t eps_temperature_raw;  /* matrix says signed but max150: preserve raw */
    int32_t eps_angle_x10000;     /* measured EPS device angle, NOT wheel angle */
    uint16_t eps_current_deci_a;
    uint8_t seb_status;
    uint8_t eps_fault_level;
    int32_t seb_stroke_um;
    uint16_t seb_pressure_kpa;
    int16_t seb_angle_half_deg;
    uint32_t seb_error_bits;      /* byte0 as MSB, then byte1,2,3 */
    uint8_t eps_fault_table[12];
    uint32_t rt49_sample_ms;
    uint32_t eps_sample_ms;
    uint32_t seb_sample_ms;
    uint32_t seb_error_sample_ms;
    uint32_t eps_fault0_sample_ms;
    uint32_t eps_fault1_sample_ms;
    uint8_t enable_valid_mask;    /* RT49/EPS enable unknown; SEB bit2 known */
    uint8_t fault_valid_mask;     /* bit0 RT49,1 EPS state,2 EPS table,3 SEB state,4 SEB errors */
    uint8_t eps_byte_order;       /* 1 Motorola314 BE; 2 Intel314 LE */
    uint32_t can_online_mask;    /* bit0..5 RT49,EPS,SEB,SEB errors,EPS page0,page1 */
    uint8_t eps_status_raw[8];
} VehicleStatus_t;

extern volatile VehicleStatus_t g_vehicle_status;
extern volatile uint32_t dbg_sta_attempt_cnt;
extern volatile uint32_t dbg_sta_send_cnt; /* accepted by DMA, not host delivery */
extern volatile uint32_t dbg_sta_fail_cnt;
extern volatile uint32_t dbg_sta_can_valid_flags;
extern volatile uint8_t dbg_sta_last_data[VEH_STA_LEN];

void VehicleStatus_Init(void);
void VehicleStatus_OnUpperCmd(uint8_t source, uint8_t counter);
void VehicleStatus_SetControlSource(uint8_t source);
void VehicleStatus_20msTask(void);
uint16_t VehicleStatus_Pack(const VehicleStatus_t *status, uint8_t *out,
                            uint16_t capacity);

/* Base-field extension interface. Active real RX is in vehicle_feedback.c.
 * Do not use target RPM, target angle or command enable to fill these fields.
 * Each call replaces only the groups in mask. Missing groups expire after
 * VEH_STA_FEEDBACK_TIMEOUT_MS. Use sample_ms in the HAL_GetTick time domain.
 * Only base CAN groups are accepted here. Caller owns calibration correctness.
 * V2 adds per-source receive timestamps, not hardware-synchronized sample time.
 * Copies use saved PRIMASK, so this interface can also be called from an ISR.
 */
void VehicleStatus_UpdateCANFeedback(const VehicleStatus_t *measured,
                                      uint32_t mask, uint32_t sample_ms);
void VehicleStatus_InvalidateCANFeedback(uint32_t mask);

#endif
