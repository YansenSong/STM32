#ifndef VEHICLE_FEEDBACK_H
#define VEHICLE_FEEDBACK_H
#include <stdint.h>
#include "vehicle_status.h"

extern volatile uint32_t dbg_feedback_rt49_cnt;
extern volatile uint32_t dbg_feedback_eps_cnt;
extern volatile uint32_t dbg_feedback_seb_cnt;
extern volatile uint32_t dbg_feedback_seb_error_cnt;
extern volatile uint32_t dbg_feedback_eps_fault_cnt;
extern volatile uint32_t dbg_feedback_reject_cnt;
extern volatile uint32_t dbg_feedback_seb_checksum_err_cnt;
extern volatile uint32_t dbg_feedback_seb_roll_duplicate_cnt;
extern volatile uint32_t dbg_feedback_seb_roll_gap_cnt;
extern volatile uint32_t dbg_feedback_online_mask;

void VehicleFeedback_Init(void);
/* Only a genuine received standard DATA frame may call this decoder. */
void VehicleFeedback_OnCAN(uint8_t bus, uint16_t id, const uint8_t *data,
                            uint8_t dlc, uint32_t received_ms);
void VehicleFeedback_FillStatus(VehicleStatus_t *status, uint32_t now);
#endif
