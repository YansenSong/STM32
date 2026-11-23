#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

extern volatile uint32_t dbg_remote_rx_byte_cnt;
extern volatile uint32_t dbg_remote_frame_cnt;
extern volatile uint32_t dbg_remote_parse_drop_cnt;
extern volatile uint32_t dbg_remote_timeout_cnt;
extern volatile uint32_t dbg_remote_override_cnt;
extern volatile uint32_t dbg_remote_safe_cnt;
extern volatile uint16_t dbg_remote_ch_us[16];
extern volatile uint8_t  dbg_remote_flags;
extern volatile uint8_t  dbg_remote_end_byte;
extern volatile uint8_t  dbg_remote_active;
extern volatile uint8_t  dbg_remote_armed;
extern volatile uint8_t  dbg_remote_brake_on;
extern volatile uint8_t  dbg_remote_fixed_speed_on;
extern volatile int16_t  dbg_remote_eps_raw;
extern volatile int16_t  dbg_remote_v_mps_x1000;
extern volatile int16_t  dbg_remote_pressure_x100;

void Remote_Init(void);
void Remote_Process(void);
uint8_t Remote_ShouldOverride(void);
void Remote_UpdateVehicleCmd(void);

#endif
