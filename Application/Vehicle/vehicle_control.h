#ifndef VEHICLE_CONTROL_H
#define VEHICLE_CONTROL_H

#include <stdint.h>
#include "vehicle_config.h"

typedef struct
{
    uint8_t valid;
    uint8_t version;
    uint8_t mode;
    uint8_t flags;
    uint8_t enable_mask;
    float   v_mps;
    float   eps_deg;
    uint8_t seb_mode;
    int16_t seb_value_raw;
    uint8_t light;
    uint8_t counter;
    uint32_t last_tick;
} VehicleCmd_t;

extern volatile VehicleCmd_t g_vehicle_cmd;

extern volatile uint8_t dbg_rt49_tx_data[8];
extern volatile uint8_t dbg_eps_tx_data[8];
extern volatile uint8_t dbg_seb_tx_data[8];
extern volatile uint32_t dbg_rt49_tx_cnt;
extern volatile uint32_t dbg_eps_tx_cnt;
extern volatile uint32_t dbg_seb_tx_cnt;
extern volatile uint32_t dbg_vehicle_20ms_cnt;
extern volatile uint32_t dbg_vehicle_timeout_cnt;
extern volatile int16_t dbg_rt49_target_rpm;
extern volatile int16_t dbg_eps_target_deg;

void Vehicle_Init(void);
uint8_t Vehicle_ParseCmd26(const uint8_t *buf, uint16_t len);
void Vehicle_20msTask(void);
void Vehicle_EStop(void);

#endif
