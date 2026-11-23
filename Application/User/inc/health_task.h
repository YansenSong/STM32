#ifndef HEALTH_TASK_H
#define HEALTH_TASK_H

#include <stdint.h>

typedef struct
{
    uint32_t uptime_ms;
    uint32_t max_control_late_us;
    uint32_t max_control_run_us;
    uint32_t missed_periods;
    uint32_t vehicle_stack_free_words;
    uint32_t can_online_mask;
    uint32_t ethernet_retries;
} HealthSnapshot;

void Health_Init(void);
void Health_Publish(const HealthSnapshot *snapshot);
void Health_SendPending(void);

#endif
