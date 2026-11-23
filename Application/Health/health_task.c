#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bsp_eth.h"
#include "health_task.h"
#include <string.h>

#define HEALTH_STACK_WORDS 256u
#define HEALTH_PRIORITY    1u
#define HEALTH_REPORT_LEN 44u
#define HEALTH_UDP_PORT 5002u

typedef struct
{
    uint8_t bytes[HEALTH_REPORT_LEN];
} HealthReport;

static StaticTask_t s_task;
static StackType_t s_stack[HEALTH_STACK_WORDS];
static StaticQueue_t s_snapshot_queue;
static StaticQueue_t s_report_queue;
static uint8_t s_snapshot_storage[sizeof(HealthSnapshot)];
static uint8_t s_report_storage[sizeof(HealthReport)];
static QueueHandle_t s_snapshots;
static QueueHandle_t s_reports;
volatile uint32_t dbg_health_report_count;
volatile uint32_t dbg_health_stack_free_words;

static void put_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value >> 24);
    out[1] = (uint8_t)(value >> 16);
    out[2] = (uint8_t)(value >> 8);
    out[3] = (uint8_t)value;
}

static void HealthTask(void *argument)
{
    TickType_t wake = xTaskGetTickCount();
    uint32_t sequence = 0u;
    HealthSnapshot snapshot;
    (void)argument;

    for (;;)
    {
        (void)xTaskDelayUntil(&wake, pdMS_TO_TICKS(1000u));
        if (xQueuePeek(s_snapshots, &snapshot, 0u) == pdPASS)
        {
            HealthReport report;
            memcpy(report.bytes, "hlt__", 5u);
            report.bytes[5] = 1u;
            report.bytes[6] = 0u;
            report.bytes[7] = 0u;
            dbg_health_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
            put_u32(&report.bytes[8], sequence++);
            put_u32(&report.bytes[12], snapshot.uptime_ms);
            put_u32(&report.bytes[16], snapshot.max_control_late_us);
            put_u32(&report.bytes[20], snapshot.max_control_run_us);
            put_u32(&report.bytes[24], snapshot.missed_periods);
            put_u32(&report.bytes[28], snapshot.vehicle_stack_free_words);
            put_u32(&report.bytes[32], dbg_health_stack_free_words);
            put_u32(&report.bytes[36], snapshot.can_online_mask);
            put_u32(&report.bytes[40], snapshot.ethernet_retries);
            (void)xQueueOverwrite(s_reports, &report);
            dbg_health_report_count++;
        }
    }
}

void Health_CreateTask(void)
{
    s_snapshots = xQueueCreateStatic(1u, sizeof(HealthSnapshot), s_snapshot_storage,
                                    &s_snapshot_queue);
    s_reports = xQueueCreateStatic(1u, sizeof(HealthReport), s_report_storage,
                                  &s_report_queue);
    configASSERT(s_snapshots != NULL && s_reports != NULL);
    TaskHandle_t task = xTaskCreateStatic(HealthTask, "Health", HEALTH_STACK_WORDS,
                                          NULL, HEALTH_PRIORITY, s_stack, &s_task);
    configASSERT(task != NULL);
}

void Health_Publish(const HealthSnapshot *snapshot)
{
    (void)xQueueOverwrite(s_snapshots, snapshot);
}

void Health_SendPending(void)
{
    HealthReport report;
    if (BSP_ETH_IsReady() &&
        xQueuePeek(s_reports, &report, 0u) == pdPASS &&
        BSP_ETH_SendTelemetry(report.bytes, sizeof(report.bytes), HEALTH_UDP_PORT))
    {
        /* Vehicle has higher priority than the sole producer, Health. */
        (void)xQueueReceive(s_reports, &report, 0u);
    }
}
