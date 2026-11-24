#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "bsp_eth.h"
#include "layout.h"
#include "ota_update.h"
#include <string.h>

#define OTA_ARM_TOKEN 0x4f544121u
#define OTA_CHUNK 256u
#define OTA_MANIFEST_BYTES 92u
#define OTA_TIMEOUT_MS 30000u

enum { OTA_BEGIN = 1, OTA_DATA, OTA_FINISH, OTA_REBOOT };
enum { OTA_OK, OTA_BAD_REQUEST, OTA_FLASH_ERROR, OTA_BAD_IMAGE };
typedef struct {
    uint16_t len;
    uint8_t bytes[12u + OTA_CHUNK];
} UpdatePacket;

volatile uint32_t dbg_ota_arm_request;
volatile uint32_t dbg_ota_maintenance;
volatile uint32_t dbg_ota_received_bytes;
static StaticTask_t s_task;
#define UPDATE_STACK_WORDS 512u
#define UPDATE_PRIORITY    1u

static StackType_t s_stack[UPDATE_STACK_WORDS];
static StaticQueue_t s_rx_cb, s_tx_cb;
static uint8_t s_rx_storage[sizeof(UpdatePacket)], s_tx_storage[12];
static QueueHandle_t s_rx, s_tx;
static uint32_t s_base, s_meta, s_length, s_offset;
static uint8_t s_manifest[OTA_MANIFEST_BYTES];
static uint8_t s_started, s_finished;

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_le32(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i < 4; ++i) p[i] = (uint8_t)(v >> (8u * i));
}

static int erase_sectors(uint32_t first, uint32_t count)
{
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS, .Sector = first,
        .NbSectors = count, .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t error;
    if (HAL_FLASH_Unlock() != HAL_OK) return 0;
    int ok = HAL_FLASHEx_Erase(&erase, &error) == HAL_OK;
    HAL_FLASH_Lock();
    return ok;
}

static int program(uint32_t address, const uint8_t *data, uint32_t length)
{
    if (HAL_FLASH_Unlock() != HAL_OK) return 0;
    int ok = 1;
    for (uint32_t i = 0; i < length; i += 4u)
    {
        uint32_t word = UINT32_MAX;
        for (uint32_t j = 0; j < 4u && i + j < length; ++j)
            word = (word & ~(0xffu << (8u * j))) | ((uint32_t)data[i + j] << (8u * j));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word) != HAL_OK)
        {
            ok = 0;
            break;
        }
    }
    HAL_FLASH_Lock();
    return ok;
}

static uint32_t image_crc(const uint8_t *data, uint32_t length)
{
    uint32_t crc = UINT32_MAX;
    while (length--)
    {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1u) ^ ((crc & 1u) ? 0xedb88320u : 0u);
    }
    return ~crc;
}

static uint8_t begin(const uint8_t *metadata)
{
    uint32_t active = SCB->VTOR;
    if (active != OTA_SLOT_A && active != OTA_SLOT_B) return OTA_BAD_REQUEST;
    uint32_t active_meta = active == OTA_SLOT_A ? OTA_META_A : OTA_META_B;
    const OtaManifest *current = (const OtaManifest *)active_meta;
    uint32_t target = active == OTA_SLOT_A ? OTA_SLOT_B : OTA_SLOT_A;
    uint32_t length = read_le32(metadata + 8);
    if (current->magic != OTA_MAGIC || current->base != active ||
        current->confirmed != 0u || read_le32(metadata) != OTA_MAGIC ||
        read_le32(metadata + 4) != target || length < 8u || length > OTA_SLOT_SIZE ||
        read_le32(metadata + 16) <= current->version ||
        read_le32(metadata + 20) != OTA_ERASED || read_le32(metadata + 24) != OTA_ERASED)
        return OTA_BAD_REQUEST;

    if (s_started)
        return memcmp(s_manifest, metadata, sizeof(s_manifest)) == 0 ? OTA_OK : OTA_BAD_REQUEST;

    s_meta = target == OTA_SLOT_A ? OTA_META_A : OTA_META_B;
    if (!erase_sectors(target == OTA_SLOT_A ? FLASH_SECTOR_4 : FLASH_SECTOR_11, 1u) ||
        !erase_sectors(target == OTA_SLOT_A ? FLASH_SECTOR_5 : FLASH_SECTOR_8, 3u))
        return OTA_FLASH_ERROR;

    memcpy(s_manifest, metadata, sizeof(s_manifest));
    s_base = target;
    s_length = length;
    s_offset = 0u;
    dbg_ota_received_bytes = 0u;
    s_started = 1u;
    return OTA_OK;
}

static uint8_t handle(const UpdatePacket *packet)
{
    const uint8_t *p = packet->bytes;
    uint32_t length = packet->len - 12u;
    uint32_t offset = read_le32(p + 8);

    switch (p[4])
    {
    case OTA_BEGIN:
        return length == sizeof(s_manifest) ? begin(p + 12) : OTA_BAD_REQUEST;
    case OTA_DATA:
        if (!s_started || s_finished || offset >= s_length || offset % OTA_CHUNK ||
            length != ((s_length - offset < OTA_CHUNK) ? s_length - offset : OTA_CHUNK))
            return OTA_BAD_REQUEST;
        if (offset < s_offset)
            return memcmp((const void *)(s_base + offset), p + 12, length) == 0 ?
                   OTA_OK : OTA_BAD_REQUEST;
        if (offset != s_offset) return OTA_BAD_REQUEST;
        if (!program(s_base + offset, p + 12, length)) return OTA_FLASH_ERROR;
        s_offset += length;
        dbg_ota_received_bytes = s_offset;
        return OTA_OK;
    case OTA_FINISH:
        if (length || !s_started || s_offset != s_length) return OTA_BAD_REQUEST;
        if (s_finished) return OTA_OK;
        if (image_crc((const uint8_t *)s_base, s_length) != read_le32(s_manifest + 12))
            return OTA_BAD_IMAGE;
        /* 最后写入魔数，标记元数据已写完。启用签名验证的引导程序会在运行固件前验签。 */
        if (!program(s_meta + 4u, s_manifest + 4u, sizeof(s_manifest) - 4u) ||
            !program(s_meta, s_manifest, 4u))
            return OTA_FLASH_ERROR;
        s_finished = 1u;
        return OTA_OK;
    case OTA_REBOOT:
        if (length || !s_finished) return OTA_BAD_REQUEST;
        NVIC_SystemReset();
        break;
    default:
        break;
    }
    return OTA_BAD_REQUEST;
}

static void UpdateTask(void *argument)
{
    (void)argument;
    UpdatePacket packet;
    for (;;)
    {
        if (dbg_ota_arm_request == OTA_ARM_TOKEN && !dbg_ota_maintenance)
        {
            dbg_ota_arm_request = 0u;
            s_started = s_finished = 0u;
            s_offset = 0u;
            xQueueReset(s_rx);
            xQueueReset(s_tx);
            dbg_ota_maintenance = 1u;
        }
        if (!dbg_ota_maintenance)
        {
            vTaskDelay(pdMS_TO_TICKS(100u));
            continue;
        }
        if (xQueueReceive(s_rx, &packet, pdMS_TO_TICKS(OTA_TIMEOUT_MS)) != pdPASS)
        {
            /* 超时后复位，退出维护模式并清除缓存的控制指令。 */
            NVIC_SystemReset();
        }
        uint8_t response[12] = { 'O','T','A','1', packet.bytes[4], 0, 0, 0 };
        response[5] = handle(&packet);
        memcpy(response + 6, packet.bytes + 6, 2);
        write_le32(response + 8, s_offset);
        xQueueOverwrite(s_tx, response);
    }
}

void OtaUpdate_CreateTask(void)
{
    s_rx = xQueueCreateStatic(1u, sizeof(UpdatePacket), s_rx_storage, &s_rx_cb);
    s_tx = xQueueCreateStatic(1u, 12u, s_tx_storage, &s_tx_cb);
    configASSERT(s_rx && s_tx);
    TaskHandle_t task = xTaskCreateStatic(UpdateTask, "Update", UPDATE_STACK_WORDS,
                                          NULL, UPDATE_PRIORITY, s_stack, &s_task);
    configASSERT(task != NULL);
}

void OtaUpdate_Receive(const uint8_t *data, uint16_t length)
{
    if (!dbg_ota_maintenance || length < 12u || length > 12u + OTA_CHUNK ||
        memcmp(data, "OTA1", 4) != 0 || data[5] != 0u)
        return;
    UpdatePacket packet = { .len = length };
    memcpy(packet.bytes, data, length);
    xQueueSend(s_rx, &packet, 0u);
}

void OtaUpdate_SendReply(void)
{
    uint8_t response[12];
    if (xQueuePeek(s_tx, response, 0u) == pdPASS &&
        BSP_ETH_SendTelemetry(response, sizeof(response), OTA_UPDATE_PORT))
        xQueueReceive(s_tx, response, 0u);
}

uint8_t OtaUpdate_InMaintenance(void)
{
    return (uint8_t)(dbg_ota_maintenance != 0u);
}
