#ifndef OTA_LAYOUT_H
#define OTA_LAYOUT_H

#include <stdint.h>

#define OTA_SLOT_A 0x08020000u
#define OTA_SLOT_B 0x08080000u
#define OTA_SLOT_SIZE 0x60000u
#define OTA_META_A 0x08010000u
#define OTA_META_B 0x080E0000u
#define OTA_MAGIC 0x4f544131u
#define OTA_ERASED UINT32_MAX

typedef struct
{
    uint32_t magic;
    uint32_t base;
    uint32_t length;
    uint32_t crc;
    uint32_t version;
    uint32_t attempted;
    uint32_t confirmed;
} OtaManifest;

#endif
