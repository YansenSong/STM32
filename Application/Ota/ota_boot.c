#include "ota_boot.h"
#include "layout.h"
#include "flash_word.h"

volatile uint32_t dbg_ota_image_version;
volatile uint32_t dbg_ota_confirmed;

void OtaBoot_Service(uint32_t now, uint8_t ready)
{
    volatile OtaManifest *manifest;
    if (SCB->VTOR == OTA_SLOT_A)
        manifest = (volatile OtaManifest *)OTA_META_A;
    else if (SCB->VTOR == OTA_SLOT_B)
        manifest = (volatile OtaManifest *)OTA_META_B;
    else
        return;

    if (manifest->magic != OTA_MAGIC || manifest->base != SCB->VTOR) return;
    dbg_ota_image_version = manifest->version;
    if (manifest->confirmed == 0u)
    {
        dbg_ota_confirmed = 1u;
        IWDG->KR = 0xAAAAu;
    }
    else if (manifest->attempted == 0u && now >= 5000u && ready)
    {
        /* 以太网就绪且控制周期正常执行后，才确认当前固件。 */
        if (ota_mark(&manifest->confirmed))
        {
            dbg_ota_confirmed = 1u;
            IWDG->KR = 0xAAAAu;
        }
    }
}
