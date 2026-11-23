#include "layout.h"
#include "flash_word.h"
#ifdef OTA_SIGNED_BOOT
#include "auth.h"
#include "ota_public_key.h"
#endif

void boot_jump(uint32_t base) __attribute__((noreturn));
void boot_main(void) __attribute__((noreturn));

static uint32_t image_crc(const uint8_t *data, uint32_t length)
{
    uint32_t crc = UINT32_MAX;
    while (length-- != 0u)
    {
        crc ^= *data++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1u) ^ ((crc & 1u) ? 0xedb88320u : 0u);
    }
    return ~crc;
}

static int image_valid(volatile const OtaManifest *m, uint32_t base)
{
    if (m->magic != OTA_MAGIC || m->base != base ||
        m->length < 8u || m->length > OTA_SLOT_SIZE || m->version == 0u)
        return 0;

    const uint32_t *vector = (const uint32_t *)base;
    int valid = vector[0] > 0x20000000u && vector[0] <= 0x20020000u &&
           (vector[0] & 7u) == 0u && (vector[1] & 1u) &&
           vector[1] >= base && (vector[1] & ~1u) < base + m->length &&
           image_crc((const uint8_t *)base, m->length) == m->crc;
#ifdef OTA_SIGNED_BOOT
    if (valid)
    {
        const OtaManifest manifest = *m;
        const uint8_t *signature = (const uint8_t *)m + sizeof(*m);
        valid = ota_image_authenticated(&manifest, (const uint8_t *)base,
                                        signature, ota_public_key);
    }
#endif
    return valid;
}

static void start_watchdog(void)
{
    IWDG->KR = 0xCCCCu;
    IWDG->KR = 0x5555u;
    IWDG->PR = 6u;
    IWDG->RLR = 4095u;
    while (IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) {}
    IWDG->KR = 0xAAAAu;
}

void boot_main(void)
{
    volatile OtaManifest *const metadata[] = {
        (volatile OtaManifest *)OTA_META_A, (volatile OtaManifest *)OTA_META_B,
    };
    const uint32_t bases[] = { OTA_SLOT_A, OTA_SLOT_B };
    volatile OtaManifest *confirmed = 0;
    volatile OtaManifest *trial = 0;

    for (unsigned i = 0; i < 2; ++i)
    {
        volatile OtaManifest *m = metadata[i];
        if (!image_valid(m, bases[i])) continue;
        if (m->confirmed == 0u)
        {
            if (!confirmed || m->version > confirmed->version) confirmed = m;
        }
        else if (m->confirmed == OTA_ERASED && m->attempted == OTA_ERASED)
        {
            if (!trial || m->version > trial->version) trial = m;
        }
    }

    /* A trial is meaningful only when a validated rollback image exists. */
    if (confirmed && trial && trial->version > confirmed->version &&
        ota_mark(&trial->attempted))
    {
        start_watchdog();
        boot_jump(trial->base);
    }
    if (confirmed) boot_jump(confirmed->base);
    for (;;) {}
}
