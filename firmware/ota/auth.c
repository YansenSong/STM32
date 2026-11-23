#include "auth.h"
#include "monocypher-ed25519.h"

int ota_image_authenticated(const OtaManifest *manifest, const uint8_t *image,
                            const uint8_t signature[OTA_SIGNATURE_SIZE],
                            const uint8_t public_key[32])
{
    const uint32_t fields[] = {
        manifest->magic, manifest->base, manifest->length,
        manifest->crc, manifest->version,
    };
    /* Sign a domain tag, immutable little-endian metadata, and SHA-512(image).
     * Boot attempt and confirmation words remain writable after verification. */
    uint8_t message[8u + sizeof(fields) + 64u] = { 'S','T','M','4','O','T','A','1' };
    if (manifest->magic != OTA_MAGIC || manifest->length < 8u ||
        manifest->length > OTA_SLOT_SIZE || manifest->version == 0u)
        return 0;

    for (unsigned i = 0; i < sizeof(fields) / sizeof(fields[0]); i++)
        for (unsigned byte = 0; byte < 4u; byte++)
            message[8u + 4u * i + byte] = (uint8_t)(fields[i] >> (8u * byte));

    crypto_sha512(message + 8u + sizeof(fields), image, manifest->length);
    return crypto_ed25519_check(signature, public_key, message, sizeof(message)) == 0;
}
