#ifndef OTA_AUTH_H
#define OTA_AUTH_H

#include "layout.h"

#define OTA_SIGNATURE_SIZE 64u

int ota_image_authenticated(const OtaManifest *manifest, const uint8_t *image,
                            const uint8_t signature[OTA_SIGNATURE_SIZE],
                            const uint8_t public_key[32]);

#endif
