#ifndef OTA_FLASH_WORD_H
#define OTA_FLASH_WORD_H

#include "stm32f4xx.h"

static inline int ota_mark(volatile uint32_t *word)
{
    const uint32_t errors = FLASH_SR_SOP | FLASH_SR_WRPERR |
                            FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR;
    if (*word != UINT32_MAX || (FLASH->SR & FLASH_SR_BSY)) return 0;
    if (FLASH->CR & FLASH_CR_LOCK)
    {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }
    FLASH->SR = errors | FLASH_SR_EOP;
    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    *word = 0u;
    while (FLASH->SR & FLASH_SR_BSY) {}
    FLASH->CR = FLASH_CR_LOCK;
    if (FLASH->ACR & FLASH_ACR_DCEN)
    {
        FLASH->ACR &= ~FLASH_ACR_DCEN;
        FLASH->ACR |= FLASH_ACR_DCRST;
        FLASH->ACR &= ~FLASH_ACR_DCRST;
        FLASH->ACR |= FLASH_ACR_DCEN;
    }
    return !(FLASH->SR & errors) && *word == 0u;
}

#endif
