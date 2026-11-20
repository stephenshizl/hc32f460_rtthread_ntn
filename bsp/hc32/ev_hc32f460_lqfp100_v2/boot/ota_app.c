/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdint.h>
#include <string.h>
#include "ota_app.h"
#include "ota_layout.h"
#include "ota_build_id.h"
#include "flash_map_backend/flash_map_backend.h"

static const uint8_t build_id[] = NTN_OTA_BUILD_ID;
static const uint8_t boot_magic[16] =
{
    0x77, 0xc2, 0x95, 0xf3, 0x60, 0xd2, 0xef, 0x7f,
    0x35, 0x52, 0x50, 0x0f, 0x2c, 0xb6, 0x79, 0x80
};

void ntn_ota_get_build_id(char out[33])
{
    static const char hex[] = "0123456789abcdef";
    for (unsigned i = 0; i < sizeof(build_id); ++i)
    {
        out[2 * i] = hex[build_id[i] >> 4];
        out[2 * i + 1] = hex[build_id[i] & 15U];
    }
    out[32] = 0;
}

static int read_mark(const struct flash_area *area, uint32_t *word)
{
    uint8_t magic[sizeof(boot_magic)];
    return flash_area_read(area, NTN_OTA_SLOT_SIZE - sizeof(magic), magic, sizeof(magic)) ||
           memcmp(magic, boot_magic, sizeof(magic)) ||
           flash_area_read(area, NTN_OTA_IMAGE_OK_OFF, word, sizeof(*word));
}

bool ntn_ota_is_confirmed(void)
{
    const struct flash_area *area;
    uint32_t word;
    return !flash_area_open(FLASH_AREA_IMAGE_PRIMARY(0), &area) &&
           !read_mark(area, &word) && word == 0xFFFFFF01UL;
}

bool ntn_ota_confirm(void)
{
    const struct flash_area *area;
    uint32_t mark, copied;
    const uint32_t confirmed = 0xFFFFFF01UL;

    if (flash_area_open(FLASH_AREA_IMAGE_PRIMARY(0), &area) || read_mark(area, &mark))
        return false;
    if (mark == confirmed)
        return true;
    return mark == UINT32_MAX &&
           !flash_area_read(area, NTN_OTA_IMAGE_OK_OFF - 8U, &copied, sizeof(copied)) &&
           copied == 0xFFFFFF01UL &&
           !flash_area_write(area, NTN_OTA_IMAGE_OK_OFF, &confirmed, sizeof(confirmed)) &&
           ntn_ota_is_confirmed();
}
