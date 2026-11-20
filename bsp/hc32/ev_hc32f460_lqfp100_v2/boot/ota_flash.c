/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "hc32_ll.h"
#include "app_flash.h"
#include "fal.h"
#include "ota_layout.h"
#include "flash_map_backend/flash_map_backend.h"

static const struct flash_area areas[] =
{
    {FLASH_AREA_IMAGE_PRIMARY(0), NTN_FLASH_INTERNAL, NTN_OTA_PRIMARY, NTN_OTA_SLOT_SIZE},
    {FLASH_AREA_IMAGE_SECONDARY(0), NTN_FLASH_NOR, NTN_OTA_SECONDARY, NTN_OTA_SLOT_SIZE},
    {FLASH_AREA_IMAGE_SCRATCH, NTN_FLASH_NOR, NTN_OTA_SCRATCH, NTN_OTA_SCRATCH_SIZE},
    {NTN_AREA_FACTORY, NTN_FLASH_NOR, NTN_OTA_FACTORY, NTN_OTA_FACTORY_SIZE},
};

static const struct fal_partition *nor_part(const struct flash_area *area)
{
    const char *name = area->fa_id == FLASH_AREA_IMAGE_SECONDARY(0) ? "download" :
                       area->fa_id == FLASH_AREA_IMAGE_SCRATCH ? "ota_scratch" : "factory";
    const struct fal_partition *part = app_flash_get_partition(name);
    return part && part->offset == area->fa_off && part->len == area->fa_size ? part : NULL;
}

static bool valid_range(const struct flash_area *area, uint32_t off, uint32_t len)
{
    for (unsigned i = 0; i < sizeof(areas) / sizeof(areas[0]); ++i)
        if (area == &areas[i])
            return off <= area->fa_size && len <= area->fa_size - off;
    return false;
}

static uint32_t sector_size(const struct flash_area *area)
{
    return area->fa_device_id == NTN_FLASH_INTERNAL ? NTN_OTA_ONCHIP_SECTOR : NTN_OTA_NOR_SECTOR;
}

int flash_device_base(uint8_t id, uintptr_t *base)
{
    if (id != NTN_FLASH_INTERNAL)
        return -1;
    *base = 0;
    return 0;
}

int flash_area_open(uint8_t id, const struct flash_area **area)
{
    for (unsigned i = 0; i < sizeof(areas) / sizeof(areas[0]); ++i)
    {
        if (areas[i].fa_id == id)
        {
            *area = &areas[i];
            return 0;
        }
    }
    return -1;
}

void flash_area_close(const struct flash_area *area) { (void)area; }

int flash_area_read(const struct flash_area *area, uint32_t off, void *dst, uint32_t len)
{
    if (!valid_range(area, off, len))
        return -1;
    if (len == 0)
        return 0;
    if (area->fa_device_id == NTN_FLASH_INTERNAL)
    {
        memcpy(dst, (const void *)(area->fa_off + off), len);
        return 0;
    }
    const struct fal_partition *part = nor_part(area);
    return part && fal_partition_read(part, off, dst, len) == (int)len ? 0 : -1;
}

static int internal_mutate(const struct flash_area *area, uint32_t off,
                           const uint8_t *source, uint32_t len)
{
    uint32_t addr = area->fa_off + off;
    uint32_t step = source ? 4U : NTN_OTA_ONCHIP_SECTOR;
    uint32_t irq = __get_PRIMASK();
    int result = 0;

    __disable_irq();
    EFM_REG_Unlock();
    EFM_CacheCmd(DISABLE);
    EFM_FWMC_Cmd(ENABLE);
    EFM_SetBusStatus(EFM_BUS_HOLD);

    for (uint32_t i = 0; i < len; i += step)
    {
        int status;
        if (source)
        {
            uint32_t word;
            memcpy(&word, source + i, sizeof(word));
            status = *(volatile const uint32_t *)(addr + i) == word ? LL_OK :
                     EFM_ProgramWordReadBack(addr + i, word);
        }
        else
            status = EFM_SectorErase(addr + i);
        if (status != LL_OK)
        {
            result = -1;
            break;
        }
    }
    EFM_FWMC_Cmd(DISABLE);
    EFM_CacheRamReset(ENABLE);
    EFM_CacheRamReset(DISABLE);
    EFM_CacheCmd(ENABLE);
    EFM_REG_Lock();
    __set_PRIMASK(irq);
    return result;
}

int flash_area_write(const struct flash_area *area, uint32_t off, const void *src, uint32_t len)
{
    if (!valid_range(area, off, len) || area->fa_id == NTN_AREA_FACTORY || (len && !src))
        return -1;
    if (len == 0)
        return 0;
    if (area->fa_device_id == NTN_FLASH_INTERNAL)
    {
        if ((off | len) & 3U)
            return -1;
#ifdef NTN_OTA_APP
        if (off != NTN_OTA_IMAGE_OK_OFF || len != 4U)
            return -1;
#endif
        return internal_mutate(area, off, src, len);
    }
#ifdef NTN_OTA_APP
    if (area->fa_id != FLASH_AREA_IMAGE_SECONDARY(0))
        return -1;
#endif
    const struct fal_partition *part = nor_part(area);
    return part && fal_partition_write(part, off, src, len) == (int)len ? 0 : -1;
}

int flash_area_erase(const struct flash_area *area, uint32_t off, uint32_t len)
{
    if (!valid_range(area, off, len) || area->fa_id == NTN_AREA_FACTORY ||
        off % sector_size(area) || len % sector_size(area))
        return -1;
    if (len == 0)
        return 0;
    if (area->fa_device_id == NTN_FLASH_INTERNAL)
    {
#ifdef NTN_OTA_APP
        return -1;
#else
        return internal_mutate(area, off, NULL, len);
#endif
    }
    const struct fal_partition *part = nor_part(area);
    return part && fal_partition_erase(part, off, len) == (int)len ? 0 : -1;
}

int flash_area_get_sector(const struct flash_area *area, uint32_t off, struct flash_sector *sector)
{
    if (!valid_range(area, off, 1))
        return -1;
    sector->fs_size = sector_size(area);
    sector->fs_off = off - off % sector->fs_size;
    return 0;
}

int flash_area_get_sectors(int id, uint32_t *count, struct flash_sector *sectors)
{
    const struct flash_area *area;
    if (flash_area_open((uint8_t)id, &area))
        return -1;
    uint32_t size = sector_size(area);
    uint32_t n = area->fa_size / size;
    if (*count < n)
        return -1;
    for (uint32_t i = 0; i < n; ++i)
    {
        sectors[i].fs_off = i * size;
        sectors[i].fs_size = size;
    }
    *count = n;
    return 0;
}

uint32_t flash_area_align(const struct flash_area *area) { (void)area; return 4; }
uint8_t flash_area_erased_val(const struct flash_area *area) { (void)area; return 0xFF; }

int flash_area_id_from_multi_image_slot(int image, int slot)
{
    return image == 0 && slot >= 0 && slot <= 2 ? slot + 1 : -1;
}

int flash_area_id_from_image_slot(int slot) { return flash_area_id_from_multi_image_slot(0, slot); }

int flash_area_id_to_multi_image_slot(int image, int id)
{
    return image == 0 && id >= 1 && id <= 3 ? id - 1 : -1;
}
