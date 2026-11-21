/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "hc32_ll.h"
#include "app_flash.h"
#include "fal.h"
#include "ota_layout.h"
#include "ota_power.h"
#include "ota_watchdog.h"
#include "flash_map_backend/flash_map_backend.h"

/* LVD1 LVL6: 2.85-3.11 V falling in SuperSpeed (200 MHz), 2.73-2.97 V in HighSpeed.
 * Guarantees VCC stays strictly above BY25Q64ES 2.70 V minimum in both modes. */
#define NTN_OTA_LVD_THRESHOLD   PWC_LVD_THRESHOLD_LVL6
#define NTN_OTA_NOR_BLOCK_SIZE  0x10000UL
#define NTN_OTA_NOR_WRITE_CHUNK 0x1000UL

static bool s_lvd_ready = false;
static bool s_low_power_fault = false;

static const struct flash_area areas[] =
{
    {FLASH_AREA_IMAGE_PRIMARY(0), NTN_FLASH_INTERNAL, NTN_OTA_PRIMARY, NTN_OTA_SLOT_SIZE},
    {FLASH_AREA_IMAGE_SECONDARY(0), NTN_FLASH_NOR, NTN_OTA_SECONDARY, NTN_OTA_SLOT_SIZE},
    {FLASH_AREA_IMAGE_SCRATCH, NTN_FLASH_NOR, NTN_OTA_SCRATCH, NTN_OTA_SCRATCH_SIZE},
    {NTN_AREA_FACTORY, NTN_FLASH_NOR, NTN_OTA_FACTORY, NTN_OTA_FACTORY_SIZE},
};

void ntn_ota_efm_lock_all(void)
{
    LL_PERIPH_WE(LL_PERIPH_EFM);
    EFM_FWMC_Cmd(DISABLE);
    /* RM 7.6.7: FPMTEW < FPMTSW disables programming and sector erase across all Flash. */
    EFM_SetWindowProtectAddr(0x0007E000UL, 0x00000000UL);
    LL_PERIPH_WP(LL_PERIPH_EFM);
}

bool ntn_ota_last_power_fault(void)
{
    return s_low_power_fault;
}

void ntn_ota_clear_power_fault(void)
{
    s_low_power_fault = false;
}

bool ntn_ota_power_init(void)
{
    stc_pwc_lvd_init_t cfg;
    int32_t ret;

    (void)PWC_LVD_StructInit(&cfg);
    cfg.u32State              = PWC_LVD_ON;
    cfg.u32CompareOutputState = PWC_LVD_CMP_ON;
    cfg.u32ExceptionType      = PWC_LVD_EXP_TYPE_NONE;
    cfg.u32Filter             = PWC_LVD_FILTER_OFF;
    cfg.u32ThresholdVoltage   = NTN_OTA_LVD_THRESHOLD;

    LL_PERIPH_WE(LL_PERIPH_LVD);
    ret = PWC_LVD_Init(PWC_LVD_CH1, &cfg);
    LL_PERIPH_WP(LL_PERIPH_LVD);
    if (ret != LL_OK)
    {
        s_lvd_ready = false;
        s_low_power_fault = true;
        return false;
    }

    /* Exceed TIPVD1 max (460 us) so comparator output settles before sampling. */
    DDL_DelayMS(1UL);
    s_lvd_ready = ((CM_PWC->PVDCR0 & PWC_PVDCR0_PVD1EN) != 0U) &&
                  ((CM_PWC->PVDCR1 & PWC_PVDCR1_PVD1CMPOE) != 0U) &&
                  ((CM_PWC->PVDLCR & PWC_PVDLCR_PVD1LVL) == NTN_OTA_LVD_THRESHOLD);
    return ntn_ota_voltage_ok();
}

bool ntn_ota_voltage_ok(void)
{
    /* RM Rev1.71 p.161: PVD1MON reads 1 when PVD1 is disabled OR VCC > VPVD1.
     * Require monitor enable, comparator enable, expected threshold, stabilization, AND MON == 1. */
    if (!s_lvd_ready && !ntn_ota_power_init())
    {
        s_low_power_fault = true;
        return false;
    }
    if ((CM_PWC->PVDCR0 & PWC_PVDCR0_PVD1EN) == 0U ||
        (CM_PWC->PVDCR1 & PWC_PVDCR1_PVD1CMPOE) == 0U ||
        ((CM_PWC->PVDLCR & PWC_PVDLCR_PVD1LVL) != NTN_OTA_LVD_THRESHOLD) ||
        PWC_LVD_GetStatus(PWC_LVD1_FLAG_MON) != SET)
    {
        s_low_power_fault = true;
        return false;
    }
    return true;
}

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
    uint32_t end = addr + len;
    uint32_t step = source ? 4U : NTN_OTA_ONCHIP_SECTOR;
    uint32_t irq;
    uint32_t prev_cache;
    int result = 0;

    /* Software half-open boundary guard strictly inside primary slot [0x20000, 0x7E000). */
    if (addr < NTN_OTA_PRIMARY || end <= addr ||
        end > (NTN_OTA_PRIMARY + NTN_OTA_SLOT_SIZE))
        return -1;

    s_low_power_fault = false;
    if (!ntn_ota_voltage_ok())
        return -1;

    irq = __get_PRIMASK();
    __disable_irq();
    EFM_REG_Unlock();
    prev_cache = CM_EFM->FRMC & EFM_CACHE_ALL;
    EFM_CacheCmd(DISABLE);
    /* Narrow hardware write window to exact target range; FPMTEW = end - 1 ensures
     * FPMTEW > FPMTSW (never equal => full unlock) and blocks adjacent address `end`. */
    EFM_SetWindowProtectAddr(addr, end - 1UL);
    EFM_FWMC_Cmd(ENABLE);
    EFM_SetBusStatus(EFM_BUS_HOLD);

    for (uint32_t i = 0; i < len; i += step)
    {
        int status;
        if ((!source || (i & 0x3FFUL) == 0UL) && !ntn_ota_voltage_ok())
        {
            result = -1;
            break;
        }
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
        if (!source || ((i + step) & 0x3FFUL) == 0UL)
            ntn_ota_feed_watchdog();
    }

    EFM_FWMC_Cmd(DISABLE);
    EFM_SetWindowProtectAddr(0x0007E000UL, 0x00000000UL);
    EFM_CacheRamReset(ENABLE);
    EFM_CacheRamReset(DISABLE);
    if (prev_cache != 0UL)
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
        return internal_mutate(area, off, (const uint8_t *)src, len);
    }
#ifdef NTN_OTA_APP
    if (area->fa_id != FLASH_AREA_IMAGE_SECONDARY(0))
        return -1;
#endif
    const struct fal_partition *part = nor_part(area);
    if (!part)
        return -1;
    s_low_power_fault = false;
    const uint8_t *bytes = (const uint8_t *)src;
    uint32_t done = 0;
    while (done < len)
    {
        uint32_t chunk = len - done;
        if (chunk > NTN_OTA_NOR_WRITE_CHUNK)
            chunk = NTN_OTA_NOR_WRITE_CHUNK;
        if (!ntn_ota_voltage_ok())
            return -1;
        if (fal_partition_write(part, off + done, bytes + done, chunk) != (int)chunk)
            return -1;
        done += chunk;
        ntn_ota_feed_watchdog();
    }
    return 0;
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
#ifdef NTN_OTA_APP
    if (area->fa_id != FLASH_AREA_IMAGE_SECONDARY(0))
        return -1;
#endif
    const struct fal_partition *part = nor_part(area);
    if (!part)
        return -1;
    s_low_power_fault = false;
    uint32_t done = 0;
    while (done < len)
    {
        uint32_t cur = off + done;
        uint32_t rem = len - done;
        /* Prefer 64 KiB aligned block chunks (<= 2 s) and 4 KiB chunks (<= 0.3 s)
         * so watchdog refreshes stay strictly bounded and gated on actual progress. */
        uint32_t chunk = (((part->offset + cur) & (NTN_OTA_NOR_BLOCK_SIZE - 1UL)) == 0UL &&
                          rem >= NTN_OTA_NOR_BLOCK_SIZE)
                             ? NTN_OTA_NOR_BLOCK_SIZE
                             : NTN_OTA_NOR_SECTOR;
        if (!ntn_ota_voltage_ok())
            return -1;
        if (fal_partition_erase(part, cur, chunk) != (int)chunk)
            return -1;
        done += chunk;
        ntn_ota_feed_watchdog();
    }
    return 0;
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
