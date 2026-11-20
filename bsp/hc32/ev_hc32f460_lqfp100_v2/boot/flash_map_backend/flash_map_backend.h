/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef NTN_FLASH_MAP_BACKEND_H
#define NTN_FLASH_MAP_BACKEND_H
#include <stdbool.h>
#include <stdint.h>
#include "sysflash/sysflash.h"

struct flash_area
{
    uint8_t fa_id, fa_device_id;
    uint32_t fa_off, fa_size;
};

struct flash_sector
{
    uint32_t fs_off, fs_size;
};

static inline uint8_t flash_area_get_id(const struct flash_area *fa) { return fa->fa_id; }
static inline uint8_t flash_area_get_device_id(const struct flash_area *fa) { return fa->fa_device_id; }
static inline uint32_t flash_area_get_off(const struct flash_area *fa) { return fa->fa_off; }
static inline uint32_t flash_area_get_size(const struct flash_area *fa) { return fa->fa_size; }
static inline uint32_t flash_sector_get_off(const struct flash_sector *fs) { return fs->fs_off; }
static inline uint32_t flash_sector_get_size(const struct flash_sector *fs) { return fs->fs_size; }
static inline bool flash_area_erase_required(const struct flash_area *fa) { (void)fa; return true; }

int flash_device_base(uint8_t id, uintptr_t *base);
int flash_area_open(uint8_t id, const struct flash_area **area);
void flash_area_close(const struct flash_area *area);
int flash_area_read(const struct flash_area *area, uint32_t off, void *dst, uint32_t len);
int flash_area_write(const struct flash_area *area, uint32_t off, const void *src, uint32_t len);
int flash_area_erase(const struct flash_area *area, uint32_t off, uint32_t len);
int flash_area_get_sector(const struct flash_area *area, uint32_t off, struct flash_sector *sector);
int flash_area_get_sectors(int area_id, uint32_t *count, struct flash_sector *sectors);
uint32_t flash_area_align(const struct flash_area *area);
uint8_t flash_area_erased_val(const struct flash_area *area);
int flash_area_id_from_multi_image_slot(int image, int slot);
int flash_area_id_from_image_slot(int slot);
int flash_area_id_to_multi_image_slot(int image, int id);
#endif
