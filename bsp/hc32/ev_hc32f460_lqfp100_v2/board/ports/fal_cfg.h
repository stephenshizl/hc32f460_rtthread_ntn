/*
 * Copyright (C) 2022-2024, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-28     CDT          first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtthread.h>
#include <board.h>

/* enable hc32f4 onchip flash driver sample */
#define FAL_FLASH_PORT_DRIVER_HC32F4
/* enable SFUD flash driver sample */
#define FAL_FLASH_PORT_DRIVER_SFUD

extern const struct fal_flash_dev hc32_onchip_flash;
extern struct fal_flash_dev ext_nor_flash0;

/* flash device table */
#define FAL_FLASH_DEV_TABLE \
    {                       \
        &hc32_onchip_flash, \
        &ext_nor_flash0,    \
    }

#ifndef FAL_USING_NOR_FLASH_DEV_NAME
#define FAL_USING_NOR_FLASH_DEV_NAME "norflash0"
#endif

/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
#if defined(NTN_OTA_APP) || defined(NTN_OTA_BL2)
#include "ota_layout.h"
#define FAL_PART_TABLE \
    { \
        { FAL_PART_MAGIC_WROD, "app", "onchip_flash", NTN_OTA_PRIMARY, NTN_OTA_SLOT_SIZE, 0 }, \
        { FAL_PART_MAGIC_WROD, "download", FAL_USING_NOR_FLASH_DEV_NAME, NTN_OTA_SECONDARY, NTN_OTA_SLOT_SIZE, 0 }, \
        { FAL_PART_MAGIC_WROD, "factory", FAL_USING_NOR_FLASH_DEV_NAME, NTN_OTA_FACTORY, NTN_OTA_FACTORY_SIZE, 0 }, \
        { FAL_PART_MAGIC_WROD, "kvdb", FAL_USING_NOR_FLASH_DEV_NAME, NTN_OTA_KVDB, NTN_OTA_KVDB_SIZE, 0 }, \
        { FAL_PART_MAGIC_WROD, "store_forward", FAL_USING_NOR_FLASH_DEV_NAME, NTN_OTA_QUEUE, NTN_OTA_QUEUE_SIZE, 0 }, \
        { FAL_PART_MAGIC_WROD, "ota_scratch", FAL_USING_NOR_FLASH_DEV_NAME, NTN_OTA_SCRATCH, NTN_OTA_SCRATCH_SIZE, 0 }, \
        { FAL_PART_MAGIC_WROD, "ota_log_a", FAL_USING_NOR_FLASH_DEV_NAME, NTN_OTA_LOG_A, NTN_OTA_NOR_SECTOR, 0 }, \
        { FAL_PART_MAGIC_WROD, "ota_log_b", FAL_USING_NOR_FLASH_DEV_NAME, NTN_OTA_LOG_B, NTN_OTA_NOR_SECTOR, 0 }, \
        { FAL_PART_MAGIC_WROD, "reserved", FAL_USING_NOR_FLASH_DEV_NAME, NTN_OTA_NOR_RESERVED, \
          NTN_OTA_NOR_SIZE - NTN_OTA_NOR_RESERVED, 0 }, \
    }
#else
/* BY25Q64ES: 8 MiB / 64 Mbit. Preserve the existing first 2 MiB layout. */
#define FAL_PART_TABLE                                                                                  \
    {                                                                                                   \
        { FAL_PART_MAGIC_WROD, "app",           "onchip_flash",                0, 512 * 1024, 0 },      \
        { FAL_PART_MAGIC_WROD, "download",      FAL_USING_NOR_FLASH_DEV_NAME,  0, 512 * 1024, 0 },      \
        { FAL_PART_MAGIC_WROD, "factory",       FAL_USING_NOR_FLASH_DEV_NAME,  512 * 1024, 512 * 1024, 0 }, \
        { FAL_PART_MAGIC_WROD, "kvdb",          FAL_USING_NOR_FLASH_DEV_NAME,  1024 * 1024, 256 * 1024, 0 }, \
        { FAL_PART_MAGIC_WROD, "store_forward", FAL_USING_NOR_FLASH_DEV_NAME,  (1024 + 256) * 1024, 768 * 1024, 0 }, \
        { FAL_PART_MAGIC_WROD, "reserved",      FAL_USING_NOR_FLASH_DEV_NAME,  2 * 1024 * 1024, 6 * 1024 * 1024, 0 }, \
    }
#endif
#endif /* FAL_PART_HAS_TABLE_CFG */

#endif /* _FAL_CFG_H_ */
