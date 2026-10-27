/*
 * Copyright (c) 2026, youyoutech.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 4: External QSPI Flash & FAL initialization
 */

#ifndef __APP_FLASH_H__
#define __APP_FLASH_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <fal.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize QSPI Flash bus, attach device, probe SFUD, and init FAL
 * @return RT_EOK on success, negative error code on failure
 */
int app_flash_init(void);

/**
 * @brief Get FAL partition handle by name
 * @param part_name Partition name ("download", "factory", "kvdb", "store_forward", "app")
 * @return Partition pointer or RT_NULL if not found
 */
const struct fal_partition *app_flash_get_partition(const char *part_name);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FLASH_H__ */
