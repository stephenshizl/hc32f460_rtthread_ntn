/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 4: External QSPI Flash (ZD25WQ16C) & FAL initialization
 */

#include "app_flash.h"
#include <drv_qspi.h>
#include <dev_spi_flash_sfud.h>
#include <fal.h>
#include "board_config.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.flash"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifndef FAL_USING_NOR_FLASH_DEV_NAME
#define FAL_USING_NOR_FLASH_DEV_NAME "norflash0"
#endif

#define QSPI_BUS_NAME   "qspi1"
#define QSPI_DEV_NAME   "qspi10"

static rt_bool_t s_flash_init_ok = RT_FALSE;

const struct fal_partition *app_flash_get_partition(const char *part_name)
{
    if (!s_flash_init_ok || part_name == RT_NULL)
    {
        return RT_NULL;
    }
    return fal_partition_find(part_name);
}

int app_flash_init(void)
{
    if (s_flash_init_ok)
    {
        return RT_EOK;
    }

    LOG_I("Initializing External QSPI NOR Flash (ZD25WQ16C 2MB)...");

    /* 1. Attach QSPI device to QSPI bus with hardware CS pin PB1 */
    if (rt_device_find(QSPI_DEV_NAME) == RT_NULL)
    {
        rt_err_t ret = rt_hw_qspi_bus_attach_device(QSPI_BUS_NAME, QSPI_DEV_NAME, FLASH_CS_PIN, 1, RT_NULL, RT_NULL);
        if (ret != RT_EOK)
        {
            LOG_E("Failed to attach %s to %s (err=%d)", QSPI_DEV_NAME, QSPI_BUS_NAME, ret);
            return ret;
        }
        LOG_D("Attached %s to %s successfully.", QSPI_DEV_NAME, QSPI_BUS_NAME);
    }

    /* 2. Probe Flash via SFUD (Serial Flash Universal Driver) */
    sfud_flash_t sfud_dev = rt_sfud_flash_find_by_dev_name(FAL_USING_NOR_FLASH_DEV_NAME);
    if (sfud_dev == RT_NULL)
    {
        rt_spi_flash_device_t flash_dev = rt_sfud_flash_probe(FAL_USING_NOR_FLASH_DEV_NAME, QSPI_DEV_NAME);
        if (flash_dev == RT_NULL)
        {
            LOG_E("SFUD flash probe failed on %s!", QSPI_DEV_NAME);
            return -RT_ERROR;
        }
        sfud_dev = rt_sfud_flash_find_by_dev_name(FAL_USING_NOR_FLASH_DEV_NAME);
    }

    if (sfud_dev != RT_NULL)
    {
        LOG_I("SFUD probed successfully: Chip=%s, Capacity=%d KB, EraseGran=%d B",
              sfud_dev->chip.name,
              sfud_dev->chip.capacity / 1024,
              sfud_dev->chip.erase_gran);
    }

    /* 3. Initialize FAL (Flash Abstraction Layer) */
    fal_init();

    s_flash_init_ok = RT_TRUE;
    LOG_I("FAL partitions initialized successfully.");
    return RT_EOK;
}
INIT_APP_EXPORT(app_flash_init);

/* MSH Diagnostics */
static void flash_info(int argc, char **argv)
{
    app_flash_init();

    sfud_flash_t sfud_dev = rt_sfud_flash_find_by_dev_name(FAL_USING_NOR_FLASH_DEV_NAME);
    rt_kprintf("\n================= Flash & FAL Status =================\n");
    if (sfud_dev != RT_NULL)
    {
        rt_kprintf(" Flash Device  : %s\n", FAL_USING_NOR_FLASH_DEV_NAME);
        rt_kprintf(" Chip Name     : %s\n", sfud_dev->chip.name);
        rt_kprintf(" Total Size    : %d KB (%d MB)\n", sfud_dev->chip.capacity / 1024, sfud_dev->chip.capacity / (1024 * 1024));
        rt_kprintf(" Sector Size   : %d Bytes\n", sfud_dev->chip.erase_gran);
    }
    else
    {
        rt_kprintf(" Flash Device  : NOT PROBED\n");
    }

    rt_kprintf("\n FAL Partition Table:\n");
    rt_kprintf(" -------------------------------------------------------------\n");
    rt_kprintf(" | %-14s | %-12s | %-10s | %-10s |\n", "Name", "Flash Dev", "Offset", "Length");
    rt_kprintf(" -------------------------------------------------------------\n");

    const char *parts[] = { "app", "download", "factory", "kvdb", "store_forward" };
    for (int i = 0; i < sizeof(parts)/sizeof(parts[0]); i++)
    {
        const struct fal_partition *p = fal_partition_find(parts[i]);
        if (p != RT_NULL)
        {
            rt_kprintf(" | %-14s | %-12s | 0x%08X | 0x%08X (%4d KB) |\n",
                       p->name, p->flash_name, p->offset, p->len, p->len / 1024);
        }
        else
        {
            rt_kprintf(" | %-14s | [NOT REGISTERED]                          |\n", parts[i]);
        }
    }
    rt_kprintf(" =============================================================\n\n");
}
MSH_CMD_EXPORT(flash_info, Display external Flash chip info and FAL partition table);

static void flash_bench(int argc, char **argv)
{
    const char *part_name = "download";
    if (argc > 1)
    {
        part_name = argv[1];
    }

    const struct fal_partition *part = fal_partition_find(part_name);
    if (part == RT_NULL)
    {
        rt_kprintf("Partition '%s' not found!\n", part_name);
        return;
    }

    rt_kprintf("Running Flash Benchmark on partition '%s' (size=%d KB)...\n", part->name, part->len / 1024);

    #define BENCH_BUF_SZ 4096
    rt_uint8_t *w_buf = (rt_uint8_t *)rt_malloc(BENCH_BUF_SZ);
    rt_uint8_t *r_buf = (rt_uint8_t *)rt_malloc(BENCH_BUF_SZ);
    if (!w_buf || !r_buf)
    {
        rt_kprintf("Out of memory for bench buffer!\n");
        if (w_buf) rt_free(w_buf);
        if (r_buf) rt_free(r_buf);
        return;
    }

    for (int i = 0; i < BENCH_BUF_SZ; i++)
    {
        w_buf[i] = (rt_uint8_t)(i ^ 0xA5);
    }

    rt_tick_t t0 = 0, t_erase = 0, t_write = 0, t_read = 0;
    int ret = 0;

    /* 1. Erase */
    t0 = rt_tick_get();
    ret = fal_partition_erase(part, 0, BENCH_BUF_SZ);
    t_erase = rt_tick_get() - t0;
    if (ret < 0)
    {
        rt_kprintf("Erase failed: %d\n", ret);
        goto __bench_exit;
    }

    /* 2. Write */
    t0 = rt_tick_get();
    ret = fal_partition_write(part, 0, w_buf, BENCH_BUF_SZ);
    t_write = rt_tick_get() - t0;
    if (ret < 0)
    {
        rt_kprintf("Write failed: %d\n", ret);
        goto __bench_exit;
    }

    /* 3. Read */
    t0 = rt_tick_get();
    ret = fal_partition_read(part, 0, r_buf, BENCH_BUF_SZ);
    t_read = rt_tick_get() - t0;
    if (ret < 0)
    {
        rt_kprintf("Read failed: %d\n", ret);
        goto __bench_exit;
    }

    /* 4. Verify */
    if (rt_memcmp(w_buf, r_buf, BENCH_BUF_SZ) == 0)
    {
        rt_kprintf("Benchmark PASSED!\n");
        rt_kprintf(" Erase 4KB: %d ms\n", t_erase * (1000 / RT_TICK_PER_SECOND));
        rt_kprintf(" Write 4KB: %d ms\n", t_write * (1000 / RT_TICK_PER_SECOND));
        rt_kprintf(" Read  4KB: %d ms\n", t_read  * (1000 / RT_TICK_PER_SECOND));
    }
    else
    {
        rt_kprintf("Data verification FAILED!\n");
    }

__bench_exit:
    rt_free(w_buf);
    rt_free(r_buf);
}
MSH_CMD_EXPORT(flash_bench, Test Flash erase/write/read on FAL partition: flash_bench [part_name]);
