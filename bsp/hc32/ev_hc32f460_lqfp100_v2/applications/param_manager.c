/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 5.2: Device Parameter NVRAM Management Framework
 */

#include "param_manager.h"
#include "app_flash.h"
#include <fal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.param"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define PARAM_SECTOR_SIZE   4096
#define PARAM_SECTOR_A_OFF  0
#define PARAM_SECTOR_B_OFF  4096

static const struct fal_partition *s_param_part = RT_NULL;
static struct rt_mutex             s_param_lock;
static param_config_t              s_config;
static rt_uint8_t                  s_active_sec = 0;   /* 0: Sector A, 1: Sector B */
static rt_bool_t                   s_param_inited = RT_FALSE;

/* IEEE 802.3 CRC-32 calculation */
static rt_uint32_t param_calc_crc(const param_config_t *cfg)
{
    rt_uint32_t crc = 0xFFFFFFFF;
    const rt_uint8_t *p = (const rt_uint8_t *)cfg;
    rt_size_t len = sizeof(param_config_t) - sizeof(rt_uint32_t); /* Exclude crc32 field itself */

    for (rt_size_t i = 0; i < len; i++)
    {
        crc ^= p[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ 0xEDB88320;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static void param_set_defaults(param_config_t *cfg)
{
    rt_memset(cfg, 0, sizeof(param_config_t));
    cfg->magic             = PARAM_MAGIC;
    cfg->version           = 1;
    cfg->save_tick         = rt_tick_get();

    cfg->work_mode         = PARAM_MODE_RAW_PASS;   /* Default: Transparent RS232 (IR-9680 compatible) */
    cfg->rs232_idle_ms     = 20;                    /* 20ms inter-character idle timeout */
    cfg->rs232_baud        = 19200;                 /* 19200 bps */
    cfg->report_interval_s = 60;                    /* 60 seconds telemetry interval */

    rt_strncpy(cfg->ble_name, "NTN-Terminal", PARAM_STR_MAX_LEN - 1);
    rt_strncpy(cfg->ntn_apn, "iridium.iot", PARAM_STR_MAX_LEN - 1);
    rt_strncpy(cfg->ntn_srv_ip, "120.79.100.18", PARAM_STR_MAX_LEN - 1);
    cfg->ntn_srv_port      = 8888;
    cfg->ntn_socket_id     = 1;
    cfg->log_output_level  = 2;                     /* INFO level */

    rt_strncpy(cfg->dev_sn, "NTN20261014001", PARAM_SN_MAX_LEN - 1);
    rt_strncpy(cfg->dev_hw_ver, "HC32F460-V2.0", 15);
    rt_strncpy(cfg->dev_sw_ver, "V1.2.0", 15);

    cfg->crc32 = param_calc_crc(cfg);
}

const param_config_t *param_get_active(void)
{
    return &s_config;
}

rt_err_t param_save(void)
{
    if (!s_param_inited || !s_param_part)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);

    /* Target the alternate sector for fail-safe Ping-Pong operation */
    rt_uint8_t target_sec = (s_active_sec == 0) ? 1 : 0;
    rt_uint32_t target_off = (target_sec == 0) ? PARAM_SECTOR_A_OFF : PARAM_SECTOR_B_OFF;

    s_config.version++;
    s_config.save_tick = rt_tick_get();
    s_config.crc32 = param_calc_crc(&s_config);

    LOG_D("Committing config revision #%d to Flash Sector %d (off=0x%05X)...",
          s_config.version, target_sec, target_off);

    /* 1. Erase target sector */
    int ret = fal_partition_erase(s_param_part, target_off, PARAM_SECTOR_SIZE);
    if (ret < 0)
    {
        LOG_E("Failed to erase Flash sector %d (err=%d)", target_sec, ret);
        s_config.version--;
        rt_mutex_release(&s_param_lock);
        return -RT_EIO;
    }

    /* 2. Write new config */
    ret = fal_partition_write(s_param_part, target_off, (const rt_uint8_t *)&s_config, sizeof(param_config_t));
    if (ret < 0)
    {
        LOG_E("Failed to write config to sector %d (err=%d)", target_sec, ret);
        s_config.version--;
        rt_mutex_release(&s_param_lock);
        return -RT_EIO;
    }

    /* 3. Read back & verify CRC */
    param_config_t verify_buf;
    ret = fal_partition_read(s_param_part, target_off, (rt_uint8_t *)&verify_buf, sizeof(param_config_t));
    if (ret < 0 || verify_buf.magic != PARAM_MAGIC || verify_buf.crc32 != s_config.crc32)
    {
        LOG_E("Verification readback failed on sector %d!", target_sec);
        s_config.version--;
        rt_mutex_release(&s_param_lock);
        return -RT_EIO;
    }

    s_active_sec = target_sec;
    rt_mutex_release(&s_param_lock);

    LOG_I("Config revision #%d saved successfully to Sector %d (CRC=0x%08X).",
          s_config.version, s_active_sec, s_config.crc32);
    return RT_EOK;
}

rt_err_t param_reset_factory(void)
{
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    LOG_W("Resetting all parameters to factory defaults...");
    param_set_defaults(&s_config);
    rt_mutex_release(&s_param_lock);

    return param_save();
}

/* Operational Mode */
rt_uint8_t param_get_work_mode(void)
{
    return s_config.work_mode;
}

rt_err_t param_set_work_mode(rt_uint8_t mode, rt_bool_t save_now)
{
    if (mode > PARAM_MODE_MODBUS)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    s_config.work_mode = mode;
    rt_mutex_release(&s_param_lock);

    if (save_now)
    {
        return param_save();
    }
    return RT_EOK;
}

/* Report Interval */
rt_uint32_t param_get_report_interval(void)
{
    return s_config.report_interval_s;
}

rt_err_t param_set_report_interval(rt_uint32_t seconds, rt_bool_t save_now)
{
    if (seconds < 5 || seconds > 86400)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    s_config.report_interval_s = seconds;
    rt_mutex_release(&s_param_lock);

    if (save_now)
    {
        return param_save();
    }
    return RT_EOK;
}

/* RS232 Baudrate */
rt_uint32_t param_get_rs232_baud(void)
{
    return s_config.rs232_baud;
}

rt_err_t param_set_rs232_baud(rt_uint32_t baud, rt_bool_t save_now)
{
    if (baud != 9600 && baud != 19200 && baud != 38400 && baud != 57600 && baud != 115200)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    s_config.rs232_baud = baud;
    rt_mutex_release(&s_param_lock);

    if (save_now)
    {
        return param_save();
    }
    return RT_EOK;
}

/* Satellite Remote Server */
const char *param_get_ntn_server(rt_uint16_t *out_port)
{
    if (out_port)
    {
        *out_port = s_config.ntn_srv_port;
    }
    return s_config.ntn_srv_ip;
}

rt_err_t param_set_ntn_server(const char *ip_or_host, rt_uint16_t port, rt_bool_t save_now)
{
    if (!ip_or_host || port == 0)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    rt_strncpy(s_config.ntn_srv_ip, ip_or_host, PARAM_STR_MAX_LEN - 1);
    s_config.ntn_srv_ip[PARAM_STR_MAX_LEN - 1] = '\0';
    s_config.ntn_srv_port = port;
    rt_mutex_release(&s_param_lock);

    if (save_now)
    {
        return param_save();
    }
    return RT_EOK;
}

/* BLE Name */
const char *param_get_ble_name(void)
{
    return s_config.ble_name;
}

rt_err_t param_set_ble_name(const char *name, rt_bool_t save_now)
{
    if (!name || rt_strlen(name) == 0)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    rt_strncpy(s_config.ble_name, name, PARAM_STR_MAX_LEN - 1);
    s_config.ble_name[PARAM_STR_MAX_LEN - 1] = '\0';
    rt_mutex_release(&s_param_lock);

    if (save_now)
    {
        return param_save();
    }
    return RT_EOK;
}

/* Device SN */
const char *param_get_dev_sn(void)
{
    return s_config.dev_sn;
}

rt_err_t param_set_dev_sn(const char *sn, rt_bool_t save_now)
{
    if (!sn || rt_strlen(sn) == 0)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    rt_strncpy(s_config.dev_sn, sn, PARAM_SN_MAX_LEN - 1);
    s_config.dev_sn[PARAM_SN_MAX_LEN - 1] = '\0';
    rt_mutex_release(&s_param_lock);

    if (save_now)
    {
        return param_save();
    }
    return RT_EOK;
}

/* Boot loader: Read Sector A & Sector B */
int param_manager_init(void)
{
    if (s_param_inited)
    {
        return RT_EOK;
    }

    /* Ensure Flash & FAL are mounted */
    app_flash_init();

    s_param_part = fal_partition_find(PARAM_PARTITION_NAME);
    if (s_param_part == RT_NULL)
    {
        LOG_E("Partition '%s' not found in FAL table!", PARAM_PARTITION_NAME);
        return -RT_ERROR;
    }

    rt_mutex_init(&s_param_lock, "parm_lck", RT_IPC_FLAG_PRIO);

    param_config_t cfg_a, cfg_b;
    rt_bool_t a_valid = RT_FALSE;
    rt_bool_t b_valid = RT_FALSE;

    /* Read Sector A */
    if (fal_partition_read(s_param_part, PARAM_SECTOR_A_OFF, (rt_uint8_t *)&cfg_a, sizeof(param_config_t)) >= 0)
    {
        if (cfg_a.magic == PARAM_MAGIC && cfg_a.crc32 == param_calc_crc(&cfg_a))
        {
            a_valid = RT_TRUE;
        }
    }

    /* Read Sector B */
    if (fal_partition_read(s_param_part, PARAM_SECTOR_B_OFF, (rt_uint8_t *)&cfg_b, sizeof(param_config_t)) >= 0)
    {
        if (cfg_b.magic == PARAM_MAGIC && cfg_b.crc32 == param_calc_crc(&cfg_b))
        {
            b_valid = RT_TRUE;
        }
    }

    if (a_valid && b_valid)
    {
        /* Choose the one with the higher revision counter */
        if (cfg_a.version >= cfg_b.version)
        {
            s_config = cfg_a;
            s_active_sec = 0;
        }
        else
        {
            s_config = cfg_b;
            s_active_sec = 1;
        }
        LOG_I("Loaded NVRAM config revision #%d from Sector %d (CRC=0x%08X)",
              s_config.version, s_active_sec, s_config.crc32);
    }
    else if (a_valid)
    {
        s_config = cfg_a;
        s_active_sec = 0;
        LOG_I("Loaded NVRAM config revision #%d from Sector A (CRC=0x%08X)",
              s_config.version, s_config.crc32);
    }
    else if (b_valid)
    {
        s_config = cfg_b;
        s_active_sec = 1;
        LOG_I("Loaded NVRAM config revision #%d from Sector B (CRC=0x%08X)",
              s_config.version, s_config.crc32);
    }
    else
    {
        /* First boot or unformatted flash -> Apply Factory Defaults */
        LOG_W("No valid NVRAM config found in Flash. Initializing factory defaults...");
        s_param_inited = RT_TRUE;
        param_reset_factory();
        return RT_EOK;
    }

    s_param_inited = RT_TRUE;
    return RT_EOK;
}
INIT_APP_EXPORT(param_manager_init);

/* MSH Diagnostic Commands */
static void param_dump(int argc, char **argv)
{
    const param_config_t *c = param_get_active();

    rt_kprintf("\n================ NVRAM Configuration (kvdb) ================\n");
    rt_kprintf(" Revision        : #%d\n", c->version);
    rt_kprintf(" Active Sector   : %s (Offset: 0x%05X)\n",
               s_active_sec == 0 ? "Sector A" : "Sector B",
               s_active_sec == 0 ? PARAM_SECTOR_A_OFF : PARAM_SECTOR_B_OFF);
    rt_kprintf(" CRC-32          : 0x%08X\n", c->crc32);
    rt_kprintf(" ------------------------------------------------------------\n");

    const char *mode_str = "UNKNOWN";
    if (c->work_mode == PARAM_MODE_RAW_PASS) mode_str = "RAW_PASS (0: Transparent RS232)";
    else if (c->work_mode == PARAM_MODE_AUTO_TELEMETRY) mode_str = "AUTO_TELEMETRY (1: GNSS/Sensor Auto Report)";
    else if (c->work_mode == PARAM_MODE_MODBUS) mode_str = "MODBUS (2: Industrial Polling)";

    rt_kprintf(" Work Mode       : %s\n", mode_str);
    rt_kprintf(" Report Interval : %d Seconds\n", c->report_interval_s);
    rt_kprintf(" RS232 Baudrate  : %d bps (Idle: %d ms)\n", c->rs232_baud, c->rs232_idle_ms);
    rt_kprintf(" BLE Name        : %s\n", c->ble_name);
    rt_kprintf(" NTN APN         : %s\n", c->ntn_apn);
    rt_kprintf(" NTN Server      : %s:%d (Socket %d)\n", c->ntn_srv_ip, c->ntn_srv_port, c->ntn_socket_id);
    rt_kprintf(" Log Level       : %d\n", c->log_output_level);
    rt_kprintf(" ------------------------------------------------------------\n");
    rt_kprintf(" Serial Number   : %s\n", c->dev_sn);
    rt_kprintf(" Hardware Ver    : %s\n", c->dev_hw_ver);
    rt_kprintf(" Firmware Ver    : %s\n", c->dev_sw_ver);
    rt_kprintf("=============================================================\n\n");
}
MSH_CMD_EXPORT(param_dump, Dump all active NVRAM configuration parameters);

static void param_get(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: param_get <key>\n");
        rt_kprintf("Keys: mode, baud, interval, ble, apn, server, port, sn, ver\n");
        return;
    }

    const char *key = argv[1];
    const param_config_t *c = param_get_active();

    if (rt_strcmp(key, "mode") == 0)
    {
        rt_kprintf("mode = %d\n", c->work_mode);
    }
    else if (rt_strcmp(key, "baud") == 0)
    {
        rt_kprintf("baud = %d\n", c->rs232_baud);
    }
    else if (rt_strcmp(key, "interval") == 0)
    {
        rt_kprintf("interval = %d\n", c->report_interval_s);
    }
    else if (rt_strcmp(key, "ble") == 0)
    {
        rt_kprintf("ble = %s\n", c->ble_name);
    }
    else if (rt_strcmp(key, "apn") == 0)
    {
        rt_kprintf("apn = %s\n", c->ntn_apn);
    }
    else if (rt_strcmp(key, "server") == 0)
    {
        rt_kprintf("server = %s\n", c->ntn_srv_ip);
    }
    else if (rt_strcmp(key, "port") == 0)
    {
        rt_kprintf("port = %d\n", c->ntn_srv_port);
    }
    else if (rt_strcmp(key, "sn") == 0)
    {
        rt_kprintf("sn = %s\n", c->dev_sn);
    }
    else if (rt_strcmp(key, "ver") == 0)
    {
        rt_kprintf("hw_ver = %s, sw_ver = %s\n", c->dev_hw_ver, c->dev_sw_ver);
    }
    else
    {
        rt_kprintf("Unknown key: %s\n", key);
    }
}
MSH_CMD_EXPORT(param_get, Query a configuration parameter: param_get <key>);

static void param_set(int argc, char **argv)
{
    if (argc < 3)
    {
        rt_kprintf("Usage: param_set <key> <val>\n");
        rt_kprintf("Examples:\n");
        rt_kprintf("  param_set mode 1\n");
        rt_kprintf("  param_set interval 300\n");
        rt_kprintf("  param_set baud 115200\n");
        rt_kprintf("  param_set ble MyNTN-01\n");
        rt_kprintf("  param_set server 120.79.100.18\n");
        rt_kprintf("  param_set port 8888\n");
        rt_kprintf("  param_set sn NTN20261014999\n");
        return;
    }

    const char *key = argv[1];
    const char *val = argv[2];
    rt_err_t res = RT_EOK;

    if (rt_strcmp(key, "mode") == 0)
    {
        res = param_set_work_mode((rt_uint8_t)atoi(val), RT_TRUE);
    }
    else if (rt_strcmp(key, "baud") == 0)
    {
        res = param_set_rs232_baud((rt_uint32_t)atoi(val), RT_TRUE);
    }
    else if (rt_strcmp(key, "interval") == 0)
    {
        res = param_set_report_interval((rt_uint32_t)atoi(val), RT_TRUE);
    }
    else if (rt_strcmp(key, "ble") == 0)
    {
        res = param_set_ble_name(val, RT_TRUE);
    }
    else if (rt_strcmp(key, "server") == 0)
    {
        rt_uint16_t port = s_config.ntn_srv_port;
        res = param_set_ntn_server(val, port, RT_TRUE);
    }
    else if (rt_strcmp(key, "port") == 0)
    {
        res = param_set_ntn_server(s_config.ntn_srv_ip, (rt_uint16_t)atoi(val), RT_TRUE);
    }
    else if (rt_strcmp(key, "sn") == 0)
    {
        res = param_set_dev_sn(val, RT_TRUE);
    }
    else
    {
        rt_kprintf("Unknown key: %s\n", key);
        return;
    }

    if (res == RT_EOK)
    {
        rt_kprintf("Successfully updated '%s' to '%s' and saved to Flash.\n", key, val);
    }
    else
    {
        rt_kprintf("Failed to set '%s' (err=%d)\n", key, res);
    }
}
MSH_CMD_EXPORT(param_set, Set and persist a configuration parameter: param_set <key> <val>);

static void param_reset(int argc, char **argv)
{
    if (param_reset_factory() == RT_EOK)
    {
        rt_kprintf("Factory defaults restored and committed to Flash successfully.\n");
    }
    else
    {
        rt_kprintf("Failed to restore factory defaults!\n");
    }
}
MSH_CMD_EXPORT(param_reset, Restore all parameters to factory defaults);

static void param_save_cmd(int argc, char **argv)
{
    if (param_save() == RT_EOK)
    {
        rt_kprintf("Active configuration committed to Flash.\n");
    }
    else
    {
        rt_kprintf("Failed to commit configuration!\n");
    }
}
MSH_CMD_EXPORT_ALIAS(param_save_cmd, param_save, Commit active parameters to Flash);
