/* SPDX-License-Identifier: Apache-2.0 */

#include "param_manager.h"
#include "app_flash.h"
#include <fal.h>
#include <string.h>

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.param"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define PARAM_SECTOR_SIZE   4096U
#define PARAM_SECTOR_A_OFF  0U
#define PARAM_SECTOR_B_OFF  PARAM_SECTOR_SIZE

static const struct fal_partition *s_param_part = RT_NULL;
static struct rt_mutex s_param_lock;
static param_config_t s_config;
static rt_uint8_t s_active_sec;
static rt_bool_t s_param_inited;
static rt_bool_t s_persisted_config;

static rt_uint32_t param_calc_crc(const param_config_t *cfg)
{
    rt_uint32_t crc = 0xFFFFFFFFUL;
    const rt_uint8_t *p = (const rt_uint8_t *)cfg;
    rt_size_t len = sizeof(*cfg) - sizeof(cfg->crc32);
    rt_size_t i;

    for (i = 0U; i < len; i++)
    {
        rt_uint8_t bit;
        crc ^= p[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return ~crc;
}

static void param_set_defaults(param_config_t *cfg)
{
    rt_memset(cfg, 0, sizeof(*cfg));
    cfg->magic = PARAM_MAGIC;
    cfg->schema_version = PARAM_SCHEMA_VERSION;
    cfg->struct_size = (rt_uint16_t)sizeof(*cfg);
    cfg->work_mode = PARAM_MODE_AT_PASSTHROUGH;
    cfg->transfer_mode = PARAM_TRANSFER_RAW;
    cfg->rx_mode = PARAM_RX_QUERY;
    cfg->dns_mode = 1U;
    cfg->pc_cfg_enabled = 1U;
    cfg->rs232_baud = 19200U;
    cfg->raw_block_len = 128U;
    cfg->raw_gap_ms = 100U;
    cfg->raw_wait_ms = 1000U;
    cfg->report_interval_s = 60U;
    cfg->location_interval_s = 60U;
    cfg->next_report_id = 1U;
    rt_strncpy(cfg->ble_name, "NTN-Terminal", sizeof(cfg->ble_name) - 1U);
    rt_strncpy(cfg->dev_hw_ver, "HC32F460-V2.0", sizeof(cfg->dev_hw_ver) - 1U);
    rt_strncpy(cfg->dev_sw_ver, "V3.0.0", sizeof(cfg->dev_sw_ver) - 1U);
    cfg->crc32 = param_calc_crc(cfg);
}

static rt_bool_t param_baud_is_valid(rt_uint32_t baud)
{
    return (baud == 9600U || baud == 19200U || baud == 38400U ||
            baud == 57600U || baud == 115200U) ? RT_TRUE : RT_FALSE;
}

static rt_bool_t param_ipv4_is_valid(const char *text)
{
    rt_uint8_t groups = 0U;
    const char *p = text;

    if (text == RT_NULL || *text == '\0')
    {
        return RT_FALSE;
    }
    while (*p != '\0')
    {
        rt_uint32_t value = 0U;
        rt_uint8_t digits = 0U;
        while (*p >= '0' && *p <= '9')
        {
            value = value * 10U + (rt_uint32_t)(*p - '0');
            digits++;
            if (digits > 3U || value > 255U)
            {
                return RT_FALSE;
            }
            p++;
        }
        if (digits == 0U)
        {
            return RT_FALSE;
        }
        groups++;
        if (*p == '\0')
        {
            break;
        }
        if (*p != '.' || groups >= 4U)
        {
            return RT_FALSE;
        }
        p++;
    }
    return (groups == 4U) ? RT_TRUE : RT_FALSE;
}

rt_bool_t param_host_is_valid(const char *host)
{
    rt_size_t len;
    rt_size_t label_len = 0U;
    rt_size_t i;
    rt_bool_t all_numeric_or_dot = RT_TRUE;

    if (host == RT_NULL)
    {
        return RT_FALSE;
    }
    len = rt_strlen(host);
    if (len == 0U || len > PARAM_HOST_MAX_LEN)
    {
        return RT_FALSE;
    }
    if (rt_strncmp(host, "http://", 7U) == 0 || rt_strncmp(host, "https://", 8U) == 0)
    {
        return RT_FALSE;
    }

    for (i = 0U; i < len; i++)
    {
        char ch = host[i];
        rt_bool_t alnum = ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
                           (ch >= '0' && ch <= '9')) ? RT_TRUE : RT_FALSE;
        if (!(alnum || ch == '-' || ch == '.'))
        {
            return RT_FALSE;
        }
        if (!((ch >= '0' && ch <= '9') || ch == '.'))
        {
            all_numeric_or_dot = RT_FALSE;
        }
        if (ch == '.')
        {
            if (label_len == 0U || label_len > 63U || host[i - 1U] == '-')
            {
                return RT_FALSE;
            }
            label_len = 0U;
        }
        else
        {
            if (label_len == 0U && ch == '-')
            {
                return RT_FALSE;
            }
            label_len++;
        }
    }
    if (label_len == 0U || label_len > 63U || host[len - 1U] == '-')
    {
        return RT_FALSE;
    }
    return all_numeric_or_dot ? param_ipv4_is_valid(host) : RT_TRUE;
}

static rt_bool_t param_config_is_valid(const param_config_t *cfg)
{
    if (cfg->magic != PARAM_MAGIC || cfg->schema_version != PARAM_SCHEMA_VERSION ||
        cfg->struct_size != sizeof(*cfg) || cfg->crc32 != param_calc_crc(cfg))
    {
        return RT_FALSE;
    }
    if (cfg->work_mode > PARAM_MODE_RAW_UPLINK || cfg->transfer_mode != PARAM_TRANSFER_RAW ||
        cfg->rx_mode > PARAM_RX_URC_PUSH || cfg->dns_mode > 2U ||
        !param_baud_is_valid(cfg->rs232_baud) ||
        cfg->raw_block_len == 0U || cfg->raw_block_len > 256U ||
        cfg->raw_gap_ms == 0U || cfg->raw_gap_ms > 10000U ||
        cfg->raw_wait_ms < cfg->raw_gap_ms || cfg->raw_wait_ms > 60000U)
    {
        return RT_FALSE;
    }
    if (cfg->next_report_id == 0U ||
        cfg->report_interval_s < 10U || cfg->report_interval_s > 86400U ||
        cfg->location_interval_s < 10U || cfg->location_interval_s > 86400U)
    {
        return RT_FALSE;
    }
    if (cfg->host[0] != '\0' && !param_host_is_valid(cfg->host))
    {
        return RT_FALSE;
    }
    return RT_TRUE;
}

static rt_err_t param_finish_update(const param_config_t *before, rt_bool_t save_now)
{
    rt_err_t result;
    if (!save_now)
    {
        return RT_EOK;
    }
    result = param_save();
    if (result != RT_EOK)
    {
        rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
        s_config = *before;
        rt_mutex_release(&s_param_lock);
    }
    return result;
}

const param_config_t *param_get_active(void)
{
    return &s_config;
}

void param_get_snapshot(param_config_t *out_config)
{
    if (out_config == RT_NULL || !s_param_inited)
    {
        return;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    *out_config = s_config;
    rt_mutex_release(&s_param_lock);
}

rt_bool_t param_has_persisted_config(void)
{
    return s_persisted_config;
}

rt_err_t param_save(void)
{
    param_config_t verify;
    rt_uint8_t target_sec;
    rt_uint32_t target_off;
    rt_uint32_t previous_revision;
    int result;

    if (!s_param_inited || s_param_part == RT_NULL)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    target_sec = (s_active_sec == 0U) ? 1U : 0U;
    target_off = (target_sec == 0U) ? PARAM_SECTOR_A_OFF : PARAM_SECTOR_B_OFF;
    previous_revision = s_config.revision;
    s_config.revision++;
    s_config.save_tick = rt_tick_get();
    s_config.crc32 = param_calc_crc(&s_config);

    result = fal_partition_erase(s_param_part, target_off, PARAM_SECTOR_SIZE);
    if (result >= 0)
    {
        result = fal_partition_write(s_param_part, target_off,
                                     (const rt_uint8_t *)&s_config, sizeof(s_config));
    }
    if (result >= 0)
    {
        result = fal_partition_read(s_param_part, target_off,
                                    (rt_uint8_t *)&verify, sizeof(verify));
    }
    if (result < 0 || !param_config_is_valid(&verify) ||
        rt_memcmp(&verify, &s_config, sizeof(verify)) != 0)
    {
        s_config.revision = previous_revision;
        s_config.crc32 = param_calc_crc(&s_config);
        rt_mutex_release(&s_param_lock);
        LOG_E("Configuration save/readback failed at sector %u", target_sec);
        return -RT_EIO;
    }
    s_active_sec = target_sec;
    s_persisted_config = RT_TRUE;
    rt_mutex_release(&s_param_lock);
    LOG_I("Configuration schema %u revision %u saved to sector %u",
          PARAM_SCHEMA_VERSION, s_config.revision, s_active_sec);
    return RT_EOK;
}

rt_err_t param_reset_factory(void)
{
    param_config_t before;
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    param_set_defaults(&s_config);
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, RT_TRUE);
}

rt_err_t param_reset_operational(void)
{
    param_config_t before;
    char sn[PARAM_SN_MAX_LEN + 1U];
    char ble_name[PARAM_BLE_NAME_MAX_LEN + 1U];
    char hw_ver[sizeof(s_config.dev_hw_ver)];
    char sw_ver[sizeof(s_config.dev_sw_ver)];

    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    rt_memcpy(sn, s_config.dev_sn, sizeof(sn));
    rt_memcpy(ble_name, s_config.ble_name, sizeof(ble_name));
    rt_memcpy(hw_ver, s_config.dev_hw_ver, sizeof(hw_ver));
    rt_memcpy(sw_ver, s_config.dev_sw_ver, sizeof(sw_ver));
    param_set_defaults(&s_config);
    rt_memcpy(s_config.dev_sn, sn, sizeof(sn));
    rt_memcpy(s_config.ble_name, ble_name, sizeof(ble_name));
    rt_memcpy(s_config.dev_hw_ver, hw_ver, sizeof(hw_ver));
    rt_memcpy(s_config.dev_sw_ver, sw_ver, sizeof(sw_ver));
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, RT_TRUE);
}

rt_uint8_t param_get_work_mode(void)
{
    return s_config.work_mode;
}

rt_err_t param_set_work_mode(rt_uint8_t mode, rt_bool_t save_now)
{
    param_config_t before;
    if (mode > PARAM_MODE_RAW_UPLINK)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.work_mode = mode;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_uint8_t param_get_transfer_mode(void)
{
    return s_config.transfer_mode;
}

rt_err_t param_set_transfer_mode(rt_uint8_t mode, rt_bool_t save_now)
{
    param_config_t before;
    if (mode != PARAM_TRANSFER_RAW)
    {
        return -RT_ENOSYS;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.transfer_mode = mode;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_uint32_t param_get_report_interval(void)
{
    return s_config.report_interval_s;
}

rt_err_t param_set_report_interval(rt_uint32_t seconds, rt_bool_t save_now)
{
    param_config_t before;
    if (seconds < 10U || seconds > 86400U)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.report_interval_s = seconds;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_uint32_t param_get_rs232_baud(void)
{
    return s_config.rs232_baud;
}

rt_err_t param_set_rs232_baud(rt_uint32_t baud, rt_bool_t save_now)
{
    param_config_t before;
    if (!param_baud_is_valid(baud))
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.rs232_baud = baud;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

const char *param_get_ntn_server(rt_uint16_t *out_port)
{
    if (out_port != RT_NULL)
    {
        *out_port = s_config.remote_port;
    }
    return s_config.host;
}

rt_err_t param_set_host(const char *host, rt_bool_t save_now)
{
    param_config_t before;
    if (!param_host_is_valid(host))
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    rt_strncpy(s_config.host, host, sizeof(s_config.host) - 1U);
    s_config.host[sizeof(s_config.host) - 1U] = '\0';
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_set_ports(rt_uint16_t remote_port, rt_uint16_t local_port, rt_bool_t save_now)
{
    param_config_t before;
    if (remote_port == 0U)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.remote_port = remote_port;
    s_config.local_port = local_port;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_set_ntn_server(const char *host, rt_uint16_t port, rt_bool_t save_now)
{
    param_config_t before;
    if (!param_host_is_valid(host) || port == 0U)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    rt_strncpy(s_config.host, host, sizeof(s_config.host) - 1U);
    s_config.host[sizeof(s_config.host) - 1U] = '\0';
    s_config.remote_port = port;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_bool_t param_endpoint_is_configured(void)
{
    return (s_config.host[0] != '\0' && s_config.remote_port != 0U) ? RT_TRUE : RT_FALSE;
}

rt_uint32_t param_get_endpoint_identity(void)
{
    param_config_t cfg;
    rt_uint32_t hash = 2166136261UL;
    const rt_uint8_t *p;
    rt_size_t i;

    param_get_snapshot(&cfg);
    if (cfg.host[0] == '\0' || cfg.remote_port == 0U)
    {
        return 0U; /* Data received before first endpoint configuration is unbound. */
    }
    p = (const rt_uint8_t *)cfg.host;
    for (i = 0U; i < rt_strlen(cfg.host); i++)
    {
        hash = (hash ^ p[i]) * 16777619UL;
    }
    hash = (hash ^ (rt_uint8_t)(cfg.remote_port & 0xFFU)) * 16777619UL;
    hash = (hash ^ (rt_uint8_t)(cfg.remote_port >> 8)) * 16777619UL;
    hash = (hash ^ (rt_uint8_t)(cfg.local_port & 0xFFU)) * 16777619UL;
    hash = (hash ^ (rt_uint8_t)(cfg.local_port >> 8)) * 16777619UL;
    return hash;
}

rt_err_t param_set_apn(const char *apn, rt_bool_t save_now)
{
    param_config_t before;
    rt_size_t i;
    rt_size_t len;
    if (apn == RT_NULL)
    {
        return -RT_EINVAL;
    }
    len = rt_strlen(apn);
    if (len > PARAM_APN_MAX_LEN)
    {
        return -RT_EINVAL;
    }
    for (i = 0U; i < len; i++)
    {
        char ch = apn[i];
        if (ch == ',' || ch == '"' || ch == '\r' || ch == '\n' || ch == ' ' || ch == '\t')
        {
            return -RT_EINVAL;
        }
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    rt_strncpy(s_config.ntn_apn, apn, sizeof(s_config.ntn_apn) - 1U);
    s_config.ntn_apn[sizeof(s_config.ntn_apn) - 1U] = '\0';
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_set_dns(rt_uint8_t mode, const char *primary,
                       const char *secondary, rt_bool_t save_now)
{
    param_config_t before;

    if (mode > 2U || primary == RT_NULL || secondary == RT_NULL)
    {
        return -RT_EINVAL;
    }
    if ((primary[0] != '\0' && !param_ipv4_is_valid(primary)) ||
        (secondary[0] != '\0' && !param_ipv4_is_valid(secondary)) ||
        (mode == 2U && primary[0] == '\0'))
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.dns_mode = mode;
    rt_strncpy(s_config.dns_primary, primary, sizeof(s_config.dns_primary) - 1U);
    s_config.dns_primary[sizeof(s_config.dns_primary) - 1U] = '\0';
    rt_strncpy(s_config.dns_secondary, secondary, sizeof(s_config.dns_secondary) - 1U);
    s_config.dns_secondary[sizeof(s_config.dns_secondary) - 1U] = '\0';
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_set_rx_mode(rt_uint8_t mode, rt_bool_t save_now)
{
    param_config_t before;

    if (mode > PARAM_RX_URC_PUSH)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.rx_mode = mode;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_set_pc_cfg_enabled(rt_bool_t enabled, rt_bool_t save_now)
{
    param_config_t before;
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.pc_cfg_enabled = enabled ? 1U : 0U;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_set_ntn_location_enabled(rt_bool_t enabled, rt_bool_t save_now)
{
    param_config_t before;
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.ntn_location_enabled = enabled ? 1U : 0U;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_set_asset_report(rt_bool_t enabled, rt_uint32_t period_s,
                                rt_bool_t save_now)
{
    param_config_t before;
    if (period_s < 10U || period_s > 86400U) return -RT_EINVAL;
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.asset_report_enabled = enabled ? 1U : 0U;
    s_config.report_interval_s = period_s;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_set_location_report(rt_bool_t enabled, rt_uint32_t period_s,
                                   rt_bool_t save_now)
{
    param_config_t before;
    if (period_s < 10U || period_s > 86400U) return -RT_EINVAL;
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.location_report_enabled = enabled ? 1U : 0U;
    s_config.location_interval_s = period_s;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_set_asset_location(rt_bool_t enabled, rt_bool_t save_now)
{
    param_config_t before;
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.asset_location_enabled = enabled ? 1U : 0U;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

rt_err_t param_reserve_report_ids(rt_uint32_t count, rt_uint64_t *out_first_id)
{
    param_config_t before;
    rt_uint64_t first;
    rt_err_t result;
    if (count == 0U || out_first_id == RT_NULL) return -RT_EINVAL;
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    first = s_config.next_report_id;
    if (first == 0U || first + count < first)
    {
        rt_mutex_release(&s_param_lock);
        return -RT_EFULL;
    }
    s_config.next_report_id += count;
    rt_mutex_release(&s_param_lock);
    result = param_finish_update(&before, RT_TRUE);
    if (result == RT_EOK) *out_first_id = first;
    return result;
}

rt_err_t param_set_raw_pack(rt_uint16_t gap_ms, rt_uint16_t block_len,
                            rt_uint16_t wait_ms, rt_bool_t save_now)
{
    param_config_t before;
    if (gap_ms < 1U || gap_ms > 10000U || block_len < 1U || block_len > 256U ||
        wait_ms < 10U || wait_ms > 60000U || wait_ms < gap_ms)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    s_config.raw_gap_ms = gap_ms;
    s_config.raw_block_len = block_len;
    s_config.raw_wait_ms = wait_ms;
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

const char *param_get_ble_name(void)
{
    return s_config.ble_name;
}

rt_err_t param_set_ble_name(const char *name, rt_bool_t save_now)
{
    param_config_t before;
    if (name == RT_NULL || name[0] == '\0' || rt_strlen(name) > PARAM_BLE_NAME_MAX_LEN)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    rt_strncpy(s_config.ble_name, name, sizeof(s_config.ble_name) - 1U);
    s_config.ble_name[sizeof(s_config.ble_name) - 1U] = '\0';
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

const char *param_get_dev_sn(void)
{
    return s_config.dev_sn;
}

rt_err_t param_set_dev_sn(const char *sn, rt_bool_t save_now)
{
    param_config_t before;
    rt_size_t i;
    rt_size_t len;
    if (sn == RT_NULL)
    {
        return -RT_EINVAL;
    }
    len = rt_strlen(sn);
    if (len == 0U || len > PARAM_SN_MAX_LEN)
    {
        return -RT_EINVAL;
    }
    for (i = 0U; i < len; i++)
    {
        char ch = sn[i];
        if (!((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
              (ch >= '0' && ch <= '9') || ch == '_' || ch == '-'))
        {
            return -RT_EINVAL;
        }
    }
    rt_mutex_take(&s_param_lock, RT_WAITING_FOREVER);
    before = s_config;
    rt_strncpy(s_config.dev_sn, sn, sizeof(s_config.dev_sn) - 1U);
    s_config.dev_sn[sizeof(s_config.dev_sn) - 1U] = '\0';
    rt_mutex_release(&s_param_lock);
    return param_finish_update(&before, save_now);
}

int param_manager_init(void)
{
    param_config_t cfg_a;
    param_config_t cfg_b;
    rt_bool_t a_valid;
    rt_bool_t b_valid;

    if (s_param_inited)
    {
        return RT_EOK;
    }
    /* Keep local interfaces usable even when external storage is unavailable. */
    if (rt_mutex_init(&s_param_lock, "parm_lck", RT_IPC_FLAG_PRIO) != RT_EOK)
    {
        return -RT_ERROR;
    }
    param_set_defaults(&s_config);
    s_active_sec = 0U;
    s_persisted_config = RT_FALSE;
    if (app_flash_init() != RT_EOK)
    {
        LOG_W("Flash unavailable; using RAM defaults (RS232 19200, mode 0), saving disabled");
        s_param_inited = RT_TRUE;
        return RT_EOK;
    }
    s_param_part = fal_partition_find(PARAM_PARTITION_NAME);
    if (s_param_part == RT_NULL)
    {
        LOG_W("Partition '%s' not found; using RAM defaults, saving disabled", PARAM_PARTITION_NAME);
        s_param_inited = RT_TRUE;
        return RT_EOK;
    }
    a_valid = (fal_partition_read(s_param_part, PARAM_SECTOR_A_OFF,
                                  (rt_uint8_t *)&cfg_a, sizeof(cfg_a)) >= 0 &&
               param_config_is_valid(&cfg_a)) ? RT_TRUE : RT_FALSE;
    b_valid = (fal_partition_read(s_param_part, PARAM_SECTOR_B_OFF,
                                  (rt_uint8_t *)&cfg_b, sizeof(cfg_b)) >= 0 &&
               param_config_is_valid(&cfg_b)) ? RT_TRUE : RT_FALSE;

    if (a_valid || b_valid)
    {
        if (a_valid && (!b_valid || cfg_a.revision >= cfg_b.revision))
        {
            s_config = cfg_a;
            s_active_sec = 0U;
        }
        else
        {
            s_config = cfg_b;
            s_active_sec = 1U;
        }
        s_persisted_config = RT_TRUE;
        LOG_I("Loaded schema %u revision %u from sector %u",
              s_config.schema_version, s_config.revision, s_active_sec);
    }
    else
    {
        /* Preserve unknown/legacy contents until an explicit V3 setting is saved. */
        param_set_defaults(&s_config);
        s_active_sec = 0U;
        s_persisted_config = RT_FALSE;
        LOG_W("No valid schema-3 config; using safe RAM defaults in mode 0");
    }
    s_param_inited = RT_TRUE;
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(param_manager_init);

static void param_dump(int argc, char **argv)
{
    param_config_t c;
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    param_get_snapshot(&c);
    rt_kprintf("schema=%u revision=%u persisted=%u mode=%u transfer=%u\n",
               c.schema_version, c.revision, s_persisted_config, c.work_mode, c.transfer_mode);
    rt_kprintf("rs232=%u host='%s' remote=%u local=%u apn='%s'\n",
               c.rs232_baud, c.host, c.remote_port, c.local_port, c.ntn_apn);
    rt_kprintf("rx_mode=%u dns_mode=%u raw=%u/%u/%u sn='%s'\n",
               c.rx_mode, c.dns_mode, c.raw_gap_ms, c.raw_block_len,
               c.raw_wait_ms, c.dev_sn);
}
MSH_CMD_EXPORT(param_dump, dump schema-3 terminal configuration);
