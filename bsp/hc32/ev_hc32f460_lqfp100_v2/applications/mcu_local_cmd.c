/* SPDX-License-Identifier: Apache-2.0 */

#include "mcu_local_cmd.h"
#include "app_telemetry.h"
#include "param_manager.h"
#include "pc_rs232.h"
#include "raw_uplink.h"
#include "ntn_ms150.h"
#include "store_forward.h"
#include "gnss_um220.h"
#include "loc2_report.h"
#include <stdlib.h>
#include <string.h>

#define MCU_CMD_COPY_MAX  384U

static rt_uint32_t s_boot_id;
static rt_uint32_t s_confirm_nonce;
static rt_tick_t s_confirm_expiry;
static rt_uint8_t s_confirm_action;

#define MCU_CONFIRM_NONE       0U
#define MCU_CONFIRM_NVCLR      1U
#define MCU_CONFIRM_QUEUECLR   2U

static void cmd_error(char *out, rt_size_t out_size, int code)
{
    rt_snprintf(out, out_size, "\r\n+MCUERROR:%d\r\nERROR\r\n", code);
}

static rt_bool_t parse_u32(const char *text, rt_uint32_t *value)
{
    char *end;
    unsigned long parsed;
    if (text == RT_NULL || *text == '\0')
    {
        return RT_FALSE;
    }
    parsed = strtoul(text, &end, 10);
    if (*end != '\0')
    {
        return RT_FALSE;
    }
    *value = (rt_uint32_t)parsed;
    return RT_TRUE;
}

static rt_bool_t cmd_is_query(const char *cmd, const char *name)
{
    rt_size_t name_len = rt_strlen(name);
    if (rt_strncmp(cmd, name, name_len) != 0)
    {
        return RT_FALSE;
    }
    return (cmd[name_len] == '\0' ||
            (cmd[name_len] == '?' && cmd[name_len + 1U] == '\0')) ? RT_TRUE : RT_FALSE;
}

static const char *cmd_set_value(const char *cmd, const char *name)
{
    rt_size_t name_len = rt_strlen(name);
    if (rt_strncmp(cmd, name, name_len) == 0 && cmd[name_len] == '=')
    {
        return &cmd[name_len + 1U];
    }
    return RT_NULL;
}

static rt_bool_t handle_identity(const char *cmd, char *out, rt_size_t out_size)
{
    param_config_t cfg;
    if (!(cmd_is_query(cmd, "AT+MCUHELLO") || cmd_is_query(cmd, "AT+MCUVER")))
    {
        return RT_FALSE;
    }
    if (s_boot_id == 0U)
    {
        s_boot_id = rt_tick_get() + 1U;
    }
    param_get_snapshot(&cfg);
    rt_snprintf(out, out_size,
                "\r\n+MCUVER:{\"boot_id\":%u,\"fw\":\"%s\",\"hw\":\"%s\","
                "\"srs\":\"3.0\",\"ctrl\":2,\"build\":\"%s %s\","
                "\"auth_required\":false}\r\nOK\r\n",
                s_boot_id, cfg.dev_sw_ver, cfg.dev_hw_ver, __DATE__, __TIME__);
    return RT_TRUE;
}

static rt_bool_t handle_capability(const char *cmd, char *out, rt_size_t out_size)
{
    if (!cmd_is_query(cmd, "AT+MCUCAP"))
    {
        return RT_FALSE;
    }
    rt_snprintf(out, out_size,
                "\r\n+MCUCAP:{\"udp\":true,\"domain\":true,\"rx_query\":true,"
                "\"rx_push\":true,\"rx_push_verified\":false,"
                "\"downlink_to_rs232\":false,\"upgrade\":false,\"raw_max\":256}\r\nOK\r\n");
    return RT_TRUE;
}

static rt_bool_t handle_mode(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value = cmd_set_value(cmd, "AT+MCUMODE");
    param_config_t cfg;
    rt_uint32_t mode;
    rt_err_t result;

    if (cmd_is_query(cmd, "AT+MCUMODE") || cmd_is_query(cmd, "AT+SETWORKMODE"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUMODE:{\"saved\":%u,\"effective\":%u,\"revision\":%u}\r\nOK\r\n",
                    cfg.work_mode, cfg.work_mode, cfg.revision);
        return RT_TRUE;
    }
    if (value == RT_NULL)
    {
        value = cmd_set_value(cmd, "AT+SETWORKMODE");
    }
    if (value == RT_NULL)
    {
        return RT_FALSE;
    }
    if (!parse_u32(value, &mode) || mode > 1U)
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    result = param_set_work_mode((rt_uint8_t)mode, RT_TRUE);
    if (result != RT_EOK)
    {
        cmd_error(out, out_size, 3);
        return RT_TRUE;
    }
    param_get_snapshot(&cfg);
    rt_snprintf(out, out_size,
                "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"rev\":%u,\"mode\":%u}\r\nOK\r\n",
                cfg.revision, cfg.work_mode);
    return RT_TRUE;
}

static rt_bool_t handle_transfer_mode(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value = cmd_set_value(cmd, "AT+MCUTRANSFERMODE");
    rt_uint32_t mode;
    if (cmd_is_query(cmd, "AT+MCUTRANSFERMODE"))
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUTRANSFERMODE:{\"saved\":0,\"effective\":0,\"name\":\"RAW\"}\r\nOK\r\n");
        return RT_TRUE;
    }
    if (value == RT_NULL)
    {
        return RT_FALSE;
    }
    if (!parse_u32(value, &mode))
    {
        cmd_error(out, out_size, 1);
    }
    else if (mode != 0U)
    {
        cmd_error(out, out_size, 2);
    }
    else
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"transfermode\":0}\r\nOK\r\n");
    }
    return RT_TRUE;
}

static rt_bool_t handle_host(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value = cmd_set_value(cmd, "AT+MCUHOST");
    param_config_t cfg;
    rt_err_t result;
    if (cmd_is_query(cmd, "AT+MCUHOST") || cmd_is_query(cmd, "AT+MCUIP"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUHOST:{\"host\":\"%s\",\"configured\":%s,"
                    "\"remote_port\":%u,\"endpoint_id\":\"%08X\","
                    "\"revision\":%u}\r\nOK\r\n",
                    cfg.host, param_endpoint_is_configured() ? "true" : "false",
                    cfg.remote_port, param_get_endpoint_identity(), cfg.revision);
        return RT_TRUE;
    }
    if (value == RT_NULL)
    {
        value = cmd_set_value(cmd, "AT+MCUIP");
    }
    if (value == RT_NULL)
    {
        return RT_FALSE;
    }
    result = param_set_host(value, RT_TRUE);
    if (result != RT_EOK)
    {
        cmd_error(out, out_size, (result == -RT_EINVAL) ? 1 : 3);
    }
    else
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"rev\":%u,"
                    "\"host\":\"%s\"}\r\nOK\r\n", cfg.revision, cfg.host);
    }
    return RT_TRUE;
}

static rt_bool_t handle_port(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value = cmd_set_value(cmd, "AT+MCUPORT");
    param_config_t cfg;
    rt_uint32_t remote_port;
    rt_uint32_t local_port;
    char *comma;
    char copy[24];

    if (cmd_is_query(cmd, "AT+MCUPORT"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUPORT:{\"remote\":%u,\"local\":%u,\"revision\":%u}\r\nOK\r\n",
                    cfg.remote_port, cfg.local_port, cfg.revision);
        return RT_TRUE;
    }
    if (value == RT_NULL)
    {
        return RT_FALSE;
    }
    if (rt_strlen(value) >= sizeof(copy))
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    rt_strncpy(copy, value, sizeof(copy) - 1U);
    copy[sizeof(copy) - 1U] = '\0';
    comma = strchr(copy, ',');
    if (comma == RT_NULL)
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    *comma = '\0';
    if (!parse_u32(copy, &remote_port) || !parse_u32(comma + 1, &local_port) ||
        remote_port == 0U || remote_port > 65535U || local_port > 65535U)
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    if (param_set_ports((rt_uint16_t)remote_port, (rt_uint16_t)local_port, RT_TRUE) != RT_EOK)
    {
        cmd_error(out, out_size, 3);
    }
    else
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"rev\":%u,"
                    "\"remote\":%u,\"local\":%u}\r\nOK\r\n",
                    cfg.revision, cfg.remote_port, cfg.local_port);
    }
    return RT_TRUE;
}

static rt_bool_t handle_baud(const char *cmd, char *out, rt_size_t out_size,
                             mcu_cmd_action_t *action)
{
    const char *value = cmd_set_value(cmd, "AT+MCURS232");
    rt_uint32_t baud;
    rt_uint32_t old_baud;
    char copy[40];
    char *suffix;

    if (cmd_is_query(cmd, "AT+MCURS232") || cmd_is_query(cmd, "AT+MCUBAUD"))
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCURS232:{\"saved_baud\":%u,\"effective_baud\":%u,"
                    "\"format\":\"8N1\"}\r\nOK\r\n",
                    param_get_rs232_baud(), pc_rs232_get_baudrate());
        return RT_TRUE;
    }
    if (value == RT_NULL)
    {
        value = cmd_set_value(cmd, "AT+MCUBAUD");
    }
    if (value == RT_NULL)
    {
        return RT_FALSE;
    }
    if (rt_strlen(value) >= sizeof(copy))
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    rt_strncpy(copy, value, sizeof(copy) - 1U);
    copy[sizeof(copy) - 1U] = '\0';
    suffix = strchr(copy, ',');
    if (suffix != RT_NULL)
    {
        *suffix++ = '\0';
        if (rt_strcmp(suffix, "8,N,1") != 0)
        {
            cmd_error(out, out_size, 1);
            return RT_TRUE;
        }
    }
    if (!parse_u32(copy, &baud))
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    old_baud = pc_rs232_get_baudrate();
    if (param_set_rs232_baud(baud, RT_TRUE) != RT_EOK)
    {
        cmd_error(out, out_size, 3);
        return RT_TRUE;
    }
    if (baud == old_baud)
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"baud\":%u}\r\nOK\r\n", baud);
    }
    else
    {
        action->type = MCU_CMD_ACTION_APPLY_RS232_BAUD;
        action->value = baud;
        action->previous_value = old_baud;
        rt_snprintf(out, out_size,
                    "\r\n+MCUPENDING:{\"action\":\"RS232_BAUD\",\"baud\":%u}\r\n", baud);
    }
    return RT_TRUE;
}

static rt_bool_t handle_raw_pack(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value = cmd_set_value(cmd, "AT+MCURAWPACK");
    param_config_t cfg;
    rt_uint32_t gap;
    rt_uint32_t block;
    rt_uint32_t wait;
    char copy[48];
    char *p1;
    char *p2;

    if (cmd_is_query(cmd, "AT+MCURAWPACK"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCURAWPACK:{\"gap_ms\":%u,\"block_len\":%u,"
                    "\"wait_ms\":%u}\r\nOK\r\n",
                    cfg.raw_gap_ms, cfg.raw_block_len, cfg.raw_wait_ms);
        return RT_TRUE;
    }
    if (value == RT_NULL)
    {
        return RT_FALSE;
    }
    if (rt_strlen(value) >= sizeof(copy))
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    rt_strncpy(copy, value, sizeof(copy) - 1U);
    copy[sizeof(copy) - 1U] = '\0';
    p1 = strchr(copy, ',');
    p2 = (p1 != RT_NULL) ? strchr(p1 + 1, ',') : RT_NULL;
    if (p1 == RT_NULL || p2 == RT_NULL)
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    *p1++ = '\0';
    *p2++ = '\0';
    if (!parse_u32(copy, &gap) || !parse_u32(p1, &block) || !parse_u32(p2, &wait) ||
        gap > 65535U || block > 65535U || wait > 65535U ||
        param_set_raw_pack((rt_uint16_t)gap, (rt_uint16_t)block,
                           (rt_uint16_t)wait, RT_TRUE) != RT_EOK)
    {
        cmd_error(out, out_size, 1);
    }
    else
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"raw_pack\":true}\r\nOK\r\n");
    }
    return RT_TRUE;
}

static rt_bool_t handle_text_setting(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value;
    param_config_t cfg;
    rt_err_t result;

    if (cmd_is_query(cmd, "AT+MCUAPN"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size, "\r\n+MCUAPN:{\"apn\":\"%s\"}\r\nOK\r\n", cfg.ntn_apn);
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUAPN");
    if (value != RT_NULL)
    {
        result = param_set_apn(value, RT_TRUE);
        if (result != RT_EOK) cmd_error(out, out_size, (result == -RT_EINVAL) ? 1 : 3);
        else rt_snprintf(out, out_size, "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"apn\":true}\r\nOK\r\n");
        return RT_TRUE;
    }

    if (cmd_is_query(cmd, "AT+MCUSN"))
    {
        rt_snprintf(out, out_size, "\r\n+MCUSN:{\"sn\":\"%s\"}\r\nOK\r\n", param_get_dev_sn());
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUSN");
    if (value != RT_NULL)
    {
        result = param_set_dev_sn(value, RT_TRUE);
        if (result != RT_EOK) cmd_error(out, out_size, (result == -RT_EINVAL) ? 1 : 3);
        else rt_snprintf(out, out_size, "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"sn\":true}\r\nOK\r\n");
        return RT_TRUE;
    }
    return RT_FALSE;
}

static rt_bool_t handle_dns(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value = cmd_set_value(cmd, "AT+MCUDNS");
    param_config_t cfg;
    rt_uint32_t mode;
    char copy[48];
    char *primary;
    char *secondary;
    char *comma;

    if (cmd_is_query(cmd, "AT+MCUDNS"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUDNS:{\"mode\":%u,\"primary\":\"%s\"," 
                    "\"secondary\":\"%s\"}\r\nOK\r\n",
                    cfg.dns_mode, cfg.dns_primary, cfg.dns_secondary);
        return RT_TRUE;
    }
    if (value == RT_NULL)
    {
        return RT_FALSE;
    }
    if (rt_strlen(value) >= sizeof(copy))
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    rt_strncpy(copy, value, sizeof(copy) - 1U);
    copy[sizeof(copy) - 1U] = '\0';
    comma = strchr(copy, ',');
    if (comma != RT_NULL)
    {
        *comma++ = '\0';
    }
    if (!parse_u32(copy, &mode) || mode > 2U)
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    primary = (comma != RT_NULL) ? comma : (char *)"";
    comma = strchr(primary, ',');
    if (comma != RT_NULL)
    {
        *comma++ = '\0';
        secondary = comma;
    }
    else
    {
        secondary = (char *)"";
    }
    if (param_set_dns((rt_uint8_t)mode, primary, secondary, RT_TRUE) != RT_EOK)
    {
        cmd_error(out, out_size, 1);
    }
    else
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"dns\":true}\r\nOK\r\n");
    }
    return RT_TRUE;
}

static rt_bool_t handle_rx_mode(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value = cmd_set_value(cmd, "AT+MCURXMODE");
    param_config_t cfg;
    rt_uint32_t mode;

    if (cmd_is_query(cmd, "AT+MCURXMODE"))
    {
        ntn_service_status_t status;
        param_get_snapshot(&cfg);
        ntn_service_get_status(&status);
        rt_snprintf(out, out_size,
                    "\r\n+MCURXMODE:{\"saved\":%u,\"effective\":%d,"
                    "\"name\":\"%s\",\"push_verified\":false}\r\nOK\r\n",
                    cfg.rx_mode, status.send_ready ? (int)cfg.rx_mode : -1,
                    cfg.rx_mode == PARAM_RX_QUERY ? "QUERY" : "URC");
        return RT_TRUE;
    }
    if (value == RT_NULL)
    {
        return RT_FALSE;
    }
    if (!parse_u32(value, &mode) || mode > PARAM_RX_URC_PUSH)
    {
        cmd_error(out, out_size, 1);
    }
    else if (mode == PARAM_RX_URC_PUSH)
    {
        /* Exact push URC syntax has not yet been verified on the target MS150. */
        cmd_error(out, out_size, 2);
    }
    else if (param_set_rx_mode((rt_uint8_t)mode, RT_TRUE) != RT_EOK)
    {
        cmd_error(out, out_size, 3);
    }
    else
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"rx_mode\":%u}\r\nOK\r\n",
                    mode);
    }
    return RT_TRUE;
}

static rt_bool_t handle_downlink(const char *cmd, char *out, rt_size_t out_size)
{
    ntn_downlink_t downlink;
    static const char hex[] = "0123456789ABCDEF";
    rt_size_t pos;
    rt_size_t i;

    if (!cmd_is_query(cmd, "AT+MCUDOWN"))
    {
        return RT_FALSE;
    }
    if (!ntn_service_get_last_downlink(&downlink))
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUDOWN:{\"valid\":false}\r\nOK\r\n");
        return RT_TRUE;
    }
    pos = (rt_size_t)rt_snprintf(out, out_size,
                                 "\r\n+MCUDOWN:{\"valid\":true,\"seq\":%u,"
                                 "\"socket_id\":%u,\"data_len\":%u,\"data_hex\":\"",
                                 downlink.sequence, downlink.socket_id,
                                 downlink.data_len);
    for (i = 0U; i < downlink.data_len && pos + 2U < out_size; i++)
    {
        out[pos++] = hex[downlink.data[i] >> 4];
        out[pos++] = hex[downlink.data[i] & 0x0FU];
    }
    if (pos + 12U < out_size)
    {
        rt_strncpy(&out[pos], "\"}\r\nOK\r\n", out_size - pos - 1U);
        out[out_size - 1U] = '\0';
    }
    else
    {
        cmd_error(out, out_size, 21);
    }
    return RT_TRUE;
}

static rt_bool_t handle_gnss_time(const char *cmd, char *out, rt_size_t out_size)
{
    gnss_data_t data;
    rt_bool_t fixed;
    rt_uint32_t age_ms;

    if (!(cmd_is_query(cmd, "AT+MCUGNSS") ||
          cmd_is_query(cmd, "AT+MCULOCATION") ||
          cmd_is_query(cmd, "AT+MCUTIME")))
    {
        return RT_FALSE;
    }
    fixed = gnss_get_data(&data);
    age_ms = data.last_update_tick == 0U ? 0xFFFFFFFFUL :
        (rt_uint32_t)(((rt_uint64_t)(rt_tick_get() - data.last_update_tick) * 1000U) /
                      RT_TICK_PER_SECOND);
    if (cmd_is_query(cmd, "AT+MCUTIME"))
    {
        if (data.time_valid)
        {
            rt_uint32_t time_age_ms = (rt_uint32_t)
                (((rt_uint64_t)(rt_tick_get() - data.time_update_tick) * 1000U) /
                 RT_TICK_PER_SECOND);
            rt_snprintf(out, out_size,
                        "\r\n+MCUTIME:1,%04u-%02u-%02uT%02u:%02u:%02uZ,GNSS,%u\r\nOK\r\n",
                        data.year, data.month, data.day,
                        data.hour, data.minute, data.second, time_age_ms);
        }
        else
        {
            rt_snprintf(out, out_size, "\r\n+MCUTIME:0,,GNSS,4294967295\r\nOK\r\n");
        }
        return RT_TRUE;
    }
    rt_snprintf(out, out_size,
                "\r\n+MCUGNSS:{\"configured\":%s,"
                "\"constellations\":\"GPS+BDS+GAL+SBAS+QZSS\","
                "\"valid\":%s,\"age_ms\":%u,"
                "\"lat_e7\":%d,\"lon_e7\":%d,\"alt_mm\":%d,"
                "\"alt_valid\":%s,\"speed_cm_s\":%u,\"sats\":%u,"
                "\"fix_quality\":%u}\r\nOK\r\n",
                gnss_is_configured() ? "true" : "false",
                (fixed && age_ms <= 5000U) ? "true" : "false", age_ms,
                data.latitude_e7, data.longitude_e7, data.altitude_mm,
                data.altitude_valid ? "true" : "false", data.speed_cm_s,
                data.sats_used, data.fix_quality);
    return RT_TRUE;
}

static rt_bool_t handle_fixed_mode_settings(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value;
    rt_uint32_t setting;
    param_config_t cfg;

    if (cmd_is_query(cmd, "AT+MCUDATAMODE"))
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUDATAMODE:{\"saved\":1,\"effective\":1,\"name\":\"HEX\"}\r\nOK\r\n");
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUDATAMODE");
    if (value != RT_NULL)
    {
        if (!parse_u32(value, &setting) || setting != 1U) cmd_error(out, out_size, 1);
        else rt_snprintf(out, out_size,
                         "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"data_mode\":1}\r\nOK\r\n");
        return RT_TRUE;
    }
    if (cmd_is_query(cmd, "AT+MCUNTNLOC"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUNTNLOC:{\"enabled\":%u}\r\nOK\r\n",
                    cfg.ntn_location_enabled);
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUNTNLOC");
    if (value == RT_NULL) return RT_FALSE;
    if (!parse_u32(value, &setting) || setting > 1U ||
        param_set_ntn_location_enabled(setting ? RT_TRUE : RT_FALSE, RT_TRUE) != RT_EOK)
    {
        cmd_error(out, out_size, 1);
    }
    else
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"ntn_location\":%u}\r\nOK\r\n",
                    setting);
    }
    return RT_TRUE;
}

static rt_bool_t handle_report_settings(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value;
    param_config_t cfg;
    rt_uint32_t enabled;
    rt_uint32_t period;
    char copy[32];
    char *comma;
    rt_err_t result;

    if (cmd_is_query(cmd, "AT+MCUASSET") || cmd_is_query(cmd, "AT+MCULOCREPORT"))
    {
        rt_bool_t asset = cmd_is_query(cmd, "AT+MCUASSET");
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+%s:{\"enabled\":%u,\"period_s\":%u}\r\nOK\r\n",
                    asset ? "MCUASSET" : "MCULOCREPORT",
                    asset ? cfg.asset_report_enabled : cfg.location_report_enabled,
                    asset ? cfg.report_interval_s : cfg.location_interval_s);
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUASSET");
    if (value == RT_NULL) value = cmd_set_value(cmd, "AT+MCULOCREPORT");
    if (value != RT_NULL)
    {
        rt_bool_t asset = (rt_strncmp(cmd, "AT+MCUASSET=", 12U) == 0) ? RT_TRUE : RT_FALSE;
        if (rt_strlen(value) >= sizeof(copy))
        {
            cmd_error(out, out_size, 1);
            return RT_TRUE;
        }
        rt_strncpy(copy, value, sizeof(copy) - 1U);
        copy[sizeof(copy) - 1U] = '\0';
        comma = strchr(copy, ',');
        if (comma == RT_NULL)
        {
            cmd_error(out, out_size, 1);
            return RT_TRUE;
        }
        *comma++ = '\0';
        if (!parse_u32(copy, &enabled) || enabled > 1U ||
            !parse_u32(comma, &period))
        {
            cmd_error(out, out_size, 1);
            return RT_TRUE;
        }
        result = asset ? param_set_asset_report(enabled ? RT_TRUE : RT_FALSE, period, RT_TRUE) :
                         param_set_location_report(enabled ? RT_TRUE : RT_FALSE, period, RT_TRUE);
        if (result != RT_EOK) cmd_error(out, out_size, result == -RT_EINVAL ? 1 : 3);
        else rt_snprintf(out, out_size,
                         "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"report\":\"%s\"}\r\nOK\r\n",
                         asset ? "ASSET" : "LOCATION");
        return RT_TRUE;
    }
    if (cmd_is_query(cmd, "AT+MCUASSETLOC"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUASSETLOC:{\"enabled\":%u}\r\nOK\r\n",
                    cfg.asset_location_enabled);
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUASSETLOC");
    if (value != RT_NULL)
    {
        if (!parse_u32(value, &enabled) || enabled > 1U ||
            param_set_asset_location(enabled ? RT_TRUE : RT_FALSE, RT_TRUE) != RT_EOK)
            cmd_error(out, out_size, 1);
        else rt_snprintf(out, out_size,
                         "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"asset_location\":%u}\r\nOK\r\n",
                         enabled);
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUREPORTNOW");
    if (value != RT_NULL)
    {
        if (!parse_u32(value, &enabled) || enabled > 1U)
        {
            cmd_error(out, out_size, 1);
            return RT_TRUE;
        }
        result = loc2_report_now(enabled == 0U ? LOC2_TYPE_ASSET : LOC2_TYPE_LOCATION);
        if (result == RT_EOK)
            rt_snprintf(out, out_size,
                        "\r\n+MCUREPORTNOW:{\"state\":\"ENQUEUED\",\"type\":%u}\r\nOK\r\n",
                        enabled);
        else if (result == -RT_EEMPTY) cmd_error(out, out_size, 9);
        else if (result == -RT_ENOSYS) cmd_error(out, out_size, 11);
        else if (result == -RT_EFULL) cmd_error(out, out_size, 7);
        else if (result == -RT_EIO) cmd_error(out, out_size, 8);
        else cmd_error(out, out_size, 5);
        return RT_TRUE;
    }
    return RT_FALSE;
}

static rt_bool_t handle_status(const char *cmd, char *out, rt_size_t out_size)
{
    raw_uplink_stats_t raw;
    ntn_service_status_t ntn;
    sf_stats_t flash;
    param_config_t cfg;
    gnss_data_t gnss;
    rt_bool_t gnss_fixed;
    rt_uint32_t gnss_age_ms;

    if (!(cmd_is_query(cmd, "AT+MCUSTAT") || cmd_is_query(cmd, "AT+MCUQUEUE")))
    {
        return RT_FALSE;
    }
    raw_uplink_get_stats(&raw);
    ntn_service_get_status(&ntn);
    store_forward_get_stats(&flash);
    param_get_snapshot(&cfg);
    gnss_fixed = gnss_get_data(&gnss);
    gnss_age_ms = gnss.last_update_tick == 0U ? 0xFFFFFFFFUL :
        (rt_uint32_t)(((rt_uint64_t)(rt_tick_get() - gnss.last_update_tick) * 1000U) /
                      RT_TICK_PER_SECOND);
    rt_snprintf(out, out_size,
                "\r\n+MCUSTAT:{\"mode\":%u,\"ntn_state\":%u,\"registered\":%u,"
                "\"socket_open\":%s,\"host\":\"%s\",\"rx_mode\":%u,"
                "\"rx_push_verified\":false,\"delivery_policy\":\"MODEM_ACCEPTED\","
                "\"last_error\":%d,\"ram_blocks\":%u,"
                "\"ram_bytes\":%u,\"building_bytes\":%u,\"input_bytes\":%u,"
                "\"accepted_bytes\":%u,\"rejected_bytes\":%u,"
                "\"rejected_blocks\":%u,\"modem_accepted_blocks\":%u,"
                "\"unknown_attempts\":%u,\"uart_overrun\":0,"
                "\"flash_pending\":%u,\"flash_used\":%u,\"flash_usable\":%u,"
                "\"storage_fault\":%s,\"history_unavailable\":%s,"
                "\"held_endpoint\":%u,\"downlink_count\":%u,"
                "\"downlink_parse_errors\":%u,\"gnss_valid\":%s,"
                "\"time_valid\":%s,\"sleep_state\":\"IDLE_ONLY\"}\r\nOK\r\n",
                cfg.work_mode, ntn.state, ntn.reg_status,
                ntn.send_ready ? "true" : "false", cfg.host, cfg.rx_mode,
                ntn.last_error,
                raw.queued_blocks, raw.queued_bytes, raw.building_bytes,
                raw.input_bytes, raw.modem_accepted_bytes, raw.rejected_bytes,
                raw.rejected_blocks, ntn.modem_accepted_count,
                ntn.unknown_result_count,
                flash.total_pending, flash.used_slots, flash.usable_slots,
                flash.storage_fault ? "true" : "false",
                flash.history_unavailable ? "true" : "false",
                raw.held_endpoint_blocks, ntn.downlink_count,
                ntn.downlink_parse_errors,
                (gnss_fixed && gnss_age_ms <= 5000U) ? "true" : "false",
                gnss.time_valid ? "true" : "false");
    return RT_TRUE;
}

static rt_bool_t handle_control_session(const char *cmd, char *out,
                                        rt_size_t out_size,
                                        mcu_cmd_action_t *action)
{
    const char *value;
    rt_uint32_t enabled;
    param_config_t cfg;

    if (cmd_is_query(cmd, "AT+MCUCFG"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUCFG:{\"pc_entry_enabled\":%u,\"active\":%s}\r\nOK\r\n",
                    cfg.pc_cfg_enabled,
                    app_telemetry_pc_config_active() ? "true" : "false");
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUCFG");
    if (value != RT_NULL)
    {
        if (!parse_u32(value, &enabled) || enabled > 1U)
        {
            cmd_error(out, out_size, 1);
        }
        else if (param_get_work_mode() != PARAM_MODE_RAW_UPLINK)
        {
            cmd_error(out, out_size, 5);
        }
        else
        {
            action->type = enabled ? MCU_CMD_ACTION_ENTER_PC_CONFIG :
                                     MCU_CMD_ACTION_EXIT_PC_CONFIG;
            rt_snprintf(out, out_size,
                        "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"pc_config\":%s}\r\nOK\r\n",
                        enabled ? "true" : "false");
        }
        return RT_TRUE;
    }

    if (cmd_is_query(cmd, "AT+MCUPCCFG"))
    {
        param_get_snapshot(&cfg);
        rt_snprintf(out, out_size,
                    "\r\n+MCUPCCFG:{\"enabled\":%u}\r\nOK\r\n",
                    cfg.pc_cfg_enabled);
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUPCCFG");
    if (value == RT_NULL) return RT_FALSE;
    if (!parse_u32(value, &enabled) || enabled > 1U)
    {
        cmd_error(out, out_size, 1);
    }
    else if (param_set_pc_cfg_enabled(enabled ? RT_TRUE : RT_FALSE,
                                      RT_TRUE) != RT_EOK)
    {
        cmd_error(out, out_size, 3);
    }
    else
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"pc_entry_enabled\":%s}\r\nOK\r\n",
                    enabled ? "true" : "false");
    }
    return RT_TRUE;
}

static rt_bool_t handle_ntn_lock(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value = cmd_set_value(cmd, "AT+MCUNTNLOCK");
    rt_uint32_t enabled;
    rt_err_t result;

    if (cmd_is_query(cmd, "AT+MCUNTNLOCK"))
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUNTNLOCK:{\"locked\":%s,\"idle_timeout_ms\":60000}\r\nOK\r\n",
                    ntn_debug_lock_is_held() ? "true" : "false");
        return RT_TRUE;
    }
    if (value == RT_NULL) return RT_FALSE;
    if (!parse_u32(value, &enabled) || enabled > 1U)
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    if (enabled)
    {
        result = ntn_debug_lock_acquire();
        if (result != RT_EOK)
        {
            cmd_error(out, out_size, 6);
            return RT_TRUE;
        }
    }
    else
    {
        ntn_debug_lock_release();
    }
    rt_snprintf(out, out_size,
                "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"ntn_lock\":%s}\r\nOK\r\n",
                enabled ? "true" : "false");
    return RT_TRUE;
}

static rt_uint32_t confirm_new_nonce(rt_uint8_t action)
{
    if (s_boot_id == 0U) s_boot_id = rt_tick_get() + 1U;
    s_confirm_nonce = (s_confirm_nonce + 0x1021U) ^ rt_tick_get() ^ s_boot_id;
    if (s_confirm_nonce == 0U) s_confirm_nonce = 1U;
    s_confirm_action = action;
    s_confirm_expiry = rt_tick_get() + rt_tick_from_millisecond(10000U);
    return s_confirm_nonce;
}

static rt_bool_t handle_destructive(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value;
    const char *comma;
    rt_uint32_t nonce;
    rt_uint8_t requested_action;
    rt_err_t result;

    if (rt_strcmp(cmd, "AT+MCUNVCLR") == 0 || rt_strcmp(cmd, "AT+NVCLR") == 0)
    {
        nonce = confirm_new_nonce(MCU_CONFIRM_NVCLR);
        rt_snprintf(out, out_size,
                    "\r\n+MCUCONFIRMREQ:{\"action\":\"NVCLR\",\"nonce\":%u,\"expires_ms\":10000}\r\n+MCUERROR:14\r\nERROR\r\n",
                    nonce);
        return RT_TRUE;
    }
    if (rt_strcmp(cmd, "AT+MCUQUEUECLR") == 0)
    {
        nonce = confirm_new_nonce(MCU_CONFIRM_QUEUECLR);
        rt_snprintf(out, out_size,
                    "\r\n+MCUCONFIRMREQ:{\"action\":\"QUEUECLR\",\"nonce\":%u,\"expires_ms\":10000}\r\n+MCUERROR:14\r\nERROR\r\n",
                    nonce);
        return RT_TRUE;
    }
    value = cmd_set_value(cmd, "AT+MCUCONFIRM");
    if (value == RT_NULL) return RT_FALSE;
    comma = strchr(value, ',');
    if (comma == RT_NULL)
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    if ((rt_size_t)(comma - value) == 5U && rt_strncmp(value, "NVCLR", 5U) == 0)
        requested_action = MCU_CONFIRM_NVCLR;
    else if ((rt_size_t)(comma - value) == 8U &&
             rt_strncmp(value, "QUEUECLR", 8U) == 0)
        requested_action = MCU_CONFIRM_QUEUECLR;
    else
    {
        cmd_error(out, out_size, 1);
        return RT_TRUE;
    }
    if (!parse_u32(comma + 1, &nonce) || nonce != s_confirm_nonce ||
        requested_action != s_confirm_action ||
        (rt_int32_t)(rt_tick_get() - s_confirm_expiry) >= 0)
    {
        s_confirm_action = MCU_CONFIRM_NONE;
        cmd_error(out, out_size, 14);
        return RT_TRUE;
    }
    s_confirm_action = MCU_CONFIRM_NONE;
    result = (requested_action == MCU_CONFIRM_NVCLR) ?
             param_reset_operational() : raw_uplink_clear();
    if (result != RT_EOK)
    {
        cmd_error(out, out_size, (result == -RT_EIO) ? 8 : 3);
    }
    else
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"action\":\"%s\"}\r\nOK\r\n",
                    requested_action == MCU_CONFIRM_NVCLR ? "NVCLR" : "QUEUECLR");
    }
    return RT_TRUE;
}

static rt_bool_t handle_power_stub(const char *cmd, char *out, rt_size_t out_size)
{
    const char *value = cmd_set_value(cmd, "AT+MCUPM");
    rt_uint32_t enabled;
    if (cmd_is_query(cmd, "AT+MCUPM"))
    {
        rt_snprintf(out, out_size,
                    "\r\n+MCUPM:{\"enabled\":false,\"sleep_state\":\"IDLE_ONLY\",\"deep_sleep_supported\":false}\r\nOK\r\n");
        return RT_TRUE;
    }
    if (value != RT_NULL)
    {
        if (!parse_u32(value, &enabled) || enabled > 1U) cmd_error(out, out_size, 1);
        else if (enabled) cmd_error(out, out_size, 2);
        else rt_snprintf(out, out_size,
                         "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"pm\":false}\r\nOK\r\n");
        return RT_TRUE;
    }
    if (rt_strcmp(cmd, "AT+MCUNTNWAKE") == 0)
    {
        cmd_error(out, out_size, 2);
        return RT_TRUE;
    }
    return RT_FALSE;
}

rt_bool_t mcu_local_cmd_execute(const rt_uint8_t *line, rt_size_t line_len,
                                char *response, rt_size_t response_size,
                                mcu_cmd_action_t *action)
{
    char cmd[MCU_CMD_COPY_MAX];

    if (line == RT_NULL || response == RT_NULL || response_size == 0U || action == RT_NULL)
    {
        return RT_FALSE;
    }
    action->type = MCU_CMD_ACTION_NONE;
    action->value = 0U;
    action->previous_value = 0U;
    response[0] = '\0';
    if (line_len == 0U || line_len >= sizeof(cmd))
    {
        return RT_FALSE;
    }
    rt_memcpy(cmd, line, line_len);
    cmd[line_len] = '\0';

    if (handle_identity(cmd, response, response_size) ||
        handle_capability(cmd, response, response_size) ||
        handle_mode(cmd, response, response_size) ||
        handle_transfer_mode(cmd, response, response_size) ||
        handle_host(cmd, response, response_size) ||
        handle_port(cmd, response, response_size) ||
        handle_baud(cmd, response, response_size, action) ||
        handle_raw_pack(cmd, response, response_size) ||
        handle_dns(cmd, response, response_size) ||
        handle_rx_mode(cmd, response, response_size) ||
        handle_downlink(cmd, response, response_size) ||
        handle_gnss_time(cmd, response, response_size) ||
        handle_fixed_mode_settings(cmd, response, response_size) ||
        handle_report_settings(cmd, response, response_size) ||
        handle_status(cmd, response, response_size) ||
        handle_text_setting(cmd, response, response_size) ||
        handle_control_session(cmd, response, response_size, action) ||
        handle_ntn_lock(cmd, response, response_size) ||
        handle_destructive(cmd, response, response_size) ||
        handle_power_stub(cmd, response, response_size))
    {
        return RT_TRUE;
    }
    if (cmd_is_query(cmd, "AT+MCUUPG"))
    {
        rt_snprintf(response, response_size,
                    "\r\n+MCUUPG:{\"supported\":false,\"planned\":["
                    "\"NTN_DIFF\",\"ANDROID_DIFF\",\"ANDROID_FULL\","
                    "\"PC_DIFF\",\"PC_FULL\"]}\r\nOK\r\n");
        return RT_TRUE;
    }
    return RT_FALSE;
}
