/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      First version: MS150 Iridium NTN modem driver & PC transparent bridge
 */

#include "ntn_ms150.h"
#include "hw_power.h"
#include "pc_rs232.h"
#include "param_manager.h"
#include "mcu_local_cmd.h"
#include "gnss_um220.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "task_wdt.h"

#define DBG_TAG "ntn.ms150"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define NTN_THREAD_STACK_SZ     2048
#define NTN_THREAD_PRIO         10
#define NTN_AT_LINE_MAX_SZ      2048U
#define NTN_AT_INTERBYTE_MS     2000U
#define NTN_AT_WHOLE_LINE_MS    30000U
#define NTN_SERVICE_STACK_SZ    5120U
#define NTN_SERVICE_THREAD_PRIO 17U
#define NTN_RX_LINE_MAX_SZ      1472U
#define NTN_RX_QUERY_PERIOD_MS  5000U

static rt_device_t              _ntn_dev           = RT_NULL;
static struct rt_semaphore      _sem_rx;
static struct rt_semaphore      _sem_resp;
static struct rt_mutex          _mutex_cmd;
static struct rt_mutex          _mutex_tx;
static struct rt_mutex          _mutex_at_line;
static rt_thread_t              _rx_thread         = RT_NULL;
static rt_thread_t              _service_thread    = RT_NULL;

static ntn_stats_t              _stats;
static rt_bool_t                _bridge_active     = RT_FALSE;
static rt_bool_t                _inited            = RT_FALSE;

/* AT Command Transaction Management */
static volatile rt_bool_t       _cmd_waiting       = RT_FALSE;
static volatile rt_err_t        _cmd_status        = RT_EOK;
static char                    *_cur_resp_buf      = RT_NULL;
static rt_size_t                _cur_resp_sz       = 0;
static rt_size_t                _cur_resp_idx      = 0;

/* V3 mode-0 AT line assembly. Buffer holds content; CR is added on transmit. */
static rt_uint8_t               _at_line_buf[NTN_AT_LINE_MAX_SZ];
static rt_size_t                _at_line_len       = 0;
static rt_bool_t                _at_discarding     = RT_FALSE;
static rt_bool_t                _at_skip_next_lf   = RT_FALSE;
static rt_tick_t                _at_line_start_tick;
static rt_timer_t               _at_timeout_timer  = RT_NULL;
static char                     _local_response[2048];
static struct rt_mutex          _service_lock;
static ntn_service_status_t     _service_status;
static rt_tick_t                _service_next_retry;
static rt_uint8_t               _service_fail_count;
static rt_uint32_t              _service_active_key;
static struct rt_mutex          _downlink_lock;
static ntn_downlink_t           _last_downlink;
static rt_uint32_t              _downlink_count;
static rt_uint32_t              _downlink_parse_errors;
static rt_uint8_t               _rx_line[NTN_RX_LINE_MAX_SZ];
static rt_size_t                _rx_line_len;
static rt_bool_t                _rx_query_requested;
static rt_tick_t                _rx_next_query_tick;
static rt_tick_t                _ntn_location_last_tick;
static volatile rt_tick_t       _last_rx_tick;
static rt_bool_t                _cmd_recovery_required;
static volatile rt_bool_t       _debug_locked;
static volatile rt_tick_t       _debug_lock_deadline;
static volatile rt_bool_t       _pc_command_active;
static volatile rt_tick_t       _pc_command_deadline;
static ntn_raw_rx_handler_t     _raw_rx_handler;
static ntn_downlink_handler_t   _downlink_handler;

static int ntn_hex_value(char ch)
{
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return -1;
}

static void ntn_publish_receive_line(const rt_uint8_t *line, rt_size_t line_len)
{
    static const char prefix[] = "+MKRECEIVE:";
    const char *p;
    char *end;
    unsigned long socket_id;
    unsigned long data_len;
    rt_size_t hex_len;
    rt_size_t i;
    ntn_downlink_t event;

    if (line_len < sizeof(prefix) - 1U ||
        rt_memcmp(line, prefix, sizeof(prefix) - 1U) != 0)
    {
        return;
    }
    p = (const char *)line + sizeof(prefix) - 1U;
    socket_id = strtoul(p, &end, 10);
    if (end == p || *end != ',') goto parse_error;
    p = end + 1;
    data_len = strtoul(p, &end, 10);
    if (end == p || socket_id < 1U || socket_id > 5U ||
        data_len > NTN_DOWNLINK_MAX_LEN) goto parse_error;
    if (*end == '\0')
    {
        if (data_len > 0U) _rx_query_requested = RT_TRUE;
        return;
    }
    if (*end != ',') goto parse_error;
    p = end + 1;
    hex_len = rt_strlen(p);
    if (hex_len != data_len * 2U) goto parse_error;

    rt_mutex_take(&_downlink_lock, RT_WAITING_FOREVER);
    for (i = 0U; i < data_len; i++)
    {
        int high = ntn_hex_value(p[i * 2U]);
        int low = ntn_hex_value(p[i * 2U + 1U]);
        if (high < 0 || low < 0)
        {
            rt_mutex_release(&_downlink_lock);
            goto parse_error;
        }
        _last_downlink.data[i] = (rt_uint8_t)((high << 4) | low);
    }
    _last_downlink.valid = RT_TRUE;
    _last_downlink.socket_id = (rt_uint8_t)socket_id;
    _last_downlink.data_len = (rt_uint16_t)data_len;
    _last_downlink.sequence++;
    _last_downlink.received_tick = rt_tick_get();
    _downlink_count++;
    event = _last_downlink;
    rt_mutex_release(&_downlink_lock);
    LOG_I("NTN downlink received: socket=%u len=%u seq=%u",
          socket_id, data_len, _last_downlink.sequence);
    if (_downlink_handler != RT_NULL)
    {
        _downlink_handler(&event);
    }
    return;

parse_error:
    _downlink_parse_errors++;
    LOG_W("Invalid +MKRECEIVE line discarded (len=%u)", line_len);
}

static void ntn_rx_line_feed(rt_uint8_t ch)
{
    if (ch == '\r' || ch == '\n')
    {
        if (_rx_line_len > 0U)
        {
            _rx_line[_rx_line_len] = '\0';
            ntn_publish_receive_line(_rx_line, _rx_line_len);
            if (_pc_command_active &&
                ((rt_strcmp((const char *)_rx_line, "OK") == 0) ||
                 (rt_strcmp((const char *)_rx_line, "ERROR") == 0) ||
                 (rt_strncmp((const char *)_rx_line, "+CME ERROR:", 11U) == 0)))
            {
                _pc_command_active = RT_FALSE;
            }
            _rx_line_len = 0U;
        }
        return;
    }
    if (_rx_line_len < sizeof(_rx_line) - 1U)
    {
        _rx_line[_rx_line_len++] = ch;
    }
    else
    {
        _rx_line_len = 0U;
        _downlink_parse_errors++;
    }
}

static rt_size_t ntn_uart_write(const void *data, rt_size_t len)
{
    rt_size_t sent = 0U;
    if (!_inited || _ntn_dev == RT_NULL || data == RT_NULL || len == 0U)
    {
        return 0U;
    }
    if (rt_mutex_take(&_mutex_tx, RT_WAITING_FOREVER) == RT_EOK)
    {
        rt_tick_t start = rt_tick_get();
        while (sent < len)
        {
            rt_size_t n = rt_device_write(_ntn_dev, 0,
                                           (const rt_uint8_t *)data + sent,
                                           len - sent);
            sent += n;
            if (n == 0U)
            {
                if ((rt_tick_get() - start) >= rt_tick_from_millisecond(5000U)) break;
                rt_thread_mdelay(1U);
            }
        }
        _stats.tx_bytes += sent;
        rt_mutex_release(&_mutex_tx);
    }
    return sent;
}

static void at_line_reset_locked(void)
{
    _at_line_len = 0U;
    _at_discarding = RT_FALSE;
    _at_line_start_tick = 0U;
}

static void local_action_apply(const mcu_cmd_action_t *action)
{
    char final_response[192];
    rt_err_t result;

    if (action->type != MCU_CMD_ACTION_APPLY_RS232_BAUD)
    {
        return;
    }
    /* The pending response is written at the old rate before this delay. */
    rt_thread_mdelay(500U);
    result = pc_rs232_set_baudrate(action->value);
    if (result != RT_EOK)
    {
        (void)param_set_rs232_baud(action->previous_value, RT_TRUE);
        rt_snprintf(final_response, sizeof(final_response),
                    "\r\n+MCUERROR:3\r\nERROR\r\n");
    }
    else
    {
        rt_snprintf(final_response, sizeof(final_response),
                    "\r\n+MCUAPPLY:{\"state\":\"APPLIED\",\"baud\":%u}\r\nOK\r\n",
                    action->value);
    }
    pc_rs232_send_str(final_response);
}

static void at_line_dispatch(const rt_uint8_t *line, rt_size_t line_len)
{
    mcu_cmd_action_t action;

    if (line_len == 0U)
    {
        return;
    }
    if (mcu_local_cmd_execute(line, line_len, _local_response,
                              sizeof(_local_response), &action))
    {
        pc_rs232_send_str(_local_response);
        local_action_apply(&action);
        return;
    }

    /* Serialize mode-0 PC commands with the Bluetooth debug lock and service. */
    if (rt_mutex_take(&_mutex_cmd, 0) != RT_EOK)
    {
        pc_rs232_send_str("\r\n+MCUERROR:6\r\nERROR\r\n");
        return;
    }
    if (ntn_debug_lock_is_held() || _pc_command_active)
    {
        rt_mutex_release(&_mutex_cmd);
        pc_rs232_send_str("\r\n+MCUERROR:6\r\nERROR\r\n");
        return;
    }
    _pc_command_active = RT_TRUE;
    _pc_command_deadline = rt_tick_get() + rt_tick_from_millisecond(60000U);
    if (ntn_uart_write(line, line_len) == line_len && ntn_uart_write("\r", 1U) == 1U)
        _stats.at_cmds_sent++;
    else
        _pc_command_active = RT_FALSE;
    rt_mutex_release(&_mutex_cmd);
}

static void at_line_timeout_entry(void *parameter)
{
    rt_bool_t timed_out = RT_FALSE;
    RT_UNUSED(parameter);

    if (rt_mutex_take(&_mutex_at_line, 0) == RT_EOK)
    {
        if (_at_line_len > 0U || _at_discarding)
        {
            at_line_reset_locked();
            timed_out = RT_TRUE;
        }
        rt_mutex_release(&_mutex_at_line);
    }
    if (timed_out)
    {
        LOG_W("Mode-0 AT line timed out and was discarded");
        pc_rs232_send_str("\r\n+MCUERROR:12\r\nERROR\r\n");
    }
}

void ntn_bridge_process_pc_data(const rt_uint8_t *data, rt_size_t len)
{
    if (!_bridge_active || !_ntn_dev || !data || len == 0U ||
        param_get_work_mode() != PARAM_MODE_AT_PASSTHROUGH)
    {
        return;
    }

    if (ntn_debug_lock_is_held())
    {
        pc_rs232_send_str("\r\n+MCUERROR:6\r\nERROR\r\n");
        return;
    }

    _stats.bridge_pc_rx += len;

    for (rt_size_t i = 0U; i < len; i++)
    {
        rt_uint8_t ch = data[i];
        rt_bool_t complete = RT_FALSE;
        rt_bool_t overflow = RT_FALSE;
        rt_bool_t whole_timeout = RT_FALSE;
        rt_size_t complete_len = 0U;
        rt_tick_t now = rt_tick_get();

        rt_mutex_take(&_mutex_at_line, RT_WAITING_FOREVER);

        if (_at_skip_next_lf)
        {
            _at_skip_next_lf = RT_FALSE;
            if (ch == '\n')
            {
                rt_mutex_release(&_mutex_at_line);
                continue;
            }
        }

        if (_at_discarding)
        {
            if (ch == '\r')
            {
                at_line_reset_locked();
                _at_skip_next_lf = RT_TRUE;
            }
            rt_mutex_release(&_mutex_at_line);
            continue;
        }

        if (_at_line_len > 0U &&
            (now - _at_line_start_tick) > rt_tick_from_millisecond(NTN_AT_WHOLE_LINE_MS))
        {
            at_line_reset_locked();
            whole_timeout = RT_TRUE;
        }

        if (!whole_timeout && ch == '\r')
        {
            complete_len = _at_line_len;
            complete = (complete_len > 0U) ? RT_TRUE : RT_FALSE;
            _at_line_len = 0U;
            _at_line_start_tick = 0U;
            _at_skip_next_lf = RT_TRUE;
        }
        else if (!whole_timeout)
        {
            if (_at_line_len == 0U)
            {
                _at_line_start_tick = now;
            }
            if (_at_line_len < (sizeof(_at_line_buf) - 1U))
            {
                _at_line_buf[_at_line_len++] = ch;
            }
            else
            {
                _at_discarding = RT_TRUE;
                _at_line_len = 0U;
                overflow = RT_TRUE;
            }
        }
        rt_mutex_release(&_mutex_at_line);

        if (_at_timeout_timer != RT_NULL)
        {
            rt_timer_stop(_at_timeout_timer);
            if (!complete && !whole_timeout)
            {
                rt_timer_start(_at_timeout_timer);
            }
        }
        if (whole_timeout)
        {
            pc_rs232_send_str("\r\n+MCUERROR:12\r\nERROR\r\n");
        }
        else if (overflow)
        {
            pc_rs232_send_str("\r\n+MCUERROR:21\r\nERROR\r\n");
        }
        else if (complete)
        {
            at_line_dispatch(_at_line_buf, complete_len);
        }
    }
}

static rt_err_t ntn_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&_sem_rx);
    return RT_EOK;
}

static void ntn_rx_thread_entry(void *parameter)
{
    rt_uint8_t rx_buf[64];

    LOG_I("NTN MS150 receiver thread running...");

    while (1)
    {
        rt_size_t read_bytes;
        while ((read_bytes = rt_device_read(_ntn_dev, 0, rx_buf, sizeof(rx_buf))) > 0)
        {
            _stats.rx_bytes += read_bytes;
            _last_rx_tick = rt_tick_get();

            for (rt_size_t i = 0U; i < read_bytes; i++)
            {
                ntn_rx_line_feed(rx_buf[i]);
            }

            if (_raw_rx_handler != RT_NULL)
            {
                _raw_rx_handler(rx_buf, read_bytes, _cmd_waiting);
            }

            /* 1. If bridge mode is active, forward bytes to PC RS232 port */
            if (_bridge_active && param_get_work_mode() == PARAM_MODE_AT_PASSTHROUGH &&
                !ntn_debug_lock_is_held())
            {
                pc_rs232_send(rx_buf, read_bytes);
                _stats.bridge_ntn_to_pc += read_bytes;
            }

            /* 2. If an AT command is synchronously waiting for a response */
            if (_cmd_waiting && _cur_resp_buf)
            {
                for (rt_size_t i = 0; i < read_bytes; i++)
                {
                    if (_cur_resp_idx < _cur_resp_sz - 1)
                    {
                        _cur_resp_buf[_cur_resp_idx++] = (char)rx_buf[i];
                        _cur_resp_buf[_cur_resp_idx] = '\0';
                    }
                }

                /* Check for completion keywords */
                if (_cur_resp_idx >= 4)
                {
                    /* Search for "OK\r\n" */
                    if (rt_strstr(_cur_resp_buf, "OK\r\n"))
                    {
                        _cmd_status = RT_EOK;
                        _cmd_waiting = RT_FALSE;
                        rt_sem_release(&_sem_resp);
                    }
                    /* Search for "ERROR\r\n" */
                    else if (rt_strstr(_cur_resp_buf, "ERROR\r\n") || rt_strstr(_cur_resp_buf, "+CME ERROR:"))
                    {
                        _cmd_status = -RT_ERROR;
                        _cmd_waiting = RT_FALSE;
                        rt_sem_release(&_sem_resp);
                    }
                }
            }
        }

        rt_sem_take(&_sem_rx, rt_tick_from_millisecond(5000));
        task_wdt_heartbeat_by_name("ntn");
    }
}

static void service_set_state(ntn_service_state_t state, rt_err_t error)
{
    rt_mutex_take(&_service_lock, RT_WAITING_FOREVER);
    _service_status.state = state;
    _service_status.last_error = error;
    _service_status.send_ready = (state == NTN_SERVICE_READY) ? RT_TRUE : RT_FALSE;
    rt_mutex_release(&_service_lock);
}

static ntn_reg_status_t service_parse_cereg(const char *response)
{
    const char *p = rt_strstr(response, "+CEREG:");
    long first;
    long second = -1;
    char *end;

    if (p == RT_NULL)
    {
        return NTN_REG_UNKNOWN;
    }
    p += 7;
    while (*p == ' ')
    {
        p++;
    }
    first = strtol(p, &end, 10);
    if (*end == ',')
    {
        second = strtol(end + 1, &end, 10);
    }
    if (second >= 0 && second <= 5)
    {
        return (ntn_reg_status_t)second;
    }
    if (first >= 0 && first <= 5)
    {
        return (ntn_reg_status_t)first;
    }
    return NTN_REG_UNKNOWN;
}

static rt_bool_t service_host_is_domain(const char *host)
{
    while (*host != '\0')
    {
        if ((*host >= 'A' && *host <= 'Z') || (*host >= 'a' && *host <= 'z'))
        {
            return RT_TRUE;
        }
        host++;
    }
    return RT_FALSE;
}

static rt_uint32_t service_hash_bytes(rt_uint32_t hash, const void *data, rt_size_t len)
{
    const rt_uint8_t *p = (const rt_uint8_t *)data;
    rt_size_t i;
    for (i = 0U; i < len; i++) hash = (hash ^ p[i]) * 16777619UL;
    return hash;
}

static rt_uint32_t service_config_key(const param_config_t *cfg)
{
    rt_uint32_t hash = 2166136261UL;
    hash = service_hash_bytes(hash, cfg->host, rt_strlen(cfg->host));
    hash = service_hash_bytes(hash, &cfg->remote_port, sizeof(cfg->remote_port));
    hash = service_hash_bytes(hash, &cfg->local_port, sizeof(cfg->local_port));
    hash = service_hash_bytes(hash, &cfg->rx_mode, sizeof(cfg->rx_mode));
    hash = service_hash_bytes(hash, &cfg->dns_mode, sizeof(cfg->dns_mode));
    hash = service_hash_bytes(hash, cfg->dns_primary, rt_strlen(cfg->dns_primary));
    hash = service_hash_bytes(hash, cfg->dns_secondary, rt_strlen(cfg->dns_secondary));
    hash = service_hash_bytes(hash, cfg->ntn_apn, rt_strlen(cfg->ntn_apn));
    hash = service_hash_bytes(hash, &cfg->ntn_location_enabled,
                              sizeof(cfg->ntn_location_enabled));
    return hash;
}

static rt_err_t service_exec(const char *command, char *response,
                             rt_size_t response_size, rt_uint32_t timeout_ms)
{
    rt_err_t result;
    rt_uint8_t attempt;
    rt_uint8_t attempts = (rt_strstr(command, "?") != RT_NULL ||
                           rt_strncmp(command, "AT+MKRECEIVE=", 13U) == 0) ? 3U : 1U;
    for (attempt = 0U; attempt < attempts; attempt++)
    {
        result = ntn_exec_cmd(command, response, response_size, timeout_ms);
        if (result != -RT_ETIMEOUT) break;
    }
    if (result != RT_EOK)
    {
        LOG_W("NTN service command failed: %s (%d)", command, result);
    }
    return result;
}

static rt_err_t service_prepare_socket(void)
{
    param_config_t cfg;
    char command[420];
    char response[NTN_RESP_BUF_MAX_SZ];
    ntn_reg_status_t reg;
    rt_err_t result;

    param_get_snapshot(&cfg);
    if (cfg.work_mode != PARAM_MODE_RAW_UPLINK || cfg.transfer_mode != PARAM_TRANSFER_RAW)
    {
        return -RT_EINVAL;
    }
    if (cfg.host[0] == '\0' || cfg.remote_port == 0U)
    {
        return -RT_ENOSYS;
    }

    service_set_state(NTN_SERVICE_PROBING, RT_EOK);
    result = service_exec("AT", response, sizeof(response), 5000U);
    if (result != RT_EOK) return result;
    result = service_exec("ATE0", response, sizeof(response), 5000U);
    if (result != RT_EOK) return result;
    result = service_exec("AT+CMEE=1", response, sizeof(response), 5000U);
    if (result != RT_EOK) return result;
    result = service_exec("AT+CPIN?", response, sizeof(response), 5000U);
    if (result != RT_EOK || rt_strstr(response, "READY") == RT_NULL)
    {
        return (result == RT_EOK) ? -RT_ERROR : result;
    }

    service_set_state(NTN_SERVICE_REGISTERING, RT_EOK);
    result = service_exec("AT+CEREG=1", response, sizeof(response), 5000U);
    if (result != RT_EOK) return result;
    result = service_exec("AT+CEREG?", response, sizeof(response), 5000U);
    if (result != RT_EOK) return result;
    reg = service_parse_cereg(response);
    rt_mutex_take(&_service_lock, RT_WAITING_FOREVER);
    _service_status.reg_status = reg;
    rt_mutex_release(&_service_lock);
    if (reg != NTN_REG_HOME && reg != NTN_REG_ROAMING)
    {
        return -RT_EBUSY;
    }

    if (cfg.ntn_apn[0] != '\0')
    {
        rt_snprintf(command, sizeof(command), "AT+CGDCONT=0,\"IP\",\"%s\"", cfg.ntn_apn);
        result = service_exec(command, response, sizeof(response), 5000U);
        if (result != RT_EOK) return result;
    }
    result = service_exec("AT+CGATT=1", response, sizeof(response), 180000U);
    if (result != RT_EOK) return result;
    result = service_exec("AT+CGACT=1,0", response, sizeof(response), 180000U);
    if (result != RT_EOK) return result;

    if (cfg.ntn_location_enabled && gnss_is_fresh(5000U))
    {
        if (ntn_sync_gnss_location() == RT_EOK)
        {
            _ntn_location_last_tick = rt_tick_get();
        }
    }

    result = service_exec("AT+MKDATTYPE=1", response, sizeof(response), 5000U);
    if (result != RT_EOK) return result;
    result = service_exec("AT+MKDATTYPE?", response, sizeof(response), 5000U);
    if (result != RT_EOK || rt_strstr(response, "+MKDATTYPE:1") == RT_NULL)
    {
        return (result == RT_EOK) ? -RT_ERROR : result;
    }

    if (service_host_is_domain(cfg.host))
    {
        if (cfg.dns_mode == 0U)
        {
            rt_snprintf(command, sizeof(command), "AT+MKDNS=0");
        }
        else if (cfg.dns_mode == 2U)
        {
            if (cfg.dns_primary[0] == '\0') return -RT_EINVAL;
            if (cfg.dns_secondary[0] != '\0')
                rt_snprintf(command, sizeof(command), "AT+MKDNS=2,%s,%s", cfg.dns_primary, cfg.dns_secondary);
            else
                rt_snprintf(command, sizeof(command), "AT+MKDNS=2,%s", cfg.dns_primary);
        }
        else if (cfg.dns_primary[0] == '\0')
        {
            rt_snprintf(command, sizeof(command), "AT+MKDNS=1");
        }
        else if (cfg.dns_secondary[0] != '\0')
        {
            rt_snprintf(command, sizeof(command), "AT+MKDNS=1,%s,%s", cfg.dns_primary, cfg.dns_secondary);
        }
        else
        {
            rt_snprintf(command, sizeof(command), "AT+MKDNS=1,%s", cfg.dns_primary);
        }
        result = service_exec(command, response, sizeof(response), 5000U);
        if (result != RT_EOK) return result;
    }

    service_set_state(NTN_SERVICE_OPENING, RT_EOK);
    (void)ntn_exec_cmd("AT+MKCLOSE=1", response, sizeof(response), 5000U);
    rt_snprintf(command, sizeof(command), "AT+MKOPEN=1,2,%s,%u,%u,%u",
                cfg.host, cfg.remote_port, cfg.local_port, cfg.rx_mode);
    result = service_exec(command, response, sizeof(response), 60000U);
    if (result != RT_EOK) return result;
    return RT_EOK;
}

static rt_uint32_t service_retry_delay_ms(rt_uint8_t fail_count)
{
    if (fail_count <= 1U) return 5000U;
    if (fail_count == 2U) return 15000U;
    if (fail_count == 3U) return 60000U;
    return 300000U;
}

static void ntn_service_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);
    while (1)
    {
        rt_err_t result;
        param_config_t cfg;
        param_get_snapshot(&cfg);

        if (ntn_debug_lock_is_held())
        {
            rt_thread_mdelay(100U);
            continue;
        }

        if (cfg.work_mode != PARAM_MODE_RAW_UPLINK)
        {
            _service_fail_count = 0U;
            service_set_state(NTN_SERVICE_DISABLED, RT_EOK);
            rt_thread_mdelay(1000U);
            continue;
        }
        if (cfg.host[0] == '\0' || cfg.remote_port == 0U)
        {
            service_set_state(NTN_SERVICE_NOT_CONFIGURED, -RT_ENOSYS);
            rt_thread_mdelay(1000U);
            continue;
        }
        if (_service_status.send_ready)
        {
            if (service_config_key(&cfg) != _service_active_key)
            {
                ntn_service_request_reconnect();
                continue;
            }
            if (cfg.rx_mode == PARAM_RX_QUERY &&
                (_rx_query_requested ||
                 (rt_int32_t)(rt_tick_get() - _rx_next_query_tick) >= 0))
            {
                char response[NTN_RESP_BUF_MAX_SZ];
                _rx_query_requested = RT_FALSE;
                _rx_next_query_tick = rt_tick_get() +
                    rt_tick_from_millisecond(NTN_RX_QUERY_PERIOD_MS);
                (void)service_exec("AT+MKRECEIVE=1", response,
                                   sizeof(response), 10000U);
            }
            if (cfg.ntn_location_enabled && gnss_is_fresh(5000U) &&
                (rt_tick_get() - _ntn_location_last_tick) >=
                    rt_tick_from_millisecond(300000U))
            {
                if (ntn_sync_gnss_location() == RT_EOK)
                {
                    _ntn_location_last_tick = rt_tick_get();
                }
            }
            rt_thread_mdelay(1000U);
            continue;
        }
        if ((rt_int32_t)(rt_tick_get() - _service_next_retry) < 0)
        {
            rt_thread_mdelay(500U);
            continue;
        }

        result = service_prepare_socket();
        if (result == RT_EOK)
        {
            _service_fail_count = 0U;
            _service_active_key = service_config_key(&cfg);
            service_set_state(NTN_SERVICE_READY, RT_EOK);
            LOG_I("NTN UDP socket 1 ready for %s:%u", cfg.host, cfg.remote_port);
        }
        else
        {
            rt_uint32_t delay;
            _service_fail_count++;
            delay = service_retry_delay_ms(_service_fail_count);
            _service_next_retry = rt_tick_get() + rt_tick_from_millisecond(delay);
            rt_mutex_take(&_service_lock, RT_WAITING_FOREVER);
            _service_status.reconnect_count++;
            rt_mutex_release(&_service_lock);
            service_set_state(NTN_SERVICE_BACKOFF, result);
            LOG_W("NTN setup failed (%d), retry in %u ms", result, delay);
        }
    }
}

int ntn_ms150_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    if (_inited)
    {
        return RT_EOK;
    }

    LOG_I("Initializing MS150 NTN Modem on %s (PB4-TX, PB5-RX @ 115200 8N1)...", NTN_UART_NAME);

    /* 1. Ensure NTN base power rails (PB15 NTN_PWR_EN, PB0 PA_PWREN) are enabled */
    ntn_power_set(RT_TRUE);

    /* 2. Hardware reset pulse (PA8 high pulse 200ms) */
    ntn_reset(200);

    /* 3. Find serial device */
    _ntn_dev = rt_device_find(NTN_UART_NAME);
    if (!_ntn_dev)
    {
        LOG_E("Cannot find NTN serial device: %s", NTN_UART_NAME);
        return -RT_ERROR;
    }

    /* 4. Configure baudrate 115200 8N1 */
    cfg.baud_rate = NTN_DEFAULT_BAUDRATE;
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity    = PARITY_NONE;
    if (rt_device_control(_ntn_dev, RT_DEVICE_CTRL_CONFIG, &cfg) != RT_EOK)
    {
        LOG_E("Failed to configure %s parameters", NTN_UART_NAME);
        return -RT_ERROR;
    }

    /* 5. Open device with interrupt RX */
    if (rt_device_open(_ntn_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Failed to open %s with INT_RX", NTN_UART_NAME);
        return -RT_ERROR;
    }

    /* 6. Initialize synchronization primitives */
    rt_memset(&_stats, 0, sizeof(_stats));
    rt_sem_init(&_sem_rx, "sntn_rx", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&_sem_resp, "sntn_rsp", 0, RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_mutex_cmd, "mntn_cmd", RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_mutex_tx, "mntn_tx", RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_mutex_at_line, "mat_line", RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_service_lock, "mntn_svc", RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_downlink_lock, "mntn_down", RT_IPC_FLAG_PRIO);
    rt_memset(&_service_status, 0, sizeof(_service_status));
    rt_memset(&_last_downlink, 0, sizeof(_last_downlink));
    _service_status.state = NTN_SERVICE_DISABLED;
    _service_status.reg_status = NTN_REG_UNKNOWN;
    _rx_next_query_tick = rt_tick_get() + rt_tick_from_millisecond(NTN_RX_QUERY_PERIOD_MS);

    /* 7. Set receive callback */
    rt_device_set_rx_indicate(_ntn_dev, ntn_uart_rx_ind);

    /* 8. Create receiver thread */
    _rx_thread = rt_thread_create("ntn_rx",
                                  ntn_rx_thread_entry,
                                  RT_NULL,
                                  NTN_THREAD_STACK_SZ,
                                  NTN_THREAD_PRIO,
                                  10);
    if (!_rx_thread)
    {
        LOG_E("Failed to create ntn_rx thread");
        return -RT_ERROR;
    }
    rt_thread_startup(_rx_thread);

    /* 9. Create AT line assembly timeout timer */
    _at_timeout_timer = rt_timer_create("at_to",
                                        at_line_timeout_entry,
                                        RT_NULL,
                                         rt_tick_from_millisecond(NTN_AT_INTERBYTE_MS),
                                        RT_TIMER_FLAG_ONE_SHOT);

    _inited = RT_TRUE;
    _bridge_active = RT_TRUE;
    _service_thread = rt_thread_create("ntn_svc", ntn_service_thread_entry,
                                       RT_NULL, NTN_SERVICE_STACK_SZ,
                                       NTN_SERVICE_THREAD_PRIO, 20U);
    if (_service_thread == RT_NULL)
    {
        LOG_E("Failed to create NTN service thread");
        return -RT_ERROR;
    }
    rt_thread_startup(_service_thread);
    LOG_I("MS150 NTN Modem ready: %s @ 115200 8N1", NTN_UART_NAME);

    return RT_EOK;
}
INIT_ENV_EXPORT(ntn_ms150_init);

void ntn_power_set(rt_bool_t on)
{
    hw_power_ntn_set(on);
}

void ntn_reset(rt_uint32_t pulse_ms)
{
    hw_reset_ntn(pulse_ms);
}

static rt_err_t ntn_exec_cmd_locked(const char *cmd, char *resp_buf,
                                    rt_size_t buf_sz, rt_uint32_t timeout_ms)
{
    rt_err_t err = RT_EOK;

    /* Setup response buffer */
    _cur_resp_buf = resp_buf;
    _cur_resp_sz  = buf_sz;
    _cur_resp_idx = 0;
    if (resp_buf && buf_sz > 0)
    {
        resp_buf[0] = '\0';
    }
    _cmd_status  = -RT_ETIMEOUT;
    _cmd_waiting = RT_TRUE;

    /* Flush any pending semaphore releases */
    while (rt_sem_trytake(&_sem_resp) == RT_EOK);

    /* Write command */
    rt_size_t len = rt_strlen(cmd);
    if (ntn_uart_write(cmd, len) != len ||
        (len > 0U && cmd[len - 1U] != '\r' &&
         ntn_uart_write("\r", 1U) != 1U))
    {
        _cmd_waiting = RT_FALSE;
        _cur_resp_buf = RT_NULL;
        return -RT_ERROR;
    }
    _stats.at_cmds_sent++;

    /* Wait for response completion */
    if (rt_sem_take(&_sem_resp, rt_tick_from_millisecond(timeout_ms)) == RT_EOK)
    {
        err = _cmd_status;
        if (err == RT_EOK)
        {
            _stats.at_cmds_ok++;
        }
        else
        {
            _stats.at_cmds_err++;
        }
    }
    else
    {
        err = -RT_ETIMEOUT;
        _stats.at_cmds_err++;
    }

    _cmd_waiting  = RT_FALSE;
    _cur_resp_buf = RT_NULL;
    return err;
}

static rt_err_t ntn_command_recover_locked(void)
{
    char response[64];
    rt_tick_t deadline = rt_tick_get() + rt_tick_from_millisecond(10000U);
    rt_err_t result;

    while ((rt_tick_get() - _last_rx_tick) < rt_tick_from_millisecond(2000U))
    {
        if ((rt_int32_t)(rt_tick_get() - deadline) >= 0)
        {
            return -RT_ETIMEOUT;
        }
        rt_thread_mdelay(100U);
    }
    result = ntn_exec_cmd_locked("AT", response, sizeof(response), 5000U);
    if (result != RT_EOK) return result;
    rt_thread_mdelay(1000U);
    result = ntn_exec_cmd_locked("AT", response, sizeof(response), 5000U);
    return result;
}

static rt_err_t ntn_exec_cmd_internal(const char *cmd, char *resp_buf,
                                       rt_size_t buf_sz, rt_uint32_t timeout_ms,
                                       rt_bool_t debug_request)
{
    rt_err_t result;
    rt_uint32_t lock_timeout = timeout_ms + 22000U;

    if (!_inited || !_ntn_dev || !cmd) return -RT_ERROR;
    if (rt_mutex_take(&_mutex_cmd, rt_tick_from_millisecond(lock_timeout)) != RT_EOK)
        return -RT_EBUSY;
    /* Re-check under the command mutex, closing the auto-service/debug race. */
    if ((!debug_request && ntn_debug_lock_is_held()) ||
        (debug_request && !ntn_debug_lock_is_held()))
    {
        rt_mutex_release(&_mutex_cmd);
        return -RT_EBUSY;
    }
    if (_cmd_recovery_required)
    {
        result = ntn_command_recover_locked();
        if (result != RT_EOK)
        {
            rt_mutex_release(&_mutex_cmd);
            return result;
        }
        _cmd_recovery_required = RT_FALSE;
    }
    result = ntn_exec_cmd_locked(cmd, resp_buf, buf_sz, timeout_ms);
    if (result == -RT_ETIMEOUT) _cmd_recovery_required = RT_TRUE;
    rt_mutex_release(&_mutex_cmd);
    return result;
}

rt_err_t ntn_exec_cmd(const char *cmd, char *resp_buf, rt_size_t buf_sz,
                      rt_uint32_t timeout_ms)
{
    return ntn_exec_cmd_internal(cmd, resp_buf, buf_sz, timeout_ms, RT_FALSE);
}

rt_err_t ntn_debug_exec_cmd(const char *cmd, char *resp_buf, rt_size_t buf_sz,
                            rt_uint32_t timeout_ms)
{
    return ntn_exec_cmd_internal(cmd, resp_buf, buf_sz, timeout_ms, RT_TRUE);
}

rt_size_t ntn_send_raw(const void *data, rt_size_t len)
{
    return ntn_uart_write(data, len);
}

static void extract_at_val(const char *resp, const char *prefix, char *out_val, rt_size_t max_len)
{
    const char *p = rt_strstr(resp, prefix);
    if (p && out_val && max_len > 0)
    {
        p += rt_strlen(prefix);
        while (*p == ' ' || *p == ':') p++;
        rt_size_t i = 0;
        while (*p && *p != '\r' && *p != '\n' && i < max_len - 1)
        {
            out_val[i++] = *p++;
        }
        out_val[i] = '\0';
    }
}

rt_err_t ntn_get_info(ntn_modem_info_t *out_info)
{
    char buf[NTN_RESP_BUF_MAX_SZ];

    if (!out_info)
    {
        return -RT_ERROR;
    }
    rt_memset(out_info, 0, sizeof(ntn_modem_info_t));

    /* Query Manufacturer */
    if (ntn_exec_cmd("AT+CGMI", buf, sizeof(buf), 1000) == RT_EOK)
    {
        extract_at_val(buf, "+CGMI", out_info->vendor, sizeof(out_info->vendor));
    }
    /* Query Model */
    if (ntn_exec_cmd("AT+CGMM", buf, sizeof(buf), 1000) == RT_EOK)
    {
        extract_at_val(buf, "+CGMM", out_info->model, sizeof(out_info->model));
    }
    /* Query Firmware Revision */
    if (ntn_exec_cmd("AT+CGMR", buf, sizeof(buf), 1000) == RT_EOK)
    {
        extract_at_val(buf, "+CGMR", out_info->fw_version, sizeof(out_info->fw_version));
    }
    /* Query IMEI */
    if (ntn_exec_cmd("AT+CGSN=1", buf, sizeof(buf), 1000) == RT_EOK)
    {
        extract_at_val(buf, "+CGSN", out_info->imei, sizeof(out_info->imei));
    }
    /* Query SN */
    if (ntn_exec_cmd("AT+CGSN=0", buf, sizeof(buf), 1000) == RT_EOK)
    {
        extract_at_val(buf, "+CGSN", out_info->sn, sizeof(out_info->sn));
    }
    /* Query ICCID */
    if (ntn_exec_cmd("AT+ICCID", buf, sizeof(buf), 1000) == RT_EOK)
    {
        extract_at_val(buf, "+ICCID", out_info->iccid, sizeof(out_info->iccid));
    }
    /* Query IMSI */
    if (ntn_exec_cmd("AT+CIMI", buf, sizeof(buf), 1000) == RT_EOK)
    {
        extract_at_val(buf, "+CIMI", out_info->imsi, sizeof(out_info->imsi));
    }

    return RT_EOK;
}

rt_err_t ntn_get_signal(ntn_signal_t *out_sig)
{
    char buf[NTN_RESP_BUF_MAX_SZ];

    if (!out_sig)
    {
        return -RT_ERROR;
    }

    /* Query Network Registration (AT+CEREG?) */
    if (ntn_exec_cmd("AT+CEREG?", buf, sizeof(buf), 1000) == RT_EOK)
    {
        const char *p = rt_strstr(buf, "+CEREG:");
        if (p)
        {
            p += 7;
            while (*p == ' ') p++;
            int n = 0, stat = 0;
            if (sscanf(p, "%d,%d", &n, &stat) >= 1)
            {
                out_sig->reg_status = (ntn_reg_status_t)stat;
            }
        }
    }

    /* Query Signal Quality (AT+CSQ) */
    if (ntn_exec_cmd("AT+CSQ", buf, sizeof(buf), 1000) == RT_EOK)
    {
        const char *p = rt_strstr(buf, "+CSQ:");
        if (p)
        {
            p += 5;
            while (*p == ' ') p++;
            int rssi = 99, ber = 99;
            if (sscanf(p, "%d,%d", &rssi, &ber) >= 1)
            {
                out_sig->rssi = (rt_uint8_t)rssi;
                out_sig->ber  = (rt_uint8_t)ber;
            }
        }
    }

    out_sig->last_check_tick = rt_tick_get();
    return RT_EOK;
}

rt_err_t ntn_sync_gnss_location(void)
{
    gnss_data_t gnss;
    char cmd[64];
    char resp[128];

    rt_int32_t lat_mdeg;
    rt_int32_t lon_mdeg;

    if (!gnss_get_data(&gnss) || !gnss_is_fresh(5000U) ||
        !gnss.altitude_valid)
    {
        LOG_W("Cannot sync location: GNSS fix not valid");
        return -RT_ERROR;
    }

    if (gnss.latitude_e7 < -900000000 || gnss.latitude_e7 > 900000000 ||
        gnss.longitude_e7 < -1800000000 || gnss.longitude_e7 > 1800000000 ||
        gnss.altitude_mm < -1000000 || gnss.altitude_mm > 6386985)
    {
        return -RT_EINVAL;
    }
    lat_mdeg = (gnss.latitude_e7 >= 0) ?
               (gnss.latitude_e7 + 5000) / 10000 :
               (gnss.latitude_e7 - 5000) / 10000;
    lon_mdeg = (gnss.longitude_e7 >= 0) ?
               (gnss.longitude_e7 + 5000) / 10000 :
               (gnss.longitude_e7 - 5000) / 10000;

    rt_snprintf(cmd, sizeof(cmd), "AT+LOCATION=0,%d,%d,%d",
                lat_mdeg, lon_mdeg, gnss.altitude_mm);
    LOG_I("Syncing GNSS location to MS150: %s", cmd);

    rt_err_t err = ntn_exec_cmd(cmd, resp, sizeof(resp), 2000);
    if (err == RT_EOK)
    {
        LOG_I("GNSS location sync SUCCESS");
    }
    else
    {
        LOG_W("GNSS location sync failed (code %d)", err);
    }

    return err;
}

void ntn_bridge_set(rt_bool_t enable)
{
    if (enable)
    {
        _bridge_active = RT_TRUE;
        rt_mutex_take(&_mutex_at_line, RT_WAITING_FOREVER);
        at_line_reset_locked();
        rt_mutex_release(&_mutex_at_line);
        LOG_I("Mode-0 AT line service enabled");
    }
    else
    {
        _bridge_active = RT_FALSE;
        rt_mutex_take(&_mutex_at_line, RT_WAITING_FOREVER);
        at_line_reset_locked();
        rt_mutex_release(&_mutex_at_line);
        if (_at_timeout_timer)
        {
            rt_timer_stop(_at_timeout_timer);
        }
        LOG_I("Mode-0 AT line service disabled");
    }
}

rt_bool_t ntn_bridge_is_enabled(void)
{
    return _bridge_active;
}

rt_err_t ntn_send_udp_hex(rt_uint8_t socket_id, const char *hex_payload)
{
    char cmd[560];
    char resp[128];
    rt_size_t hex_len;

    if (!hex_payload || socket_id < 1U || socket_id > 5U)
    {
        return -RT_EINVAL;
    }
    hex_len = rt_strlen(hex_payload);
    if (hex_len == 0U || hex_len > 512U || (hex_len & 1U) != 0U)
    {
        return -RT_EINVAL;
    }

    rt_snprintf(cmd, sizeof(cmd), "AT+MKUDPSEND=%u,%s,0", socket_id, hex_payload);
    return ntn_exec_cmd(cmd, resp, sizeof(resp), 60000U);
}

rt_bool_t ntn_service_is_send_ready(void)
{
    return (!ntn_debug_lock_is_held() && _service_status.send_ready) ? RT_TRUE : RT_FALSE;
}

rt_err_t ntn_debug_lock_acquire(void)
{
    rt_tick_t now;
    /* Refuse (rather than steal) an in-flight PC or automatic AT transaction. */
    if (rt_mutex_take(&_mutex_cmd, 0) != RT_EOK) return -RT_EBUSY;
    now = rt_tick_get();
    if ((_pc_command_active &&
         (rt_int32_t)(now - _pc_command_deadline) < 0) || _cmd_waiting ||
        _at_line_len != 0U || _at_discarding)
    {
        rt_mutex_release(&_mutex_cmd);
        return -RT_EBUSY;
    }
    _debug_locked = RT_TRUE;
    _debug_lock_deadline = now + rt_tick_from_millisecond(60000U);
    rt_mutex_release(&_mutex_cmd);
    return RT_EOK;
}

void ntn_debug_lock_release(void)
{
    _debug_locked = RT_FALSE;
    _debug_lock_deadline = 0U;
}

rt_bool_t ntn_debug_lock_is_held(void)
{
    if (_debug_locked &&
        (rt_int32_t)(rt_tick_get() - _debug_lock_deadline) >= 0)
    {
        ntn_debug_lock_release();
    }
    if (_pc_command_active &&
        (rt_int32_t)(rt_tick_get() - _pc_command_deadline) >= 0)
    {
        _pc_command_active = RT_FALSE;
    }
    return _debug_locked;
}

void ntn_set_raw_rx_handler(ntn_raw_rx_handler_t handler)
{
    _raw_rx_handler = handler;
}

void ntn_set_downlink_handler(ntn_downlink_handler_t handler)
{
    _downlink_handler = handler;
}

void ntn_service_request_reconnect(void)
{
    if (!_inited)
    {
        return;
    }
    _service_next_retry = rt_tick_get() + rt_tick_from_millisecond(5000U);
    service_set_state(NTN_SERVICE_BACKOFF, -RT_ERROR);
}

void ntn_service_get_status(ntn_service_status_t *out_status)
{
    if (out_status == RT_NULL)
    {
        return;
    }
    rt_mutex_take(&_service_lock, RT_WAITING_FOREVER);
    *out_status = _service_status;
    rt_mutex_release(&_service_lock);
    rt_mutex_take(&_downlink_lock, RT_WAITING_FOREVER);
    out_status->downlink_count = _downlink_count;
    out_status->downlink_parse_errors = _downlink_parse_errors;
    rt_mutex_release(&_downlink_lock);
}

rt_bool_t ntn_service_get_last_downlink(ntn_downlink_t *out_downlink)
{
    rt_bool_t valid;
    if (out_downlink == RT_NULL || !_inited)
    {
        return RT_FALSE;
    }
    rt_mutex_take(&_downlink_lock, RT_WAITING_FOREVER);
    *out_downlink = _last_downlink;
    valid = _last_downlink.valid;
    rt_mutex_release(&_downlink_lock);
    return valid;
}

rt_err_t ntn_service_send_raw(const rt_uint8_t *payload, rt_size_t len)
{
    char command[560];
    char response[NTN_RESP_BUF_MAX_SZ];
    static const char hex[] = "0123456789ABCDEF";
    rt_size_t i;
    rt_size_t pos;
    rt_err_t result;

    if (payload == RT_NULL || len == 0U || len > 256U)
    {
        return -RT_EINVAL;
    }
    if (!ntn_service_is_send_ready())
    {
        return -RT_EBUSY;
    }

    pos = (rt_size_t)rt_snprintf(command, sizeof(command), "AT+MKUDPSEND=1,");
    for (i = 0U; i < len; i++)
    {
        command[pos++] = hex[payload[i] >> 4];
        command[pos++] = hex[payload[i] & 0x0FU];
    }
    command[pos++] = ',';
    command[pos++] = '0';
    command[pos] = '\0';

    result = ntn_exec_cmd(command, response, sizeof(response), 60000U);
    rt_mutex_take(&_service_lock, RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        _service_status.modem_accepted_count++;
    }
    else if (result == -RT_ETIMEOUT)
    {
        _service_status.unknown_result_count++;
    }
    rt_mutex_release(&_service_lock);
    if (result != RT_EOK)
    {
        _service_next_retry = rt_tick_get() + rt_tick_from_millisecond(5000U);
        service_set_state(NTN_SERVICE_BACKOFF, result);
    }
    return result;
}

void ntn_get_stats(ntn_stats_t *out_stats)
{
    if (out_stats)
    {
        rt_memcpy(out_stats, &_stats, sizeof(ntn_stats_t));
    }
}

/* ------------------- FinSH / MSH Commands ------------------- */
#ifdef RT_USING_FINSH
#include <finsh.h>

static int ntn_status(int argc, char **argv)
{
    ntn_signal_t sig;
    rt_memset(&sig, 0, sizeof(sig));
    ntn_get_signal(&sig);

    const char *reg_str = "Unknown";
    switch (sig.reg_status)
    {
        case NTN_REG_NOT_REGISTERED: reg_str = "Not Registered (Idle)"; break;
        case NTN_REG_HOME:           reg_str = "Registered (Home Network)"; break;
        case NTN_REG_SEARCHING:      reg_str = "Searching Satellite Network..."; break;
        case NTN_REG_DENIED:         reg_str = "Registration Denied"; break;
        case NTN_REG_UNKNOWN:        reg_str = "No Satellite Coverage"; break;
        case NTN_REG_ROAMING:        reg_str = "Registered (Roaming)"; break;
    }

    rt_kprintf("\r\n================ MS150 NTN Modem Status ================\r\n");
    rt_kprintf(" Port           : %s (PB4-TX, PB5-RX @ 115200 8N1)\r\n", NTN_UART_NAME);
    rt_kprintf(" Bridge Mode    : %s\r\n", _bridge_active ? "ENABLED (PC uart1 <-> MS150)" : "DISABLED");
    rt_kprintf(" NTN Core Power : %s (PB15)\r\n", hw_power_ntn_get() ? "ON" : "OFF");
    rt_kprintf(" RF PA Power    : %s (PB0)\r\n", rt_pin_read(PA_PWREN_PIN) ? "ON" : "OFF");
    rt_kprintf(" NTN_STATUS Pin : %s (PB3)\r\n", hw_ntn_status_get() ? "HIGH (Active)" : "LOW (Inactive)");
    rt_kprintf(" Registration   : %d (%s)\r\n", sig.reg_status, reg_str);
    rt_kprintf(" Signal (CSQ)   : RSSI=%d, BER=%d\r\n", sig.rssi, sig.ber);
    rt_kprintf("---------------- Statistics -----------------\r\n");
    rt_kprintf(" TX Total Bytes : %u\r\n", _stats.tx_bytes);
    rt_kprintf(" RX Total Bytes : %u\r\n", _stats.rx_bytes);
    rt_kprintf(" AT Commands    : Total=%u, OK=%u, Err=%u\r\n",
               _stats.at_cmds_sent, _stats.at_cmds_ok, _stats.at_cmds_err);
    rt_kprintf(" Bridge Traffic : PC->NTN: %u bytes, NTN->PC: %u bytes\r\n",
               _stats.bridge_pc_rx, _stats.bridge_ntn_to_pc);
    rt_kprintf("========================================================\r\n\r\n");

    return 0;
}
MSH_CMD_EXPORT(ntn_status, display MS150 Iridium NTN modem status and signal metrics);

static int ntn_at(int argc, char **argv)
{
    char resp[NTN_RESP_BUF_MAX_SZ];

    if (argc < 2)
    {
        rt_kprintf("Usage: ntn_at <AT command string>\r\n");
        rt_kprintf("Examples:\r\n");
        rt_kprintf("  ntn_at ATI\r\n");
        rt_kprintf("  ntn_at AT+CSQ\r\n");
        rt_kprintf("  ntn_at AT+CEREG?\r\n");
        rt_kprintf("  ntn_at AT+MKCELLINFO\r\n");
        return -1;
    }

    rt_kprintf("[AT TX] %s\r\n", argv[1]);
    rt_err_t err = ntn_exec_cmd(argv[1], resp, sizeof(resp), 3000);
    if (err == RT_EOK)
    {
        rt_kprintf("[AT RX] (OK):\r\n%s\r\n", resp);
    }
    else if (err == -RT_ETIMEOUT)
    {
        rt_kprintf("[AT RX] (TIMEOUT):\r\n%s\r\n", resp);
    }
    else
    {
        rt_kprintf("[AT RX] (ERROR %d):\r\n%s\r\n", err, resp);
    }

    return 0;
}
MSH_CMD_EXPORT(ntn_at, execute raw AT command on MS150 modem);

static int ntn_bridge(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: ntn_bridge <on|off>\r\n");
        rt_kprintf("Current state: %s\r\n", _bridge_active ? "ON" : "OFF");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        ntn_bridge_set(RT_TRUE);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        ntn_bridge_set(RT_FALSE);
    }
    else
    {
        rt_kprintf("Unknown parameter: %s, use 'on' or 'off'\r\n", argv[1]);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(ntn_bridge, toggle PC RS232 <-> NTN MS150 transparent bridge);

static int ntn_power(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: ntn_power <on|off>\r\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        ntn_power_set(RT_TRUE);
        rt_kprintf("NTN modem & PA power ENABLED\r\n");
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        ntn_power_set(RT_FALSE);
        rt_kprintf("NTN modem & PA power DISABLED\r\n");
    }
    else
    {
        rt_kprintf("Unknown parameter: %s, use 'on' or 'off'\r\n", argv[1]);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(ntn_power, control MS150 modem power supply);

static int ntn_reset_cmd(int argc, char **argv)
{
    rt_kprintf("Triggering MS150 NTN hardware reset (PA8 high pulse 200ms)...\r\n");
    ntn_reset(200);
    rt_kprintf("Reset complete.\r\n");
    return 0;
}
MSH_CMD_EXPORT_ALIAS(ntn_reset_cmd, ntn_reset, hardware reset MS150 modem);

static int ntn_loc_sync(int argc, char **argv)
{
    return (int)ntn_sync_gnss_location();
}
MSH_CMD_EXPORT(ntn_loc_sync, synchronize current GNSS location to MS150);

static int ntn_udp_send(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: ntn_udp_send <hex_payload> [socket_id]\r\n");
        rt_kprintf("Example: ntn_udp_send 0031323334 1\r\n");
        return -1;
    }

    rt_uint8_t sock = 1;
    if (argc >= 3)
    {
        sock = (rt_uint8_t)atoi(argv[2]);
    }

    rt_err_t err = ntn_send_udp_hex(sock, argv[1]);
    rt_kprintf("ntn_send_udp_hex returned: %d\r\n", err);
    return (int)err;
}
MSH_CMD_EXPORT(ntn_udp_send, send hex payload via UDP socket over satellite);

#endif /* RT_USING_FINSH */
