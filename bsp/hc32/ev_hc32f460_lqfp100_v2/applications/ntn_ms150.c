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

static rt_device_t              _ntn_dev           = RT_NULL;
static struct rt_semaphore      _sem_rx;
static struct rt_semaphore      _sem_resp;
static struct rt_mutex          _mutex_cmd;
static rt_thread_t              _rx_thread         = RT_NULL;

static ntn_stats_t              _stats;
static rt_bool_t                _bridge_active     = RT_FALSE;
static rt_bool_t                _inited            = RT_FALSE;

/* AT Command Transaction Management */
static volatile rt_bool_t       _cmd_waiting       = RT_FALSE;
static volatile rt_err_t        _cmd_status        = RT_EOK;
static char                    *_cur_resp_buf      = RT_NULL;
static rt_size_t                _cur_resp_sz       = 0;
static rt_size_t                _cur_resp_idx      = 0;

/* AT Line Assembly & Incomplete Command Guard */
static rt_uint8_t               _at_line_buf[512];
static rt_size_t                _at_line_len       = 0;
static rt_timer_t               _at_timeout_timer  = RT_NULL;

static void at_line_timeout_entry(void *parameter)
{
    RT_UNUSED(parameter);

    if (_at_line_len > 0)
    {
        LOG_W("[NTN BRIDGE] PC AT cmd timeout (>2s, %d B discarded, missing CRLF): %.*s",
              _at_line_len, _at_line_len, _at_line_buf);
        _at_line_len = 0;
        /* Notify PC that the truncated command resulted in ERROR */
        pc_rs232_send_str("\r\nERROR\r\n");
    }
}

void ntn_bridge_process_pc_data(const rt_uint8_t *data, rt_size_t len)
{
    if (!_bridge_active || !_ntn_dev || !data || len == 0)
    {
        return;
    }

    _stats.bridge_pc_rx += len;

    for (rt_size_t i = 0; i < len; i++)
    {
        rt_uint8_t ch = data[i];

        if (_at_line_len == 0)
        {
            /* Skip leading whitespace or standalone CRLF */
            if (ch == '\r' || ch == '\n' || ch == ' ')
            {
                continue;
            }

            /* If first character does not start an AT command, treat as raw transparent stream */
            if (ch != 'A' && ch != 'a' && ch != '+')
            {
                rt_size_t rem = len - i;
                rt_device_write(_ntn_dev, 0, &data[i], rem);
                _stats.tx_bytes += rem;
                return;
            }

            /* Start 2-second timeout timer for incomplete command detection */
            if (_at_timeout_timer)
            {
                rt_timer_stop(_at_timeout_timer);
                rt_timer_start(_at_timeout_timer);
            }
        }

        /* Append character to assembly line buffer */
        if (_at_line_len < sizeof(_at_line_buf) - 2)
        {
            _at_line_buf[_at_line_len++] = ch;
        }
        else
        {
            /* Line buffer overflow protection */
            LOG_E("[NTN BRIDGE] AT line buffer overflow (>512 B)!");
            _at_line_len = 0;
            if (_at_timeout_timer)
            {
                rt_timer_stop(_at_timeout_timer);
            }
            pc_rs232_send_str("\r\nERROR\r\n");
            return;
        }

        /* Check for command line termination: \r or \n */
        if (ch == '\r' || ch == '\n')
        {
            if (_at_timeout_timer)
            {
                rt_timer_stop(_at_timeout_timer);
            }

            /* If \r is followed by \n in the same buffer, consume it */
            if (ch == '\r' && (i + 1 < len) && data[i + 1] == '\n')
            {
                i++;
                if (_at_line_len < sizeof(_at_line_buf) - 1)
                {
                    _at_line_buf[_at_line_len++] = '\n';
                }
            }
            else if (ch == '\r')
            {
                /* Standardize: append \n if missing */
                if (_at_line_len < sizeof(_at_line_buf) - 1)
                {
                    _at_line_buf[_at_line_len++] = '\n';
                }
            }

            _at_line_buf[_at_line_len] = '\0';

            /* Deliver full command atomically to NTN modem at 115200 bps */
            rt_device_write(_ntn_dev, 0, _at_line_buf, _at_line_len);
            _stats.tx_bytes += _at_line_len;
            _stats.at_cmds_sent++;

            LOG_D("[PC->NTN Bridge] Forwarded complete AT cmd (%d B): %s", _at_line_len, _at_line_buf);

            _at_line_len = 0;
        }
    }
}

static void ntn_pc_bridge_rx_handler(const rt_uint8_t *data, rt_size_t len)
{
    ntn_bridge_process_pc_data(data, len);
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

            /* 1. If bridge mode is active, forward bytes to PC RS232 port */
            if (_bridge_active)
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
                                        rt_tick_from_millisecond(2000),
                                        RT_TIMER_FLAG_ONE_SHOT);

    _inited = RT_TRUE;
    LOG_I("MS150 NTN Modem ready: %s @ 115200 8N1", NTN_UART_NAME);

    return RT_EOK;
}
INIT_APP_EXPORT(ntn_ms150_init);

void ntn_power_set(rt_bool_t on)
{
    hw_power_ntn_set(on);
}

void ntn_reset(rt_uint32_t pulse_ms)
{
    hw_reset_ntn(pulse_ms);
}

rt_err_t ntn_exec_cmd(const char *cmd, char *resp_buf, rt_size_t buf_sz, rt_uint32_t timeout_ms)
{
    rt_err_t err = RT_EOK;

    if (!_inited || !_ntn_dev || !cmd)
    {
        return -RT_ERROR;
    }

    if (rt_mutex_take(&_mutex_cmd, rt_tick_from_millisecond(timeout_ms)) != RT_EOK)
    {
        return -RT_EBUSY;
    }

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
    rt_device_write(_ntn_dev, 0, cmd, len);
    if (len == 0 || cmd[len - 1] != '\n')
    {
        rt_device_write(_ntn_dev, 0, "\r\n", 2);
        _stats.tx_bytes += 2;
    }
    _stats.tx_bytes += len;
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
    rt_mutex_release(&_mutex_cmd);

    return err;
}

rt_size_t ntn_send_raw(const void *data, rt_size_t len)
{
    if (!_inited || !_ntn_dev || !data || len == 0)
    {
        return 0;
    }
    rt_size_t sent = rt_device_write(_ntn_dev, 0, data, len);
    _stats.tx_bytes += sent;
    return sent;
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

    if (!gnss_get_data(&gnss) || !gnss.is_fixed)
    {
        LOG_W("Cannot sync location: GNSS fix not valid");
        return -RT_ERROR;
    }

    /*
     * MS150 Format: AT+LOCATION=0.<lat_int>.<lon_int>.<alt_mm>
     * Example: AT+LOCATION=0.22498.113920.3200 (Lat 22.498, Lon 113.920, Alt 3.2m)
     */
    int lat_val = (int)(abs(gnss.latitude_e7) / 10000);
    int lon_val = (int)(abs(gnss.longitude_e7) / 10000);
    int alt_mm  = (int)(gnss.altitude * 1000.0f);

    rt_snprintf(cmd, sizeof(cmd), "AT+LOCATION=0.%d.%d.%d", lat_val, lon_val, alt_mm);
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
        _at_line_len   = 0;
        pc_rs232_set_rx_handler(ntn_pc_bridge_rx_handler);
        LOG_I("PC RS232 <-> NTN MS150 Transparent Bridge ENABLED (Default Mode 0)");
        pc_rs232_send_str("\r\n[NTN BRIDGE] PC <-> NTN MS150 Bridge Active (19200 <-> 115200)\r\n");
    }
    else
    {
        _bridge_active = RT_FALSE;
        _at_line_len   = 0;
        if (_at_timeout_timer)
        {
            rt_timer_stop(_at_timeout_timer);
        }
        LOG_I("PC RS232 <-> NTN MS150 Transparent Bridge DISABLED");
        pc_rs232_send_str("\r\n[NTN BRIDGE] Bridge Disabled\r\n");
    }
}

rt_bool_t ntn_bridge_is_enabled(void)
{
    return _bridge_active;
}

rt_err_t ntn_send_udp_hex(rt_uint8_t socket_id, const char *hex_payload)
{
    char cmd[128];
    char resp[128];

    if (!hex_payload)
    {
        return -RT_ERROR;
    }

    rt_snprintf(cmd, sizeof(cmd), "AT+MKUDPSEND=%d,%s", socket_id, hex_payload);
    return ntn_exec_cmd(cmd, resp, sizeof(resp), 5000);
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
