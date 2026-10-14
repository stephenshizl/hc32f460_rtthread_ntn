/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 5.4: Industrial Modbus RTU Master Auto-Polling Collector
 */

#include "app_modbus.h"
#include "app_telemetry.h"
#include "param_manager.h"
#include "pc_rs232.h"
#include "store_forward.h"
#include "task_wdt.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.modbus"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define MODBUS_THREAD_STACK_SZ  2048
#define MODBUS_THREAD_PRIO      16

static modbus_channel_t s_channels[MODBUS_MAX_CHANNELS];
static modbus_stats_t   s_stats;
static struct rt_mutex  s_mb_lock;
static struct rt_semaphore s_mb_resp_sem;
static rt_thread_t      s_mb_thread = RT_NULL;

static rt_uint8_t       s_resp_buf[256];
static rt_size_t        s_resp_len = 0;
static rt_uint8_t       s_expecting_slave = 0;
static rt_uint8_t       s_expecting_fc = 0;
static volatile rt_bool_t s_is_waiting_resp = RT_FALSE;
static rt_bool_t        s_mb_inited = RT_FALSE;

/* Standard Modbus CRC-16 (Polynomial 0xA001, Initial 0xFFFF, Low byte first) */
static rt_uint16_t modbus_crc16(const rt_uint8_t *buf, rt_size_t len)
{
    rt_uint16_t crc = 0xFFFF;

    for (rt_size_t i = 0; i < len; i++)
    {
        crc ^= (rt_uint16_t)buf[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* Callback from RS232 driver when a frame arrives in MODBUS mode */
void app_modbus_on_rx_frame(const rt_uint8_t *data, rt_size_t len)
{
    if (!data || len < 4 || !s_is_waiting_resp)
    {
        return;
    }

    /* Check if response matches the current awaiting slave and function code */
    if (data[0] == s_expecting_slave && (data[1] == s_expecting_fc || data[1] == (s_expecting_fc | 0x80)))
    {
        rt_size_t copy_len = (len < sizeof(s_resp_buf)) ? len : sizeof(s_resp_buf);
        rt_memcpy(s_resp_buf, data, copy_len);
        s_resp_len = copy_len;
        s_is_waiting_resp = RT_FALSE;
        rt_sem_release(&s_mb_resp_sem);
    }
}

rt_err_t app_modbus_read_registers(rt_uint8_t slave, rt_uint8_t fc, rt_uint16_t start_reg,
                                  rt_uint16_t reg_count, rt_uint16_t *out_regs, rt_uint32_t timeout_ms)
{
    if (slave == 0 || (fc != MODBUS_FC_READ_HOLDING && fc != MODBUS_FC_READ_INPUT) ||
        reg_count == 0 || reg_count > MODBUS_MAX_REGS_PER_POLL || !out_regs)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_mb_lock, RT_WAITING_FOREVER);

    /* Construct 8-byte Modbus RTU Read Request */
    rt_uint8_t req[8];
    req[0] = slave;
    req[1] = fc;
    req[2] = (rt_uint8_t)(start_reg >> 8);
    req[3] = (rt_uint8_t)(start_reg & 0xFF);
    req[4] = (rt_uint8_t)(reg_count >> 8);
    req[5] = (rt_uint8_t)(reg_count & 0xFF);

    rt_uint16_t crc = modbus_crc16(req, 6);
    req[6] = (rt_uint8_t)(crc & 0xFF);        /* CRC Low */
    req[7] = (rt_uint8_t)((crc >> 8) & 0xFF); /* CRC High */

    /* Reset response synchronization */
    while (rt_sem_trytake(&s_mb_resp_sem) == RT_EOK);
    s_resp_len = 0;
    s_expecting_slave = slave;
    s_expecting_fc = fc;
    s_is_waiting_resp = RT_TRUE;

    s_stats.polls_sent++;

    /* Transmit request over PC RS232 port */
    pc_rs232_send(req, sizeof(req));

    /* Wait for slave response */
    rt_err_t ret = rt_sem_take(&s_mb_resp_sem, rt_tick_from_millisecond(timeout_ms));
    s_is_waiting_resp = RT_FALSE;

    if (ret != RT_EOK)
    {
        s_stats.polls_timeout++;
        rt_mutex_release(&s_mb_lock);
        return -RT_ETIMEOUT;
    }

    /* Validate response length */
    rt_size_t expected_len = 3 + 2 * reg_count + 2;
    if (s_resp_len < 5)
    {
        s_stats.polls_crc_err++;
        rt_mutex_release(&s_mb_lock);
        return -RT_EIO;
    }

    /* Check Modbus Exception */
    if (s_resp_buf[1] & 0x80)
    {
        s_stats.polls_exception++;
        LOG_W("Modbus slave %d returned Exception code: 0x%02X", slave, s_resp_buf[2]);
        rt_mutex_release(&s_mb_lock);
        return -RT_ERROR;
    }

    if (s_resp_len != expected_len)
    {
        s_stats.polls_crc_err++;
        LOG_W("Modbus response length mismatch! Expected %d B, got %d B", expected_len, s_resp_len);
        rt_mutex_release(&s_mb_lock);
        return -RT_EIO;
    }

    /* Verify CRC16 */
    rt_uint16_t calc_crc = modbus_crc16(s_resp_buf, s_resp_len - 2);
    rt_uint16_t recv_crc = (rt_uint16_t)s_resp_buf[s_resp_len - 2] |
                           ((rt_uint16_t)s_resp_buf[s_resp_len - 1] << 8);

    if (calc_crc != recv_crc)
    {
        s_stats.polls_crc_err++;
        LOG_W("Modbus response CRC Error! Calc=0x%04X, Recv=0x%04X", calc_crc, recv_crc);
        rt_mutex_release(&s_mb_lock);
        return -RT_EIO;
    }

    /* Extract 16-bit register values */
    for (rt_uint16_t i = 0; i < reg_count; i++)
    {
        out_regs[i] = ((rt_uint16_t)s_resp_buf[3 + i * 2] << 8) |
                      (rt_uint16_t)s_resp_buf[4 + i * 2];
    }

    s_stats.polls_ok++;
    rt_mutex_release(&s_mb_lock);
    return RT_EOK;
}

rt_err_t app_modbus_write_single_register(rt_uint8_t slave, rt_uint16_t reg, rt_uint16_t value, rt_uint32_t timeout_ms)
{
    if (slave == 0)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_mb_lock, RT_WAITING_FOREVER);

    /* Construct 8-byte Modbus RTU Write Single Register Request */
    rt_uint8_t req[8];
    req[0] = slave;
    req[1] = MODBUS_FC_WRITE_SINGLE;
    req[2] = (rt_uint8_t)(reg >> 8);
    req[3] = (rt_uint8_t)(reg & 0xFF);
    req[4] = (rt_uint8_t)(value >> 8);
    req[5] = (rt_uint8_t)(value & 0xFF);

    rt_uint16_t crc = modbus_crc16(req, 6);
    req[6] = (rt_uint8_t)(crc & 0xFF);
    req[7] = (rt_uint8_t)((crc >> 8) & 0xFF);

    while (rt_sem_trytake(&s_mb_resp_sem) == RT_EOK);
    s_resp_len = 0;
    s_expecting_slave = slave;
    s_expecting_fc = MODBUS_FC_WRITE_SINGLE;
    s_is_waiting_resp = RT_TRUE;

    pc_rs232_send(req, sizeof(req));

    rt_err_t ret = rt_sem_take(&s_mb_resp_sem, rt_tick_from_millisecond(timeout_ms));
    s_is_waiting_resp = RT_FALSE;

    if (ret != RT_EOK)
    {
        rt_mutex_release(&s_mb_lock);
        return -RT_ETIMEOUT;
    }

    /* Normal response echoes request bytes */
    if (s_resp_len == 8 && rt_memcmp(req, s_resp_buf, 8) == 0)
    {
        rt_mutex_release(&s_mb_lock);
        return RT_EOK;
    }

    rt_mutex_release(&s_mb_lock);
    return -RT_EIO;
}

/* Poll channel, extract data, package NTAP Modbus packet, and push to Store & Forward */
rt_err_t app_modbus_poll_channel(rt_uint8_t ch_idx)
{
    if (ch_idx >= MODBUS_MAX_CHANNELS)
    {
        return -RT_EINVAL;
    }

    modbus_channel_t *ch = &s_channels[ch_idx];
    if (!ch->is_enabled)
    {
        return -RT_EINVAL;
    }

    ch->last_poll_tick = rt_tick_get();

    rt_uint16_t regs[MODBUS_MAX_REGS_PER_POLL];
    rt_err_t res = app_modbus_read_registers(ch->slave_addr, ch->func_code, ch->start_reg,
                                            ch->reg_count, regs, MODBUS_DEFAULT_TIMEOUT_MS);
    if (res == RT_EOK)
    {
        ch->last_poll_ok = RT_TRUE;
        rt_memcpy(ch->last_regs, regs, ch->reg_count * sizeof(rt_uint16_t));

        /* Package compact NTAP Modbus Telemetry Packet */
        ntap_modbus_pkt_t pkt;
        rt_memset(&pkt, 0, sizeof(pkt));
        pkt.msg_type   = NTAP_MSG_TYPE_MODBUS_DATA;
        pkt.slave_addr = ch->slave_addr;
        pkt.func_code  = ch->func_code;
        pkt.byte_count = (rt_uint8_t)(ch->reg_count * 2);
        pkt.start_reg  = ch->start_reg;
        pkt.reg_count  = ch->reg_count;
        pkt.timestamp  = rt_tick_get();
        rt_memcpy(pkt.reg_values, regs, ch->reg_count * sizeof(rt_uint16_t));

        rt_size_t pkt_len = 12 + ch->reg_count * 2; /* Header(10B) + Regs(2*NB) + CRC(2B) */

        /* Calculate CRC16 excluding crc16 field itself */
        pkt.payload_crc16 = modbus_crc16((const rt_uint8_t *)&pkt, pkt_len - 2);

        /* Push to Store & Forward offline Flash queue */
        rt_uint32_t flash_seq = 0;
        rt_err_t push_ret = store_forward_push(&pkt, pkt_len, &flash_seq);
        if (push_ret == RT_EOK)
        {
            s_stats.packets_enqueued++;
            LOG_I("[MODBUS] Ch%d Polled OK (Slave %d, Regs 0x%04X..0x%04X) -> Store&Forward Seq=%d",
                  ch_idx, ch->slave_addr, ch->start_reg, ch->start_reg + ch->reg_count - 1, flash_seq);

            /* Trigger satellite flush */
            store_forward_trigger_flush();
        }
        return RT_EOK;
    }
    else
    {
        ch->last_poll_ok = RT_FALSE;
        LOG_W("[MODBUS] Ch%d Poll failed! Slave=%d, Func=0x%02X, Reg=0x%04X (err=%d)",
              ch_idx, ch->slave_addr, ch->func_code, ch->start_reg, res);
        return res;
    }
}

/* Polling daemon thread */
static void modbus_master_thread_entry(void *param)
{
    LOG_I("Modbus RTU Master auto-polling daemon started.");

    while (1)
    {
        rt_thread_mdelay(1000);
        task_wdt_heartbeat_by_name("modbus");

        /* Only poll when system is configured for MODBUS mode */
        if (param_get_work_mode() != PARAM_MODE_MODBUS)
        {
            continue;
        }

        rt_tick_t now = rt_tick_get();

        for (rt_uint8_t i = 0; i < MODBUS_MAX_CHANNELS; i++)
        {
            modbus_channel_t *ch = &s_channels[i];
            if (ch->is_enabled)
            {
                rt_uint32_t period_ticks = rt_tick_from_millisecond(ch->poll_period_s * 1000);
                if (now - ch->last_poll_tick >= period_ticks)
                {
                    app_modbus_poll_channel(i);
                    rt_thread_mdelay(200); /* Small spacing between polls */
                }
            }
        }
    }
}

void app_modbus_get_stats(modbus_stats_t *out_stats)
{
    if (out_stats)
    {
        *out_stats = s_stats;
    }
}

rt_err_t app_modbus_get_channel(rt_uint8_t channel_idx, modbus_channel_t *out_ch)
{
    if (channel_idx >= MODBUS_MAX_CHANNELS || !out_ch)
    {
        return -RT_EINVAL;
    }

    *out_ch = s_channels[channel_idx];
    return RT_EOK;
}

int app_modbus_init(void)
{
    if (s_mb_inited)
    {
        return RT_EOK;
    }

    rt_memset(s_channels, 0, sizeof(s_channels));
    rt_memset(&s_stats, 0, sizeof(s_stats));

    rt_mutex_init(&s_mb_lock, "mb_lck", RT_IPC_FLAG_PRIO);
    rt_sem_init(&s_mb_resp_sem, "mb_sem", 0, RT_IPC_FLAG_PRIO);

    /* Configure Channel 0 as default industrial sensor channel */
    s_channels[0].slave_addr    = 1;                        /* Slave ID 1 (e.g. Tanker fuel/level sensor) */
    s_channels[0].func_code     = MODBUS_FC_READ_HOLDING;   /* 0x03 */
    s_channels[0].start_reg     = 0x0000;                   /* Reg 0 */
    s_channels[0].reg_count     = 4;                        /* 4 registers: Fuel level, Temp, Volt, Status */
    s_channels[0].poll_period_s = 60;                       /* Poll every 60 seconds */
    s_channels[0].is_enabled    = RT_TRUE;

    /* Register with Task Watchdog */
    task_wdt_register("modbus", 60);

    /* Start background master polling thread */
    s_mb_thread = rt_thread_create("mb_poll",
                                   modbus_master_thread_entry,
                                   RT_NULL,
                                   MODBUS_THREAD_STACK_SZ,
                                   MODBUS_THREAD_PRIO,
                                   10);
    if (s_mb_thread != RT_NULL)
    {
        rt_thread_startup(s_mb_thread);
    }
    else
    {
        LOG_E("Failed to create Modbus Master polling thread!");
        return -RT_ERROR;
    }

    s_mb_inited = RT_TRUE;
    LOG_I("Modbus RTU Master Collector ready (Default Ch0: Slave 1, 4 Regs @ 60s)");
    return RT_EOK;
}
INIT_APP_EXPORT(app_modbus_init);

/* MSH Diagnostic Commands */
static void mb_status(int argc, char **argv)
{
    modbus_stats_t st;
    app_modbus_get_stats(&st);

    rt_kprintf("\n============= Modbus RTU Master Status =============\n");
    rt_kprintf(" Operating Mode     : %s\n",
               param_get_work_mode() == PARAM_MODE_MODBUS ? "ACTIVE (PARAM_MODE_MODBUS)" : "INACTIVE (Switch via: telem_mode 2)");
    rt_kprintf(" Total Polls Sent   : %d\n", st.polls_sent);
    rt_kprintf(" Successful Polls   : %d\n", st.polls_ok);
    rt_kprintf(" Response Timeouts  : %d\n", st.polls_timeout);
    rt_kprintf(" CRC Errors         : %d\n", st.polls_crc_err);
    rt_kprintf(" Exception Responses: %d\n", st.polls_exception);
    rt_kprintf(" Packets Enqueued   : %d\n\n", st.packets_enqueued);

    rt_kprintf(" Polling Channels Table:\n");
    rt_kprintf(" -----------------------------------------------------------------\n");
    rt_kprintf(" | Ch | Slave | Func | StartReg | Count | Period | Status | Values\n");
    rt_kprintf(" -----------------------------------------------------------------\n");

    for (int i = 0; i < MODBUS_MAX_CHANNELS; i++)
    {
        modbus_channel_t ch;
        app_modbus_get_channel(i, &ch);

        if (ch.is_enabled)
        {
            rt_kprintf(" | %2d | %5d | 0x%02X |   0x%04X | %5d | %5ds | %-6s | ",
                       i, ch.slave_addr, ch.func_code, ch.start_reg, ch.reg_count,
                       ch.poll_period_s, ch.last_poll_ok ? "OK" : "ERR");

            for (int r = 0; r < ch.reg_count && r < 4; r++)
            {
                rt_kprintf("[%d]=%d ", r, ch.last_regs[r]);
            }
            rt_kprintf("\n");
        }
    }
    rt_kprintf(" =================================================================\n\n");
}
MSH_CMD_EXPORT(mb_status, Display Modbus Master status and register values);

static void mb_poll_cmd(int argc, char **argv)
{
    rt_uint8_t ch = 0;
    if (argc > 1)
    {
        ch = (rt_uint8_t)atoi(argv[1]);
    }

    rt_kprintf("Polling Modbus channel %d...\n", ch);
    rt_err_t res = app_modbus_poll_channel(ch);
    if (res == RT_EOK)
    {
        modbus_channel_t c;
        app_modbus_get_channel(ch, &c);
        rt_kprintf("Poll Succeeded! Slave %d, Regs 0x%04X..0x%04X:\n",
                   c.slave_addr, c.start_reg, c.start_reg + c.reg_count - 1);
        for (int i = 0; i < c.reg_count; i++)
        {
            rt_kprintf("  [0x%04X] = %5d (0x%04X)\n", c.start_reg + i, c.last_regs[i], c.last_regs[i]);
        }
    }
    else
    {
        rt_kprintf("Poll Failed (err=%d)!\n", res);
    }
}
MSH_CMD_EXPORT_ALIAS(mb_poll_cmd, mb_poll, Manually poll a Modbus channel: mb_poll [ch_idx]);

static void mb_read_cmd(int argc, char **argv)
{
    if (argc < 5)
    {
        rt_kprintf("Usage: mb_read <slave> <fc:3|4> <start_reg> <count>\n");
        rt_kprintf("Example: mb_read 1 3 0 4\n");
        return;
    }

    rt_uint8_t slave = (rt_uint8_t)atoi(argv[1]);
    rt_uint8_t fc = (rt_uint8_t)atoi(argv[2]);
    rt_uint16_t start_reg = (rt_uint16_t)strtoul(argv[3], NULL, 0);
    rt_uint16_t count = (rt_uint16_t)atoi(argv[4]);

    rt_uint16_t regs[MODBUS_MAX_REGS_PER_POLL];
    rt_kprintf("Reading from Slave %d, FC 0x%02X, Reg 0x%04X, Count %d...\n", slave, fc, start_reg, count);

    rt_err_t res = app_modbus_read_registers(slave, fc, start_reg, count, regs, 1000);
    if (res == RT_EOK)
    {
        rt_kprintf("Read OK:\n");
        for (int i = 0; i < count; i++)
        {
            rt_kprintf("  Reg[0x%04X] = %5d (0x%04X)\n", start_reg + i, regs[i], regs[i]);
        }
    }
    else
    {
        rt_kprintf("Read Failed (err=%d)!\n", res);
    }
}
MSH_CMD_EXPORT_ALIAS(mb_read_cmd, mb_read, Read Modbus registers: mb_read <slave> <fc> <reg> <count>);

static void mb_write_cmd(int argc, char **argv)
{
    if (argc < 4)
    {
        rt_kprintf("Usage: mb_write <slave> <reg> <val>\n");
        rt_kprintf("Example: mb_write 1 0 100\n");
        return;
    }

    rt_uint8_t slave = (rt_uint8_t)atoi(argv[1]);
    rt_uint16_t reg = (rt_uint16_t)strtoul(argv[2], NULL, 0);
    rt_uint16_t val = (rt_uint16_t)strtoul(argv[3], NULL, 0);

    rt_kprintf("Writing to Slave %d, Reg 0x%04X = %d (0x%04X)...\n", slave, reg, val, val);
    rt_err_t res = app_modbus_write_single_register(slave, reg, val, 1000);
    if (res == RT_EOK)
    {
        rt_kprintf("Write Register OK!\n");
    }
    else
    {
        rt_kprintf("Write Register Failed (err=%d)!\n", res);
    }
}
MSH_CMD_EXPORT_ALIAS(mb_write_cmd, mb_write, Write Modbus register: mb_write <slave> <reg> <val>);
