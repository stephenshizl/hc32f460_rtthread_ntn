/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 5.3: End-to-End Automated Telemetry Engine & NTAP Packaging
 */

#include "app_telemetry.h"
#include "app_modbus.h"
#include "param_manager.h"
#include "gnss_um220.h"
#include "ntn_ms150.h"
#include "store_forward.h"
#include "pc_rs232.h"
#include "task_wdt.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.telem"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define TELEM_THREAD_STACK_SZ   2048
#define TELEM_THREAD_PRIO       15

static telem_stats_t    s_stats;
static rt_thread_t      s_telem_thread = RT_NULL;
static rt_uint16_t      s_telem_seq = 1;
static rt_uint16_t      s_rs232_seq = 1;
static volatile rt_bool_t s_trigger_now = RT_FALSE;
static rt_bool_t        s_telem_inited = RT_FALSE;

/* Standard CCITT CRC-16 (poly 0x1021, init 0xFFFF) */
static rt_uint16_t ntap_calc_crc16(const void *buf, rt_size_t len)
{
    rt_uint16_t crc = 0xFFFF;
    const rt_uint8_t *p = (const rt_uint8_t *)buf;

    for (rt_size_t i = 0; i < len; i++)
    {
        crc ^= (rt_uint16_t)p[i] << 8;
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* Callback invoked when PC RS232 port receives a complete idle-detected frame */
static void app_telemetry_rs232_rx_handler(const rt_uint8_t *data, rt_size_t len)
{
    if (!data || len == 0)
    {
        return;
    }

    rt_uint8_t mode = param_get_work_mode();

    /* In RAW_PASS mode, all PC RS232 frames are enveloped and enqueued for satellite transmission */
    if (mode == PARAM_MODE_RAW_PASS)
    {
        rt_size_t total_len = sizeof(ntap_rs232_hdr_t) + len;
        rt_uint8_t *env_buf = (rt_uint8_t *)rt_malloc(total_len);
        if (!env_buf)
        {
            LOG_E("Out of memory for RS232 NTAP envelope!");
            return;
        }

        ntap_rs232_hdr_t *hdr = (ntap_rs232_hdr_t *)env_buf;
        hdr->msg_type  = NTAP_MSG_TYPE_RS232_RAW;
        hdr->reserved  = 0x00;
        hdr->seq_id    = s_rs232_seq++;
        hdr->timestamp = rt_tick_get();
        hdr->raw_len   = (rt_uint16_t)len;

        rt_memcpy(env_buf + sizeof(ntap_rs232_hdr_t), data, len);

        rt_uint32_t flash_seq = 0;
        rt_err_t res = store_forward_push(env_buf, total_len, &flash_seq);
        rt_free(env_buf);

        if (res == RT_EOK)
        {
            s_stats.rs232_frames_fwd++;
            LOG_I("[RS232->NTN] Enqueued %d B raw payload to Store&Forward (SeqID=%d)", len, flash_seq);
            store_forward_trigger_flush();
        }
        else
        {
            LOG_E("Failed to enqueue RS232 payload to Store&Forward (err=%d)", res);
        }
    }
    else if (mode == PARAM_MODE_MODBUS)
    {
        /* In MODBUS mode, RS232 frames are slave responses -> route to Modbus engine */
        app_modbus_on_rx_frame(data, len);
    }
}

/* Sample GNSS, construct NTAP telemetry packet, and push to Store & Forward */
static rt_err_t app_telemetry_generate_frame(void)
{
    gnss_data_t gnss;
    ntn_signal_t sig;
    ntap_telemetry_pkt_t pkt;

    /* 1. Fetch latest GNSS data */
    gnss_get_data(&gnss);

    /* 2. Automatically sync latest GNSS coordinates to MS150 modem for ephemeris calculation */
    ntn_sync_gnss_location();

    /* 3. Query satellite signal */
    ntn_get_signal(&sig);

    /* 4. Construct compact 28-byte NTAP binary packet */
    rt_memset(&pkt, 0, sizeof(pkt));
    pkt.msg_type = NTAP_MSG_TYPE_TELEMETRY;

    pkt.flags = 0;
    if (gnss.is_fixed)
    {
        pkt.flags |= NTAP_FLAG_GNSS_VALID;
    }
    pkt.flags |= NTAP_FLAG_EXT_POWER; /* Powered by main DC */

    pkt.seq_id         = s_telem_seq++;
    pkt.timestamp      = rt_tick_get();
    pkt.lat_e7         = gnss.latitude_e7;
    pkt.lon_e7         = gnss.longitude_e7;
    pkt.alt_m          = (rt_int16_t)gnss.altitude;
    pkt.speed_kmh_x10  = (rt_uint16_t)((gnss.speed_cm_s * 36) / 100); /* cm/s to 0.1 km/h */
    pkt.course_deg     = (rt_uint16_t)gnss.course;
    pkt.num_sats       = gnss.sats_used;
    pkt.battery_pct    = 100;
    pkt.rssi           = (rt_int8_t)sig.rssi;
    pkt.work_mode      = param_get_work_mode();

    /* Calculate CRC-16 over all packet fields except payload_crc16 itself */
    pkt.payload_crc16  = ntap_calc_crc16(&pkt, sizeof(pkt) - sizeof(rt_uint16_t));

    /* 5. Push packet into Store & Forward persistent Flash queue */
    rt_uint32_t flash_seq = 0;
    rt_err_t ret = store_forward_push(&pkt, sizeof(pkt), &flash_seq);

    if (ret == RT_EOK)
    {
        s_stats.telem_frames_gen++;
        s_stats.last_sample_tick = rt_tick_get();

        int lat_int = (int)(pkt.lat_e7 / 10000000);
        int lat_frac = abs((int)(pkt.lat_e7 % 10000000));
        int lon_int = (int)(pkt.lon_e7 / 10000000);
        int lon_frac = abs((int)(pkt.lon_e7 % 10000000));

        LOG_I("[TELEMETRY] Frame #%d generated: Lat=%d.%07d, Lon=%d.%07d, Alt=%dm, Spd=%.1fkm/h, Sats=%d -> Store&Forward Seq=%d",
              pkt.seq_id, lat_int, lat_frac, lon_int, lon_frac, pkt.alt_m, (float)pkt.speed_kmh_x10 / 10.0f, pkt.num_sats, flash_seq);

        /* Trigger satellite flush worker */
        store_forward_trigger_flush();
    }
    else
    {
        LOG_E("[TELEMETRY] Failed to push telemetry packet into Store&Forward (err=%d)", ret);
    }

    return ret;
}

/* Background worker thread */
static void telemetry_thread_entry(void *param)
{
    LOG_I("Automated Telemetry Engine running...");
    s_stats.engine_running = RT_TRUE;

    while (1)
    {
        rt_uint32_t interval_s = param_get_report_interval();
        if (interval_s < 5)
        {
            interval_s = 5;
        }

        /* 1-second interval loop for responsive trigger & watchdog kicking */
        for (rt_uint32_t i = 0; i < interval_s; i++)
        {
            rt_thread_mdelay(1000);
            task_wdt_heartbeat_by_name("telem");

            if (s_trigger_now)
            {
                s_trigger_now = RT_FALSE;
                break;
            }
        }

        rt_uint8_t mode = param_get_work_mode();
        if (mode == PARAM_MODE_AUTO_TELEMETRY || s_trigger_now)
        {
            s_trigger_now = RT_FALSE;
            app_telemetry_generate_frame();
        }
    }
}

rt_err_t app_telemetry_trigger_now(void)
{
    s_trigger_now = RT_TRUE;
    return RT_EOK;
}

void app_telemetry_get_stats(telem_stats_t *out_stats)
{
    if (out_stats)
    {
        *out_stats = s_stats;
    }
}

int app_telemetry_init(void)
{
    if (s_telem_inited)
    {
        return RT_EOK;
    }

    rt_memset(&s_stats, 0, sizeof(s_stats));

    /* 1. Register RS232 receiver hook for transparent envelope packaging */
    pc_rs232_set_rx_handler(app_telemetry_rs232_rx_handler);

    /* 2. Register with Task Watchdog framework */
    task_wdt_register("telem", 60);

    /* 3. Start telemetry background daemon thread */
    s_telem_thread = rt_thread_create("telem",
                                      telemetry_thread_entry,
                                      RT_NULL,
                                      TELEM_THREAD_STACK_SZ,
                                      TELEM_THREAD_PRIO,
                                      10);
    if (s_telem_thread != RT_NULL)
    {
        rt_thread_startup(s_telem_thread);
    }
    else
    {
        LOG_E("Failed to create telemetry thread!");
        return -RT_ERROR;
    }

    s_telem_inited = RT_TRUE;
    LOG_I("Automated Telemetry Engine initialized successfully.");
    return RT_EOK;
}
INIT_APP_EXPORT(app_telemetry_init);

/* MSH Diagnostic Commands */
static void telem_status(int argc, char **argv)
{
    telem_stats_t st;
    app_telemetry_get_stats(&st);

    gnss_data_t gnss;
    gnss_get_data(&gnss);

    rt_kprintf("\n============= Automated Telemetry Status =============\n");
    rt_kprintf(" Engine State       : %s\n", st.engine_running ? "RUNNING" : "STOPPED");
    rt_kprintf(" Work Mode          : %d (%s)\n",
               param_get_work_mode(),
               param_get_work_mode() == 0 ? "RAW_PASS" :
               param_get_work_mode() == 1 ? "AUTO_TELEMETRY" : "MODBUS");
    rt_kprintf(" Report Interval    : %d Seconds\n", param_get_report_interval());
    rt_kprintf(" Telemetry Packets  : %d Generated\n", st.telem_frames_gen);
    rt_kprintf(" RS232 Raw Envelopes: %d Enqueued\n", st.rs232_frames_fwd);

    if (st.last_sample_tick > 0)
    {
        rt_uint32_t elapsed_s = (rt_tick_get() - st.last_sample_tick) / RT_TICK_PER_SECOND;
        rt_kprintf(" Last Sample Time   : %d seconds ago\n", elapsed_s);
    }
    else
    {
        rt_kprintf(" Last Sample Time   : Never\n");
    }

    rt_kprintf(" -----------------------------------------------------\n");
    rt_kprintf(" Latest GNSS Status :\n");
    rt_kprintf("   Fix Status       : %s (Quality=%d)\n", gnss.is_fixed ? "FIXED" : "NO FIX", gnss.fix_quality);
    rt_kprintf("   Satellites       : %d\n", gnss.sats_used);

    int lat_int = (int)(gnss.latitude_e7 / 10000000);
    int lat_frac = abs((int)(gnss.latitude_e7 % 10000000));
    int lon_int = (int)(gnss.longitude_e7 / 10000000);
    int lon_frac = abs((int)(gnss.longitude_e7 % 10000000));

    rt_kprintf("   Latitude         : %d.%07d deg\n", lat_int, lat_frac);
    rt_kprintf("   Longitude        : %d.%07d deg\n", lon_int, lon_frac);
    rt_kprintf("   Altitude         : %d m\n", (int)gnss.altitude);
    rt_kprintf("   Speed            : %.2f km/h\n", gnss.speed_kmh);
    rt_kprintf("   Heading          : %.1f deg\n", gnss.course);
    rt_kprintf("======================================================\n\n");
}
MSH_CMD_EXPORT(telem_status, Display automated telemetry status and GNSS position);

static void telem_trigger(int argc, char **argv)
{
    rt_kprintf("Triggering manual telemetry frame generation...\n");
    app_telemetry_trigger_now();
}
MSH_CMD_EXPORT(telem_trigger, Trigger immediate telemetry generation & enqueue);

static void telem_mode(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: telem_mode <0:RawPass, 1:AutoTelemetry, 2:Modbus>\n");
        return;
    }

    int mode = atoi(argv[1]);
    if (mode < 0 || mode > 2)
    {
        rt_kprintf("Invalid mode! 0:RawPass, 1:AutoTelemetry, 2:Modbus\n");
        return;
    }

    param_set_work_mode((rt_uint8_t)mode, RT_TRUE);
    rt_kprintf("Work mode switched to %d and committed to Flash.\n", mode);
}
MSH_CMD_EXPORT(telem_mode, Set terminal operational mode: telem_mode <0|1|2>);

static void telem_interval(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: telem_interval <seconds>\n");
        return;
    }

    int sec = atoi(argv[1]);
    if (sec < 5 || sec > 86400)
    {
        rt_kprintf("Invalid interval! Must be between 5 and 86400 seconds.\n");
        return;
    }

    param_set_report_interval((rt_uint32_t)sec, RT_TRUE);
    rt_kprintf("Telemetry interval set to %d seconds and committed to Flash.\n", sec);
}
MSH_CMD_EXPORT(telem_interval, Set telemetry report interval: telem_interval <sec>);
