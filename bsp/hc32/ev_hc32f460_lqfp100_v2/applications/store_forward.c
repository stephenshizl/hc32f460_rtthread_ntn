/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 4: Store & Forward offline persistent queue manager
 */

#include "store_forward.h"
#include "app_flash.h"
#include "ntn_ms150.h"
#include "task_wdt.h"
#include <fal.h>
#include <stdio.h>
#include <string.h>

#define DBG_TAG "app.sf"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define SF_RECORD_ALIGN(len)    (((len) + 3) & ~3)
#define SF_HDR_SZ               sizeof(sf_record_hdr_t)
#define SF_STATUS_OFFSET        12  /* Byte offset of 'status' field in sf_record_hdr_t */

static const struct fal_partition *s_sf_part = RT_NULL;
static struct rt_mutex             s_sf_lock;
static struct rt_semaphore         s_sf_sem;
static rt_thread_t                 s_flush_thread = RT_NULL;

static rt_uint16_t s_head_sector = 0;
static rt_uint16_t s_head_offset = 0;
static rt_uint16_t s_tail_sector = 0;
static rt_uint32_t s_next_seq_id = 1;

static rt_uint32_t s_total_pushed    = 0;
static rt_uint32_t s_total_pending   = 0;
static rt_uint32_t s_total_confirmed = 0;
static rt_uint32_t s_total_dropped   = 0;
static rt_bool_t   s_auto_flush      = RT_TRUE;
static rt_bool_t   s_sf_init_ok      = RT_FALSE;

/* Fast CRC-16 (CCITT: poly 0x1021, init 0xFFFF) */
static rt_uint16_t sf_calc_crc(const sf_record_hdr_t *hdr, const rt_uint8_t *payload)
{
    rt_uint16_t crc = 0xFFFF;
    const rt_uint8_t *p_hdr = (const rt_uint8_t *)hdr;

    /* Calculate CRC over immutable fields: magic(2B) + seq_id(4B) + timestamp(4B) + data_len(2B) = 12 bytes */
    for (int i = 0; i < 12; i++)
    {
        crc ^= (rt_uint16_t)p_hdr[i] << 8;
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

    if (payload && hdr->data_len > 0)
    {
        for (int i = 0; i < hdr->data_len; i++)
        {
            crc ^= (rt_uint16_t)payload[i] << 8;
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
    }

    return crc;
}

/* Fast startup scan to rebuild in-memory queue state from Flash */
static void sf_scan_and_recover(void)
{
    rt_uint32_t max_seq = 0;
    rt_int32_t  last_written_sec = -1;
    rt_uint16_t last_written_off = 0;
    rt_int32_t  first_pending_sec = -1;

    s_total_pending   = 0;
    s_total_confirmed = 0;

    sf_record_hdr_t hdr;

    for (rt_uint16_t sec = 0; sec < SF_TOTAL_SECTORS; sec++)
    {
        rt_uint32_t sec_base = sec * SF_SECTOR_SIZE;
        rt_uint16_t off = 0;

        while (off + SF_HDR_SZ <= SF_SECTOR_SIZE)
        {
            if (fal_partition_read(s_sf_part, sec_base + off, (rt_uint8_t *)&hdr, SF_HDR_SZ) < 0)
            {
                break;
            }

            if (hdr.magic != SF_RECORD_MAGIC || hdr.data_len == 0 || hdr.data_len > SF_MAX_PAYLOAD_LEN)
            {
                /* No more valid records in this sector */
                break;
            }

            rt_size_t rec_len = SF_RECORD_ALIGN(SF_HDR_SZ + hdr.data_len);
            if (off + rec_len > SF_SECTOR_SIZE)
            {
                break;
            }

            /* Record is structural valid */
            if (hdr.seq_id > max_seq)
            {
                max_seq = hdr.seq_id;
                last_written_sec = sec;
                last_written_off = off + rec_len;
            }

            if (hdr.status == SF_STATUS_PENDING)
            {
                s_total_pending++;
                if (first_pending_sec < 0)
                {
                    first_pending_sec = sec;
                }
            }
            else if (hdr.status == SF_STATUS_CONFIRMED)
            {
                s_total_confirmed++;
            }

            off += rec_len;
        }
    }

    if (last_written_sec >= 0)
    {
        s_head_sector = (rt_uint16_t)last_written_sec;
        s_head_offset = last_written_off;
        s_next_seq_id = max_seq + 1;
    }
    else
    {
        s_head_sector = 0;
        s_head_offset = 0;
        s_next_seq_id = 1;
    }

    if (first_pending_sec >= 0)
    {
        s_tail_sector = (rt_uint16_t)first_pending_sec;
    }
    else
    {
        s_tail_sector = s_head_sector;
    }

    s_total_pushed = s_total_pending + s_total_confirmed;

    LOG_I("Queue recovery complete: HeadSec=%d, HeadOff=%d, TailSec=%d, NextSeq=%d, Pending=%d, Confirmed=%d",
          s_head_sector, s_head_offset, s_tail_sector, s_next_seq_id, s_total_pending, s_total_confirmed);
}

/* Push a record into Flash */
rt_err_t store_forward_push(const void *data, rt_size_t len, rt_uint32_t *out_seq)
{
    if (!s_sf_init_ok || data == RT_NULL || len == 0 || len > SF_MAX_PAYLOAD_LEN)
    {
        return -RT_EINVAL;
    }

    rt_size_t rec_len = SF_RECORD_ALIGN(SF_HDR_SZ + len);
    rt_uint8_t *rec_buf = (rt_uint8_t *)rt_malloc(rec_len);
    if (!rec_buf)
    {
        return -RT_ENOMEM;
    }

    rt_mutex_take(&s_sf_lock, RT_WAITING_FOREVER);

    /* Check if current head sector has room */
    if (s_head_offset + rec_len > SF_SECTOR_SIZE)
    {
        rt_uint16_t next_sec = (s_head_sector + 1) % SF_TOTAL_SECTORS;

        /* Check for circular queue full condition */
        if (next_sec == s_tail_sector)
        {
            LOG_W("Queue wrapped around! Advancing tail sector to make room for new records.");
            s_tail_sector = (s_tail_sector + 1) % SF_TOTAL_SECTORS;
            s_total_dropped++;
        }

        /* Erase the new sector */
        fal_partition_erase(s_sf_part, next_sec * SF_SECTOR_SIZE, SF_SECTOR_SIZE);
        s_head_sector = next_sec;
        s_head_offset = 0;
    }
    else if (s_head_offset == 0)
    {
        /* Erase brand new head sector */
        fal_partition_erase(s_sf_part, s_head_sector * SF_SECTOR_SIZE, SF_SECTOR_SIZE);
    }

    /* Build header */
    sf_record_hdr_t *hdr = (sf_record_hdr_t *)rec_buf;
    hdr->magic     = SF_RECORD_MAGIC;
    hdr->seq_id    = s_next_seq_id++;
    hdr->timestamp = rt_tick_get();
    hdr->data_len  = (rt_uint16_t)len;
    hdr->status    = SF_STATUS_PENDING;
    hdr->reserved  = 0xFF;
    hdr->crc16     = sf_calc_crc(hdr, (const rt_uint8_t *)data);

    /* Copy payload and pad trailing bytes with 0xFF */
    rt_memcpy(rec_buf + SF_HDR_SZ, data, len);
    for (rt_size_t i = SF_HDR_SZ + len; i < rec_len; i++)
    {
        rec_buf[i] = 0xFF;
    }

    /* Write to Flash */
    rt_uint32_t flash_addr = s_head_sector * SF_SECTOR_SIZE + s_head_offset;
    int ret = fal_partition_write(s_sf_part, flash_addr, rec_buf, rec_len);
    if (ret >= 0)
    {
        s_head_offset += (rt_uint16_t)rec_len;
        s_total_pushed++;
        s_total_pending++;
        if (out_seq)
        {
            *out_seq = hdr->seq_id;
        }
        LOG_D("Pushed record seq=%d, len=%d at sec=%d, off=%d", hdr->seq_id, len, s_head_sector, flash_addr);
    }
    else
    {
        LOG_E("Failed to write record to Flash (err=%d)", ret);
    }

    rt_mutex_release(&s_sf_lock);
    rt_free(rec_buf);

    if (ret >= 0)
    {
        rt_sem_release(&s_sf_sem);
        return RT_EOK;
    }
    return -RT_EIO;
}

/* Peek the oldest pending record without confirming */
rt_err_t store_forward_peek(sf_record_hdr_t *hdr, void *data_buf, rt_size_t buf_sz)
{
    if (!s_sf_init_ok || hdr == RT_NULL || data_buf == RT_NULL)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_sf_lock, RT_WAITING_FOREVER);

    if (s_total_pending == 0)
    {
        rt_mutex_release(&s_sf_lock);
        return -RT_EEMPTY;
    }

    rt_uint16_t sec = s_tail_sector;
    rt_bool_t   found = RT_FALSE;

    for (rt_uint16_t count = 0; count < SF_TOTAL_SECTORS; count++)
    {
        rt_uint32_t sec_base = sec * SF_SECTOR_SIZE;
        rt_uint16_t off = 0;

        while (off + SF_HDR_SZ <= SF_SECTOR_SIZE)
        {
            sf_record_hdr_t cur_hdr;
            if (fal_partition_read(s_sf_part, sec_base + off, (rt_uint8_t *)&cur_hdr, SF_HDR_SZ) < 0)
            {
                break;
            }

            if (cur_hdr.magic != SF_RECORD_MAGIC || cur_hdr.data_len == 0 || cur_hdr.data_len > SF_MAX_PAYLOAD_LEN)
            {
                break;
            }

            rt_size_t rec_len = SF_RECORD_ALIGN(SF_HDR_SZ + cur_hdr.data_len);
            if (off + rec_len > SF_SECTOR_SIZE)
            {
                break;
            }

            if (cur_hdr.status == SF_STATUS_PENDING)
            {
                /* Read payload and verify CRC */
                rt_size_t copy_len = (cur_hdr.data_len < buf_sz) ? cur_hdr.data_len : buf_sz;
                fal_partition_read(s_sf_part, sec_base + off + SF_HDR_SZ, (rt_uint8_t *)data_buf, copy_len);

                rt_uint16_t expected_crc = sf_calc_crc(&cur_hdr, (const rt_uint8_t *)data_buf);
                if (cur_hdr.crc16 == expected_crc)
                {
                    *hdr = cur_hdr;
                    found = RT_TRUE;
                    break;
                }
                else
                {
                    LOG_W("CRC mismatch on record seq=%d! Expected=0x%04X, Read=0x%04X",
                          cur_hdr.seq_id, cur_hdr.crc16, expected_crc);
                }
            }

            off += rec_len;
        }

        if (found)
        {
            s_tail_sector = sec;
            break;
        }

        if (sec == s_head_sector)
        {
            break;
        }
        sec = (sec + 1) % SF_TOTAL_SECTORS;
    }

    rt_mutex_release(&s_sf_lock);
    return found ? RT_EOK : -RT_EEMPTY;
}

/* Mark a record acknowledged via zero-erase write (0xFF -> 0x00) */
rt_err_t store_forward_ack(rt_uint32_t seq_id)
{
    if (!s_sf_init_ok)
    {
        return -RT_EINVAL;
    }

    rt_mutex_take(&s_sf_lock, RT_WAITING_FOREVER);

    rt_uint16_t sec = s_tail_sector;
    rt_bool_t   found = RT_FALSE;

    for (rt_uint16_t count = 0; count < SF_TOTAL_SECTORS; count++)
    {
        rt_uint32_t sec_base = sec * SF_SECTOR_SIZE;
        rt_uint16_t off = 0;

        while (off + SF_HDR_SZ <= SF_SECTOR_SIZE)
        {
            sf_record_hdr_t cur_hdr;
            if (fal_partition_read(s_sf_part, sec_base + off, (rt_uint8_t *)&cur_hdr, SF_HDR_SZ) < 0)
            {
                break;
            }

            if (cur_hdr.magic != SF_RECORD_MAGIC || cur_hdr.data_len == 0 || cur_hdr.data_len > SF_MAX_PAYLOAD_LEN)
            {
                break;
            }

            rt_size_t rec_len = SF_RECORD_ALIGN(SF_HDR_SZ + cur_hdr.data_len);

            if (cur_hdr.seq_id == seq_id)
            {
                if (cur_hdr.status == SF_STATUS_PENDING)
                {
                    /* Perform Zero-Erase write directly to 'status' byte */
                    rt_uint8_t zero_status = SF_STATUS_CONFIRMED;
                    rt_uint32_t status_addr = sec_base + off + SF_STATUS_OFFSET;

                    fal_partition_write(s_sf_part, status_addr, &zero_status, 1);

                    if (s_total_pending > 0)
                    {
                        s_total_pending--;
                    }
                    s_total_confirmed++;
                    LOG_D("Record seq=%d confirmed in Flash via Zero-Erase at 0x%08X", seq_id, status_addr);
                }
                found = RT_TRUE;
                break;
            }

            off += rec_len;
        }

        if (found)
        {
            break;
        }

        if (sec == s_head_sector)
        {
            break;
        }
        sec = (sec + 1) % SF_TOTAL_SECTORS;
    }

    rt_mutex_release(&s_sf_lock);
    return found ? RT_EOK : -RT_ERROR;
}

/* Erase entire store_forward partition */
rt_err_t store_forward_clear(void)
{
    if (!s_sf_init_ok)
    {
        return -RT_EINVAL;
    }

    LOG_I("Erasing Store & Forward partition (768 KB)...");
    rt_mutex_take(&s_sf_lock, RT_WAITING_FOREVER);

    fal_partition_erase(s_sf_part, 0, SF_TOTAL_SECTORS * SF_SECTOR_SIZE);

    s_head_sector     = 0;
    s_head_offset     = 0;
    s_tail_sector     = 0;
    s_next_seq_id     = 1;
    s_total_pushed    = 0;
    s_total_pending   = 0;
    s_total_confirmed = 0;
    s_total_dropped   = 0;

    rt_mutex_release(&s_sf_lock);
    LOG_I("Store & Forward queue cleared successfully.");
    return RT_EOK;
}

void store_forward_get_stats(sf_stats_t *stats)
{
    if (!stats)
    {
        return;
    }

    rt_mutex_take(&s_sf_lock, RT_WAITING_FOREVER);
    stats->total_pushed       = s_total_pushed;
    stats->total_pending      = s_total_pending;
    stats->total_confirmed    = s_total_confirmed;
    stats->total_dropped      = s_total_dropped;
    stats->next_seq_id        = s_next_seq_id;
    stats->head_sector        = s_head_sector;
    stats->head_offset        = s_head_offset;
    stats->tail_sector        = s_tail_sector;
    stats->auto_flush_enabled = s_auto_flush;
    rt_mutex_release(&s_sf_lock);
}

void store_forward_trigger_flush(void)
{
    rt_sem_release(&s_sf_sem);
}

void store_forward_set_auto_flush(rt_bool_t enable)
{
    s_auto_flush = enable;
    if (enable)
    {
        rt_sem_release(&s_sf_sem);
    }
}

/* Auto-flush background worker thread */
static void sf_flush_entry(void *param)
{
    sf_record_hdr_t hdr;
    rt_uint8_t payload[SF_MAX_PAYLOAD_LEN + 1];
    char hex_str[SF_MAX_PAYLOAD_LEN * 2 + 1];

    while (1)
    {
        /* Wait on semaphore with 10-second periodic timeout */
        rt_sem_take(&s_sf_sem, rt_tick_from_millisecond(10000));
        task_wdt_heartbeat_by_name("sf_flush");

        if (!s_auto_flush || s_total_pending == 0)
        {
            continue;
        }

        /* Check NTN modem network registration status */
        ntn_signal_t sig;
        if (ntn_get_signal(&sig) != RT_EOK ||
            (sig.reg_status != NTN_REG_HOME && sig.reg_status != NTN_REG_ROAMING))
        {
            /* No satellite coverage or not registered yet */
            continue;
        }

        LOG_I("NTN satellite link active (Reg=%d, RSSI=%d). Flushing %d pending records...",
              sig.reg_status, sig.rssi, s_total_pending);

        while (s_total_pending > 0)
        {
            rt_err_t res = store_forward_peek(&hdr, payload, sizeof(payload));
            if (res != RT_EOK)
            {
                break;
            }

            /* Convert binary payload to HEX string for NTN AT command */
            for (rt_uint16_t i = 0; i < hdr.data_len; i++)
            {
                rt_sprintf(&hex_str[i * 2], "%02X", payload[i]);
            }
            hex_str[hdr.data_len * 2] = '\0';

            /* Send via MS150 UDP socket */
            rt_err_t send_res = ntn_send_udp_hex(1, hex_str);
            if (send_res == RT_EOK)
            {
                /* Mark acknowledged via zero-erase write */
                store_forward_ack(hdr.seq_id);
                LOG_I("Transmitted & ACKed record [seq=%d, len=%d] successfully.", hdr.seq_id, hdr.data_len);
                rt_thread_mdelay(200);
            }
            else
            {
                LOG_W("Failed to transmit record [seq=%d] over NTN (err=%d). Pausing flush.", hdr.seq_id, send_res);
                break;
            }
        }
    }
}

/* Initialization */
int store_forward_init(void)
{
    if (s_sf_init_ok)
    {
        return RT_EOK;
    }

    /* Ensure external flash & FAL are initialized first */
    app_flash_init();

    s_sf_part = fal_partition_find(SF_PARTITION_NAME);
    if (s_sf_part == RT_NULL)
    {
        LOG_E("Partition '%s' not found in FAL table!", SF_PARTITION_NAME);
        return -RT_ERROR;
    }

    rt_mutex_init(&s_sf_lock, "sf_lock", RT_IPC_FLAG_PRIO);
    rt_sem_init(&s_sf_sem, "sf_sem", 0, RT_IPC_FLAG_PRIO);

    /* Fast recovery scan */
    sf_scan_and_recover();

    s_flush_thread = rt_thread_create("sf_flush",
                                      sf_flush_entry,
                                      RT_NULL,
                                      2048,
                                      22,
                                      20);
    if (s_flush_thread != RT_NULL)
    {
        rt_thread_startup(s_flush_thread);
    }
    else
    {
        LOG_E("Failed to create Store & Forward flush thread!");
    }

    s_sf_init_ok = RT_TRUE;
    LOG_I("Store & Forward persistent queue ready (Part='%s', Capacity=768 KB)", SF_PARTITION_NAME);
    return RT_EOK;
}
INIT_APP_EXPORT(store_forward_init);

/* MSH Diagnostics */
static void sf_status(int argc, char **argv)
{
    sf_stats_t s;
    store_forward_get_stats(&s);

    rt_kprintf("\n=========== Store & Forward Offline Queue ===========\n");
    rt_kprintf(" Partition Name     : %s\n", SF_PARTITION_NAME);
    rt_kprintf(" Total Size         : %d KB (%d Sectors)\n", (SF_TOTAL_SECTORS * SF_SECTOR_SIZE) / 1024, SF_TOTAL_SECTORS);
    rt_kprintf(" Head Pointer       : Sector %d, Offset %d\n", s.head_sector, s.head_offset);
    rt_kprintf(" Tail Pointer       : Sector %d\n", s.tail_sector);
    rt_kprintf(" Next Sequence ID   : %d\n", s.next_seq_id);
    rt_kprintf(" Total Pushed       : %d\n", s.total_pushed);
    rt_kprintf(" Total Pending      : %d (Awaiting Satellite Link)\n", s.total_pending);
    rt_kprintf(" Total Confirmed    : %d (Zero-Erase ACKed)\n", s.total_confirmed);
    rt_kprintf(" Total Dropped      : %d (Queue Overflows)\n", s.total_dropped);
    rt_kprintf(" Auto-Flush Enabled : %s\n", s.auto_flush_enabled ? "YES" : "NO");
    rt_kprintf("=====================================================\n\n");
}
MSH_CMD_EXPORT(sf_status, Display Store & Forward queue status and statistics);

static void sf_push(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: sf_push <message_text>\n");
        return;
    }

    const char *msg = argv[1];
    rt_size_t len = rt_strlen(msg);
    rt_uint32_t seq = 0;

    rt_err_t res = store_forward_push(msg, len, &seq);
    if (res == RT_EOK)
    {
        rt_kprintf("Successfully pushed into offline queue: SeqID=%d, Len=%d\n", seq, len);
    }
    else
    {
        rt_kprintf("Failed to push message (err=%d)\n", res);
    }
}
MSH_CMD_EXPORT(sf_push, Push a message into offline persistent queue: sf_push <text>);

static void sf_peek(int argc, char **argv)
{
    sf_record_hdr_t hdr;
    rt_uint8_t buf[SF_MAX_PAYLOAD_LEN + 1];

    rt_err_t res = store_forward_peek(&hdr, buf, sizeof(buf) - 1);
    if (res == RT_EOK)
    {
        buf[hdr.data_len] = '\0';
        rt_kprintf("Oldest Pending Record:\n");
        rt_kprintf(" Seq ID    : %d\n", hdr.seq_id);
        rt_kprintf(" Timestamp : %d ticks\n", hdr.timestamp);
        rt_kprintf(" Length    : %d bytes\n", hdr.data_len);
        rt_kprintf(" Status    : 0x%02X (PENDING)\n", hdr.status);
        rt_kprintf(" CRC16     : 0x%04X\n", hdr.crc16);
        rt_kprintf(" Data(Str) : %s\n", (char *)buf);
        rt_kprintf(" Data(Hex) : ");
        for (int i = 0; i < hdr.data_len; i++)
        {
            rt_kprintf("%02X ", buf[i]);
        }
        rt_kprintf("\n");
    }
    else if (res == -RT_EEMPTY)
    {
        rt_kprintf("Queue is empty. No pending records.\n");
    }
    else
    {
        rt_kprintf("Peek failed: err=%d\n", res);
    }
}
MSH_CMD_EXPORT(sf_peek, Peek oldest pending record in queue);

static void sf_pop(int argc, char **argv)
{
    sf_record_hdr_t hdr;
    rt_uint8_t buf[SF_MAX_PAYLOAD_LEN + 1];

    rt_err_t res = store_forward_peek(&hdr, buf, sizeof(buf) - 1);
    if (res == RT_EOK)
    {
        buf[hdr.data_len] = '\0';
        store_forward_ack(hdr.seq_id);
        rt_kprintf("Popped & Confirmed Record: SeqID=%d, Data='%s'\n", hdr.seq_id, (char *)buf);
    }
    else if (res == -RT_EEMPTY)
    {
        rt_kprintf("Queue is empty.\n");
    }
    else
    {
        rt_kprintf("Pop failed: err=%d\n", res);
    }
}
MSH_CMD_EXPORT(sf_pop, Pop and acknowledge oldest pending record in queue);

static void sf_clear(int argc, char **argv)
{
    store_forward_clear();
}
MSH_CMD_EXPORT(sf_clear, Clear all records in Store & Forward partition);

static void sf_flush(int argc, char **argv)
{
    rt_kprintf("Triggering Store & Forward satellite flush...\n");
    store_forward_trigger_flush();
}
MSH_CMD_EXPORT(sf_flush, Trigger immediate flush of pending records over NTN);

static void sf_autoflush(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: sf_autoflush <1|0>\n");
        return;
    }

    int enable = argv[1][0] - '0';
    store_forward_set_auto_flush(enable ? RT_TRUE : RT_FALSE);
    rt_kprintf("Auto-flush %s\n", enable ? "ENABLED" : "DISABLED");
}
MSH_CMD_EXPORT(sf_autoflush, Enable or disable auto satellite flush: sf_autoflush <1|0>);
