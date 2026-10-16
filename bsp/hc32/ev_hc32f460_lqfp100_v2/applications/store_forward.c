/* SPDX-License-Identifier: Apache-2.0 */

#include "store_forward.h"
#include "app_flash.h"
#include "param_manager.h"
#include <fal.h>
#include <stddef.h>
#include <string.h>

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.sfq3"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static const struct fal_partition *s_part;
static struct rt_mutex s_lock;
static rt_bool_t s_ready;
static rt_bool_t s_storage_fault;
static rt_bool_t s_history_unavailable;
static rt_uint32_t s_head_slot;
static rt_uint32_t s_tail_slot;
static rt_uint32_t s_next_seq = 1U;
static rt_uint32_t s_total_pushed;
static rt_uint32_t s_total_pending;
static rt_uint32_t s_total_accepted;
static rt_uint32_t s_total_rejected;
static rt_uint32_t s_used_slots;
static rt_uint16_t s_sector_pending[SF_TOTAL_SECTORS];
static rt_uint16_t s_sector_used[SF_TOTAL_SECTORS];
static rt_uint16_t s_sector_accepted[SF_TOTAL_SECTORS];
static rt_bool_t s_sector_bad[SF_TOTAL_SECTORS];

static rt_uint32_t sf_slot_address(rt_uint32_t slot)
{
    rt_uint32_t sector = slot / SF_SLOTS_PER_SECTOR;
    rt_uint32_t index = slot % SF_SLOTS_PER_SECTOR;
    return sector * SF_SECTOR_SIZE + index * SF_SLOT_SIZE;
}

static rt_uint32_t sf_next_slot(rt_uint32_t slot)
{
    slot++;
    return (slot >= SF_TOTAL_SLOTS) ? 0U : slot;
}

static rt_uint32_t sf_crc_update(rt_uint32_t crc, const rt_uint8_t *data, rt_size_t len)
{
    rt_size_t i;
    for (i = 0U; i < len; i++)
    {
        rt_uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return crc;
}

static rt_uint32_t sf_record_crc(const sf_record_hdr_t *hdr, const rt_uint8_t *payload)
{
    rt_uint32_t crc = 0xFFFFFFFFUL;
    crc = sf_crc_update(crc, (const rt_uint8_t *)hdr, offsetof(sf_record_hdr_t, status));
    crc = sf_crc_update(crc, payload, hdr->data_len);
    return ~crc;
}

static rt_bool_t sf_header_is_structural(const sf_record_hdr_t *hdr)
{
    return (hdr->magic == SF_RECORD_MAGIC &&
            hdr->format_version == SF_FORMAT_VERSION &&
            hdr->header_size == sizeof(sf_record_hdr_t) &&
            hdr->data_len > 0U && hdr->data_len <= SF_MAX_PAYLOAD_LEN &&
            hdr->commit == SF_COMMIT_VALID &&
            (hdr->status == SF_STATUS_PENDING || hdr->status == SF_STATUS_ACCEPTED)) ?
            RT_TRUE : RT_FALSE;
}

static rt_err_t sf_read_record(rt_uint32_t slot, sf_record_hdr_t *hdr,
                               rt_uint8_t *payload, rt_size_t payload_size)
{
    rt_uint32_t address;
    int result;

    if (slot >= SF_TOTAL_SLOTS || hdr == RT_NULL || payload == RT_NULL)
    {
        return -RT_EINVAL;
    }
    address = sf_slot_address(slot);
    result = fal_partition_read(s_part, address, (rt_uint8_t *)hdr, sizeof(*hdr));
    if (result < 0)
    {
        return -RT_EIO;
    }
    if (!sf_header_is_structural(hdr))
    {
        return -RT_ERROR;
    }
    if (payload_size < hdr->data_len)
    {
        return -RT_EINVAL;
    }
    result = fal_partition_read(s_part, address + sizeof(*hdr), payload, hdr->data_len);
    if (result < 0)
    {
        return -RT_EIO;
    }
    return (sf_record_crc(hdr, payload) == hdr->crc32) ? RT_EOK : -RT_ERROR;
}

static rt_err_t sf_read_magic(rt_uint32_t slot, rt_uint32_t *magic)
{
    int result = fal_partition_read(s_part, sf_slot_address(slot),
                                    (rt_uint8_t *)magic, sizeof(*magic));
    return (result < 0) ? -RT_EIO : RT_EOK;
}

static void sf_mark_io_fault(rt_bool_t history_unavailable)
{
    s_storage_fault = RT_TRUE;
    if (history_unavailable)
    {
        s_history_unavailable = RT_TRUE;
    }
}

static void sf_scan(void)
{
    rt_uint32_t slot;
    rt_uint32_t max_seq = 0U;
    rt_uint32_t min_pending_seq = 0xFFFFFFFFUL;
    rt_uint32_t max_slot = 0U;
    rt_uint32_t min_pending_slot = 0U;
    rt_uint8_t payload[SF_MAX_PAYLOAD_LEN];

    rt_memset(s_sector_pending, 0, sizeof(s_sector_pending));
    rt_memset(s_sector_used, 0, sizeof(s_sector_used));
    rt_memset(s_sector_accepted, 0, sizeof(s_sector_accepted));
    rt_memset(s_sector_bad, 0, sizeof(s_sector_bad));
    s_total_pending = 0U;
    s_total_accepted = 0U;
    s_used_slots = 0U;

    for (slot = 0U; slot < SF_TOTAL_SLOTS; slot++)
    {
        sf_record_hdr_t hdr;
        rt_err_t result = sf_read_record(slot, &hdr, payload, sizeof(payload));
        rt_uint32_t sector = slot / SF_SLOTS_PER_SECTOR;
        if (result == -RT_EIO)
        {
            sf_mark_io_fault(RT_TRUE);
            s_sector_bad[sector] = RT_TRUE;
            slot += SF_SLOTS_PER_SECTOR - (slot % SF_SLOTS_PER_SECTOR) - 1U;
            continue;
        }
        if (result != RT_EOK)
        {
            continue;
        }
        s_used_slots++;
        s_sector_used[sector]++;
        if (hdr.seq_id > max_seq)
        {
            max_seq = hdr.seq_id;
            max_slot = slot;
        }
        if (hdr.status == SF_STATUS_PENDING)
        {
            s_total_pending++;
            s_sector_pending[sector]++;
            if (hdr.seq_id < min_pending_seq)
            {
                min_pending_seq = hdr.seq_id;
                min_pending_slot = slot;
            }
        }
        else
        {
            s_total_accepted++;
            s_sector_accepted[sector]++;
        }
    }
    s_next_seq = (max_seq == 0U) ? 1U : max_seq + 1U;
    s_head_slot = (max_seq == 0U) ? 0U : sf_next_slot(max_slot);
    s_tail_slot = (min_pending_seq == 0xFFFFFFFFUL) ? s_head_slot : min_pending_slot;
    s_total_pushed = s_used_slots;
    LOG_I("SFQ3 recovered: pending=%u accepted=%u used=%u next=%u",
          s_total_pending, s_total_accepted, s_used_slots, s_next_seq);
}

static rt_err_t sf_erase_sector_locked(rt_uint32_t sector)
{
    int result;
    if (sector >= SF_TOTAL_SECTORS || s_sector_pending[sector] != 0U ||
        s_sector_bad[sector])
    {
        return -RT_EFULL;
    }
    result = fal_partition_erase(s_part, sector * SF_SECTOR_SIZE, SF_SECTOR_SIZE);
    if (result < 0)
    {
        s_sector_bad[sector] = RT_TRUE;
        sf_mark_io_fault(RT_FALSE);
        return -RT_EIO;
    }
    s_used_slots -= s_sector_used[sector];
    s_total_accepted -= s_sector_accepted[sector];
    s_sector_used[sector] = 0U;
    s_sector_accepted[sector] = 0U;
    return RT_EOK;
}

static rt_err_t sf_prepare_head_locked(void)
{
    rt_uint32_t magic;
    rt_uint32_t current_sector;
    rt_uint32_t count;

    if (sf_read_magic(s_head_slot, &magic) != RT_EOK)
    {
        sf_mark_io_fault(RT_FALSE);
        return -RT_EIO;
    }
    if (magic == 0xFFFFFFFFUL)
    {
        return RT_EOK;
    }

    current_sector = s_head_slot / SF_SLOTS_PER_SECTOR;
    for (count = 0U; count < SF_TOTAL_SECTORS; count++)
    {
        rt_uint32_t sector = (current_sector + count) % SF_TOTAL_SECTORS;
        if (!s_sector_bad[sector] && s_sector_pending[sector] == 0U)
        {
            rt_err_t result = sf_erase_sector_locked(sector);
            if (result == RT_EOK)
            {
                s_head_slot = sector * SF_SLOTS_PER_SECTOR;
                return RT_EOK;
            }
            if (result == -RT_EIO)
            {
                return result;
            }
        }
    }
    s_total_rejected++;
    return -RT_EFULL;
}

static rt_err_t sf_find_next_pending_locked(rt_uint32_t start, rt_uint32_t *out_slot)
{
    rt_uint32_t count;
    rt_uint8_t payload[SF_MAX_PAYLOAD_LEN];
    rt_uint32_t slot = start;

    for (count = 0U; count < SF_TOTAL_SLOTS; count++)
    {
        sf_record_hdr_t hdr;
        rt_err_t result = sf_read_record(slot, &hdr, payload, sizeof(payload));
        if (result == -RT_EIO)
        {
            sf_mark_io_fault(RT_TRUE);
        }
        else if (result == RT_EOK && hdr.status == SF_STATUS_PENDING)
        {
            *out_slot = slot;
            return RT_EOK;
        }
        slot = sf_next_slot(slot);
    }
    return -RT_EEMPTY;
}

rt_err_t store_forward_push_ex(const void *data, rt_size_t len,
                               rt_uint32_t endpoint_id,
                               rt_uint32_t config_revision,
                               rt_uint32_t *out_seq)
{
    rt_uint8_t slot_data[SF_SLOT_SIZE];
    rt_uint8_t verify[sizeof(sf_record_hdr_t) + SF_MAX_PAYLOAD_LEN];
    sf_record_hdr_t *hdr = (sf_record_hdr_t *)slot_data;
    rt_uint32_t address;
    rt_uint32_t sector;
    rt_uint8_t commit = SF_COMMIT_VALID;
    int result;

    if (!s_ready || data == RT_NULL || len == 0U || len > SF_MAX_PAYLOAD_LEN)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    if (sf_prepare_head_locked() != RT_EOK)
    {
        rt_mutex_release(&s_lock);
        return s_storage_fault ? -RT_EIO : -RT_EFULL;
    }

    rt_memset(slot_data, 0xFF, sizeof(slot_data));
    hdr->magic = SF_RECORD_MAGIC;
    hdr->format_version = SF_FORMAT_VERSION;
    hdr->header_size = sizeof(*hdr);
    hdr->seq_id = s_next_seq;
    hdr->timestamp = rt_tick_get();
    hdr->endpoint_id = endpoint_id;
    hdr->config_revision = config_revision;
    hdr->data_len = (rt_uint16_t)len;
    hdr->status = SF_STATUS_PENDING;
    hdr->commit = 0xFFU;
    rt_memcpy(slot_data + sizeof(*hdr), data, len);
    hdr->crc32 = sf_record_crc(hdr, slot_data + sizeof(*hdr));
    address = sf_slot_address(s_head_slot);
    result = fal_partition_write(s_part, address, slot_data, sizeof(*hdr) + len);
    if (result >= 0)
    {
        result = fal_partition_read(s_part, address, verify, sizeof(*hdr) + len);
    }
    if (result < 0 || rt_memcmp(verify, slot_data, sizeof(*hdr) + len) != 0)
    {
        s_sector_bad[s_head_slot / SF_SLOTS_PER_SECTOR] = RT_TRUE;
        sf_mark_io_fault(RT_FALSE);
        rt_mutex_release(&s_lock);
        return -RT_EIO;
    }
    result = fal_partition_write(s_part, address + offsetof(sf_record_hdr_t, commit),
                                 &commit, sizeof(commit));
    if (result < 0 ||
        fal_partition_read(s_part, address + offsetof(sf_record_hdr_t, commit),
                           &verify[0], 1U) < 0 || verify[0] != SF_COMMIT_VALID)
    {
        s_sector_bad[s_head_slot / SF_SLOTS_PER_SECTOR] = RT_TRUE;
        sf_mark_io_fault(RT_FALSE);
        rt_mutex_release(&s_lock);
        return -RT_EIO;
    }

    sector = s_head_slot / SF_SLOTS_PER_SECTOR;
    if (s_total_pending == 0U)
    {
        s_tail_slot = s_head_slot;
    }
    s_sector_pending[sector]++;
    s_sector_used[sector]++;
    s_total_pending++;
    s_used_slots++;
    s_total_pushed++;
    if (out_seq != RT_NULL)
    {
        *out_seq = s_next_seq;
    }
    s_next_seq++;
    s_head_slot = sf_next_slot(s_head_slot);
    rt_mutex_release(&s_lock);
    return RT_EOK;
}

rt_err_t store_forward_push(const void *data, rt_size_t len, rt_uint32_t *out_seq)
{
    param_config_t cfg;
    param_get_snapshot(&cfg);
    return store_forward_push_ex(data, len, param_get_endpoint_identity(),
                                 cfg.revision, out_seq);
}

rt_err_t store_forward_peek(sf_record_hdr_t *hdr, void *data_buf, rt_size_t buf_sz)
{
    rt_err_t result;
    if (!s_ready || hdr == RT_NULL || data_buf == RT_NULL)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    if (s_total_pending == 0U)
    {
        rt_mutex_release(&s_lock);
        return -RT_EEMPTY;
    }
    result = sf_read_record(s_tail_slot, hdr, (rt_uint8_t *)data_buf, buf_sz);
    if (result != RT_EOK || hdr->status != SF_STATUS_PENDING)
    {
        if (result == -RT_EIO)
        {
            sf_mark_io_fault(RT_TRUE);
        }
        if (sf_find_next_pending_locked(sf_next_slot(s_tail_slot), &s_tail_slot) == RT_EOK)
        {
            result = sf_read_record(s_tail_slot, hdr, (rt_uint8_t *)data_buf, buf_sz);
        }
    }
    rt_mutex_release(&s_lock);
    return result;
}

rt_err_t store_forward_ack(rt_uint32_t seq_id)
{
    sf_record_hdr_t hdr;
    rt_uint8_t payload[SF_MAX_PAYLOAD_LEN];
    rt_uint8_t state = SF_STATUS_ACCEPTED;
    rt_uint8_t verify;
    rt_uint32_t address;
    rt_uint32_t sector;
    rt_err_t result;

    if (!s_ready)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    result = sf_read_record(s_tail_slot, &hdr, payload, sizeof(payload));
    if (result != RT_EOK || hdr.status != SF_STATUS_PENDING || hdr.seq_id != seq_id)
    {
        rt_mutex_release(&s_lock);
        return -RT_ERROR;
    }
    address = sf_slot_address(s_tail_slot) + offsetof(sf_record_hdr_t, status);
    if (fal_partition_write(s_part, address, &state, 1U) < 0 ||
        fal_partition_read(s_part, address, &verify, 1U) < 0 || verify != state)
    {
        sf_mark_io_fault(RT_TRUE);
        rt_mutex_release(&s_lock);
        return -RT_EIO;
    }
    sector = s_tail_slot / SF_SLOTS_PER_SECTOR;
    s_sector_pending[sector]--;
    s_sector_accepted[sector]++;
    s_total_pending--;
    s_total_accepted++;
    if (s_total_pending == 0U)
    {
        s_tail_slot = s_head_slot;
    }
    else
    {
        (void)sf_find_next_pending_locked(sf_next_slot(s_tail_slot), &s_tail_slot);
    }
    rt_mutex_release(&s_lock);
    return RT_EOK;
}

rt_err_t store_forward_clear(void)
{
    int result;
    if (!s_ready)
    {
        return -RT_EINVAL;
    }
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    result = fal_partition_erase(s_part, 0U, SF_TOTAL_SECTORS * SF_SECTOR_SIZE);
    if (result < 0)
    {
        sf_mark_io_fault(RT_TRUE);
        rt_mutex_release(&s_lock);
        return -RT_EIO;
    }
    rt_memset(s_sector_pending, 0, sizeof(s_sector_pending));
    rt_memset(s_sector_used, 0, sizeof(s_sector_used));
    rt_memset(s_sector_accepted, 0, sizeof(s_sector_accepted));
    rt_memset(s_sector_bad, 0, sizeof(s_sector_bad));
    s_head_slot = 0U;
    s_tail_slot = 0U;
    s_next_seq = 1U;
    s_total_pushed = 0U;
    s_total_pending = 0U;
    s_total_accepted = 0U;
    s_total_rejected = 0U;
    s_used_slots = 0U;
    s_storage_fault = RT_FALSE;
    s_history_unavailable = RT_FALSE;
    rt_mutex_release(&s_lock);
    return RT_EOK;
}

void store_forward_get_stats(sf_stats_t *stats)
{
    if (stats == RT_NULL)
    {
        return;
    }
    rt_memset(stats, 0, sizeof(*stats));
    if (!s_ready)
    {
        stats->storage_fault = RT_TRUE;
        stats->history_unavailable = RT_TRUE;
        return;
    }
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    stats->total_pushed = s_total_pushed;
    stats->total_pending = s_total_pending;
    stats->total_confirmed = s_total_accepted;
    stats->total_rejected = s_total_rejected;
    stats->next_seq_id = s_next_seq;
    stats->used_slots = s_used_slots;
    {
        rt_uint32_t sector;
        rt_uint32_t bad_slots = 0U;
        for (sector = 0U; sector < SF_TOTAL_SECTORS; sector++)
        {
            if (s_sector_bad[sector]) bad_slots += SF_SLOTS_PER_SECTOR;
        }
        stats->usable_slots = (bad_slots + s_total_pending >= SF_TOTAL_SLOTS) ?
                              0U : SF_TOTAL_SLOTS - bad_slots - s_total_pending;
    }
    stats->head_sector = (rt_uint16_t)(s_head_slot / SF_SLOTS_PER_SECTOR);
    stats->head_slot = (rt_uint16_t)(s_head_slot % SF_SLOTS_PER_SECTOR);
    stats->tail_sector = (rt_uint16_t)(s_tail_slot / SF_SLOTS_PER_SECTOR);
    stats->storage_fault = s_storage_fault;
    stats->history_unavailable = s_history_unavailable;
    rt_mutex_release(&s_lock);
}

rt_bool_t store_forward_is_ready(void)
{
    return s_ready;
}

int store_forward_init(void)
{
    if (s_ready)
    {
        return RT_EOK;
    }
    if (app_flash_init() != RT_EOK)
    {
        s_storage_fault = RT_TRUE;
        s_history_unavailable = RT_TRUE;
        return -RT_ERROR;
    }
    s_part = fal_partition_find(SF_PARTITION_NAME);
    if (s_part == RT_NULL || s_part->len < SF_TOTAL_SECTORS * SF_SECTOR_SIZE)
    {
        s_storage_fault = RT_TRUE;
        s_history_unavailable = RT_TRUE;
        LOG_E("SFQ3 partition unavailable or too small");
        return -RT_ERROR;
    }
    rt_mutex_init(&s_lock, "sfq3", RT_IPC_FLAG_PRIO);
    s_ready = RT_TRUE;
    sf_scan();
    LOG_I("SFQ3 ready: %u slots, REJECT_NEW policy", SF_TOTAL_SLOTS);
    return RT_EOK;
}
INIT_ENV_EXPORT(store_forward_init);

static void sf_status(int argc, char **argv)
{
    sf_stats_t stats;
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    store_forward_get_stats(&stats);
    rt_kprintf("sfq3 pending=%u accepted=%u used=%u usable=%u rejected=%u\n",
               stats.total_pending, stats.total_confirmed, stats.used_slots,
               stats.usable_slots, stats.total_rejected);
    rt_kprintf("sfq3 head=%u/%u tail_sec=%u next=%u fault=%u history_unknown=%u\n",
               stats.head_sector, stats.head_slot, stats.tail_sector,
               stats.next_seq_id, stats.storage_fault, stats.history_unavailable);
}
MSH_CMD_EXPORT(sf_status, show V3 Flash store-and-forward queue);

static void sf_push(int argc, char **argv)
{
    rt_uint32_t seq;
    rt_err_t result;
    if (argc != 2)
    {
        rt_kprintf("Usage: sf_push <text>\n");
        return;
    }
    result = store_forward_push(argv[1], rt_strlen(argv[1]), &seq);
    rt_kprintf("sf_push result=%d seq=%u\n", result,
               (result == RT_EOK) ? seq : 0U);
}
MSH_CMD_EXPORT(sf_push, append one test record to SFQ3);

static void sf_peek(int argc, char **argv)
{
    sf_record_hdr_t hdr;
    rt_uint8_t data[SF_MAX_PAYLOAD_LEN];
    rt_err_t result;
    rt_uint16_t i;
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    result = store_forward_peek(&hdr, data, sizeof(data));
    rt_kprintf("sf_peek result=%d", result);
    if (result == RT_EOK)
    {
        rt_kprintf(" seq=%u len=%u endpoint=%08X data=", hdr.seq_id,
                   hdr.data_len, hdr.endpoint_id);
        for (i = 0U; i < hdr.data_len; i++) rt_kprintf("%02X", data[i]);
    }
    rt_kprintf("\n");
}
MSH_CMD_EXPORT(sf_peek, show oldest valid pending SFQ3 record);

static void sf_pop(int argc, char **argv)
{
    sf_record_hdr_t hdr;
    rt_uint8_t data[SF_MAX_PAYLOAD_LEN];
    rt_err_t result;
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    result = store_forward_peek(&hdr, data, sizeof(data));
    if (result == RT_EOK) result = store_forward_ack(hdr.seq_id);
    rt_kprintf("sf_pop result=%d\n", result);
}
MSH_CMD_EXPORT(sf_pop, mark oldest SFQ3 record modem-accepted);

static void sf_clear(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    rt_kprintf("sf_clear result=%d\n", store_forward_clear());
}
MSH_CMD_EXPORT(sf_clear, erase the V3 business queue from maintenance console);
