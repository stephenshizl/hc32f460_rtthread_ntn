/* SPDX-License-Identifier: Apache-2.0 */

#include "raw_uplink.h"
#include "param_manager.h"
#include "ntn_ms150.h"
#include "store_forward.h"
#include "task_wdt.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.raw"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define RAW_BLOCK_MAX_LEN       256U
#define RAW_RAM_BLOCK_COUNT     32U
#define RAW_RAM_HIGH_WATER      6144U
#define RAW_OLDEST_SPILL_MS     2000U
#define RAW_SEND_GAP_MS         1000U
#define RAW_THREAD_STACK_SZ     5120U
#define RAW_THREAD_PRIO         14U

typedef struct
{
    rt_uint16_t len;
    rt_uint16_t reserved;
    rt_uint32_t sequence;
    rt_uint32_t endpoint_id;
    rt_uint32_t config_revision;
    rt_tick_t queued_tick;
    rt_uint8_t data[RAW_BLOCK_MAX_LEN];
} raw_block_t;

static raw_block_t s_blocks[RAW_RAM_BLOCK_COUNT];
static rt_uint8_t s_build[RAW_BLOCK_MAX_LEN];
static rt_uint16_t s_build_len;
static rt_uint32_t s_build_endpoint_id;
static rt_uint32_t s_build_config_revision;
static rt_tick_t s_build_first_tick;
static rt_tick_t s_build_last_tick;
static rt_uint16_t s_head;
static rt_uint16_t s_tail;
static rt_uint16_t s_count;
static rt_uint32_t s_queued_bytes;
static rt_uint32_t s_next_sequence = 1U;
static rt_uint8_t s_retry_count;
static rt_tick_t s_next_send_tick;
static rt_tick_t s_next_flash_attempt_tick;
static rt_bool_t s_flash_handoff;
static struct rt_mutex s_lock;
static struct rt_semaphore s_sem;
static rt_thread_t s_thread;
static raw_uplink_stats_t s_stats;
static rt_bool_t s_initialized;

static rt_err_t raw_commit_builder_locked(void)
{
    raw_block_t *block;
    if (s_build_len == 0U)
    {
        return RT_EOK;
    }
    if (s_count >= RAW_RAM_BLOCK_COUNT)
    {
        return -RT_EFULL;
    }
    block = &s_blocks[s_tail];
    block->len = s_build_len;
    block->sequence = s_next_sequence++;
    block->endpoint_id = s_build_endpoint_id;
    block->config_revision = s_build_config_revision;
    block->queued_tick = rt_tick_get();
    rt_memcpy(block->data, s_build, s_build_len);
    s_tail = (rt_uint16_t)((s_tail + 1U) % RAW_RAM_BLOCK_COUNT);
    s_count++;
    s_queued_bytes += s_build_len;
    s_build_len = 0U;
    s_build_endpoint_id = 0U;
    s_build_config_revision = 0U;
    s_build_first_tick = 0U;
    s_build_last_tick = 0U;
    return RT_EOK;
}

static void raw_pop_head_locked(rt_uint32_t sequence)
{
    if (s_count > 0U && s_blocks[s_head].sequence == sequence)
    {
        s_queued_bytes -= s_blocks[s_head].len;
        s_head = (rt_uint16_t)((s_head + 1U) % RAW_RAM_BLOCK_COUNT);
        s_count--;
    }
}

static rt_uint32_t raw_retry_delay_ms(rt_uint8_t retry_count)
{
    if (retry_count <= 1U) return 5000U;
    if (retry_count == 2U) return 15000U;
    if (retry_count == 3U) return 60000U;
    return 300000U;
}

rt_err_t raw_uplink_input(const rt_uint8_t *data, rt_size_t len)
{
    param_config_t cfg;
    rt_size_t offset = 0U;
    rt_size_t accepted = 0U;

    if (!s_initialized || data == RT_NULL || len == 0U)
    {
        return -RT_EINVAL;
    }
    if (param_get_work_mode() != PARAM_MODE_RAW_UPLINK)
    {
        return -RT_EINVAL;
    }
    param_get_snapshot(&cfg);
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    while (offset < len)
    {
        rt_size_t room;
        rt_size_t copy_len;
        if (s_build_len >= cfg.raw_block_len && raw_commit_builder_locked() != RT_EOK)
        {
            break;
        }
        room = cfg.raw_block_len - s_build_len;
        copy_len = len - offset;
        if (copy_len > room) copy_len = room;
        if (s_build_len == 0U)
        {
            s_build_first_tick = rt_tick_get();
            s_build_endpoint_id = param_get_endpoint_identity();
            s_build_config_revision = cfg.revision;
        }
        rt_memcpy(&s_build[s_build_len], &data[offset], copy_len);
        s_build_len += (rt_uint16_t)copy_len;
        s_build_last_tick = rt_tick_get();
        offset += copy_len;
        accepted += copy_len;
        if (s_build_len >= cfg.raw_block_len)
        {
            (void)raw_commit_builder_locked();
        }
    }
    s_stats.input_bytes += accepted;
    if (accepted < len)
    {
        s_stats.rejected_bytes += (rt_uint32_t)(len - accepted);
        s_stats.rejected_blocks++;
    }
    rt_mutex_release(&s_lock);
    rt_sem_release(&s_sem);
    return (accepted == len) ? RT_EOK : -RT_EFULL;
}

rt_err_t raw_uplink_enqueue_generated(const rt_uint8_t *data, rt_size_t len)
{
    param_config_t cfg;
    raw_block_t *block;

    if (!s_initialized || data == RT_NULL || len == 0U || len > RAW_BLOCK_MAX_LEN ||
        param_get_work_mode() != PARAM_MODE_RAW_UPLINK)
    {
        return -RT_EINVAL;
    }
    param_get_snapshot(&cfg);
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    /* Preserve byte order: any earlier partial RAW block owns the next queue position. */
    if (s_build_len > 0U && raw_commit_builder_locked() != RT_EOK)
    {
        s_stats.generated_rejected_blocks++;
        rt_mutex_release(&s_lock);
        return -RT_EFULL;
    }
    if (s_count >= RAW_RAM_BLOCK_COUNT)
    {
        s_stats.generated_rejected_blocks++;
        rt_mutex_release(&s_lock);
        return -RT_EFULL;
    }
    block = &s_blocks[s_tail];
    block->len = (rt_uint16_t)len;
    block->sequence = s_next_sequence++;
    block->endpoint_id = param_get_endpoint_identity();
    block->config_revision = cfg.revision;
    block->queued_tick = rt_tick_get();
    rt_memcpy(block->data, data, len);
    s_tail = (rt_uint16_t)((s_tail + 1U) % RAW_RAM_BLOCK_COUNT);
    s_count++;
    s_queued_bytes += len;
    s_stats.generated_bytes += len;
    rt_mutex_release(&s_lock);
    rt_sem_release(&s_sem);
    return RT_EOK;
}

static rt_bool_t raw_copy_ram_head(raw_block_t *out_block)
{
    rt_bool_t available = RT_FALSE;
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    if (s_count > 0U)
    {
        *out_block = s_blocks[s_head];
        available = RT_TRUE;
    }
    rt_mutex_release(&s_lock);
    return available;
}

static void raw_note_send_failure(rt_err_t result)
{
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    s_stats.send_failures++;
    if (result == -RT_ETIMEOUT) s_stats.unknown_results++;
    s_retry_count++;
    s_next_send_tick = rt_tick_get() +
        rt_tick_from_millisecond(raw_retry_delay_ms(s_retry_count));
    rt_mutex_release(&s_lock);
}

static rt_bool_t raw_send_flash_head(rt_uint32_t current_endpoint)
{
    sf_record_hdr_t hdr;
    rt_uint8_t payload[SF_MAX_PAYLOAD_LEN];
    rt_err_t result;

    result = store_forward_peek(&hdr, payload, sizeof(payload));
    if (result != RT_EOK)
    {
        return RT_FALSE;
    }
    if (hdr.endpoint_id != 0U && hdr.endpoint_id != current_endpoint)
    {
        rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
        s_stats.held_endpoint_blocks = 1U;
        rt_mutex_release(&s_lock);
        return RT_FALSE;
    }
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    s_stats.held_endpoint_blocks = 0U;
    rt_mutex_release(&s_lock);
    if (!ntn_service_is_send_ready() ||
        (rt_int32_t)(rt_tick_get() - s_next_send_tick) < 0)
    {
        return RT_FALSE;
    }
    result = ntn_service_send_raw(payload, hdr.data_len);
    if (result == RT_EOK)
    {
        result = store_forward_ack(hdr.seq_id);
        if (result == RT_EOK)
        {
            rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
            s_stats.modem_accepted_bytes += hdr.data_len;
            s_retry_count = 0U;
            s_next_send_tick = rt_tick_get() + rt_tick_from_millisecond(RAW_SEND_GAP_MS);
            rt_mutex_release(&s_lock);
            return RT_TRUE;
        }
    }
    raw_note_send_failure(result);
    return RT_FALSE;
}

static rt_bool_t raw_spill_ram_head(const raw_block_t *block)
{
    rt_uint32_t current_endpoint = param_get_endpoint_identity();
    rt_uint32_t endpoint = (block->endpoint_id == 0U) ? current_endpoint : block->endpoint_id;
    rt_err_t result = store_forward_push_ex(block->data, block->len,
                                            endpoint,
                                            block->config_revision, RT_NULL);
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    if (s_count > 0U && s_blocks[s_head].sequence == block->sequence)
    {
        if (result == RT_EOK)
        {
            s_stats.flash_persisted_bytes += block->len;
            raw_pop_head_locked(block->sequence);
            s_flash_handoff = RT_TRUE;
        }
        else
        {
            s_stats.flash_write_failures++;
            s_next_flash_attempt_tick = rt_tick_get() + rt_tick_from_millisecond(5000U);
        }
    }
    rt_mutex_release(&s_lock);
    return (result == RT_EOK) ? RT_TRUE : RT_FALSE;
}

static void raw_send_ram_head(const raw_block_t *block)
{
    rt_err_t result;
    if (!ntn_service_is_send_ready() ||
        (rt_int32_t)(rt_tick_get() - s_next_send_tick) < 0)
    {
        return;
    }
    result = ntn_service_send_raw(block->data, block->len);
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    if (s_count > 0U && s_blocks[s_head].sequence == block->sequence)
    {
        if (result == RT_EOK)
        {
            s_stats.modem_accepted_bytes += block->len;
            raw_pop_head_locked(block->sequence);
            s_retry_count = 0U;
            s_next_send_tick = rt_tick_get() + rt_tick_from_millisecond(RAW_SEND_GAP_MS);
        }
        else
        {
            s_stats.send_failures++;
            if (result == -RT_ETIMEOUT) s_stats.unknown_results++;
            s_retry_count++;
            s_next_send_tick = rt_tick_get() +
                rt_tick_from_millisecond(raw_retry_delay_ms(s_retry_count));
        }
    }
    rt_mutex_release(&s_lock);
}

static void raw_finish_builder_if_due(const param_config_t *cfg)
{
    rt_tick_t now = rt_tick_get();
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    if (s_build_len > 0U &&
        ((now - s_build_last_tick) >= rt_tick_from_millisecond(cfg->raw_gap_ms) ||
         (now - s_build_first_tick) >= rt_tick_from_millisecond(cfg->raw_wait_ms)))
    {
        (void)raw_commit_builder_locked();
    }
    rt_mutex_release(&s_lock);
}

static void raw_uplink_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);
    while (1)
    {
        param_config_t cfg;
        sf_stats_t flash;
        raw_block_t block;
        rt_uint32_t current_endpoint;
        rt_bool_t have_ram;
        rt_bool_t spill;

        (void)rt_sem_take(&s_sem, rt_tick_from_millisecond(50U));
        task_wdt_heartbeat_by_name("raw_tx");
        param_get_snapshot(&cfg);
        raw_finish_builder_if_due(&cfg);
        if (cfg.work_mode != PARAM_MODE_RAW_UPLINK)
        {
            continue;
        }
        current_endpoint = param_get_endpoint_identity();
        store_forward_get_stats(&flash);
        if (!flash.history_unavailable && flash.total_pending > 0U)
        {
            s_flash_handoff = RT_TRUE;
            (void)raw_send_flash_head(current_endpoint);
            store_forward_get_stats(&flash);
        }

        have_ram = raw_copy_ram_head(&block);
        if (!have_ram)
        {
            if (flash.history_unavailable || flash.total_pending == 0U) s_flash_handoff = RT_FALSE;
            continue;
        }

        if (flash.history_unavailable)
        {
            /* Keep unreadable history intact and send new, correctly routed data from RAM. */
            if (block.endpoint_id == 0U || block.endpoint_id == current_endpoint)
                raw_send_ram_head(&block);
            continue;
        }

        spill = s_flash_handoff || flash.total_pending > 0U ||
                !ntn_service_is_send_ready() ||
                (block.endpoint_id != 0U && block.endpoint_id != current_endpoint) ||
                s_queued_bytes >= RAW_RAM_HIGH_WATER ||
                (rt_tick_get() - block.queued_tick) >=
                    rt_tick_from_millisecond(RAW_OLDEST_SPILL_MS);
        if (spill && store_forward_is_ready() &&
            (rt_int32_t)(rt_tick_get() - s_next_flash_attempt_tick) >= 0)
        {
            if (!raw_spill_ram_head(&block) && flash.total_pending == 0U &&
                (block.endpoint_id == 0U || block.endpoint_id == current_endpoint))
            {
                raw_send_ram_head(&block);
            }
        }
        else if ((!spill || flash.storage_fault) &&
                 flash.total_pending == 0U &&
                 (block.endpoint_id == 0U || block.endpoint_id == current_endpoint))
        {
            /* Flash fault/full degrades to online RAM transfer when link is usable. */
            raw_send_ram_head(&block);
        }
    }
}

void raw_uplink_get_stats(raw_uplink_stats_t *out_stats)
{
    if (out_stats == RT_NULL)
    {
        return;
    }
    rt_memset(out_stats, 0, sizeof(*out_stats));
    if (!s_initialized) return;
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    *out_stats = s_stats;
    out_stats->queued_blocks = s_count;
    out_stats->building_bytes = s_build_len;
    out_stats->queued_bytes = s_queued_bytes + s_build_len;
    rt_mutex_release(&s_lock);
}

rt_err_t raw_uplink_clear(void)
{
    rt_err_t result;
    if (!s_initialized) return -RT_ERROR;
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    s_build_len = 0U;
    s_build_endpoint_id = 0U;
    s_build_config_revision = 0U;
    s_build_first_tick = 0U;
    s_build_last_tick = 0U;
    s_head = 0U;
    s_tail = 0U;
    s_count = 0U;
    s_queued_bytes = 0U;
    s_retry_count = 0U;
    s_flash_handoff = RT_FALSE;
    rt_memset(s_blocks, 0, sizeof(s_blocks));
    rt_mutex_release(&s_lock);
    result = store_forward_clear();
    return result;
}

int raw_uplink_init(void)
{
    sf_stats_t flash;
    if (s_initialized) return RT_EOK;
    rt_memset(s_blocks, 0, sizeof(s_blocks));
    rt_memset(&s_stats, 0, sizeof(s_stats));
    rt_mutex_init(&s_lock, "mraw_q", RT_IPC_FLAG_PRIO);
    rt_sem_init(&s_sem, "sraw_q", 0, RT_IPC_FLAG_PRIO);
    store_forward_get_stats(&flash);
    s_flash_handoff = (flash.total_pending > 0U) ? RT_TRUE : RT_FALSE;
    s_initialized = RT_TRUE;
    task_wdt_register("raw_tx", 10U);
    s_thread = rt_thread_create("raw_tx", raw_uplink_thread_entry, RT_NULL,
                                RAW_THREAD_STACK_SZ, RAW_THREAD_PRIO, 10U);
    if (s_thread == RT_NULL)
    {
        s_initialized = RT_FALSE;
        return -RT_ERROR;
    }
    rt_thread_startup(s_thread);
    LOG_I("RAW queue ready: RAM 8 KiB, Flash spill and old-first replay enabled");
    return RT_EOK;
}
INIT_ENV_EXPORT(raw_uplink_init);

static void raw_status(int argc, char **argv)
{
    raw_uplink_stats_t stats;
    ntn_service_status_t ntn;
    sf_stats_t flash;
    RT_UNUSED(argc);
    RT_UNUSED(argv);
    raw_uplink_get_stats(&stats);
    ntn_service_get_status(&ntn);
    store_forward_get_stats(&flash);
    rt_kprintf("raw input=%u accepted=%u rejected=%u ram=%u/%u build=%u\n",
               stats.input_bytes, stats.modem_accepted_bytes, stats.rejected_bytes,
               stats.queued_bytes, stats.queued_blocks, stats.building_bytes);
    rt_kprintf("raw flash_pending=%u persisted=%u write_fail=%u held=%u fault=%u\n",
               flash.total_pending, stats.flash_persisted_bytes,
               stats.flash_write_failures, stats.held_endpoint_blocks,
               flash.storage_fault);
    rt_kprintf("raw send_fail=%u unknown=%u ntn_state=%u ready=%u last_err=%d\n",
               stats.send_failures, stats.unknown_results, ntn.state,
               ntn.send_ready, ntn.last_error);
}
MSH_CMD_EXPORT(raw_status, show V3 RAW RAM and Flash transport status);
