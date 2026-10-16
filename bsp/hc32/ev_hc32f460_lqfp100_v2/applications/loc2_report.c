/* SPDX-License-Identifier: Apache-2.0 */

#include "loc2_report.h"
#include "gnss_um220.h"
#include "param_manager.h"
#include "raw_uplink.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "app.loc2"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define LOC2_FLAG_POSITION    0x0001U
#define LOC2_FLAG_UTC         0x0002U
#define LOC2_FLAG_ALTITUDE    0x0004U
#define LOC2_FLAG_SPEED       0x0008U
#define LOC2_FLAG_COURSE      0x0010U
#define LOC2_ID_RESERVE_COUNT 64U
#define LOC2_THREAD_STACK_SZ  2048U
#define LOC2_THREAD_PRIO      20U

static struct rt_mutex s_lock;
static rt_thread_t s_thread;
static rt_bool_t s_initialized;
static rt_uint64_t s_next_id;
static rt_uint64_t s_id_limit;
static rt_tick_t s_last_asset_tick;
static rt_tick_t s_last_location_tick;
static loc2_stats_t s_stats;

static void loc2_put_u16(rt_uint8_t *p, rt_uint16_t value)
{
    p[0] = (rt_uint8_t)(value >> 8);
    p[1] = (rt_uint8_t)value;
}

static void loc2_put_u32(rt_uint8_t *p, rt_uint32_t value)
{
    p[0] = (rt_uint8_t)(value >> 24);
    p[1] = (rt_uint8_t)(value >> 16);
    p[2] = (rt_uint8_t)(value >> 8);
    p[3] = (rt_uint8_t)value;
}

static void loc2_put_u64(rt_uint8_t *p, rt_uint64_t value)
{
    rt_uint8_t i;
    for (i = 0U; i < 8U; i++)
    {
        p[i] = (rt_uint8_t)(value >> (56U - i * 8U));
    }
}

static rt_uint32_t loc2_crc32(const rt_uint8_t *data, rt_size_t len)
{
    rt_uint32_t crc = 0xFFFFFFFFUL;
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
    return ~crc;
}

static rt_bool_t loc2_is_leap(rt_uint16_t year)
{
    return ((year % 4U == 0U && year % 100U != 0U) || year % 400U == 0U) ?
           RT_TRUE : RT_FALSE;
}

static rt_bool_t loc2_utc_seconds(const gnss_data_t *gnss, rt_uint64_t *out_seconds)
{
    static const rt_uint8_t month_days[12] =
        {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
    rt_uint64_t days = 0U;
    rt_uint16_t year;
    rt_uint8_t month;
    rt_uint8_t max_day;

    if (!gnss->time_valid || gnss->year < 1970U || gnss->month < 1U ||
        gnss->month > 12U || gnss->second > 59U)
    {
        return RT_FALSE;
    }
    max_day = month_days[gnss->month - 1U];
    if (gnss->month == 2U && loc2_is_leap(gnss->year)) max_day = 29U;
    if (gnss->day < 1U || gnss->day > max_day) return RT_FALSE;
    for (year = 1970U; year < gnss->year; year++)
    {
        days += loc2_is_leap(year) ? 366U : 365U;
    }
    for (month = 1U; month < gnss->month; month++)
    {
        days += month_days[month - 1U];
        if (month == 2U && loc2_is_leap(gnss->year)) days++;
    }
    days += gnss->day - 1U;
    *out_seconds = days * 86400U + (rt_uint64_t)gnss->hour * 3600U +
                   (rt_uint64_t)gnss->minute * 60U + gnss->second;
    return RT_TRUE;
}

static rt_err_t loc2_take_id(rt_uint64_t *out_id)
{
    rt_uint64_t first;
    rt_err_t result = RT_EOK;
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    if (s_next_id >= s_id_limit)
    {
        result = param_reserve_report_ids(LOC2_ID_RESERVE_COUNT, &first);
        if (result == RT_EOK)
        {
            s_next_id = first;
            s_id_limit = first + LOC2_ID_RESERVE_COUNT;
        }
        else
        {
            s_stats.id_reserve_failures++;
        }
    }
    if (result == RT_EOK)
    {
        *out_id = s_next_id++;
    }
    rt_mutex_release(&s_lock);
    return result;
}

static rt_err_t loc2_build(rt_uint8_t type, rt_uint64_t record_id,
                           rt_uint8_t *packet, rt_size_t *out_len)
{
    param_config_t cfg;
    gnss_data_t gnss;
    rt_bool_t fixed;
    rt_bool_t fresh;
    rt_bool_t include_position;
    rt_uint16_t flags = 0U;
    rt_uint64_t utc_seconds = 0U;
    rt_size_t sn_len;
    rt_size_t body_len;

    param_get_snapshot(&cfg);
    if (cfg.dev_sn[0] == '\0') return -RT_ENOSYS;
    if (type != LOC2_TYPE_ASSET && type != LOC2_TYPE_LOCATION) return -RT_EINVAL;
    fixed = gnss_get_data(&gnss);
    fresh = (fixed && gnss_is_fresh(5000U)) ? RT_TRUE : RT_FALSE;
    if (type == LOC2_TYPE_LOCATION && !fresh) return -RT_EEMPTY;
    include_position = (type == LOC2_TYPE_LOCATION ||
                        (type == LOC2_TYPE_ASSET && cfg.asset_location_enabled)) && fresh;
    if (include_position)
    {
        flags |= LOC2_FLAG_POSITION;
        if (gnss.altitude_valid) flags |= LOC2_FLAG_ALTITUDE;
        if (gnss.speed_valid) flags |= LOC2_FLAG_SPEED;
        if (gnss.course_valid) flags |= LOC2_FLAG_COURSE;
    }
    if (loc2_utc_seconds(&gnss, &utc_seconds)) flags |= LOC2_FLAG_UTC;
    sn_len = rt_strlen(cfg.dev_sn);
    body_len = 44U + sn_len;
    rt_memset(packet, 0, body_len + 4U);
    packet[0] = 0x4EU;
    packet[1] = 0x4CU;
    packet[2] = 0x02U;
    packet[3] = type;
    loc2_put_u16(&packet[4], flags);
    packet[6] = (rt_uint8_t)sn_len;
    loc2_put_u64(&packet[8], record_id);
    if ((flags & LOC2_FLAG_UTC) != 0U) loc2_put_u64(&packet[16], utc_seconds);
    if (include_position)
    {
        loc2_put_u32(&packet[24], (rt_uint32_t)gnss.latitude_e7);
        loc2_put_u32(&packet[28], (rt_uint32_t)gnss.longitude_e7);
        if (gnss.altitude_valid) loc2_put_u32(&packet[32], (rt_uint32_t)gnss.altitude_mm);
        if (gnss.speed_valid) loc2_put_u32(&packet[36], gnss.speed_cm_s);
        if (gnss.course_valid) loc2_put_u16(&packet[40], gnss.course_cdeg);
        packet[42] = gnss.sats_used;
        packet[43] = gnss.fix_quality;
    }
    rt_memcpy(&packet[44], cfg.dev_sn, sn_len);
    loc2_put_u32(&packet[body_len], loc2_crc32(packet, body_len));
    *out_len = body_len + 4U;
    return RT_EOK;
}

rt_err_t loc2_report_now(rt_uint8_t type)
{
    rt_uint8_t packet[LOC2_MAX_PACKET_LEN];
    rt_size_t length;
    rt_uint64_t record_id;
    rt_err_t result;

    if (!s_initialized || param_get_work_mode() != PARAM_MODE_RAW_UPLINK)
    {
        return -RT_EINVAL;
    }
    result = loc2_take_id(&record_id);
    if (result != RT_EOK) return -RT_EIO;
    result = loc2_build(type, record_id, packet, &length);
    if (result != RT_EOK)
    {
        if (result == -RT_EEMPTY) s_stats.no_fix_skipped++;
        return result;
    }
    result = raw_uplink_enqueue_generated(packet, length);
    if (result == RT_EOK)
    {
        if (type == LOC2_TYPE_ASSET) s_stats.asset_enqueued++;
        else s_stats.location_enqueued++;
    }
    else
    {
        s_stats.queue_rejected++;
    }
    return result;
}

void loc2_report_get_stats(loc2_stats_t *out_stats)
{
    if (out_stats == RT_NULL) return;
    rt_mutex_take(&s_lock, RT_WAITING_FOREVER);
    *out_stats = s_stats;
    rt_mutex_release(&s_lock);
}

static void loc2_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);
    while (1)
    {
        param_config_t cfg;
        rt_tick_t now;
        rt_thread_mdelay(1000U);
        param_get_snapshot(&cfg);
        if (cfg.work_mode != PARAM_MODE_RAW_UPLINK) continue;
        now = rt_tick_get();
        if (cfg.asset_report_enabled &&
            (now - s_last_asset_tick) >= rt_tick_from_millisecond(cfg.report_interval_s * 1000U))
        {
            if (loc2_report_now(LOC2_TYPE_ASSET) == RT_EOK) s_last_asset_tick = now;
        }
        if (cfg.location_report_enabled &&
            (now - s_last_location_tick) >= rt_tick_from_millisecond(cfg.location_interval_s * 1000U))
        {
            if (loc2_report_now(LOC2_TYPE_LOCATION) == RT_EOK) s_last_location_tick = now;
        }
    }
}

int loc2_report_init(void)
{
    if (s_initialized) return RT_EOK;
    rt_mutex_init(&s_lock, "mloc2", RT_IPC_FLAG_PRIO);
    rt_memset(&s_stats, 0, sizeof(s_stats));
    s_initialized = RT_TRUE;
    s_thread = rt_thread_create("loc2", loc2_thread_entry, RT_NULL,
                                LOC2_THREAD_STACK_SZ, LOC2_THREAD_PRIO, 20U);
    if (s_thread == RT_NULL)
    {
        s_initialized = RT_FALSE;
        return -RT_ERROR;
    }
    rt_thread_startup(s_thread);
    return RT_EOK;
}
INIT_APP_EXPORT(loc2_report_init);
