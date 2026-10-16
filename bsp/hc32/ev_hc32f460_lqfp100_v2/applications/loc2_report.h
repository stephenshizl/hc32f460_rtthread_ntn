/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __LOC2_REPORT_H__
#define __LOC2_REPORT_H__

#include <rtthread.h>

#define LOC2_TYPE_ASSET       1U
#define LOC2_TYPE_LOCATION    2U
#define LOC2_MAX_PACKET_LEN   80U

typedef struct
{
    rt_uint32_t asset_enqueued;
    rt_uint32_t location_enqueued;
    rt_uint32_t no_fix_skipped;
    rt_uint32_t queue_rejected;
    rt_uint32_t id_reserve_failures;
} loc2_stats_t;

int loc2_report_init(void);
rt_err_t loc2_report_now(rt_uint8_t type);
void loc2_report_get_stats(loc2_stats_t *out_stats);

#endif /* __LOC2_REPORT_H__ */
