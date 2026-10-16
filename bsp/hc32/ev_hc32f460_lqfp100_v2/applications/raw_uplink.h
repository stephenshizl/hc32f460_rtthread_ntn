/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __RAW_UPLINK_H__
#define __RAW_UPLINK_H__

#include <rtthread.h>

typedef struct
{
    rt_uint32_t input_bytes;
    rt_uint32_t modem_accepted_bytes;
    rt_uint32_t rejected_bytes;
    rt_uint32_t rejected_blocks;
    rt_uint32_t send_failures;
    rt_uint32_t unknown_results;
    rt_uint32_t flash_persisted_bytes;
    rt_uint32_t flash_write_failures;
    rt_uint32_t held_endpoint_blocks;
    rt_uint32_t generated_bytes;
    rt_uint32_t generated_rejected_blocks;
    rt_uint16_t queued_blocks;
    rt_uint16_t building_bytes;
    rt_uint32_t queued_bytes;
} raw_uplink_stats_t;

int raw_uplink_init(void);
rt_err_t raw_uplink_input(const rt_uint8_t *data, rt_size_t len);
rt_err_t raw_uplink_enqueue_generated(const rt_uint8_t *data, rt_size_t len);
void raw_uplink_get_stats(raw_uplink_stats_t *out_stats);
rt_err_t raw_uplink_clear(void);

#endif /* __RAW_UPLINK_H__ */
