/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __STORE_FORWARD_H__
#define __STORE_FORWARD_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SF_PARTITION_NAME       "store_forward"
#define SF_SECTOR_SIZE          4096U
#define SF_TOTAL_SECTORS        192U
#define SF_RECORD_MAGIC         0x53465133UL /* "SFQ3" */
#define SF_FORMAT_VERSION       3U
#define SF_MAX_PAYLOAD_LEN      256U
#define SF_SLOT_SIZE            288U
#define SF_SLOTS_PER_SECTOR     (SF_SECTOR_SIZE / SF_SLOT_SIZE)
#define SF_TOTAL_SLOTS          (SF_TOTAL_SECTORS * SF_SLOTS_PER_SECTOR)
#define SF_STATUS_PENDING       0xFFU
#define SF_STATUS_ACCEPTED      0x7FU
#define SF_COMMIT_VALID         0xA5U

#pragma pack(1)
typedef struct
{
    rt_uint32_t magic;
    rt_uint16_t format_version;
    rt_uint16_t header_size;
    rt_uint32_t seq_id;
    rt_uint32_t timestamp;
    rt_uint32_t endpoint_id;
    rt_uint32_t config_revision;
    rt_uint16_t data_len;
    rt_uint8_t  status;
    rt_uint8_t  commit;
    rt_uint32_t crc32;
} sf_record_hdr_t;
#pragma pack()

typedef struct
{
    rt_uint32_t total_pushed;
    rt_uint32_t total_pending;
    rt_uint32_t total_confirmed;
    rt_uint32_t total_rejected;
    rt_uint32_t next_seq_id;
    rt_uint32_t used_slots;
    rt_uint32_t usable_slots;
    rt_uint16_t head_sector;
    rt_uint16_t head_slot;
    rt_uint16_t tail_sector;
    rt_bool_t storage_fault;
    rt_bool_t history_unavailable;
} sf_stats_t;

int store_forward_init(void);
rt_bool_t store_forward_is_ready(void);
rt_err_t store_forward_push(const void *data, rt_size_t len, rt_uint32_t *out_seq);
rt_err_t store_forward_push_ex(const void *data, rt_size_t len,
                               rt_uint32_t endpoint_id,
                               rt_uint32_t config_revision,
                               rt_uint32_t *out_seq);
rt_err_t store_forward_peek(sf_record_hdr_t *hdr, void *data_buf, rt_size_t buf_sz);
rt_err_t store_forward_ack(rt_uint32_t seq_id);
rt_err_t store_forward_clear(void);
void store_forward_get_stats(sf_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* __STORE_FORWARD_H__ */
