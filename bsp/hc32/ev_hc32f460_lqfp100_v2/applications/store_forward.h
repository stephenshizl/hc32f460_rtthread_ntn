/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 4: Store & Forward offline persistent queue manager
 */

#ifndef __STORE_FORWARD_H__
#define __STORE_FORWARD_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SF_PARTITION_NAME       "store_forward" /* FAL Partition Name (768 KB) */
#define SF_SECTOR_SIZE          4096            /* NOR Flash Sector Size (4 KB) */
#define SF_TOTAL_SECTORS        192             /* 768 KB / 4 KB = 192 Sectors */
#define SF_RECORD_MAGIC         0xA55A          /* Record Magic Word */
#define SF_STATUS_PENDING       0xFF            /* Unacknowledged / pending transmit */
#define SF_STATUS_CONFIRMED     0x00            /* Acknowledged / successfully sent (Zero-Erase) */
#define SF_MAX_PAYLOAD_LEN      256             /* Maximum satellite payload length in bytes */

#pragma pack(1)
/**
 * @brief Store & Forward On-Flash Record Header (16 bytes)
 */
typedef struct
{
    rt_uint16_t magic;          /* Magic: 0xA55A */
    rt_uint32_t seq_id;         /* Monotonically increasing sequence ID */
    rt_uint32_t timestamp;      /* Timestamp (system tick or epoch) */
    rt_uint16_t data_len;       /* Payload length in bytes */
    rt_uint8_t  status;         /* 0xFF=Pending, 0x00=Confirmed (Zero-Erase bit modification) */
    rt_uint8_t  reserved;       /* Reserved (0xFF) */
    rt_uint16_t crc16;          /* CRC-16 over (magic, seq_id, timestamp, data_len) + payload */
} sf_record_hdr_t;
#pragma pack()

/**
 * @brief Queue Diagnostic Statistics
 */
typedef struct
{
    rt_uint32_t total_pushed;       /* Total records pushed into queue */
    rt_uint32_t total_pending;      /* Current unconfirmed records in flash */
    rt_uint32_t total_confirmed;    /* Current confirmed records in flash */
    rt_uint32_t total_dropped;      /* Records dropped due to partition wrap-around */
    rt_uint32_t next_seq_id;        /* Next sequence ID to be assigned */
    rt_uint16_t head_sector;        /* Current writing sector (0..191) */
    rt_uint16_t head_offset;        /* Current write offset within head sector */
    rt_uint16_t tail_sector;        /* Oldest sector containing pending records */
    rt_bool_t   auto_flush_enabled; /* Auto-flush daemon active flag */
} sf_stats_t;

/**
 * @brief Initialize Store & Forward subsystem, mount FAL partition, recover queue state
 * @return RT_EOK on success, error code on failure
 */
int store_forward_init(void);

/**
 * @brief Push a message into the persistent offline Flash queue
 * @param data Pointer to payload data
 * @param len Payload length (1 .. SF_MAX_PAYLOAD_LEN)
 * @param out_seq Optional output pointer to receive assigned sequence ID (can be RT_NULL)
 * @return RT_EOK on success, negative error code on failure
 */
rt_err_t store_forward_push(const void *data, rt_size_t len, rt_uint32_t *out_seq);

/**
 * @brief Peek the oldest unconfirmed record without acknowledging it
 * @param hdr Output pointer to receive record header
 * @param data_buf Output buffer to receive payload
 * @param buf_sz Size of output buffer
 * @return RT_EOK if pending record retrieved, -RT_ENOSYS if partition uninitialized, -RT_EEMPTY if queue empty
 */
rt_err_t store_forward_peek(sf_record_hdr_t *hdr, void *data_buf, rt_size_t buf_sz);

/**
 * @brief Acknowledge and mark a record as sent via zero-erase bit transition
 * @param seq_id Sequence ID of the record to acknowledge
 * @return RT_EOK on success, negative error code on failure
 */
rt_err_t store_forward_ack(rt_uint32_t seq_id);

/**
 * @brief Erase the entire Store & Forward partition and reset queue pointers
 * @return RT_EOK on success
 */
rt_err_t store_forward_clear(void);

/**
 * @brief Get queue statistics
 * @param stats Pointer to output statistics structure
 */
void store_forward_get_stats(sf_stats_t *stats);

/**
 * @brief Trigger the background flush task immediately
 */
void store_forward_trigger_flush(void);

/**
 * @brief Enable or disable automatic satellite recovery flush
 * @param enable RT_TRUE to enable, RT_FALSE to disable
 */
void store_forward_set_auto_flush(rt_bool_t enable);

#ifdef __cplusplus
}
#endif

#endif /* __STORE_FORWARD_H__ */
