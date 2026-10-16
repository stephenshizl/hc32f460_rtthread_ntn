/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-14     Stephen      Stage 5.3: End-to-End Automated Telemetry Engine & NTAP Packaging
 */

#ifndef __APP_TELEMETRY_H__
#define __APP_TELEMETRY_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NTAP (NTN Application Protocol) Message Types */
#define NTAP_MSG_TYPE_TELEMETRY     0x01    /* Periodic GNSS/Sensor Telemetry */
#define NTAP_MSG_TYPE_ALARM         0x02    /* Emergency / Geofence / Tamper Alarm */
#define NTAP_MSG_TYPE_RS232_RAW     0x03    /* Transparent RS232 User Payload */
#define NTAP_MSG_TYPE_MODBUS_DATA   0x04    /* Industrial Modbus Polled Records */

/* NTAP Status Flags */
#define NTAP_FLAG_GNSS_VALID        (1 << 0)    /* GNSS has valid fix (FixType > 0) */
#define NTAP_FLAG_EXT_POWER         (1 << 1)    /* Running on external main DC power */
#define NTAP_FLAG_ALARM_ACTIVE      (1 << 2)    /* Alarm condition present */

#pragma pack(1)
/**
 * @brief Compact 28-Byte Satellite Telemetry Packet
 */
typedef struct
{
    rt_uint8_t  msg_type;       /* NTAP_MSG_TYPE_TELEMETRY (0x01) */
    rt_uint8_t  flags;          /* NTAP_FLAG_* bitmask */
    rt_uint16_t seq_id;         /* Telemetry frame sequence counter */
    rt_uint32_t timestamp;      /* System tick or UTC epoch */
    rt_int32_t  lat_e7;         /* Latitude * 1e7 (10cm precision) */
    rt_int32_t  lon_e7;         /* Longitude * 1e7 (10cm precision) */
    rt_int16_t  alt_m;          /* Altitude in meters (-1000 .. +9000) */
    rt_uint16_t speed_kmh_x10;  /* Speed in km/h * 10 (0.1 km/h resolution) */
    rt_uint16_t course_deg;     /* Heading / Course (0 .. 359 deg) */
    rt_uint8_t  num_sats;       /* Satellites tracked */
    rt_uint8_t  battery_pct;    /* Battery percent (0..100) or 0xFF if external DC */
    rt_int8_t   rssi;           /* Satellite RF signal RSSI */
    rt_uint8_t  work_mode;      /* Current terminal mode */
    rt_uint16_t payload_crc16;  /* CRC16 over preceding bytes */
} ntap_telemetry_pkt_t;

/**
 * @brief Transparent RS232 Payload Envelope Header (10 bytes)
 */
typedef struct
{
    rt_uint8_t  msg_type;       /* NTAP_MSG_TYPE_RS232_RAW (0x03) */
    rt_uint8_t  reserved;       /* 0x00 */
    rt_uint16_t seq_id;         /* Envelope sequence counter */
    rt_uint32_t timestamp;      /* Timestamp */
    rt_uint16_t raw_len;        /* Length of following raw RS232 payload */
} ntap_rs232_hdr_t;
#pragma pack()

/**
 * @brief Diagnostic telemetry statistics
 */
typedef struct
{
    rt_uint32_t telem_frames_gen;   /* Total periodic telemetry frames generated */
    rt_uint32_t rs232_frames_fwd;   /* Total RS232 frames packaged & pushed to Store&Forward */
    rt_uint32_t alarms_triggered;   /* Total alarm events generated */
    rt_tick_t   last_sample_tick;   /* Tick of last telemetry generation */
    rt_bool_t   engine_running;     /* Telemetry engine running status */
} telem_stats_t;

/**
 * @brief Initialize Telemetry Engine and start periodic background task
 * @return RT_EOK on success, negative error code on failure
 */
int app_telemetry_init(void);

/**
 * @brief Manually trigger a telemetry sample & push to Store & Forward queue immediately
 * @return RT_EOK on success
 */
rt_err_t app_telemetry_trigger_now(void);

/**
 * @brief Set terminal operational mode (0: NTN Bridge, 1: Auto Telemetry, 2: Modbus, 3: CAN)
 * @param mode Target operational mode
 */
void app_telemetry_set_mode(rt_uint8_t mode);

/**
 * @brief Get diagnostic statistics
 * @param out_stats Pointer to output statistics structure
 */
void app_telemetry_get_stats(telem_stats_t *out_stats);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TELEMETRY_H__ */
