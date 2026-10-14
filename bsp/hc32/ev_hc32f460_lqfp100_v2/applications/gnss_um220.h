/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: GNSS (Beidou/GPS) UM220-IV-M driver
 * 2026-10-14     Stephen      Enhanced with TankerFlow NMEA engine: fixed-point E7,
 *                             zero-copy field parsing, freshness check, cm/s speed
 */

#ifndef __GNSS_UM220_H__
#define __GNSS_UM220_H__

#include <rtthread.h>
#include <rtdevice.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_UART_NAME          "uart2"         /* HC32F460 USART2: PA3(RX), PA4(TX) */
#define GNSS_DEFAULT_BAUDRATE   BAUD_RATE_115200/* UM220 standard baudrate */
#define GNSS_NMEA_LINE_MAX_SZ   128             /* Maximum NMEA sentence length */
#define GNSS_E7_SCALE           (10000000UL)    /* 10^7 scaling for high-precision fixed-point */

/**
 * @brief GNSS parsed position & time data structure
 * Features both integer fixed-point E7 (ideal for compact NTN packet binary packing)
 * and floating-point representations (convenient for UI and logging).
 */
typedef struct
{
    /* Positioning status */
    rt_bool_t   is_fixed;          /* RT_TRUE if positioning fix is valid */
    rt_uint8_t  fix_quality;       /* 0=Invalid, 1=SPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float */
    rt_uint8_t  sats_used;         /* Number of satellites in use */

    /* Fixed-point coordinates (10^7 scale, signed, +North/East, -South/West) */
    rt_int32_t  latitude_e7;       /* Latitude * 10^7 (e.g. 31.2057600 -> 312057600) */
    rt_int32_t  longitude_e7;      /* Longitude * 10^7 (e.g. 121.5076120 -> 1215076120) */

    /* Floating-point coordinates */
    double      latitude;          /* Latitude in signed decimal degrees */
    char        lat_dir;           /* 'N' or 'S' */
    double      longitude;         /* Longitude in signed decimal degrees */
    char        lon_dir;           /* 'E' or 'W' */

    /* Navigation metrics */
    float       altitude;          /* Altitude in meters above sea level */
    float       hdop;              /* Horizontal Dilution of Precision */
    rt_uint32_t speed_cm_s;        /* Ground speed in cm/s (fixed-point integer) */
    float       speed_kmh;         /* Ground speed in km/h */
    float       course;            /* Heading / Course over ground in degrees (0.0~360.0) */

    /* UTC Date & Time components */
    rt_uint8_t  hour;              /* UTC Hour: 0-23 */
    rt_uint8_t  minute;            /* UTC Minute: 0-59 */
    rt_uint8_t  second;            /* UTC Second: 0-60 */
    rt_uint16_t year;              /* UTC Year: e.g. 2026 */
    rt_uint8_t  month;             /* UTC Month: 1-12 */
    rt_uint8_t  day;               /* UTC Day: 1-31 */

    rt_uint32_t utc_date;          /* Legacy composite Date: DDMMYY (e.g. 141026) */
    rt_uint32_t utc_time;          /* Legacy composite Time: HHMMSS (e.g. 101500) */

    rt_tick_t   last_update_tick;  /* System tick of last valid update */
} gnss_data_t;

/**
 * @brief Diagnostic statistics structure
 */
typedef struct
{
    rt_uint32_t bytes;             /* Total raw bytes received */
    rt_uint32_t sentences;         /* Total sentences processed */
    rt_uint32_t rmc_ok;            /* Successfully parsed RMC sentences */
    rt_uint32_t gga_ok;            /* Successfully parsed GGA sentences */
    rt_uint32_t checksum_error;    /* XOR Checksum mismatch count */
    rt_uint32_t format_error;      /* Syntax / field validation errors */
    rt_uint32_t overflow_error;    /* Buffer overflow / frame too long */
    rt_uint32_t unsupported;       /* Other NMEA sentences (e.g. GSV, GSA) */
} gnss_stats_t;

/**
 * @brief GNSS sentence receive callback function prototype
 */
typedef void (*gnss_sentence_handler_t)(const char *sentence);

/**
 * @brief Initialize GNSS module interface on uart2
 * @return RT_EOK on success, error code on failure
 */
int gnss_init(void);

/**
 * @brief Get latest parsed GNSS positioning data
 * @param out_data Pointer to output structure
 * @return RT_TRUE if fix is valid, RT_FALSE otherwise
 */
rt_bool_t gnss_get_data(gnss_data_t *out_data);

/**
 * @brief Check if current positioning data is valid and fresh
 * @param timeout_ms Maximum allowed age in milliseconds (e.g. 10000ms)
 * @return RT_TRUE if positioning is fixed and updated within timeout_ms
 */
rt_bool_t gnss_is_fresh(rt_uint32_t timeout_ms);

/**
 * @brief Get copy of GNSS diagnostic statistics
 * @param out_stats Pointer to statistics structure
 */
void gnss_get_stats(gnss_stats_t *out_stats);

/**
 * @brief Send configuration/command string to GNSS module
 * @param cmd Null-terminated command string
 * @return Bytes sent
 */
rt_size_t gnss_send_cmd(const char *cmd);

/**
 * @brief Set GNSS baudrate dynamically
 */
rt_err_t gnss_set_baudrate(rt_uint32_t baud_rate);

/**
 * @brief Enable or disable raw NMEA sentence logging to RTT console
 */
void gnss_set_raw_log(rt_bool_t enable);

/**
 * @brief Register raw sentence callback
 */
void gnss_set_sentence_handler(gnss_sentence_handler_t handler);

#ifdef __cplusplus
}
#endif

#endif /* __GNSS_UM220_H__ */
