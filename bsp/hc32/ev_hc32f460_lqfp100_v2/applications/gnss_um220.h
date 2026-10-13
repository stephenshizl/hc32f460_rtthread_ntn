/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: GNSS (Beidou/GPS) UM220-IV-M driver
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

/**
 * @brief GNSS parsed position & time data structure
 */
typedef struct
{
    rt_bool_t   is_fixed;          /* RT_TRUE if positioning fix is valid */
    rt_uint8_t  fix_quality;       /* 0=Invalid, 1=SPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float */
    rt_uint8_t  sats_used;         /* Number of satellites in use */

    double      latitude;          /* Latitude in signed decimal degrees (+North, -South) */
    char        lat_dir;           /* 'N' or 'S' */
    double      longitude;         /* Longitude in signed decimal degrees (+East, -West) */
    char        lon_dir;           /* 'E' or 'W' */

    float       altitude;          /* Altitude in meters above sea level */
    float       hdop;              /* Horizontal Dilution of Precision */
    float       speed_kmh;         /* Ground speed in km/h */
    float       course;            /* Heading / Course over ground in degrees */

    rt_uint32_t utc_date;          /* Date: DDMMYY (e.g. 131026 for 2026-10-13) */
    rt_uint32_t utc_time;          /* Time: HHMMSS (e.g. 204500 for 20:45:00 UTC) */

    rt_tick_t   last_update_tick;  /* System tick of last valid update */
} gnss_data_t;

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
