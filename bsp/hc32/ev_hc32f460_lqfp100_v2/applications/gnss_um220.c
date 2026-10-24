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

#include "gnss_um220.h"
#include "hw_power.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "task_wdt.h"

#define DBG_TAG "gnss"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define GNSS_THREAD_STACK_SZ    2048
#define GNSS_THREAD_PRIO        12
#define GNSS_CFG_THREAD_STACK_SZ 1536
#define GNSS_CFG_THREAD_PRIO     13
#define GNSS_CFG_RESP_MAX_SZ     128
#define GNSS_CFG_CMD_MAX_SZ      64
#define GNSS_CFG_TIMEOUT_MS      1500U
#define GNSS_CFG_RETRY_COUNT     2U

#define NMEA_MAX_FIELDS         20

/* Zero-copy slice referencing field within sentence buffer */
typedef struct
{
    const char  *ptr;
    rt_uint8_t   length;
} nmea_field_t;

/* Parsing result codes */
typedef enum
{
    NMEA_RES_OK_RMC = 0,
    NMEA_RES_OK_GGA,
    NMEA_RES_UNSUPPORTED,
    NMEA_RES_CHECKSUM_ERROR,
    NMEA_RES_FORMAT_ERROR
} nmea_res_t;

static rt_device_t              _gnss_dev          = RT_NULL;
static struct rt_semaphore      _sem_rx;
static struct rt_semaphore      _sem_cmd;
static struct rt_mutex          _mutex_data;
static struct rt_mutex          _mutex_cmd;
static rt_thread_t              _rx_thread         = RT_NULL;
static rt_thread_t              _cfg_thread        = RT_NULL;

static gnss_data_t              _gnss_data;
static gnss_stats_t             _gnss_stats;
static gnss_sentence_handler_t  _sentence_handler  = RT_NULL;
static rt_bool_t                _raw_log           = RT_FALSE;
static rt_bool_t                _inited            = RT_FALSE;
static volatile rt_bool_t       _cmd_waiting       = RT_FALSE;
static volatile rt_err_t        _cmd_status        = -RT_ETIMEOUT;
static char                    *_cmd_resp_buf       = RT_NULL;
static rt_size_t                _cmd_resp_size      = 0U;
static rt_size_t                _cmd_resp_index     = 0U;
static volatile rt_bool_t       _configured        = RT_FALSE;

/* UM220-IV-M protocol bodies. Checksums are generated when commands are sent. */
static const char * const _gnss_cfg_bodies[] =
{
    "CFGSYS,H11",       /* GPS + BDS + Galileo + SBAS + QZSS */
    "CFGNMEA,H51",      /* NMEA 4.1 */
    "CFGMSG,0,0,1",     /* GGA, 1 Hz */
    "CFGMSG,0,2,1",     /* GSA, 1 Hz */
    "CFGMSG,0,3,1",     /* GSV, 1 Hz */
    "CFGMSG,0,4,1"      /* RMC, 1 Hz */
};

/* ------------------- High-Robustness NMEA Parsing Engine ------------------- */

static rt_uint8_t nmea_hex_val(char ch, rt_uint8_t *val)
{
    if ((ch >= '0') && (ch <= '9'))
    {
        *val = (rt_uint8_t)(ch - '0');
        return 1U;
    }
    if ((ch >= 'A') && (ch <= 'F'))
    {
        *val = (rt_uint8_t)(ch - 'A' + 10);
        return 1U;
    }
    if ((ch >= 'a') && (ch <= 'f'))
    {
        *val = (rt_uint8_t)(ch - 'a' + 10);
        return 1U;
    }
    return 0U;
}

static rt_uint8_t nmea_parse_uint(const nmea_field_t *field, rt_uint32_t *val)
{
    rt_uint32_t res = 0U;

    if (!field || !val || field->length == 0U)
    {
        return 0U;
    }

    for (rt_uint8_t i = 0U; i < field->length; i++)
    {
        char ch = field->ptr[i];
        if ((ch < '0') || (ch > '9'))
        {
            return 0U;
        }
        rt_uint32_t digit = (rt_uint32_t)(ch - '0');
        if ((res > 429496729U) || ((res == 429496729U) && (digit > 5U)))
        {
            return 0U; /* Overflow prevention */
        }
        res = (res * 10U) + digit;
    }

    *val = res;
    return 1U;
}

static rt_uint8_t nmea_parse_time(const nmea_field_t *field,
                                  rt_uint8_t *hour,
                                  rt_uint8_t *minute,
                                  rt_uint8_t *second)
{
    if (!field || !hour || !minute || !second || (field->length < 6U))
    {
        return 0U;
    }

    for (rt_uint8_t i = 0U; i < 6U; i++)
    {
        if ((field->ptr[i] < '0') || (field->ptr[i] > '9'))
        {
            return 0U;
        }
    }
    if ((field->length > 6U) && (field->ptr[6] != '.'))
    {
        return 0U;
    }

    rt_uint8_t hh = (rt_uint8_t)(((field->ptr[0] - '0') * 10) + (field->ptr[1] - '0'));
    rt_uint8_t mm = (rt_uint8_t)(((field->ptr[2] - '0') * 10) + (field->ptr[3] - '0'));
    rt_uint8_t ss = (rt_uint8_t)(((field->ptr[4] - '0') * 10) + (field->ptr[5] - '0'));

    /* Second up to 60 for leap seconds */
    if ((hh > 23U) || (mm > 59U) || (ss > 60U))
    {
        return 0U;
    }

    *hour   = hh;
    *minute = mm;
    *second = ss;
    return 1U;
}

static rt_uint8_t nmea_parse_date(const nmea_field_t *field,
                                  rt_uint16_t *year,
                                  rt_uint8_t *month,
                                  rt_uint8_t *day)
{
    if (!field || !year || !month || !day || (field->length != 6U))
    {
        return 0U;
    }

    for (rt_uint8_t i = 0U; i < 6U; i++)
    {
        if ((field->ptr[i] < '0') || (field->ptr[i] > '9'))
        {
            return 0U;
        }
    }

    rt_uint8_t dd = (rt_uint8_t)(((field->ptr[0] - '0') * 10) + (field->ptr[1] - '0'));
    rt_uint8_t mm = (rt_uint8_t)(((field->ptr[2] - '0') * 10) + (field->ptr[3] - '0'));
    rt_uint8_t yy = (rt_uint8_t)(((field->ptr[4] - '0') * 10) + (field->ptr[5] - '0'));

    if ((dd == 0U) || (dd > 31U) || (mm == 0U) || (mm > 12U))
    {
        return 0U;
    }

    *day   = dd;
    *month = mm;
    *year  = (yy >= 80U) ? (rt_uint16_t)(1900U + yy) : (rt_uint16_t)(2000U + yy);
    return 1U;
}

/**
 * @brief Parse coordinate into fixed-point 10^7 scale without float calculation
 * @param field Pointer to field slice (e.g. "3112.3456" or "12130.4567")
 * @param hemisphere 'N', 'S', 'E', or 'W'
 * @param degree_digits 2 for latitude, 3 for longitude
 * @param value_e7 Output integer scaled by 10^7
 */
static rt_uint8_t nmea_parse_coordinate(const nmea_field_t *field,
                                        char hemisphere,
                                        rt_uint8_t degree_digits,
                                        rt_int32_t *value_e7)
{
    rt_uint8_t dot_index;
    rt_uint8_t fraction_digits;
    rt_uint32_t degrees;
    rt_uint32_t minutes;
    rt_uint32_t minute_fraction_e7;
    rt_uint32_t coordinate_e7;
    rt_uint32_t scale;

    if (!field || !value_e7 || (field->length < (rt_uint8_t)(degree_digits + 2U)))
    {
        return 0U;
    }

    dot_index = field->length;
    for (rt_uint8_t i = 0U; i < field->length; i++)
    {
        char ch = field->ptr[i];
        if (ch == '.')
        {
            if (dot_index != field->length)
            {
                return 0U; /* Duplicate dots */
            }
            dot_index = i;
        }
        else if ((ch < '0') || (ch > '9'))
        {
            return 0U;
        }
    }

    if (dot_index != (rt_uint8_t)(degree_digits + 2U))
    {
        return 0U;
    }

    degrees = 0U;
    for (rt_uint8_t i = 0U; i < degree_digits; i++)
    {
        degrees = (degrees * 10U) + (rt_uint32_t)(field->ptr[i] - '0');
    }

    minutes = (rt_uint32_t)((field->ptr[degree_digits] - '0') * 10) +
              (rt_uint32_t)(field->ptr[degree_digits + 1U] - '0');
    if (minutes >= 60U)
    {
        return 0U;
    }
    if (((degree_digits == 2U) && (degrees > 90U)) ||
        ((degree_digits == 3U) && (degrees > 180U)))
    {
        return 0U;
    }

    minute_fraction_e7 = 0U;
    fraction_digits = 0U;
    if (dot_index < field->length)
    {
        for (rt_uint8_t i = (rt_uint8_t)(dot_index + 1U); i < field->length; i++)
        {
            if (fraction_digits < 7U)
            {
                minute_fraction_e7 = (minute_fraction_e7 * 10U) + (rt_uint32_t)(field->ptr[i] - '0');
                fraction_digits++;
            }
        }
    }

    scale = 1U;
    while (fraction_digits < 7U)
    {
        scale *= 10U;
        fraction_digits++;
    }
    minute_fraction_e7 *= scale;

    if (((degree_digits == 2U) && (degrees == 90U) && ((minutes != 0U) || (minute_fraction_e7 != 0U))) ||
        ((degree_digits == 3U) && (degrees == 180U) && ((minutes != 0U) || (minute_fraction_e7 != 0U))))
    {
        return 0U;
    }

    /* Fixed-point arithmetic: 1 minute = 1/60 degree */
    coordinate_e7 = (degrees * GNSS_E7_SCALE) +
                    (((minutes * GNSS_E7_SCALE) + minute_fraction_e7) / 60U);

    if ((hemisphere == 'S') || (hemisphere == 'W'))
    {
        *value_e7 = -(rt_int32_t)coordinate_e7;
    }
    else if ((hemisphere == 'N') || (hemisphere == 'E'))
    {
        *value_e7 = (rt_int32_t)coordinate_e7;
    }
    else
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief Parse Knots to cm/s using pure integer math (1 Knot = 463/9000 m/s = 463/90 cm/s)
 */
static rt_uint8_t nmea_parse_knots_to_cm_s(const nmea_field_t *field, rt_uint32_t *speed_cm_s)
{
    rt_uint8_t seen_dot = 0U;
    rt_uint8_t fraction_digits = 0U;
    rt_uint32_t integer_part = 0U;
    rt_uint32_t fraction_part = 0U;
    rt_uint32_t knots_milli;

    if (!field || !speed_cm_s || (field->length == 0U))
    {
        return 0U;
    }

    for (rt_uint8_t i = 0U; i < field->length; i++)
    {
        char ch = field->ptr[i];
        if (ch == '.')
        {
            if (seen_dot != 0U) return 0U;
            seen_dot = 1U;
            continue;
        }
        if ((ch < '0') || (ch > '9')) return 0U;

        if (seen_dot == 0U)
        {
            rt_uint32_t digit = (rt_uint32_t)(ch - '0');
            if ((integer_part > 999U) || ((integer_part == 999U) && (digit > 9U))) return 0U;
            integer_part = (integer_part * 10U) + digit;
        }
        else if (fraction_digits < 3U)
        {
            fraction_part = (fraction_part * 10U) + (rt_uint32_t)(ch - '0');
            fraction_digits++;
        }
    }

    while (fraction_digits < 3U)
    {
        fraction_part *= 10U;
        fraction_digits++;
    }

    if (integer_part > 9999U) return 0U;

    knots_milli = (integer_part * 1000U) + fraction_part;
    /* (knots_milli / 1000) * 1.852 * 100000 / 3600 = knots_milli * 463 / 9000 */
    *speed_cm_s = ((knots_milli / 9000U) * 463U) +
                  ((((knots_milli % 9000U) * 463U) + 4500U) / 9000U);
    return 1U;
}

/**
 * @brief Safe float parser for altitude, course, and HDOP
 */
static rt_uint8_t nmea_parse_float(const nmea_field_t *field, float *val)
{
    char tmp[16];
    if (!field || !val || field->length == 0U || field->length >= sizeof(tmp))
    {
        return 0U;
    }
    rt_memcpy(tmp, field->ptr, field->length);
    tmp[field->length] = '\0';
    *val = (float)atof(tmp);
    return 1U;
}

static rt_uint8_t nmea_sentence_matches(const nmea_field_t *field, char a, char b, char c)
{
    if (!field || (field->length != 5U))
    {
        return 0U;
    }
    return ((field->ptr[2] == a) && (field->ptr[3] == b) && (field->ptr[4] == c)) ? 1U : 0U;
}

/**
 * @brief Zero-copy field splitter handling empty fields ",," correctly
 */
static rt_uint8_t nmea_split_fields(const char *sentence,
                                    rt_uint16_t body_start,
                                    rt_uint16_t body_end,
                                    nmea_field_t *fields,
                                    rt_uint8_t *field_count)
{
    rt_uint16_t start = body_start;
    rt_uint8_t count = 0U;

    for (rt_uint16_t i = body_start; i <= body_end; i++)
    {
        if ((i == body_end) || (sentence[i] == ','))
        {
            if (count >= NMEA_MAX_FIELDS) return 0U;
            fields[count].ptr = &sentence[start];
            fields[count].length = (rt_uint8_t)(i - start);
            count++;
            start = (rt_uint16_t)(i + 1U);
        }
    }

    *field_count = count;
    return (count > 0U) ? 1U : 0U;
}

/**
 * @brief Parse RMC sentence into decoded components
 */
static nmea_res_t nmea_parse_rmc(const nmea_field_t *fields, rt_uint8_t field_count)
{
    rt_uint8_t hour, minute, second;
    rt_uint16_t year;
    rt_uint8_t month, day;
    rt_int32_t lat_e7, lon_e7;
    rt_uint32_t speed_cm_s = 0U;
    float course = 0.0f;
    rt_bool_t speed_valid = RT_FALSE;
    rt_bool_t course_valid = RT_FALSE;
    rt_bool_t valid;

    if (field_count < 10U)
    {
        return NMEA_RES_FORMAT_ERROR;
    }

    if (nmea_parse_time(&fields[1], &hour, &minute, &second) == 0U)
    {
        return NMEA_RES_FORMAT_ERROR;
    }

    if ((fields[2].length != 1U) || ((fields[2].ptr[0] != 'A') && (fields[2].ptr[0] != 'V')))
    {
        return NMEA_RES_FORMAT_ERROR;
    }
    valid = (fields[2].ptr[0] == 'A') ? RT_TRUE : RT_FALSE;

    if (valid)
    {
        if ((fields[4].length != 1U) || (fields[6].length != 1U) ||
            (nmea_parse_coordinate(&fields[3], fields[4].ptr[0], 2U, &lat_e7) == 0U) ||
            (nmea_parse_coordinate(&fields[5], fields[6].ptr[0], 3U, &lon_e7) == 0U))
        {
            return NMEA_RES_FORMAT_ERROR;
        }
    }

    if (fields[7].length > 0U)
    {
        speed_valid = nmea_parse_knots_to_cm_s(&fields[7], &speed_cm_s) ? RT_TRUE : RT_FALSE;
    }
    if (fields[8].length > 0U)
    {
        course_valid = nmea_parse_float(&fields[8], &course) ? RT_TRUE : RT_FALSE;
        if (course < 0.0f || course >= 360.0f) course_valid = RT_FALSE;
    }

    if (nmea_parse_date(&fields[9], &year, &month, &day) == 0U)
    {
        return NMEA_RES_FORMAT_ERROR;
    }

    /* Update shared data under mutex */
    rt_mutex_take(&_mutex_data, RT_WAITING_FOREVER);
    _gnss_data.hour   = hour;
    _gnss_data.minute = minute;
    _gnss_data.second = second;
    _gnss_data.utc_time = ((rt_uint32_t)hour * 10000) + ((rt_uint32_t)minute * 100) + second;

    _gnss_data.year   = year;
    _gnss_data.month  = month;
    _gnss_data.day    = day;
    _gnss_data.utc_date = ((rt_uint32_t)day * 10000) + ((rt_uint32_t)month * 100) + (year % 100);
    _gnss_data.time_valid = RT_TRUE;
    _gnss_data.time_update_tick = rt_tick_get();

    _gnss_data.is_fixed   = valid;
    _gnss_data.speed_cm_s = speed_cm_s;
    _gnss_data.speed_valid = speed_valid;
    _gnss_data.speed_kmh  = (float)speed_cm_s * 0.036f;
    _gnss_data.course     = course;
    _gnss_data.course_valid = course_valid;
    if (course_valid)
    {
        rt_uint32_t course_cdeg = (rt_uint32_t)(course * 100.0f + 0.5f);
        _gnss_data.course_cdeg = (rt_uint16_t)((course_cdeg > 35999U) ? 35999U : course_cdeg);
    }
    else
    {
        _gnss_data.course_cdeg = 0U;
    }

    if (valid)
    {
        _gnss_data.latitude_e7  = lat_e7;
        _gnss_data.longitude_e7 = lon_e7;
        _gnss_data.lat_dir      = fields[4].ptr[0];
        _gnss_data.lon_dir      = fields[6].ptr[0];
        _gnss_data.latitude     = (double)lat_e7 / 10000000.0;
        _gnss_data.longitude    = (double)lon_e7 / 10000000.0;
        _gnss_data.last_update_tick = rt_tick_get();
    }
    rt_mutex_release(&_mutex_data);

    return NMEA_RES_OK_RMC;
}

/**
 * @brief Parse GGA sentence into decoded components
 */
static nmea_res_t nmea_parse_gga(const nmea_field_t *fields, rt_uint8_t field_count)
{
    rt_uint8_t hour, minute, second;
    rt_uint32_t fix_quality;
    rt_uint32_t satellites;
    rt_int32_t lat_e7, lon_e7;
    float hdop = 0.0f;
    float alt = 0.0f;
    rt_bool_t altitude_valid = RT_FALSE;

    if (field_count < 10U)
    {
        return NMEA_RES_FORMAT_ERROR;
    }

    if (nmea_parse_time(&fields[1], &hour, &minute, &second) == 0U)
    {
        return NMEA_RES_FORMAT_ERROR;
    }

    if ((nmea_parse_uint(&fields[6], &fix_quality) == 0U) || (fix_quality > 255U))
    {
        return NMEA_RES_FORMAT_ERROR;
    }

    if ((nmea_parse_uint(&fields[7], &satellites) == 0U) || (satellites > 255U))
    {
        return NMEA_RES_FORMAT_ERROR;
    }

    if (fields[8].length > 0U)
    {
        nmea_parse_float(&fields[8], &hdop);
    }
    if (fields[9].length > 0U)
    {
        altitude_valid = nmea_parse_float(&fields[9], &alt) ? RT_TRUE : RT_FALSE;
    }

    if (fix_quality > 0U)
    {
        if ((fields[3].length != 1U) || (fields[5].length != 1U) ||
            (nmea_parse_coordinate(&fields[2], fields[3].ptr[0], 2U, &lat_e7) == 0U) ||
            (nmea_parse_coordinate(&fields[4], fields[5].ptr[0], 3U, &lon_e7) == 0U))
        {
            return NMEA_RES_FORMAT_ERROR;
        }
    }

    /* Update shared data under mutex */
    rt_mutex_take(&_mutex_data, RT_WAITING_FOREVER);
    _gnss_data.hour        = hour;
    _gnss_data.minute      = minute;
    _gnss_data.second      = second;
    _gnss_data.utc_time    = ((rt_uint32_t)hour * 10000) + ((rt_uint32_t)minute * 100) + second;

    _gnss_data.fix_quality = (rt_uint8_t)fix_quality;
    _gnss_data.sats_used   = (rt_uint8_t)satellites;
    _gnss_data.hdop        = hdop;
    _gnss_data.altitude    = alt;
    _gnss_data.altitude_valid = (fix_quality > 0U && altitude_valid) ? RT_TRUE : RT_FALSE;
    _gnss_data.altitude_mm = _gnss_data.altitude_valid ?
        (rt_int32_t)((alt >= 0.0f) ? (alt * 1000.0f + 0.5f) : (alt * 1000.0f - 0.5f)) : 0;
    _gnss_data.is_fixed    = (fix_quality > 0U) ? RT_TRUE : RT_FALSE;

    if (fix_quality > 0U)
    {
        _gnss_data.latitude_e7  = lat_e7;
        _gnss_data.longitude_e7 = lon_e7;
        _gnss_data.lat_dir      = fields[3].ptr[0];
        _gnss_data.lon_dir      = fields[5].ptr[0];
        _gnss_data.latitude     = (double)lat_e7 / 10000000.0;
        _gnss_data.longitude    = (double)lon_e7 / 10000000.0;
        _gnss_data.last_update_tick = rt_tick_get();
    }
    rt_mutex_release(&_mutex_data);

    return NMEA_RES_OK_GGA;
}

/**
 * @brief Dispatch complete line to NMEA parser with checksum and split verification
 */
static void nmea_process_sentence(const char *sentence, rt_uint16_t length)
{
    nmea_field_t fields[NMEA_MAX_FIELDS];
    rt_uint16_t star_idx = length;
    rt_uint8_t field_cnt;
    rt_uint8_t calc_crc = 0U;
    rt_uint8_t high, low;

    _gnss_stats.sentences++;

    if (!sentence || (length < 7U) || (sentence[0] != '$'))
    {
        _gnss_stats.format_error++;
        return;
    }

    /* Locate '*' checksum separator */
    for (rt_uint16_t i = 1U; i < length; i++)
    {
        if (sentence[i] == '*')
        {
            star_idx = i;
            break;
        }
    }

    if ((star_idx == length) || (star_idx < 2U) || ((rt_uint16_t)(star_idx + 3U) != length))
    {
        _gnss_stats.format_error++;
        return;
    }

    /* XOR Checksum verification */
    for (rt_uint16_t i = 1U; i < star_idx; i++)
    {
        calc_crc ^= (rt_uint8_t)sentence[i];
    }

    if ((nmea_hex_val(sentence[star_idx + 1U], &high) == 0U) ||
        (nmea_hex_val(sentence[star_idx + 2U], &low) == 0U))
    {
        _gnss_stats.format_error++;
        return;
    }

    if (calc_crc != (rt_uint8_t)((high << 4) | low))
    {
        _gnss_stats.checksum_error++;
        return;
    }

    /* Print raw log if enabled */
    if (_raw_log)
    {
        rt_kprintf("[NMEA] %s\r\n", sentence);
    }

    /* Forward to custom callback if registered */
    if (_sentence_handler)
    {
        _sentence_handler(sentence);
    }

    /* Split body fields between '$' and '*' */
    if (nmea_split_fields(sentence, 1U, star_idx, fields, &field_cnt) == 0U)
    {
        _gnss_stats.format_error++;
        return;
    }

    if (nmea_sentence_matches(&fields[0], 'R', 'M', 'C') != 0U)
    {
        nmea_res_t res = nmea_parse_rmc(fields, field_cnt);
        if (res == NMEA_RES_OK_RMC) _gnss_stats.rmc_ok++;
        else _gnss_stats.format_error++;
    }
    else if (nmea_sentence_matches(&fields[0], 'G', 'G', 'A') != 0U)
    {
        nmea_res_t res = nmea_parse_gga(fields, field_cnt);
        if (res == NMEA_RES_OK_GGA) _gnss_stats.gga_ok++;
        else _gnss_stats.format_error++;
    }
    else
    {
        _gnss_stats.unsupported++;
    }
}

/* ------------------- RT-Thread Device Interface & Threads ------------------- */

static rt_bool_t gnss_line_is_navigation_nmea(const char *line, rt_uint16_t length)
{
    if (line == RT_NULL || length < 6U || line[0] != '$')
    {
        return RT_FALSE;
    }

    /* Standard GNSS talker IDs: GP, GN, BD/GB, GA, GL and GQ. */
    if ((line[1] == 'G' &&
         (line[2] == 'P' || line[2] == 'N' || line[2] == 'B' ||
          line[2] == 'A' || line[2] == 'L' || line[2] == 'Q')) ||
        (line[1] == 'B' && line[2] == 'D'))
    {
        return RT_TRUE;
    }
    return RT_FALSE;
}

static void gnss_command_process_line(const char *line, rt_uint16_t length)
{
    rt_size_t copy_len;

    if (!_cmd_waiting || line == RT_NULL || length == 0U)
    {
        return;
    }

    if (_cmd_resp_buf != RT_NULL && _cmd_resp_size > 1U &&
        _cmd_resp_index < (_cmd_resp_size - 1U))
    {
        copy_len = length;
        if (copy_len > (_cmd_resp_size - 1U - _cmd_resp_index))
        {
            copy_len = _cmd_resp_size - 1U - _cmd_resp_index;
        }
        rt_memcpy(&_cmd_resp_buf[_cmd_resp_index], line, copy_len);
        _cmd_resp_index += copy_len;
        if (_cmd_resp_index < (_cmd_resp_size - 1U))
        {
            _cmd_resp_buf[_cmd_resp_index++] = '\n';
        }
        _cmd_resp_buf[_cmd_resp_index] = '\0';
    }

    if (length >= 3U && rt_strncmp(line, "$OK", 3U) == 0)
    {
        _cmd_status = RT_EOK;
        _cmd_waiting = RT_FALSE;
        rt_sem_release(&_sem_cmd);
    }
    else if (length >= 5U && rt_strncmp(line, "$FAIL", 5U) == 0)
    {
        _cmd_status = -RT_ERROR;
        _cmd_waiting = RT_FALSE;
        rt_sem_release(&_sem_cmd);
    }
}

static rt_err_t gnss_exec_body(const char *body, char *response,
                               rt_size_t response_size, rt_uint32_t timeout_ms)
{
    char frame[GNSS_CFG_CMD_MAX_SZ];
    rt_uint8_t checksum = 0U;
    rt_size_t body_len;
    rt_size_t frame_len;
    rt_size_t sent;
    rt_err_t result;

    if (!_inited || _gnss_dev == RT_NULL || body == RT_NULL)
    {
        return -RT_ERROR;
    }
    body_len = rt_strlen(body);
    if (body_len == 0U || body_len > (GNSS_CFG_CMD_MAX_SZ - 8U))
    {
        return -RT_EINVAL;
    }
    if (rt_mutex_take(&_mutex_cmd, rt_tick_from_millisecond(timeout_ms + 500U)) != RT_EOK)
    {
        return -RT_EBUSY;
    }

    for (rt_size_t i = 0U; i < body_len; i++)
    {
        checksum ^= (rt_uint8_t)body[i];
    }
    frame_len = (rt_size_t)rt_snprintf(frame, sizeof(frame),
                                       "$%s*%02X\r\n", body, checksum);
    if (frame_len >= sizeof(frame))
    {
        rt_mutex_release(&_mutex_cmd);
        return -RT_EINVAL;
    }

    while (rt_sem_trytake(&_sem_cmd) == RT_EOK)
    {
        /* Drain completion left by a previous timed-out transaction. */
    }
    _cmd_resp_buf = response;
    _cmd_resp_size = response_size;
    _cmd_resp_index = 0U;
    if (response != RT_NULL && response_size > 0U)
    {
        response[0] = '\0';
    }
    _cmd_status = -RT_ETIMEOUT;
    _cmd_waiting = RT_TRUE;

    sent = rt_device_write(_gnss_dev, 0, frame, frame_len);
    if (sent != frame_len)
    {
        _cmd_waiting = RT_FALSE;
        result = -RT_EIO;
    }
    else if (rt_sem_take(&_sem_cmd, rt_tick_from_millisecond(timeout_ms)) == RT_EOK)
    {
        result = _cmd_status;
    }
    else
    {
        _cmd_waiting = RT_FALSE;
        result = -RT_ETIMEOUT;
    }

    _cmd_resp_buf = RT_NULL;
    _cmd_resp_size = 0U;
    _cmd_resp_index = 0U;
    rt_mutex_release(&_mutex_cmd);
    return result;
}

rt_err_t gnss_configure_all(void)
{
    char response[GNSS_CFG_RESP_MAX_SZ];
    rt_err_t result = -RT_ERROR;

    if (!_inited)
    {
        return -RT_ERROR;
    }
    _configured = RT_FALSE;

    for (rt_size_t i = 0U;
         i < (sizeof(_gnss_cfg_bodies) / sizeof(_gnss_cfg_bodies[0])); i++)
    {
        result = -RT_ERROR;
        for (rt_uint8_t attempt = 0U; attempt < GNSS_CFG_RETRY_COUNT; attempt++)
        {
            result = gnss_exec_body(_gnss_cfg_bodies[i], response,
                                    sizeof(response), GNSS_CFG_TIMEOUT_MS);
            if (result == RT_EOK)
            {
                break;
            }
            rt_thread_mdelay(150U);
        }
        if (result != RT_EOK)
        {
            LOG_E("UM220 config failed: %s (%d)", _gnss_cfg_bodies[i], result);
            return result;
        }
        LOG_I("UM220 config OK: %s", _gnss_cfg_bodies[i]);

        /* Constellation switching may briefly reconfigure the navigation engine. */
        rt_thread_mdelay((i == 0U) ? 300U : 60U);
    }

    _configured = RT_TRUE;

    /* Read back constellation mode for diagnostics. A failed query does not undo
     * the acknowledged set commands above. */
    result = gnss_exec_body("CFGSYS", response, sizeof(response), GNSS_CFG_TIMEOUT_MS);
    if (result == RT_EOK)
    {
        LOG_I("UM220 constellation readback: %s", response);
    }
    else
    {
        LOG_W("UM220 CFGSYS readback unavailable (%d)", result);
    }
    LOG_I("UM220 runtime GNSS configuration complete; no CFGSAVE flash write");
    return RT_EOK;
}

rt_bool_t gnss_is_configured(void)
{
    return _configured;
}

static void gnss_cfg_thread_entry(void *parameter)
{
    (void)parameter;

    /* UM220 requires startup time after power-on/reset before accepting commands. */
    rt_thread_mdelay(1000U);
    if (gnss_configure_all() != RT_EOK)
    {
        LOG_E("Automatic UM220 constellation configuration failed");
    }
}

static rt_err_t gnss_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&_sem_rx);
    return RT_EOK;
}

static void gnss_rx_thread_entry(void *parameter)
{
    char line_buf[GNSS_NMEA_LINE_MAX_SZ];
    rt_uint16_t line_len = 0U;
    rt_uint8_t collecting = 0U;
    rt_uint8_t ch = 0U;

    LOG_I("GNSS parser thread (high-robustness engine) running...");

    while (1)
    {
        while (rt_device_read(_gnss_dev, 0, &ch, 1) == 1)
        {
            _gnss_stats.bytes++;

            if (ch == (rt_uint8_t)'$')
            {
                if ((collecting != 0U) && (line_len > 0U))
                {
                    _gnss_stats.format_error++;
                }
                collecting = 1U;
                line_len = 0U;
                line_buf[line_len++] = '$';
                continue;
            }

            if (collecting == 0U)
            {
                continue;
            }

            if (ch == (rt_uint8_t)'\n')
            {
                rt_uint16_t parse_len = line_len;
                if ((parse_len > 0U) && (line_buf[parse_len - 1U] == '\r'))
                {
                    parse_len--;
                }
                if (parse_len > 0U)
                {
                    line_buf[parse_len] = '\0';
                    if (gnss_line_is_navigation_nmea(line_buf, parse_len))
                    {
                        nmea_process_sentence(line_buf, parse_len);
                    }
                    else
                    {
                        gnss_command_process_line(line_buf, parse_len);
                    }
                }
                line_len = 0U;
                collecting = 0U;
                continue;
            }

            if (line_len >= (GNSS_NMEA_LINE_MAX_SZ - 1U))
            {
                _gnss_stats.overflow_error++;
                line_len = 0U;
                collecting = 0U;
                continue;
            }

            line_buf[line_len++] = (char)ch;
        }

        rt_sem_take(&_sem_rx, rt_tick_from_millisecond(5000));
        task_wdt_heartbeat_by_name("gnss");
    }
}

int gnss_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    if (_inited)
    {
        return RT_EOK;
    }

    LOG_I("Initializing GNSS (UM220-IV-M) on %s (PA4-RX, PA3-TX)...", GNSS_UART_NAME);

    /* 1. Ensure RNSS power rail (PH2) is enabled */
    hw_power_rnss_set(RT_TRUE);

    /* 2. Hardware reset pulse (PC13, 100ms) */
    hw_reset_rnss(100);

    /* 3. Find serial device */
    _gnss_dev = rt_device_find(GNSS_UART_NAME);
    if (!_gnss_dev)
    {
        LOG_E("Cannot find GNSS serial device: %s", GNSS_UART_NAME);
        return -RT_ERROR;
    }

    /* 4. Configure baudrate 115200 8N1 */
    cfg.baud_rate = GNSS_DEFAULT_BAUDRATE;
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity    = PARITY_NONE;
    if (rt_device_control(_gnss_dev, RT_DEVICE_CTRL_CONFIG, &cfg) != RT_EOK)
    {
        LOG_E("Failed to configure %s parameters", GNSS_UART_NAME);
        return -RT_ERROR;
    }

    /* 5. Open device with interrupt RX */
    if (rt_device_open(_gnss_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Failed to open %s with INT_RX", GNSS_UART_NAME);
        return -RT_ERROR;
    }

    /* 6. Initialize synchronization & data */
    rt_memset(&_gnss_data, 0, sizeof(_gnss_data));
    rt_memset(&_gnss_stats, 0, sizeof(_gnss_stats));
    rt_sem_init(&_sem_rx, "sgnss_rx", 0, RT_IPC_FLAG_PRIO);
    rt_sem_init(&_sem_cmd, "sgnss_cmd", 0, RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_mutex_data, "mgnss_dt", RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_mutex_cmd, "mgnss_cmd", RT_IPC_FLAG_PRIO);

    /* 7. Set receive callback */
    rt_device_set_rx_indicate(_gnss_dev, gnss_uart_rx_ind);

    /* 8. Create receiver thread */
    _rx_thread = rt_thread_create("gnss_rx",
                                  gnss_rx_thread_entry,
                                  RT_NULL,
                                  GNSS_THREAD_STACK_SZ,
                                  GNSS_THREAD_PRIO,
                                  10);
    if (!_rx_thread)
    {
        LOG_E("Failed to create gnss_rx thread");
        return -RT_ERROR;
    }
    rt_thread_startup(_rx_thread);

    _inited = RT_TRUE;
    _cfg_thread = rt_thread_create("gnss_cfg", gnss_cfg_thread_entry, RT_NULL,
                                   GNSS_CFG_THREAD_STACK_SZ,
                                   GNSS_CFG_THREAD_PRIO, 10U);
    if (_cfg_thread == RT_NULL)
    {
        LOG_E("Failed to create gnss_cfg thread");
        return -RT_ERROR;
    }
    rt_thread_startup(_cfg_thread);
    LOG_I("GNSS module ready: %s @ 115200 8N1; constellation config scheduled", GNSS_UART_NAME);

    return RT_EOK;
}
INIT_ENV_EXPORT(gnss_init);

rt_bool_t gnss_get_data(gnss_data_t *out_data)
{
    rt_bool_t fixed = RT_FALSE;

    if (!out_data)
    {
        return RT_FALSE;
    }

    rt_mutex_take(&_mutex_data, RT_WAITING_FOREVER);
    rt_memcpy(out_data, &_gnss_data, sizeof(gnss_data_t));
    fixed = _gnss_data.is_fixed;
    rt_mutex_release(&_mutex_data);

    return fixed;
}

rt_bool_t gnss_is_fresh(rt_uint32_t timeout_ms)
{
    rt_bool_t fresh = RT_FALSE;

    rt_mutex_take(&_mutex_data, RT_WAITING_FOREVER);
    if (_gnss_data.is_fixed)
    {
        rt_tick_t elapsed_ticks = rt_tick_get() - _gnss_data.last_update_tick;
        rt_uint32_t elapsed_ms = (rt_uint32_t)(elapsed_ticks * (1000 / RT_TICK_PER_SECOND));
        if (elapsed_ms <= timeout_ms)
        {
            fresh = RT_TRUE;
        }
    }
    rt_mutex_release(&_mutex_data);

    return fresh;
}

void gnss_get_stats(gnss_stats_t *out_stats)
{
    if (!out_stats)
    {
        return;
    }
    rt_mutex_take(&_mutex_data, RT_WAITING_FOREVER);
    rt_memcpy(out_stats, &_gnss_stats, sizeof(gnss_stats_t));
    rt_mutex_release(&_mutex_data);
}

rt_size_t gnss_send_cmd(const char *cmd)
{
    if (!_inited || !_gnss_dev || !cmd)
    {
        return 0;
    }

    rt_size_t len = rt_strlen(cmd);
    rt_size_t sent = rt_device_write(_gnss_dev, 0, cmd, len);
    if (len > 0 && cmd[len - 1] != '\n')
    {
        rt_device_write(_gnss_dev, 0, "\r\n", 2);
        sent += 2;
    }

    return sent;
}

rt_err_t gnss_set_baudrate(rt_uint32_t baud_rate)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    if (!_inited || !_gnss_dev)
    {
        return -RT_ERROR;
    }

    cfg.baud_rate = baud_rate;
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity    = PARITY_NONE;

    rt_err_t err = rt_device_control(_gnss_dev, RT_DEVICE_CTRL_CONFIG, &cfg);
    if (err == RT_EOK)
    {
        LOG_I("GNSS baudrate changed to %d bps", baud_rate);
    }
    return err;
}

void gnss_set_raw_log(rt_bool_t enable)
{
    _raw_log = enable;
    rt_kprintf("GNSS raw NMEA logging %s\r\n", enable ? "ENABLED" : "DISABLED");
}

void gnss_set_sentence_handler(gnss_sentence_handler_t handler)
{
    _sentence_handler = handler;
}

/* ------------------- FinSH / MSH Commands ------------------- */
#ifdef RT_USING_FINSH
#include <finsh.h>

static int gnss_status(int argc, char **argv)
{
    gnss_data_t data;
    gnss_stats_t stats;
    rt_bool_t fixed = gnss_get_data(&data);
    gnss_get_stats(&stats);
    rt_bool_t fresh = gnss_is_fresh(10000); /* 10s freshness threshold */

    rt_kprintf("\r\n================ GNSS Status ================\r\n");
    rt_kprintf(" Port           : %s (PA4-RX, PA3-TX)\r\n", GNSS_UART_NAME);
    rt_kprintf(" Constellations : GPS+BDS+GAL+SBAS+QZSS (%s)\r\n",
               gnss_is_configured() ? "CONFIG OK" : "CONFIG PENDING/FAILED");
    rt_kprintf(" NMEA Output    : GGA+GSA+GSV+RMC @ 1 Hz\r\n");
    rt_kprintf(" Fix Status     : %s\r\n", fixed ? "FIXED (VALID)" : "NO FIX (INVALID)");
    rt_kprintf(" Freshness      : %s (Threshold: 10s)\r\n", fresh ? "FRESH" : "STALE / EXPIRED");
    rt_kprintf(" Fix Quality    : %d (%s)\r\n", data.fix_quality,
               (data.fix_quality == 1) ? "GPS SPS" :
               (data.fix_quality == 2) ? "DGPS" :
               (data.fix_quality == 4) ? "RTK Fixed" :
               (data.fix_quality == 5) ? "RTK Float" : "Invalid/None");
    rt_kprintf(" Satellites     : %d in use\r\n", data.sats_used);
    rt_kprintf(" HDOP           : %d.%02d\r\n", (int)data.hdop, (int)(data.hdop * 100) % 100);
    rt_kprintf(" Altitude       : %d.%01d m\r\n", (int)data.altitude, (int)(data.altitude * 10) % 10);
    rt_kprintf(" Speed          : %u cm/s (%d.%02d km/h)\r\n",
               data.speed_cm_s, (int)data.speed_kmh, (int)(data.speed_kmh * 100) % 100);
    rt_kprintf(" Course         : %d.%01d deg\r\n", (int)data.course, (int)(data.course * 10) % 10);

    /* High Precision E7 vs Formatted */
    rt_kprintf(" Lat (E7)       : %d (%.7f deg %c)\r\n",
               data.latitude_e7, data.latitude, data.lat_dir ? data.lat_dir : '-');
    rt_kprintf(" Lon (E7)       : %d (%.7f deg %c)\r\n",
               data.longitude_e7, data.longitude, data.lon_dir ? data.lon_dir : '-');

    /* UTC Date & Time */
    rt_kprintf(" UTC Time       : %02d:%02d:%02d\r\n", data.hour, data.minute, data.second);
    rt_kprintf(" UTC Date       : %04d-%02d-%02d\r\n", data.year, data.month, data.day);

    if (fixed)
    {
        rt_tick_t elapsed_ms = (rt_tick_get() - data.last_update_tick) * (1000 / RT_TICK_PER_SECOND);
        rt_kprintf(" Last Update    : %u ms ago\r\n", elapsed_ms);
    }

    rt_kprintf("---------------- Statistics -----------------\r\n");
    rt_kprintf(" RX Total Bytes : %u\r\n", stats.bytes);
    rt_kprintf(" Sentences Total: %u\r\n", stats.sentences);
    rt_kprintf(" RMC OK         : %u\r\n", stats.rmc_ok);
    rt_kprintf(" GGA OK         : %u\r\n", stats.gga_ok);
    rt_kprintf(" Checksum Errors: %u\r\n", stats.checksum_error);
    rt_kprintf(" Format Errors  : %u\r\n", stats.format_error);
    rt_kprintf(" Overflow Errors: %u\r\n", stats.overflow_error);
    rt_kprintf(" Unsupported    : %u\r\n", stats.unsupported);
    rt_kprintf(" Raw Log Stream : %s\r\n", _raw_log ? "ON" : "OFF");
    rt_kprintf("=============================================\r\n\r\n");

    return 0;
}
MSH_CMD_EXPORT(gnss_status, display Beidou/GNSS positioning status and statistics);

static int gnss_raw(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: gnss_raw <on|off>\r\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        gnss_set_raw_log(RT_TRUE);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        gnss_set_raw_log(RT_FALSE);
    }
    else
    {
        rt_kprintf("Unknown parameter: %s, use 'on' or 'off'\r\n", argv[1]);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(gnss_raw, toggle raw NMEA sentence logging <on|off>);

static int gnss_baud(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: gnss_baud <9600|115200|38400>\r\n");
        return -1;
    }

    rt_uint32_t baud = (rt_uint32_t)atoi(argv[1]);
    if (baud == 9600 || baud == 115200 || baud == 38400 || baud == 19200 || baud == 57600)
    {
        gnss_set_baudrate(baud);
    }
    else
    {
        rt_kprintf("Unsupported baudrate: %d\r\n", baud);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(gnss_baud, set GNSS UART baudrate);

static int gnss_reset(int argc, char **argv)
{
    rt_err_t result;

    rt_kprintf("Triggering UM220 GNSS hardware reset (PC13 low pulse 100ms)...\r\n");
    _configured = RT_FALSE;
    hw_reset_rnss(100);
    rt_thread_mdelay(1000U);
    result = gnss_configure_all();
    rt_kprintf("GNSS reset and constellation configuration %s.\r\n",
               (result == RT_EOK) ? "complete" : "failed");
    return (result == RT_EOK) ? 0 : -1;
}
MSH_CMD_EXPORT(gnss_reset, perform hardware reset on GNSS module);

static int gnss_config(int argc, char **argv)
{
    rt_err_t result;

    (void)argc;
    (void)argv;
    result = gnss_configure_all();
    rt_kprintf("UM220 all-constellation runtime configuration: %s\r\n",
               (result == RT_EOK) ? "OK" : "FAILED");
    return (result == RT_EOK) ? 0 : -1;
}
MSH_CMD_EXPORT(gnss_config, enable GPS BDS Galileo SBAS QZSS and required NMEA);

static int gnss_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: gnss_cmd <NMEA command string>\r\n");
        rt_kprintf("Example: gnss_cmd $CFGSYS,H11*checksum\r\n");
        return -1;
    }

    rt_size_t sent = gnss_send_cmd(argv[1]);
    rt_kprintf("Sent %u bytes to GNSS: %s\r\n", sent, argv[1]);
    return 0;
}
MSH_CMD_EXPORT(gnss_cmd, send command string to GNSS module);

#endif /* RT_USING_FINSH */
