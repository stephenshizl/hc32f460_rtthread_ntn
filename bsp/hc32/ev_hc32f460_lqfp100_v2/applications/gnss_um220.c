/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: GNSS (Beidou/GPS) UM220-IV-M driver
 */

#include "gnss_um220.h"
#include "hw_power.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define DBG_TAG "gnss"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define GNSS_THREAD_STACK_SZ    2048
#define GNSS_THREAD_PRIO        12

#define NMEA_FIELD_MAX_LEN      32

static rt_device_t              _gnss_dev          = RT_NULL;
static struct rt_semaphore      _sem_rx;
static struct rt_mutex          _mutex_data;
static rt_thread_t              _rx_thread         = RT_NULL;

static gnss_data_t              _gnss_data;
static gnss_sentence_handler_t  _sentence_handler  = RT_NULL;
static rt_bool_t                _raw_log           = RT_FALSE;
static rt_bool_t                _inited            = RT_FALSE;

/* Diagnostic statistics */
static volatile rt_uint32_t     _stat_rx_bytes     = 0;
static volatile rt_uint32_t     _stat_sentences    = 0;
static volatile rt_uint32_t     _stat_crc_err      = 0;
static volatile rt_uint32_t     _stat_gga_count    = 0;
static volatile rt_uint32_t     _stat_rmc_count    = 0;

/**
 * @brief Calculate and verify NMEA checksum
 * @param sentence Sentence string starting with '$' and containing '*'
 * @return RT_TRUE if checksum matches, RT_FALSE otherwise
 */
static rt_bool_t nmea_verify_checksum(const char *sentence)
{
    const char *p = sentence;
    rt_uint8_t calc_crc = 0;
    rt_uint32_t expected_crc = 0;

    if (!sentence || *sentence != '$')
    {
        return RT_FALSE;
    }

    p++; /* Skip '$' */

    while (*p && *p != '*' && *p != '\r' && *p != '\n')
    {
        calc_crc ^= (rt_uint8_t)(*p);
        p++;
    }

    if (*p != '*')
    {
        return RT_FALSE;
    }

    p++; /* Skip '*' */

    if (*p == '\0')
    {
        return RT_FALSE;
    }

    /* Convert hex checksum */
    char crc_str[3] = {0};
    crc_str[0] = p[0];
    if (p[1] != '\0' && p[1] != '\r' && p[1] != '\n')
    {
        crc_str[1] = p[1];
    }
    expected_crc = (rt_uint32_t)strtoul(crc_str, RT_NULL, 16);

    return (calc_crc == (rt_uint8_t)expected_crc) ? RT_TRUE : RT_FALSE;
}

/**
 * @brief Extract the next comma-separated field from an NMEA sentence
 * @param s Pointer to current position in string
 * @param field Output buffer for the field
 * @param max_len Size of output buffer
 * @return Pointer to start of next field, or RT_NULL if end of sentence reached
 */
static const char *nmea_next_field(const char *s, char *field, rt_size_t max_len)
{
    rt_size_t idx = 0;

    if (!s)
    {
        if (field && max_len > 0)
        {
            field[0] = '\0';
        }
        return RT_NULL;
    }

    while (*s && *s != ',' && *s != '*' && *s != '\r' && *s != '\n')
    {
        if (idx < max_len - 1)
        {
            field[idx++] = *s;
        }
        s++;
    }

    if (field && max_len > 0)
    {
        field[idx] = '\0';
    }

    if (*s == ',')
    {
        return s + 1; /* Position at start of next field */
    }

    return RT_NULL; /* End of sentence reached */
}

/**
 * @brief Convert NMEA coordinate (ddmm.mmmm / dddmm.mmmm) to decimal degrees
 * @param str NMEA coordinate string
 * @return Decimal degrees value
 */
static double nmea_coord_to_deg(const char *str)
{
    if (!str || *str == '\0')
    {
        return 0.0;
    }

    double raw = atof(str);
    int deg = (int)(raw / 100.0);
    double min = raw - ((double)deg * 100.0);

    return (double)deg + (min / 60.0);
}

/**
 * @brief Parse GGA sentence ($GNGGA, $GPGGA, $BDGGA)
 * Format: $xxGGA,time,lat,lat_dir,lon,lon_dir,quality,num_sats,hdop,alt,M,geoid,M,age,ref*cs
 */
static void nmea_parse_gga(const char *sentence)
{
    char field[NMEA_FIELD_MAX_LEN];
    const char *p = sentence;

    /* Field 0: Sentence ID ($xxGGA) */
    p = nmea_next_field(p, field, sizeof(field));

    /* Field 1: UTC Time (hhmmss.sss) */
    p = nmea_next_field(p, field, sizeof(field));
    if (field[0] != '\0')
    {
        rt_uint32_t raw_time = (rt_uint32_t)strtoul(field, RT_NULL, 10);
        rt_mutex_take(&_mutex_data, RT_WAITING_FOREVER);
        _gnss_data.utc_time = raw_time;
        rt_mutex_release(&_mutex_data);
    }

    /* Field 2: Latitude (ddmm.mmmm) */
    p = nmea_next_field(p, field, sizeof(field));
    char lat_str[NMEA_FIELD_MAX_LEN];
    rt_strncpy(lat_str, field, sizeof(lat_str));

    /* Field 3: N/S Direction */
    p = nmea_next_field(p, field, sizeof(field));
    char lat_dir = field[0];

    /* Field 4: Longitude (dddmm.mmmm) */
    p = nmea_next_field(p, field, sizeof(field));
    char lon_str[NMEA_FIELD_MAX_LEN];
    rt_strncpy(lon_str, field, sizeof(lon_str));

    /* Field 5: E/W Direction */
    p = nmea_next_field(p, field, sizeof(field));
    char lon_dir = field[0];

    /* Field 6: Fix Quality (0=Invalid, 1=SPS, 2=DGPS, etc.) */
    p = nmea_next_field(p, field, sizeof(field));
    rt_uint8_t fix_quality = (rt_uint8_t)atoi(field);

    /* Field 7: Satellites used */
    p = nmea_next_field(p, field, sizeof(field));
    rt_uint8_t sats = (rt_uint8_t)atoi(field);

    /* Field 8: HDOP */
    p = nmea_next_field(p, field, sizeof(field));
    float hdop = (float)atof(field);

    /* Field 9: Altitude */
    p = nmea_next_field(p, field, sizeof(field));
    float alt = (float)atof(field);

    /* Update shared data under mutex */
    rt_mutex_take(&_mutex_data, RT_WAITING_FOREVER);
    _gnss_data.fix_quality = fix_quality;
    _gnss_data.is_fixed    = (fix_quality > 0) ? RT_TRUE : RT_FALSE;
    _gnss_data.sats_used   = sats;
    _gnss_data.hdop        = hdop;
    _gnss_data.altitude    = alt;

    if (lat_str[0] != '\0' && (lat_dir == 'N' || lat_dir == 'S'))
    {
        double lat = nmea_coord_to_deg(lat_str);
        if (lat_dir == 'S') lat = -lat;
        _gnss_data.latitude = lat;
        _gnss_data.lat_dir  = lat_dir;
    }

    if (lon_str[0] != '\0' && (lon_dir == 'E' || lon_dir == 'W'))
    {
        double lon = nmea_coord_to_deg(lon_str);
        if (lon_dir == 'W') lon = -lon;
        _gnss_data.longitude = lon;
        _gnss_data.lon_dir   = lon_dir;
    }

    if (_gnss_data.is_fixed)
    {
        _gnss_data.last_update_tick = rt_tick_get();
    }
    rt_mutex_release(&_mutex_data);

    _stat_gga_count++;
}

/**
 * @brief Parse RMC sentence ($GNRMC, $GPRMC, $BDRMC)
 * Format: $xxRMC,time,status,lat,lat_dir,lon,lon_dir,speed,course,date,mag_var,mag_dir,mode*cs
 */
static void nmea_parse_rmc(const char *sentence)
{
    char field[NMEA_FIELD_MAX_LEN];
    const char *p = sentence;

    /* Field 0: Sentence ID ($xxRMC) */
    p = nmea_next_field(p, field, sizeof(field));

    /* Field 1: UTC Time (hhmmss.sss) */
    p = nmea_next_field(p, field, sizeof(field));
    rt_uint32_t utc_time = 0;
    if (field[0] != '\0')
    {
        utc_time = (rt_uint32_t)strtoul(field, RT_NULL, 10);
    }

    /* Field 2: Status ('A'=Valid, 'V'=Warning) */
    p = nmea_next_field(p, field, sizeof(field));
    rt_bool_t is_valid = (field[0] == 'A') ? RT_TRUE : RT_FALSE;

    /* Field 3: Latitude */
    p = nmea_next_field(p, field, sizeof(field));
    char lat_str[NMEA_FIELD_MAX_LEN];
    rt_strncpy(lat_str, field, sizeof(lat_str));

    /* Field 4: N/S */
    p = nmea_next_field(p, field, sizeof(field));
    char lat_dir = field[0];

    /* Field 5: Longitude */
    p = nmea_next_field(p, field, sizeof(field));
    char lon_str[NMEA_FIELD_MAX_LEN];
    rt_strncpy(lon_str, field, sizeof(lon_str));

    /* Field 6: E/W */
    p = nmea_next_field(p, field, sizeof(field));
    char lon_dir = field[0];

    /* Field 7: Speed over ground in knots */
    p = nmea_next_field(p, field, sizeof(field));
    float speed_kmh = (float)(atof(field) * 1.852); /* 1 knot = 1.852 km/h */

    /* Field 8: Course over ground in degrees */
    p = nmea_next_field(p, field, sizeof(field));
    float course = (float)atof(field);

    /* Field 9: UTC Date (ddmmyy) */
    p = nmea_next_field(p, field, sizeof(field));
    rt_uint32_t utc_date = 0;
    if (field[0] != '\0')
    {
        utc_date = (rt_uint32_t)strtoul(field, RT_NULL, 10);
    }

    /* Update shared data under mutex */
    rt_mutex_take(&_mutex_data, RT_WAITING_FOREVER);
    _gnss_data.is_fixed   = is_valid;
    _gnss_data.speed_kmh  = speed_kmh;
    _gnss_data.course     = course;

    if (utc_time > 0)
    {
        _gnss_data.utc_time = utc_time;
    }
    if (utc_date > 0)
    {
        _gnss_data.utc_date = utc_date;
    }

    if (lat_str[0] != '\0' && (lat_dir == 'N' || lat_dir == 'S'))
    {
        double lat = nmea_coord_to_deg(lat_str);
        if (lat_dir == 'S') lat = -lat;
        _gnss_data.latitude = lat;
        _gnss_data.lat_dir  = lat_dir;
    }

    if (lon_str[0] != '\0' && (lon_dir == 'E' || lon_dir == 'W'))
    {
        double lon = nmea_coord_to_deg(lon_str);
        if (lon_dir == 'W') lon = -lon;
        _gnss_data.longitude = lon;
        _gnss_data.lon_dir   = lon_dir;
    }

    if (is_valid)
    {
        _gnss_data.last_update_tick = rt_tick_get();
    }
    rt_mutex_release(&_mutex_data);

    _stat_rmc_count++;
}

/**
 * @brief Dispatch a complete NMEA sentence to appropriate parsers
 */
static void nmea_process_sentence(const char *sentence)
{
    _stat_sentences++;

    /* Check checksum first */
    if (!nmea_verify_checksum(sentence))
    {
        _stat_crc_err++;
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

    /* Identify sentence type: 3rd through 5th chars, e.g. GGA, RMC */
    if (rt_strstr(sentence, "GGA,"))
    {
        nmea_parse_gga(sentence);
    }
    else if (rt_strstr(sentence, "RMC,"))
    {
        nmea_parse_rmc(sentence);
    }
}

/**
 * @brief UART RX callback indicating incoming data
 */
static rt_err_t gnss_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&_sem_rx);
    return RT_EOK;
}

/**
 * @brief GNSS receiver and parser thread
 */
static void gnss_rx_thread_entry(void *parameter)
{
    char line_buf[GNSS_NMEA_LINE_MAX_SZ];
    rt_size_t line_idx = 0;
    rt_uint8_t ch = 0;

    LOG_I("GNSS parser thread running...");

    while (1)
    {
        /* Wait for incoming UART data */
        while (rt_device_read(_gnss_dev, 0, &ch, 1) == 1)
        {
            _stat_rx_bytes++;

            if (ch == '$')
            {
                /* Start of new NMEA sentence */
                line_idx = 0;
                line_buf[line_idx++] = '$';
            }
            else if (line_idx > 0)
            {
                if (ch == '\r' || ch == '\n')
                {
                    /* End of sentence */
                    if (line_idx > 4) /* Minimum valid sentence like $xx*hh */
                    {
                        line_buf[line_idx] = '\0';
                        nmea_process_sentence(line_buf);
                    }
                    line_idx = 0;
                }
                else if (line_idx < sizeof(line_buf) - 1)
                {
                    line_buf[line_idx++] = (char)ch;
                }
                else
                {
                    /* Buffer overflow protection: discard corrupted long line */
                    line_idx = 0;
                }
            }
        }

        /* Block waiting for next RX interrupt */
        rt_sem_take(&_sem_rx, RT_WAITING_FOREVER);
    }
}

/**
 * @brief Initialize GNSS module interface on uart2
 */
int gnss_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    if (_inited)
    {
        return RT_EOK;
    }

    LOG_I("Initializing GNSS (UM220-IV-M) on %s (PA3-RX, PA4-TX)...", GNSS_UART_NAME);

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
    rt_sem_init(&_sem_rx, "sgnss_rx", 0, RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_mutex_data, "mgnss_dt", RT_IPC_FLAG_PRIO);

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
    LOG_I("GNSS module ready: %s @ 115200 8N1", GNSS_UART_NAME);

    return RT_EOK;
}
INIT_APP_EXPORT(gnss_init);

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
    rt_bool_t fixed = gnss_get_data(&data);

    rt_kprintf("\r\n================ GNSS Status ================\r\n");
    rt_kprintf(" Port           : %s (PA3-RX, PA4-TX)\r\n", GNSS_UART_NAME);
    rt_kprintf(" Fix Status     : %s\r\n", fixed ? "FIXED (VALID)" : "NO FIX (INVALID)");
    rt_kprintf(" Fix Quality    : %d (%s)\r\n", data.fix_quality,
               (data.fix_quality == 1) ? "GPS SPS" :
               (data.fix_quality == 2) ? "DGPS" :
               (data.fix_quality == 4) ? "RTK Fixed" :
               (data.fix_quality == 5) ? "RTK Float" : "Invalid/None");
    rt_kprintf(" Satellites     : %d in use\r\n", data.sats_used);
    rt_kprintf(" HDOP           : %d.%02d\r\n", (int)data.hdop, (int)(data.hdop * 100) % 100);
    rt_kprintf(" Altitude       : %d.%01d m\r\n", (int)data.altitude, (int)(data.altitude * 10) % 10);
    rt_kprintf(" Speed          : %d.%02d km/h\r\n", (int)data.speed_kmh, (int)(data.speed_kmh * 100) % 100);
    rt_kprintf(" Course         : %d.%01d deg\r\n", (int)data.course, (int)(data.course * 10) % 10);

    /* Latitude & Longitude with 6 decimal places */
    int lat_deg = (int)data.latitude;
    int lat_frac = (int)((data.latitude > 0 ? data.latitude - lat_deg : lat_deg - data.latitude) * 1000000.0);
    int lon_deg = (int)data.longitude;
    int lon_frac = (int)((data.longitude > 0 ? data.longitude - lon_deg : lon_deg - data.longitude) * 1000000.0);

    rt_kprintf(" Latitude       : %d.%06d deg %c\r\n", lat_deg, lat_frac, data.lat_dir ? data.lat_dir : '-');
    rt_kprintf(" Longitude      : %d.%06d deg %c\r\n", lon_deg, lon_frac, data.lon_dir ? data.lon_dir : '-');

    /* UTC Date & Time */
    rt_uint32_t t = data.utc_time;
    rt_uint32_t d = data.utc_date;
    rt_kprintf(" UTC Time       : %02d:%02d:%02d\r\n", (int)(t / 10000), (int)((t % 10000) / 100), (int)(t % 100));
    rt_kprintf(" UTC Date       : 20%02d-%02d-%02d\r\n", (int)(d % 100), (int)((d % 10000) / 100), (int)(d / 10000));

    if (fixed)
    {
        rt_tick_t elapsed_ms = (rt_tick_get() - data.last_update_tick) * (1000 / RT_TICK_PER_SECOND);
        rt_kprintf(" Last Update    : %u ms ago\r\n", elapsed_ms);
    }

    rt_kprintf("---------------- Statistics -----------------\r\n");
    rt_kprintf(" RX Total Bytes : %u\r\n", _stat_rx_bytes);
    rt_kprintf(" NMEA Sentences : %u\r\n", _stat_sentences);
    rt_kprintf(" Checksum Errors: %u\r\n", _stat_crc_err);
    rt_kprintf(" GGA Sentences  : %u\r\n", _stat_gga_count);
    rt_kprintf(" RMC Sentences  : %u\r\n", _stat_rmc_count);
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
    rt_kprintf("Triggering UM220 GNSS hardware reset (PC13 low pulse 100ms)...\r\n");
    hw_reset_rnss(100);
    rt_kprintf("GNSS reset complete.\r\n");
    return 0;
}
MSH_CMD_EXPORT(gnss_reset, perform hardware reset on GNSS module);

static int gnss_cmd(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: gnss_cmd <NMEA command string>\r\n");
        rt_kprintf("Example: gnss_cmd $PCAS03,1,1,1,1,1,1,1,1*02\r\n");
        return -1;
    }

    rt_size_t sent = gnss_send_cmd(argv[1]);
    rt_kprintf("Sent %u bytes to GNSS: %s\r\n", sent, argv[1]);
    return 0;
}
MSH_CMD_EXPORT(gnss_cmd, send command string to GNSS module);

#endif /* RT_USING_FINSH */
