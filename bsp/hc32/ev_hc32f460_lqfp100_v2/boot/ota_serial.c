/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "ota_layout.h"
#include "ota_serial.h"
#include "flash_map_backend/flash_map_backend.h"
#include "bootutil/bootutil_public.h"

enum { SOH = 1, STX = 2, EOT = 4, ACK = 6, NAK = 21, CAN = 24 };
static uint8_t frame[1028];
static char last_error[144];

void ntn_ota_log_error(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    rt_vsnprintf(last_error, sizeof(last_error), format, args);
    va_end(args);
}

static bool send_bytes(rt_device_t port, const void *data, unsigned length)
{
    return rt_device_write(port, 0, data, length) == length;
}

static bool signal_byte(rt_device_t port, uint8_t value)
{
    return send_bytes(port, &value, 1);
}

static int read_byte(rt_device_t port, unsigned timeout)
{
    rt_tick_t start = rt_tick_get();
    do
    {
        uint8_t byte;
        if (rt_device_read(port, 0, &byte, 1) == 1)
            return byte;
        rt_thread_mdelay(1);
    } while (rt_tick_get() - start < rt_tick_from_millisecond(timeout));
    return -1;
}

static bool read_line(rt_device_t port, char *line, unsigned capacity, unsigned timeout)
{
    unsigned used = 0;
    rt_tick_t start = rt_tick_get();
    while (rt_tick_get() - start < rt_tick_from_millisecond(timeout))
    {
        int c = read_byte(port, 10);
        if (c < 0 || c == '\r')
            continue;
        if (c == '\n')
        {
            if (used < capacity)
            {
                line[used] = 0;
                return true;
            }
            used = 0;
        }
        else if (used < capacity)
            line[used++] = (char)c;
    }
    return false;
}

static bool parse_length(const char *str, uint32_t *length)
{
    uint32_t n = 0;
    for (; *str >= '0' && *str <= '9'; ++str)
    {
        n = 10U * n + (unsigned)(*str - '0');
        if (n > NTN_OTA_PACKAGE_MAX)
            return false;
    }
    if ((*str != 0 && *str != ' ') || n == 0)
        return false;
    *length = n;
    return true;
}

static uint16_t crc16(const uint8_t *data, unsigned len)
{
    uint16_t crc = 0;
    while (len--)
    {
        crc ^= (uint16_t)*data++ << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    }
    return crc;
}

static unsigned read_frame(rt_device_t port, int type)
{
    unsigned size = type == SOH ? 128 : type == STX ? 1024 : 0;
    if (!size)
        return 0;
    for (unsigned i = 0; i < size + 4U; ++i)
    {
        int next = read_byte(port, 300);
        if (next < 0)
            return 0;
        frame[i] = (uint8_t)next;
    }
    return (uint8_t)(frame[0] ^ frame[1]) == 0xFF &&
           crc16(frame + 2, size) == (uint16_t)(frame[size + 2] << 8 | frame[size + 3])
           ? size : 0;
}

static bool last_block_matches(const struct flash_area *area, uint32_t offset, unsigned count)
{
    uint8_t bytes[32];
    for (unsigned checked = 0; checked < count;)
    {
        unsigned n = count - checked > sizeof(bytes) ? sizeof(bytes) : count - checked;
        if (flash_area_read(area, offset + checked, bytes, n) ||
            memcmp(bytes, frame + 2 + checked, n))
            return false;
        checked += n;
    }
    return true;
}

static bool receive_file(rt_device_t port, const struct flash_area *area, uint32_t length)
{
    uint32_t written = 0;
    unsigned previous = 0;
    uint8_t sequence = 0;
    unsigned phase = 0;
    unsigned errors = 0;
    if (!signal_byte(port, 'C'))
        return false;

    while (errors < 6)
    {
        int code = read_byte(port, 3000);
        unsigned size;
        if (code == CAN)
            return false;
        if (code == EOT && phase >= 1 && written == length)
        {
            if (phase++ == 1)
                signal_byte(port, NAK);
            else
            {
                phase = 3;
                send_bytes(port, "\x06" "C", 2);
            }
            continue;
        }
        size = read_frame(port, code);
        if (size)
        {
            if (phase == 0 && frame[0] == 0)
            {
                const uint8_t *end = memchr(frame + 2, 0, size);
                uint32_t stated;
                frame[size + 2] = 0;
                if (!end || end == frame + 2 || end + 1 >= frame + 2 + size ||
                    !parse_length((const char *)end + 1, &stated) || stated != length)
                    return false;
                phase = 1;
                sequence = 1;
                errors = 0;
                send_bytes(port, "\x06" "C", 2);
                continue;
            }
            if (phase == 3 && frame[0] == 0 && frame[2] == 0)
                return signal_byte(port, ACK);
            if (phase == 1 && frame[0] == sequence && written < length)
            {
                previous = length - written < size ? length - written : size;
                if (flash_area_write(area, written, frame + 2, previous))
                    return false;
                written += previous;
                ++sequence;
                errors = 0;
                signal_byte(port, ACK);
                continue;
            }
            if (phase == 1 && frame[0] == (uint8_t)(sequence - 1U))
            {
                if (!written)
                {
                    send_bytes(port, "\x06" "C", 2);
                    continue;
                }
                if (size >= previous && last_block_matches(area, written - previous, previous))
                {
                    signal_byte(port, ACK);
                    continue;
                }
            }
        }
        ++errors;
        signal_byte(port, phase == 0 || phase == 3 ? 'C' : NAK);
    }
    return false;
}

static rt_device_t open_debug_uart(void)
{
    rt_device_t port = rt_device_find("uart1");
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = 19200;
    if (!port || rt_device_control(port, RT_DEVICE_CTRL_CONFIG, &config) != RT_EOK ||
        rt_device_open(port, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
        return RT_NULL;
    return port;
}

static void send_boot_log(rt_device_t port, const char *status)
{
    send_bytes(port, status, strlen(status));
    if (last_error[0])
    {
        const char prefix[] = "BOOT2 DETAIL ";
        send_bytes(port, prefix, sizeof(prefix) - 1U);
        send_bytes(port, last_error, strlen(last_error));
        send_bytes(port, "\r\n", 2);
    }
}

void ntn_ota_report_boot_error(const char *error)
{
    rt_device_t port = open_debug_uart();
    if (port)
    {
        send_boot_log(port, error);
        rt_thread_mdelay(40);
        rt_device_close(port);
    }
}

int ntn_ota_serial(unsigned listen_ms, const char *status)
{
    rt_device_t port = open_debug_uart();
    const struct flash_area *area;
    char line[80];
    uint32_t length;
    int result = 0;
    if (!port)
        return 0;

    if (!read_line(port, line, sizeof(line), listen_ms) || strcmp(line, "NTN-MCU-OTA/1 HELLO"))
        goto close;
    send_boot_log(port, status);
    rt_snprintf(line, sizeof(line), "READY %u %u %u\r\n",
                NTN_OTA_BOOT_ABI, NTN_OTA_LAYOUT_ID, NTN_OTA_PACKAGE_MAX);
    if (!send_bytes(port, line, strlen(line)) ||
        !read_line(port, line, sizeof(line), 10000) || strncmp(line, "BEGIN ", 6) ||
        !parse_length(line + 6, &length))
        goto close;

    if (flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &area) ||
        flash_area_erase(area, 0, NTN_OTA_SLOT_SIZE))
    {
        send_bytes(port, "ERROR FLASH_IO\r\n", 16);
        goto close;
    }
    if (!send_bytes(port, "RX 1\r\n", 6) || !receive_file(port, area, length))
    {
        send_bytes(port, "\x18\x18" "ERROR RX\r\n", 12);
        goto close;
    }
    if (!ntn_ota_check_image(area, length))
    {
        send_bytes(port, "ERROR BAD_IMAGE\r\n", 17);
        goto close;
    }
    if (boot_set_pending(0))
    {
        const char *error = "ERROR TEST_MARK\r\n";
        send_bytes(port, error, strlen(error));
        goto close;
    }
    send_bytes(port, "VERIFIED 1\r\n", 12);
    rt_thread_mdelay(100);
    result = 1;
close:
    rt_device_close(port);
    return result;
}
