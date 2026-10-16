/*
 * Copyright (c) 2026, Antigravity AI pair programming.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-10-13     Stephen      First version: BLE UART driver on uart4 (PB6/PB7)
 */

#include "ble_uart.h"
#include "hw_power.h"
#include "param_manager.h"
#include "gnss_um220.h"
#include "ntn_ms150.h"
#include <stdlib.h>
#include <stdio.h>
#include "task_wdt.h"

#define DBG_TAG "ble.uart"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define BLE_THREAD_STACK_SZ     2048
#define BLE_THREAD_PRIO         10

static rt_device_t            _ble_dev          = RT_NULL;
static struct rt_semaphore    _sem_rx;
static struct rt_mutex        _mutex_tx;
static rt_thread_t            _rx_thread        = RT_NULL;

static rt_uint8_t             _rx_buf[BLE_RX_BUF_MAX_SZ];
static ble_rx_handler_t       _rx_handler       = RT_NULL;
static rt_bool_t              _loopback_enabled = RT_FALSE;
static rt_bool_t              _log_stream_on    = RT_FALSE;
static rt_bool_t              _inited           = RT_FALSE;

static volatile rt_uint32_t   _stat_rx_frames   = 0;
static volatile rt_uint32_t   _stat_rx_bytes    = 0;
static volatile rt_uint32_t   _stat_tx_frames   = 0;
static volatile rt_uint32_t   _stat_tx_bytes    = 0;

/**
 * @brief Standard CRC-16 (Modbus polynomial 0xA001) for packet integrity
 */
static rt_uint16_t ble_crc16(const rt_uint8_t *data, rt_size_t len)
{
    rt_uint16_t crc = 0xFFFF;
    for (rt_size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Serial RX interrupt indication callback
 */
static rt_err_t ble_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    RT_UNUSED(dev);
    RT_UNUSED(size);

    rt_sem_release(&_sem_rx);
    return RT_EOK;
}

/**
 * @brief Handle received standard protocol frame
 */
static void ble_handle_frame(const rt_uint8_t *frame, rt_size_t len)
{
    if (len < 6)
    {
        return;
    }

    /* Check SOF */
    if (frame[0] == BLE_FRAME_SOF1 && frame[1] == BLE_FRAME_SOF2)
    {
        rt_uint8_t  msg_type = frame[2];
        rt_uint8_t  seq      = frame[3];
        rt_uint16_t payload_len = ((rt_uint16_t)frame[4] << 8) | frame[5];

        if (len >= (rt_size_t)(6 + payload_len + 2))
        {
            rt_uint16_t calc_crc = ble_crc16(frame, 6 + payload_len);
            rt_uint16_t recv_crc = ((rt_uint16_t)frame[6 + payload_len] << 8) | frame[6 + payload_len + 1];

            if (calc_crc == recv_crc)
            {
                LOG_I("[BLE] Valid Frame: Type=0x%02X, Seq=%u, Len=%u", msg_type, seq, payload_len);

                switch (msg_type)
                {
                case BLE_CMD_SYS_STATUS:
                {
                    /* Response: System health payload */
                    char resp[128];
                    rt_uint32_t uptime_s = rt_tick_get() / RT_TICK_PER_SECOND;
                    rt_snprintf(resp, sizeof(resp), "{\"dev\":\"NTN-MS150\",\"ver\":\"V1.2\",\"up_s\":%u,\"pwr_ntn\":%d}",
                                uptime_s, hw_power_ntn_get());
                    ble_send_frame(BLE_CMD_SYS_STATUS, seq, (const rt_uint8_t *)resp, rt_strlen(resp));
                    break;
                }
                case BLE_CMD_LOG_STREAM:
                {
                    if (payload_len >= 1)
                    {
                        ble_log_stream_set(frame[6] ? RT_TRUE : RT_FALSE);
                    }
                    rt_uint8_t ack = _log_stream_on ? 1 : 0;
                    ble_send_frame(BLE_CMD_LOG_STREAM, seq, &ack, 1);
                    break;
                }
                case BLE_CMD_DEV_CONFIG:
                {
                    /*
                     * Payload: [SubCmd] [Params...]
                     * SubCmd 0x01: Query Work Mode -> Resp: [0x01][Mode]
                     * SubCmd 0x02: Set Work Mode   -> Resp: [0x02][Status 0:OK,1:ERR][Mode]
                     */
                    if (payload_len >= 1)
                    {
                        rt_uint8_t sub_cmd = frame[6];
                        if (sub_cmd == 0x01)
                        {
                            rt_uint8_t resp[2];
                            resp[0] = 0x01;
                            resp[1] = param_get_work_mode();
                            ble_send_frame(BLE_CMD_DEV_CONFIG, seq, resp, sizeof(resp));
                            LOG_I("[BLE] Phone App queried work mode: %d", resp[1]);
                        }
                        else if (sub_cmd == 0x02 && payload_len >= 2)
                        {
                            rt_uint8_t target_mode = frame[7];
                            rt_err_t err = param_set_work_mode(target_mode, RT_TRUE);
                            if (err == RT_EOK)
                            {
                                ntn_bridge_set(target_mode == PARAM_MODE_NTN_BRIDGE);
                            }
                            rt_uint8_t resp[3];
                            resp[0] = 0x02;
                            resp[1] = (err == RT_EOK) ? 0x00 : 0x01;
                            resp[2] = param_get_work_mode();
                            ble_send_frame(BLE_CMD_DEV_CONFIG, seq, resp, sizeof(resp));
                            LOG_I("[BLE] Phone App switched work mode to %d (status=%d)", target_mode, resp[1]);
                        }
                    }
                    break;
                }
                case BLE_CMD_GNSS_INFO:
                {
                    gnss_data_t gnss;
                    gnss_get_data(&gnss);
                    char resp[128];
                    rt_snprintf(resp, sizeof(resp),
                                "{\"fix\":%d,\"sats\":%d,\"lat\":%d,\"lon\":%d,\"spd\":%d,\"hdg\":%d}",
                                gnss.is_fixed ? 1 : 0,
                                gnss.sats_used,
                                (int)(gnss.latitude_e7 / 10),
                                (int)(gnss.longitude_e7 / 10),
                                (int)(gnss.speed_kmh * 10),
                                (int)gnss.course);
                    ble_send_frame(BLE_CMD_GNSS_INFO, seq, (const rt_uint8_t *)resp, rt_strlen(resp));
                    break;
                }
                case BLE_CMD_NTN_AT_PASS:
                {
                    /* Phone App sends AT command to MS150 modem over BLE */
                    if (payload_len > 0)
                    {
                        char at_cmd[128];
                        char at_resp[256];
                        rt_size_t c_len = (payload_len < sizeof(at_cmd) - 1) ? payload_len : sizeof(at_cmd) - 1;
                        rt_memcpy(at_cmd, &frame[6], c_len);
                        at_cmd[c_len] = '\0';

                        rt_err_t res = ntn_exec_cmd(at_cmd, at_resp, sizeof(at_resp), 3000);
                        if (res == RT_EOK)
                        {
                            ble_send_frame(BLE_CMD_NTN_AT_PASS, seq, (const rt_uint8_t *)at_resp, rt_strlen(at_resp));
                        }
                        else
                        {
                            ble_send_frame(BLE_CMD_NTN_AT_PASS, seq, (const rt_uint8_t *)"\r\nERROR\r\n", 9);
                        }
                    }
                    break;
                }
                default:
                    break;
                }
            }
            else
            {
                LOG_W("[BLE] Frame CRC Error! Calc=0x%04X, Recv=0x%04X", calc_crc, recv_crc);
            }
        }
    }
}

/**
 * @brief BLE receiver thread with 15ms inter-character idle timeout
 */
static void ble_rx_thread_entry(void *parameter)
{
    RT_UNUSED(parameter);

    while (1)
    {
        /* Wait for first incoming byte or 5s heartbeat timeout */
        if (rt_sem_take(&_sem_rx, rt_tick_from_millisecond(5000)) != RT_EOK)
        {
            task_wdt_heartbeat_by_name("ble");
            continue;
        }
        task_wdt_heartbeat_by_name("ble");

        rt_size_t frame_len = 0;

        while (1)
        {
            rt_uint8_t ch;
            while (rt_device_read(_ble_dev, 0, &ch, 1) == 1)
            {
                if (frame_len < sizeof(_rx_buf))
                {
                    _rx_buf[frame_len++] = ch;
                }
                else
                {
                    break;
                }
            }

            /* Wait for next byte with 15ms idle timeout */
            if (rt_sem_take(&_sem_rx, rt_tick_from_millisecond(BLE_IDLE_TIMEOUT_MS)) != RT_EOK)
            {
                /* Idle gap reached -> Frame complete */
                break;
            }
        }

        if (frame_len > 0)
        {
            _stat_rx_frames++;
            _stat_rx_bytes += frame_len;

            LOG_D("[BLE] Rx Frame #%u (%d bytes)", _stat_rx_frames, frame_len);

            /* If loopback echo is enabled */
            if (_loopback_enabled)
            {
                ble_uart_send(_rx_buf, frame_len);
            }

            /* Parse standard protocol frame */
            ble_handle_frame(_rx_buf, frame_len);

            /* Dispatch to upper-layer custom handler if registered */
            if (_rx_handler != RT_NULL)
            {
                _rx_handler(_rx_buf, frame_len);
            }
        }
    }
}

/**
 * @brief Initialize BLE UART interface on uart4 (115200 8N1)
 */
int ble_uart_init(void)
{
    struct serial_configure cfg = RT_SERIAL_CONFIG_DEFAULT;

    if (_inited)
    {
        return RT_EOK;
    }

    LOG_I("Initializing BLE UART on %s (PB6-TX, PB7-RX @ 115200 8N1)...", BLE_UART_NAME);

    /* 1. Ensure BLE power is turned on via hw_power */
    hw_power_bt_set(RT_TRUE);

    /* 2. Find device */
    _ble_dev = rt_device_find(BLE_UART_NAME);
    if (!_ble_dev)
    {
        LOG_E("Cannot find serial device: %s", BLE_UART_NAME);
        return -RT_ERROR;
    }

    /* 3. Configure 115200 8N1 */
    cfg.baud_rate = BLE_DEFAULT_BAUDRATE;
    cfg.data_bits = DATA_BITS_8;
    cfg.stop_bits = STOP_BITS_1;
    cfg.parity    = PARITY_NONE;
    if (rt_device_control(_ble_dev, RT_DEVICE_CTRL_CONFIG, &cfg) != RT_EOK)
    {
        LOG_E("Failed to configure %s parameters", BLE_UART_NAME);
        return -RT_ERROR;
    }

    /* 4. Open device with interrupt RX */
    if (rt_device_open(_ble_dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) != RT_EOK)
    {
        LOG_E("Failed to open %s with INT_RX", BLE_UART_NAME);
        return -RT_ERROR;
    }

    /* 5. Initialize synchronization primitives */
    rt_sem_init(&_sem_rx, "sble_rx", 0, RT_IPC_FLAG_PRIO);
    rt_mutex_init(&_mutex_tx, "mble_tx", RT_IPC_FLAG_PRIO);

    /* 6. Set receive callback */
    rt_device_set_rx_indicate(_ble_dev, ble_uart_rx_ind);

    /* 7. Create receiver thread */
    _rx_thread = rt_thread_create("ble_rx",
                                  ble_rx_thread_entry,
                                  RT_NULL,
                                  BLE_THREAD_STACK_SZ,
                                  BLE_THREAD_PRIO,
                                  10);
    if (!_rx_thread)
    {
        LOG_E("Failed to create ble_rx thread");
        return -RT_ERROR;
    }
    rt_thread_startup(_rx_thread);

    _inited = RT_TRUE;
    LOG_I("BLE UART ready: 115200/8N1, Idle Timeout: %dms", BLE_IDLE_TIMEOUT_MS);

    return RT_EOK;
}
INIT_APP_EXPORT(ble_uart_init);

void ble_uart_set_rx_handler(ble_rx_handler_t handler)
{
    _rx_handler = handler;
}

rt_size_t ble_uart_send(const void *buf, rt_size_t len)
{
    rt_size_t sent = 0;

    if (!_inited || !_ble_dev || !buf || len == 0)
    {
        return 0;
    }

    if (rt_mutex_take(&_mutex_tx, RT_WAITING_FOREVER) == RT_EOK)
    {
        sent = rt_device_write(_ble_dev, 0, buf, len);
        if (sent > 0)
        {
            _stat_tx_frames++;
            _stat_tx_bytes += sent;
        }
        rt_mutex_release(&_mutex_tx);
    }

    return sent;
}

rt_size_t ble_uart_send_str(const char *str)
{
    if (!str)
    {
        return 0;
    }
    return ble_uart_send(str, rt_strlen(str));
}

rt_size_t ble_send_frame(rt_uint8_t msg_type, rt_uint8_t seq, const rt_uint8_t *payload, rt_uint16_t len)
{
    rt_uint8_t header[6];
    rt_uint8_t crc_bytes[2];
    rt_uint16_t crc;
    rt_size_t total = 0;

    header[0] = BLE_FRAME_SOF1;
    header[1] = BLE_FRAME_SOF2;
    header[2] = msg_type;
    header[3] = seq;
    header[4] = (rt_uint8_t)(len >> 8);
    header[5] = (rt_uint8_t)(len & 0xFF);

    /* Calculate CRC across header + payload */
    crc = ble_crc16(header, 6);
    if (payload && len > 0)
    {
        /* Continued CRC calculation */
        for (rt_size_t i = 0; i < len; i++)
        {
            crc ^= payload[i];
            for (int j = 0; j < 8; j++)
            {
                if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
                else crc >>= 1;
            }
        }
    }
    crc_bytes[0] = (rt_uint8_t)(crc >> 8);
    crc_bytes[1] = (rt_uint8_t)(crc & 0xFF);

    if (rt_mutex_take(&_mutex_tx, RT_WAITING_FOREVER) == RT_EOK)
    {
        total += rt_device_write(_ble_dev, 0, header, 6);
        if (payload && len > 0)
        {
            total += rt_device_write(_ble_dev, 0, payload, len);
        }
        total += rt_device_write(_ble_dev, 0, crc_bytes, 2);

        _stat_tx_frames++;
        _stat_tx_bytes += total;
        rt_mutex_release(&_mutex_tx);
    }

    return total;
}

rt_bool_t ble_log_stream_enabled(void)
{
    return _log_stream_on;
}

void ble_log_stream_set(rt_bool_t enable)
{
    _log_stream_on = enable;
    LOG_I("BLE wireless log streaming: %s", enable ? "ENABLED" : "DISABLED");
}

void ble_uart_set_loopback(rt_bool_t enable)
{
    _loopback_enabled = enable;
    LOG_I("BLE loopback echo mode: %s", enable ? "ENABLED" : "DISABLED");
}

/* ==================== FinSH / MSH Commands ==================== */

#if defined(RT_USING_FINSH)

static int ble_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("\r\n+-------------------------------------------------------------+\r\n");
    rt_kprintf("|              BLE Bluetooth UART (uart4) Status              |\r\n");
    rt_kprintf("+-----------------------------------+-------------------------+\r\n");
    rt_kprintf("| Parameter / Item                  | Current Value           |\r\n");
    rt_kprintf("+-----------------------------------+-------------------------+\r\n");
    rt_kprintf("| Device Name                       | %-23s |\r\n", BLE_UART_NAME);
    rt_kprintf("| Hardware Pins                     | PB6(TX), PB7(RX)        |\r\n");
    rt_kprintf("| Power Supply LDO (PB9)            | %-23s |\r\n", hw_power_bt_get() ? "ON (3.3V High)" : "OFF (Low)");
    rt_kprintf("| Baudrate & Format                 | 115200 bps, 8N1         |\r\n");
    rt_kprintf("| Framing Mechanism                 | Inter-char Idle (%d ms)  |\r\n", BLE_IDLE_TIMEOUT_MS);
    rt_kprintf("| Wireless Log Streaming            | %-23s |\r\n", _log_stream_on ? "ACTIVE" : "INACTIVE");
    rt_kprintf("| Loopback Echo Mode                | %-23s |\r\n", _loopback_enabled ? "ENABLED" : "DISABLED");
    rt_kprintf("| Rx Frames Count                   | %-23u |\r\n", _stat_rx_frames);
    rt_kprintf("| Rx Bytes Count                    | %-23u |\r\n", _stat_rx_bytes);
    rt_kprintf("| Tx Frames Count                   | %-23u |\r\n", _stat_tx_frames);
    rt_kprintf("| Tx Bytes Count                    | %-23u |\r\n", _stat_tx_bytes);
    rt_kprintf("+-----------------------------------+-------------------------+\r\n\r\n");

    return 0;
}
MSH_CMD_EXPORT(ble_status, display BLE uart4 status and statistics);

static int ble_send(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: ble_send <message_string>\r\n");
        return -1;
    }

    rt_size_t len = rt_strlen(argv[1]);
    rt_size_t sent = ble_uart_send(argv[1], len);
    ble_uart_send("\r\n", 2);

    rt_kprintf("[BLE] Sent %d bytes to BLE SoC: \"%s\"\r\n", sent, argv[1]);
    return 0;
}
MSH_CMD_EXPORT(ble_send, send raw text to BLE SoC over uart4);

static int ble_send_test_frame(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    const char *test_payload = "{\"test\":\"BLE_FRAME_OK\",\"code\":200}";
    rt_size_t total = ble_send_frame(BLE_CMD_SYS_STATUS, 1, (const rt_uint8_t *)test_payload, rt_strlen(test_payload));

    rt_kprintf("[BLE] Generated & sent standard 0xAA 0x55 protocol frame (%d bytes)\r\n", total);
    return 0;
}
MSH_CMD_EXPORT(ble_send_test_frame, send standard 0xAA 0x55 protocol test frame);

static int ble_log_stream(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: ble_log_stream <on|off>\r\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        ble_log_stream_set(RT_TRUE);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        ble_log_stream_set(RT_FALSE);
    }
    else
    {
        rt_kprintf("Unknown parameter: %s, use 'on' or 'off'\r\n", argv[1]);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(ble_log_stream, control wireless log stream over BLE <on|off>);

static int ble_echo(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Usage: ble_echo <on|off>\r\n");
        return -1;
    }

    if (rt_strcmp(argv[1], "on") == 0)
    {
        ble_uart_set_loopback(RT_TRUE);
    }
    else if (rt_strcmp(argv[1], "off") == 0)
    {
        ble_uart_set_loopback(RT_FALSE);
    }
    else
    {
        rt_kprintf("Unknown parameter: %s, use 'on' or 'off'\r\n", argv[1]);
        return -1;
    }

    return 0;
}
MSH_CMD_EXPORT(ble_echo, toggle BLE UART loopback echo mode <on|off>);

#endif /* RT_USING_FINSH */
